#include "gesture_recognition.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <raymath.h>

GestureRecognizer::GestureRecognizer() {
    gestureStartTime = 0.0f;
    currentGesture = GestureType::NONE;
    motionHistory.reserve(10);
}

GestureData GestureRecognizer::RecognizeGesture(const HandLandmarks& hand) {
    GestureData result;
    result.type = GestureType::NONE;
    result.confidence = 0.0f;
    result.position = hand.landmarks[0]; // Wrist position
    result.direction = { 0, 0, 0 };
    result.duration = 0.0f;
    result.isActive = false;

    if (!hand.active[0]) return result; // Hand not tracked

    // Check static gestures (in order of priority)
    if (IsPinchGesture(hand)) {
        result.type = GestureType::PINCH;
        result.confidence = 0.9f;
    }
    else if (IsIndexFingerExtended(hand)) {
        result.type = GestureType::POINT;
        result.confidence = 0.8f;
    }
    else if (IsFistGesture(hand)) {
        result.type = GestureType::FIST;
        result.confidence = 0.8f;
    }
    else if (IsOpenPalmGesture(hand)) {
        result.type = GestureType::OPEN_PALM;
        result.confidence = 0.7f;
    }
    else if (IsPeaceSignGesture(hand)) {
        result.type = GestureType::PEACE_SIGN;
        result.confidence = 0.8f;
    }
    else if (IsThumbsUpGesture(hand)) {
        result.type = GestureType::THUMBS_UP;
        result.confidence = 0.8f;
    }
    else if (IsOKSignGesture(hand)) {
        result.type = GestureType::OK_SIGN;
        result.confidence = 0.8f;
    }

    // Check motion gestures
    Vector3 swipeDirection;
    if (IsSwipeGesture(hand, swipeDirection)) {
        if (swipeDirection.x > 0.5f) {
            result.type = GestureType::SWIPE_RIGHT;
        }
        else if (swipeDirection.x < -0.5f) {
            result.type = GestureType::SWIPE_LEFT;
        }
        result.direction = swipeDirection;
        result.confidence = 0.7f;
    }

    result.isActive = (result.type != GestureType::NONE);

    // Update motion history
    UpdateMotionHistory(hand.landmarks[0]);

    return result;
}

bool GestureRecognizer::IsIndexFingerExtended(const HandLandmarks& hand) {
    if (!hand.active[5] || !hand.active[8]) return false;

    Vector3 mcp = hand.landmarks[5];   // Index MCP
    Vector3 tip = hand.landmarks[8];   // Index tip
    Vector3 wrist = hand.landmarks[0]; // Wrist

    // Check if index finger is extended
    float indexLength = Vector3Distance(mcp, tip);
    float wristToTip = Vector3Distance(wrist, tip);

    // Index finger should be extended and other fingers relatively closed
    bool indexExtended = indexLength > 0.07f && wristToTip > 0.12f;

    // Check if other fingers are more closed than index
    float middleLength = Vector3Distance(hand.landmarks[9], hand.landmarks[12]);
    float ringLength = Vector3Distance(hand.landmarks[13], hand.landmarks[16]);

    return indexExtended && (indexLength > middleLength * 1.2f) && (indexLength > ringLength * 1.2f);
}

bool GestureRecognizer::IsPinchGesture(const HandLandmarks& hand, float threshold) {
    if (!hand.active[4] || !hand.active[8]) return false;

    Vector3 thumbTip = hand.landmarks[4];
    Vector3 indexTip = hand.landmarks[8];

    float distance = Vector3Distance(thumbTip, indexTip);
    return distance < threshold;
}

bool GestureRecognizer::IsFistGesture(const HandLandmarks& hand) {
    // Check if all fingertips are close to palm
    Vector3 palm = hand.landmarks[0]; // Wrist as palm reference

    float avgDistance = 0.0f;
    int fingerCount = 0;

    // Check fingertips: thumb(4), index(8), middle(12), ring(16), pinky(20)
    int fingertips[] = { 4, 8, 12, 16, 20 };

    for (int tip : fingertips) {
        if (hand.active[tip]) {
            avgDistance += Vector3Distance(palm, hand.landmarks[tip]);
            fingerCount++;
        }
    }

    if (fingerCount == 0) return false;
    avgDistance /= fingerCount;

    return avgDistance < 0.08f; // All fingers close to palm
}

bool GestureRecognizer::IsOpenPalmGesture(const HandLandmarks& hand) {
    // Check if all fingers are extended
    float openness = GetHandOpenness(hand);
    return openness > 0.8f;
}

bool GestureRecognizer::IsPeaceSignGesture(const HandLandmarks& hand) {
    if (!hand.active[8] || !hand.active[12]) return false;

    // Index and middle fingers extended, others closed
    float indexExt = GetFingerExtension(hand, 1); // Index finger
    float middleExt = GetFingerExtension(hand, 2); // Middle finger
    float ringExt = GetFingerExtension(hand, 3);   // Ring finger
    float pinkyExt = GetFingerExtension(hand, 4);  // Pinky finger

    return (indexExt > 0.7f) && (middleExt > 0.7f) && (ringExt < 0.4f) && (pinkyExt < 0.4f);
}

bool GestureRecognizer::IsThumbsUpGesture(const HandLandmarks& hand) {
    if (!hand.active[4]) return false;

    // Thumb extended upward, other fingers closed
    Vector3 thumbTip = hand.landmarks[4];
    Vector3 wrist = hand.landmarks[0];

    // Check if thumb is pointing up
    bool thumbUp = (thumbTip.y > wrist.y + 0.05f);

    // Check if other fingers are closed
    float otherFingersOpenness = 0.0f;
    for (int i = 1; i <= 4; i++) {
        if (i != 0) { // Skip thumb
            otherFingersOpenness += GetFingerExtension(hand, i);
        }
    }
    otherFingersOpenness /= 4.0f;

    return thumbUp && (otherFingersOpenness < 0.3f);
}

bool GestureRecognizer::IsOKSignGesture(const HandLandmarks& hand) {
    if (!hand.active[4] || !hand.active[8]) return false;

    // Thumb and index form circle, other fingers extended
    Vector3 thumbTip = hand.landmarks[4];
    Vector3 indexTip = hand.landmarks[8];

    float distance = Vector3Distance(thumbTip, indexTip);
    bool circleFormed = (distance < 0.04f);

    // Check if other fingers are extended
    float middleExt = GetFingerExtension(hand, 2);
    float ringExt = GetFingerExtension(hand, 3);
    float pinkyExt = GetFingerExtension(hand, 4);

    return circleFormed && (middleExt > 0.6f) && (ringExt > 0.6f) && (pinkyExt > 0.6f);
}

bool GestureRecognizer::IsSwipeGesture(const HandLandmarks& hand, Vector3& direction) {
    if (motionHistory.size() < 5) return false;

    Vector3 startPos = motionHistory[0];
    Vector3 endPos = motionHistory.back();

    Vector3 movement = Vector3Subtract(endPos, startPos);
    float distance = Vector3Length(movement);

    if (distance > 0.15f) { // 15cm movement threshold
        direction = Vector3Normalize(movement);
        return true;
    }

    return false;
}

float GestureRecognizer::GetFingerExtension(const HandLandmarks& hand, int fingerIndex) {
    // Finger landmark indices: thumb(1-4), index(5-8), middle(9-12), ring(13-16), pinky(17-20)
    int baseIndices[] = { 1, 5, 9, 13, 17 };
    int tipIndices[] = { 4, 8, 12, 16, 20 };

    if (fingerIndex < 0 || fingerIndex > 4) return 0.0f;

    int baseIdx = baseIndices[fingerIndex];
    int tipIdx = tipIndices[fingerIndex];

    if (!hand.active[baseIdx] || !hand.active[tipIdx]) return 0.0f;

    Vector3 base = hand.landmarks[baseIdx];
    Vector3 tip = hand.landmarks[tipIdx];
    Vector3 wrist = hand.landmarks[0];

    float fingerLength = Vector3Distance(base, tip);
    float maxLength = 0.09f; // Approximate max finger length

    return Clamp(fingerLength / maxLength, 0.0f, 1.0f);
}

float GestureRecognizer::GetHandOpenness(const HandLandmarks& hand) {
    float totalExtension = 0.0f;
    for (int i = 0; i < 5; i++) {
        totalExtension += GetFingerExtension(hand, i);
    }
    return totalExtension / 5.0f;
}

void GestureRecognizer::UpdateMotionHistory(const Vector3& position) {
    motionHistory.push_back(position);
    if (motionHistory.size() > 10) {
        motionHistory.erase(motionHistory.begin());
    }
}
