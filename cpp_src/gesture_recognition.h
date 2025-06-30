#ifndef GESTURE_RECOGNITION_H
#define GESTURE_RECOGNITION_H

#include "raylib.h"
#include <vector>
#include <string>

enum class GestureType {
    NONE,
    POINT,           // Index finger extended
    PINCH,           // Thumb and index touching
    FIST,            // All fingers closed
    OPEN_PALM,       // All fingers extended
    PEACE_SIGN,      // Index and middle extended
    THUMBS_UP,       // Thumb extended, others closed
    OK_SIGN,         // Thumb and index in circle
    SWIPE_LEFT,      // Hand moving left
    SWIPE_RIGHT,     // Hand moving right
    GRAB,            // Fingers closing motion
    RELEASE          // Fingers opening motion
};

struct GestureData {
    GestureType type;
    float confidence;
    Vector3 position;
    Vector3 direction;
    float duration;
    bool isActive;
};

struct HandLandmarks {
    Vector3 landmarks[21];
    bool active[21];
    std::string handedness;
    float confidence;
};

class GestureRecognizer {
public:
    GestureRecognizer();

    // Core gesture recognition
    GestureData RecognizeGesture(const HandLandmarks& hand);

    // Individual gesture checks
    bool IsIndexFingerExtended(const HandLandmarks& hand);
    bool IsPinchGesture(const HandLandmarks& hand, float threshold = 0.03f);
    bool IsFistGesture(const HandLandmarks& hand);
    bool IsOpenPalmGesture(const HandLandmarks& hand);
    bool IsPeaceSignGesture(const HandLandmarks& hand);
    bool IsThumbsUpGesture(const HandLandmarks& hand);
    bool IsOKSignGesture(const HandLandmarks& hand);

    // Motion-based gestures
    bool IsSwipeGesture(const HandLandmarks& hand, Vector3& direction);
    bool IsGrabGesture(const HandLandmarks& hand);
    bool IsReleaseGesture(const HandLandmarks& hand);

    // Utility functions
    float GetFingerExtension(const HandLandmarks& hand, int fingerIndex);
    Vector3 GetFingerDirection(const HandLandmarks& hand, int fingerIndex);
    float GetHandOpenness(const HandLandmarks& hand);

private:
    HandLandmarks previousHand;
    std::vector<Vector3> motionHistory;
    float gestureStartTime;
    GestureType currentGesture;

    void UpdateMotionHistory(const Vector3& position);
    float CalculateMotionDirection(Vector3& direction);
};

#endif // GESTURE_RECOGNITION_H
