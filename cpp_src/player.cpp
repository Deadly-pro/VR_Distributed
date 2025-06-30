#include "player.h"
#include <cmath>
#include <algorithm>
#include <fstream>

Player::Player() : position({ 0.0f, 1.7f, 0.0f }), target({ 0.0f, 1.7f, -1.0f }),
up({ 0.0f, 1.0f, 0.0f }), yaw(0.0f), pitch(0.0f), roll(0.0f), sensitivity(0.002f) {

    calibrationYaw = 0.0f;
    calibrationPitch = 0.0f;
    calibrationRoll = 0.0f;
    isCalibrated = false;

    leftHand.is_tracked = false;
    rightHand.is_tracked = false;
    leftHand.landmarks.resize(21);
    rightHand.landmarks.resize(21);
    handPoints.resize(42);
}
void Player::Update() {
    // Direct calculation using phone orientation
    // Based on search results: use target and up vectors

    float cosYaw = cosf(yaw);
    float sinYaw = sinf(yaw);
    float cosPitch = cosf(pitch);
    float sinPitch = sinf(pitch);
    float cosRoll = cosf(roll);
    float sinRoll = sinf(roll);

    // Calculate forward direction (where camera looks)
    Vector3 forward = {
        cosPitch * sinYaw,
        sinPitch,
        cosPitch * cosYaw
    };
    forward = Vector3Normalize(forward);

    // Calculate up vector with roll applied
    Vector3 baseUp = { 0.0f, 1.0f, 0.0f };
    up = {
        sinRoll,
        cosRoll,
        0.0f
    };
    up = Vector3Normalize(up);

    // Set target based on forward direction
    target = Vector3Add(position, forward);
}


void Player::HandleMouseLook(Vector2 mouseDelta) {
    yaw += mouseDelta.x * sensitivity;
    pitch -= mouseDelta.y * sensitivity;
    pitch = Clamp(pitch, -PI / 2.0f + 0.1f, PI / 2.0f - 0.1f);
}

void Player::Move(Vector3 direction, float deltaTime) {
    const float speed = 5.0f;
    Vector3 forward = GetForward();
    Vector3 right = GetRight();

    Vector3 movement = { 0 };
    movement = Vector3Add(movement, Vector3Scale(forward, direction.z * speed * deltaTime));
    movement = Vector3Add(movement, Vector3Scale(right, direction.x * speed * deltaTime));
    movement.y += direction.y * speed * deltaTime;

    position = Vector3Add(position, movement);
}
Vector3 Player::TransformHolisticToVR(const Vector3& holisticCoords, const std::string& handedness, float depth_scale) {
    // Completely static world positioning
    float vr_x = holisticCoords.x;
    float vr_y = 1.0f - holisticCoords.y;
    float vr_z = holisticCoords.z;

    // Mirror for natural interaction
    if (handedness == "Right") {
        vr_x = 1.0f - vr_x;
    }

    // Fixed world coordinates (no player position dependency)
    Vector3 static_position = {
        (vr_x - 0.5f) * 1.5f,      // X: -0.75m to +0.75m from world origin
        1.5f + (vr_y - 0.5f) * 0.6f, // Y: 1.2m to 1.8m height (chest area)
        1.5f + vr_z * 0.3f         // Z: 1.5m to 1.8m from world origin
    };

    return static_position;
}

void Player::UpdateHolisticHand(VRHand& hand, const HolisticHandData& holistic_data) {
    hand.label = holistic_data.handedness;
    hand.is_tracked = !holistic_data.landmarks.empty();
    hand.confidence = holistic_data.confidence;
    hand.estimated_depth = holistic_data.depth_scale;

    if (hand.is_tracked && holistic_data.landmarks.size() >= 21) {
        for (size_t i = 0; i < 21 && i < holistic_data.landmarks.size(); ++i) {
            Vector3 vr_pos = TransformHolisticToVR(
                holistic_data.landmarks[i],
                holistic_data.handedness,
                holistic_data.depth_scale
            );

            hand.landmarks[i].position = vr_pos;
            hand.landmarks[i].active = true;
            hand.landmarks[i].confidence = holistic_data.confidence;
            hand.landmarks[i].landmark_id = static_cast<int>(i);
        }
    }
    else {
        hand.is_tracked = false;
        for (auto& landmark : hand.landmarks) {
            landmark.active = false;
        }
    }
}

// Simplified hand rendering - removed complex coloring
void Player::DrawHolisticHands(const std::vector<HolisticHandData>& holisticData) {
    // Update hand data
    for (const auto& holistic_hand : holisticData) {
        if (holistic_hand.handedness == "Left") {
            UpdateHolisticHand(leftHand, holistic_hand);
        }
        else if (holistic_hand.handedness == "Right") {
            UpdateHolisticHand(rightHand, holistic_hand);
        }
    }

    // Draw left hand - simple red coloring
    if (leftHand.is_tracked) {
        for (size_t i = 0; i < leftHand.landmarks.size(); ++i) {
            if (leftHand.landmarks[i].active) {
                float pointSize = (i == 0) ? 0.03f : 0.02f; // Wrist slightly larger
                DrawSphere(leftHand.landmarks[i].position, pointSize, RED);
            }
        }

        // Simple hand skeleton
        const int connections[][2] = {
            {0,1}, {1,2}, {2,3}, {3,4},     // Thumb
            {0,5}, {5,6}, {6,7}, {7,8},     // Index
            {0,9}, {9,10}, {10,11}, {11,12}, // Middle
            {0,13}, {13,14}, {14,15}, {15,16}, // Ring
            {0,17}, {17,18}, {18,19}, {19,20}, // Pinky
            {5,9}, {9,13}, {13,17}          // Palm
        };

        for (const auto& conn : connections) {
            if (leftHand.landmarks[conn[0]].active && leftHand.landmarks[conn[1]].active) {
                DrawLine3D(leftHand.landmarks[conn[0]].position,
                    leftHand.landmarks[conn[1]].position, RED);
            }
        }
    }

    // Draw right hand - simple blue coloring
    if (rightHand.is_tracked) {
        for (size_t i = 0; i < rightHand.landmarks.size(); ++i) {
            if (rightHand.landmarks[i].active) {
                float pointSize = (i == 0) ? 0.03f : 0.02f; // Wrist slightly larger
                DrawSphere(rightHand.landmarks[i].position, pointSize, BLUE);
            }
        }

        const int connections[][2] = {
            {0,1}, {1,2}, {2,3}, {3,4},
            {0,5}, {5,6}, {6,7}, {7,8},
            {0,9}, {9,10}, {10,11}, {11,12},
            {0,13}, {13,14}, {14,15}, {15,16},
            {0,17}, {17,18}, {18,19}, {19,20},
            {5,9}, {9,13}, {13,17}
        };

        for (const auto& conn : connections) {
            if (rightHand.landmarks[conn[0]].active && rightHand.landmarks[conn[1]].active) {
                DrawLine3D(rightHand.landmarks[conn[0]].position,
                    rightHand.landmarks[conn[1]].position, BLUE);
            }
        }
    }
}

// Legacy support - simplified
void Player::DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData) {
    std::vector<HolisticHandData> holistic_data;
    for (const auto& hand : handData) {
        HolisticHandData holistic_hand;
        holistic_hand.handedness = hand.first;
        holistic_hand.landmarks = hand.second;
        holistic_hand.distance_factor = 1.0f;
        holistic_hand.depth_scale = 1.0f;
        holistic_hand.shoulder_calibrated = true; // Enable by default
        holistic_hand.confidence = 0.8f;
        holistic_data.push_back(holistic_hand);
    }
    DrawHolisticHands(holistic_data);
}

Camera Player::GetLeftEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, -eyeSeparation / 2.0f));
    camera.target = Vector3Add(target, Vector3Scale(right, -eyeSeparation / 2.0f));
    camera.up = up;
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

Camera Player::GetRightEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, eyeSeparation / 2.0f));
    camera.target = Vector3Add(target, Vector3Scale(right, eyeSeparation / 2.0f));
    camera.up = up;
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

Vector3 Player::GetForward() const {
    return Vector3Normalize(Vector3Subtract(target, position));
}

Vector3 Player::GetRight() const {
    return Vector3Normalize(Vector3CrossProduct(GetForward(), up));
}

Vector3 Player::GetPosition() const {
    return position;
}

Vector3 Player::Getup() const {
    return Vector3Normalize(Vector3CrossProduct(GetRight(), GetForward()));
}

void Player::SetYawPitchRoll(float newYaw, float newPitch, float newRoll) {
    yaw = newYaw;
    pitch = -newPitch;  // Invert pitch for natural VR feel
    roll = newRoll;

    // Apply calibration if needed
    if (isCalibrated) {
        yaw -= calibrationYaw;
        pitch -= calibrationPitch;
        roll -= calibrationRoll;
    }

    // Simple normalization
    while (yaw > 2.0f * PI) yaw -= 2.0f * PI;
    while (yaw < 0.0f) yaw += 2.0f * PI;

    // Convert yaw to [-π, π] range
    if (yaw > PI) yaw -= 2.0f * PI;

    // Clamp pitch for comfort
    pitch = Clamp(pitch, -PI / 2.0f, PI / 2.0f);
}
