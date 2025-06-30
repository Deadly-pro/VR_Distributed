#include "player.h"
#include <cmath>
#include <algorithm>

float calibrationYaw = 0.0f;
float calibrationPitch = 0.0f;
bool isCalibrated = false;
float yaw;
float pitch;


Player::Player() : position({ 0.0f, 1.7f, 5.0f }), target({ 0.0f, 1.7f, 0.0f }),
up({ 0.0f, 1.0f, 0.0f }), yaw(0.0f), pitch(0.0f), sensitivity(0.002f) {
    // Initialize hand data
    leftHand.is_tracked = false;
    rightHand.is_tracked = false;
    leftHand.landmarks.resize(21);
    rightHand.landmarks.resize(21);
    handPoints.resize(42); // 21 points per hand * 2 hands

}

void Player::Update() {
    // Update target based on yaw and pitch
    target.x = position.x + cosf(pitch) * sinf(yaw);
    target.y = position.y + sinf(pitch);
    target.z = position.z + cosf(pitch) * cosf(yaw);
}

void Player::HandleMouseLook(Vector2 mouseDelta) {
    yaw += mouseDelta.x * sensitivity;
    pitch -= mouseDelta.y * sensitivity;

    // Clamp pitch to prevent over-rotation
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

Vector3 Player::CorrectHandMirroring(const Vector3& coords) const {
    // Flip X coordinate to correct webcam mirroring
    return { 1.0f - coords.x, coords.y, coords.z };
}

float Player::EstimateDepthFromScale(float hand_scale) const {
    // Larger hand scale = closer to camera = smaller depth value
    // Using inverse relationship with clamping
    if (hand_scale <= 0.0f) return MAX_DEPTH;

    float estimated_depth = REFERENCE_HAND_SPAN / hand_scale;
    return Clamp(estimated_depth, MIN_DEPTH, MAX_DEPTH);
}

Vector3 Player::GetShoulderPosition(bool is_left_hand) const {
    Vector3 forward = GetForward();
    Vector3 right = GetRight();
    Vector3 up_vec = Getup();

    // Calculate shoulder position relative to head/camera
    Vector3 shoulder_offset = Vector3Scale(right, is_left_hand ? -SHOULDER_WIDTH / 2 : SHOULDER_WIDTH / 2);
    Vector3 shoulder_pos = Vector3Add(position, shoulder_offset);
    shoulder_pos = Vector3Add(shoulder_pos, Vector3Scale(up_vec, -SHOULDER_HEIGHT));

    return shoulder_pos;
}

Vector3 Player::TransformMediaPipeToWorld(const Vector3& mediapipeCoords, const std::string& handedness, float estimated_depth, float hand_scale) {
    // First, correct the mirroring from webcam
    Vector3 corrected_coords = CorrectHandMirroring(mediapipeCoords);

    bool is_left_hand = (handedness == "Left");

    // Use hand scale for better depth estimation
    if (hand_scale > 0.0f) {
        estimated_depth = EstimateDepthFromScale(hand_scale);
    }
    else if (estimated_depth <= 0.0f) {
        // Fallback depth estimation from MediaPipe z-coordinate
        estimated_depth = MIN_DEPTH + (fabsf(corrected_coords.z) * (MAX_DEPTH - MIN_DEPTH));
    }

    // Convert normalized coordinates to camera space
    float fov_rad = 60.0f * DEG2RAD;
    float aspect_ratio = (float)GetScreenWidth() / GetScreenHeight();

    // Camera space coordinates with corrected mirroring
    float cam_x = (corrected_coords.x - 0.5f) * 2.0f * estimated_depth * tanf(fov_rad * 0.5f) * aspect_ratio;
    float cam_y = (0.5f - corrected_coords.y) * 2.0f * estimated_depth * tanf(fov_rad * 0.5f);
    float cam_z = -estimated_depth; // Negative for forward direction

    Vector3 camera_space = { cam_x, cam_y, cam_z };

    // Transform to world space using player's orientation
    Vector3 forward = GetForward();
    Vector3 right = GetRight();
    Vector3 up_vec = Getup();

    Vector3 world_pos = Vector3Add(position,
        Vector3Add(Vector3Add(
            Vector3Scale(right, camera_space.x),
            Vector3Scale(up_vec, camera_space.y)),
            Vector3Scale(forward, -camera_space.z)));

    // Get anatomically correct shoulder position
    Vector3 shoulder_pos = GetShoulderPosition(is_left_hand);

    // Add arm extension from shoulder toward camera space
    Vector3 arm_direction = Vector3Normalize(Vector3Subtract(world_pos, shoulder_pos));
    Vector3 anatomical_pos = Vector3Add(shoulder_pos, Vector3Scale(arm_direction, HAND_FORWARD_OFFSET));

    // Blend camera tracking with anatomical positioning
    Vector3 final_pos = Vector3Add(
        Vector3Scale(world_pos, BLEND_FACTOR),
        Vector3Scale(anatomical_pos, 1.0f - BLEND_FACTOR)
    );

    return final_pos;
}

void Player::UpdateHandFromData(VRHand& hand, const std::vector<Vector3>& landmarks, const std::string& handedness) {
    hand.label = handedness;
    hand.is_tracked = !landmarks.empty();
    hand.confidence = 1.0f;

    if (hand.is_tracked && landmarks.size() >= 21) {
        // Calculate hand scale using multiple reference points for better depth estimation
        Vector3 thumb_tip = landmarks[4];
        Vector3 index_tip = landmarks[8];
        Vector3 middle_tip = landmarks[12];
        Vector3 wrist = landmarks[0];

        // Calculate multiple spans for more robust depth estimation
        float thumb_index_span = Vector3Distance(thumb_tip, index_tip);
        float wrist_middle_span = Vector3Distance(wrist, middle_tip);

        // Use the larger span for more stable depth estimation
        hand.hand_scale = fmaxf(thumb_index_span, wrist_middle_span * 0.7f); // Scale factor for different measurements

        // Estimate depth from hand scale
        hand.estimated_depth = EstimateDepthFromScale(hand.hand_scale);

        for (size_t i = 0; i < 21 && i < landmarks.size(); ++i) {
            Vector3 world_pos = TransformMediaPipeToWorld(landmarks[i], handedness, hand.estimated_depth, hand.hand_scale);

            hand.landmarks[i].position = world_pos;
            hand.landmarks[i].active = true;
            hand.landmarks[i].confidence = 1.0f;
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

void Player::DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData) {
    // Update hand data
    for (const auto& hand : handData) {
        if (hand.first == "Left") {
            UpdateHandFromData(leftHand, hand.second, hand.first);
        }
        else if (hand.first == "Right") {
            UpdateHandFromData(rightHand, hand.second, hand.first);
        }
    }

    // Draw left hand (RED for left, corrected positioning)
    if (leftHand.is_tracked) {
        for (const auto& point : leftHand.landmarks) {
            if (point.active) {
                DrawSphere(point.position, 0.008f, RED);
            }
        }

        // Draw hand skeleton connections
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

    // Draw right hand (BLUE for right, corrected positioning)
    if (rightHand.is_tracked) {
        for (const auto& point : rightHand.landmarks) {
            if (point.active) {
                DrawSphere(point.position, 0.008f, BLUE);
            }
        }

        // Draw hand skeleton connections
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

Camera Player::GetLeftEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, -eyeSeparation));
    camera.target = Vector3Add(target, Vector3Scale(right, -eyeSeparation));
    camera.up = up;
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

Camera Player::GetRightEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, eyeSeparation));
    camera.target = Vector3Add(target, Vector3Scale(right, eyeSeparation));
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

void Player::SetYawPitch(float newYaw, float newPitch) {
    float transformedYaw, transformedPitch;

    // Swap axes due to landscape orientation:
    transformedYaw = newPitch;       // phone pitch → VR yaw
    transformedPitch = -newYaw;      // phone yaw → VR pitch (invert for correct up/down)

    // Apply calibration if user calibrated
    if (isCalibrated) {
        transformedYaw -= calibrationYaw;
        transformedPitch -= calibrationPitch;
    }

    // Normalize yaw to [-PI, PI]
    while (transformedYaw > PI) transformedYaw -= 2.0f * PI;
    while (transformedYaw < -PI) transformedYaw += 2.0f * PI;

    // Optional smoothing
    const float smoothingFactor = 0.15f;
    yaw = yaw * (1.0f - smoothingFactor) + transformedYaw * smoothingFactor;

    float clampedPitch = Clamp(transformedPitch, -PI/2.0f + 0.2f, PI/2.0f - 0.2f);
    pitch = pitch * (1.0f - smoothingFactor) + clampedPitch * smoothingFactor;

    // Optional deadzone to reduce micro-drift
    const float deadzone = 0.01f;
    if (fabsf(transformedYaw) < deadzone) transformedYaw = 0.0f;
    if (fabsf(transformedPitch) < deadzone) transformedPitch = 0.0f;
}

void CalibrateVROrientation() {
    // Call this when user is looking straight forward to set baseline
    calibrationYaw = yaw;
    calibrationPitch = pitch;
    isCalibrated = true;
}

void ResetVRDrift() {
    // Call periodically or when user presses a button to combat gyro drift
    CalibrateVROrientation();
}
