#ifndef PLAYER_H
#define PLAYER_H

#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <string>
#include <unordered_map>

struct HandPoint {
    Vector3 position;
    bool active;
    float confidence;
    int landmark_id;
};

struct VRHand {
    std::vector<HandPoint> landmarks;
    std::string label;
    float confidence;
    bool is_tracked;
    float estimated_depth;
    float hand_scale; // For size-based depth estimation
};

class Player {
public:
    Player();
    void Update();
    void HandleMouseLook(Vector2 mouseDelta);
    void Move(Vector3 direction, float deltaTime);
    void DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData);

    Camera GetLeftEyeCamera(float eyeSeparation) const;
    Camera GetRightEyeCamera(float eyeSeparation) const;
    Vector3 GetForward() const;
    Vector3 GetRight() const;
    Vector3 GetPosition() const;
    Vector3 Getup() const;
    void SetYawPitch(float newYaw, float newPitch);

private:
    float calibrationYaw = 0.0f;
    float calibrationPitch = 0.0f;
    bool isCalibrated = false; 
    Vector3 TransformMediaPipeToWorld(const Vector3& mediapipeCoords, const std::string& handedness, float estimated_depth = 0.5f, float hand_scale = 1.0f);
    Vector3 GetShoulderPosition(bool is_left_hand) const;
    float EstimateDepthFromScale(float hand_scale) const;
    Vector3 CorrectHandMirroring(const Vector3& coords) const;

    Vector3 position;
    Vector3 target;
    Vector3 up;
    float yaw;
    float pitch;
    float sensitivity;

    // Anthropometric constants
    static constexpr float SHOULDER_WIDTH = 0.35f;
    static constexpr float SHOULDER_HEIGHT = 0.12f;
    static constexpr float ARM_LENGTH = 0.55f;
    static constexpr float HAND_FORWARD_OFFSET = 0.25f;
    static constexpr float BLEND_FACTOR = 0.8f; // More camera tracking for responsive feel
    static constexpr float REFERENCE_HAND_SPAN = 0.19f; // 19cm average hand span
    static constexpr float MIN_DEPTH = 0.15f; // 15cm minimum
    static constexpr float MAX_DEPTH = 1.5f;  // 1.5m maximum

    // VR Hand data
    VRHand leftHand;
    VRHand rightHand;

    // Hand visualization
    std::vector<HandPoint> handPoints;
    void UpdateHandFromData(VRHand& hand, const std::vector<Vector3>& landmarks, const std::string& handedness);
};

#endif  // PLAYER_H
