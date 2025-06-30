#ifndef PLAYER_H
#define PLAYER_H

#include "raylib.h"
#include "raymath.h"
#include "holistic_data.h"
#include <vector>
#include <string>

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
    float hand_scale;
};

class Player {
public:
    Player();
    void Update();
    void HandleMouseLook(Vector2 mouseDelta);
    void Move(Vector3 direction, float deltaTime);

    // Enhanced holistic methods
    void DrawHolisticHands(const std::vector<HolisticHandData>& holisticData);
    void DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData);

    Camera GetLeftEyeCamera(float eyeSeparation) const;
    Camera GetRightEyeCamera(float eyeSeparation) const;
    Vector3 GetForward() const;
    Vector3 GetRight() const;
    Vector3 GetPosition() const;
    Vector3 Getup() const;

    void SetYawPitch(float newYaw, float newPitch);
    void SetYawPitchRoll(float newYaw, float newPitch, float newRoll);
    void Calibrate();
    void ResetVRDrift();

private:
    float calibrationYaw;
    float calibrationPitch;
    float calibrationRoll;
    bool isCalibrated;

    Vector3 TransformHolisticToVR(const Vector3& holisticCoords, const std::string& handedness, float depth_scale);
    void UpdateHolisticHand(VRHand& hand, const HolisticHandData& holistic_data);

    Vector3 position;
    Vector3 target;
    Vector3 up;
    float yaw;
    float pitch;
    float roll;
    float sensitivity;

    VRHand leftHand;
    VRHand rightHand;
    std::vector<HandPoint> handPoints;
};

#endif // PLAYER_H
