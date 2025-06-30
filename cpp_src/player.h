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

struct VRMouse {
    Vector3 position;      // Index fingertip position
    Vector3 hitPoint;      // Where the ray hits the panel
    Vector2 panelUV;       // UV coordinates on panel
    bool isActive;
    bool isClicking;       // Right click (pinch)
    bool leftClicking;     // Left click (scissors)
    bool isDragging;
    float clickCooldown;
    float cursorSize;      // Added missing member
    float clickThreshold;  // Added missing member
    //lazer pointer
    bool showLaser;
    float laserIntensity;
};

class Player {
public:
    Player();
    void Update();
    void HandleMouseLook(Vector2 mouseDelta);
    void Move(Vector3 direction, float deltaTime);

    // Hand and VR mouse methods
    void DrawHolisticHands(const std::vector<HolisticHandData>& holisticData);
    void DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData);
    void SetPanelInfo(const Vector3& position, const Vector3& size);
    bool GetVRMouseData(Vector2& mouseUV, bool& isLeftClicking, bool& isRightClicking, bool& isDragging);
    void DrawLaserPointer();
    
    Camera GetLeftEyeCamera(float eyeSeparation) const;
    Camera GetRightEyeCamera(float eyeSeparation) const;
    Vector3 GetForward() const;
    Vector3 GetRight() const;
    Vector3 GetPosition() const;
    Vector3 Getup() const;

    void SetYawPitchRoll(float newYaw, float newPitch, float newRoll);
    void Calibrate();
    void ResetVRDrift();

private:
    // VR Mouse system
    VRMouse vrMouse;
    Vector3 panelPosition;  // Added missing member
    Vector3 panelSize;      // Added missing member

    // VR Mouse methods - Added missing declarations
    bool IsIndexFingerExtended(const VRHand& hand);
    bool IsPinchGesture(const VRHand& hand);
    bool IsScissorsGesture(const VRHand& hand);
    Vector3 GetIndexFingerRayDirection(const VRHand& hand);
    bool GetIndexFingerPanelIntersection(const VRHand& hand, Vector3& hitPoint, Vector2& hitUV);
    Vector2 GetPanelUVFromWorldPos(const Vector3& worldPos);
    void UpdateVRMouse(const VRHand& rightHand);
    void DrawVRMouse();

    // Calibration variables
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
