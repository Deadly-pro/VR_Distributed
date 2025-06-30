#ifndef VR_MOUSE_H
#define VR_MOUSE_H

#include "raylib.h"
#include "gesture_recognition.h"

struct VRMouse {
    Vector3 position;
    Vector2 panelUV;
    bool isActive;
    bool isClicking;
    bool isDragging;
    float clickCooldown;
    float dragThreshold;
    Vector2 lastClickUV;

    // Future gesture support
    GestureType activeGesture;
    float gestureStartTime;
};

class VRMouseController {
public:
    VRMouseController();

    void SetPanelInfo(const Vector3& position, const Vector3& size);
    void Update(const HandLandmarks& rightHand, float deltaTime);
    void Draw();

    // Mouse data access
    bool GetMouseData(Vector2& mouseUV, bool& isClicking, bool& isDragging);
    GestureType GetActiveGesture() const { return vrMouse.activeGesture; }

    // Configuration
    void SetClickThreshold(float threshold) { clickThreshold = threshold; }
    void SetDragThreshold(float threshold) { vrMouse.dragThreshold = threshold; }

private:
    VRMouse vrMouse;
    GestureRecognizer gestureRecognizer;
    Vector3 panelPosition;
    Vector3 panelSize;
    float clickThreshold;

    Vector2 GetPanelUVFromWorldPos(const Vector3& worldPos);
    bool IsPointingAtPanel(const HandLandmarks& hand);
    void UpdateMousePosition(const HandLandmarks& hand);
    void UpdateClickState(const HandLandmarks& hand);
    void UpdateDragState(const HandLandmarks& hand);
    void DrawCursor();
    void DrawRayToPanel();
};

#endif // VR_MOUSE_H
