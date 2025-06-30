#include "vr_mouse.h"
#include <fstream>
#include "raymath.h"
#include <cmath>
VRMouseController::VRMouseController() {
    vrMouse.isActive = false;
    vrMouse.isClicking = false;
    vrMouse.isDragging = false;
    vrMouse.clickCooldown = 0.0f;
    vrMouse.dragThreshold = 0.02f;
    vrMouse.position = { 0, 0, 0 };
    vrMouse.panelUV = { 0.5f, 0.5f };
    vrMouse.activeGesture = GestureType::NONE;
    clickThreshold = 0.03f;
}

void VRMouseController::SetPanelInfo(const Vector3& position, const Vector3& size) {
    panelPosition = position;
    panelSize = size;
}

void VRMouseController::Update(const HandLandmarks& rightHand, float deltaTime) {
    // Update cooldowns
    if (vrMouse.clickCooldown > 0.0f) {
        vrMouse.clickCooldown -= deltaTime;
    }

    if (!rightHand.active[0]) {
        vrMouse.isActive = false;
        vrMouse.activeGesture = GestureType::NONE;
        return;
    }

    // Recognize current gesture
    GestureData gesture = gestureRecognizer.RecognizeGesture(rightHand);
    vrMouse.activeGesture = gesture.type;

    // Check if pointing at panel
    if (IsPointingAtPanel(rightHand)) {
        vrMouse.isActive = true;
        UpdateMousePosition(rightHand);
        UpdateClickState(rightHand);
        UpdateDragState(rightHand);
    }
    else {
        vrMouse.isActive = false;
        vrMouse.isClicking = false;
        vrMouse.isDragging = false;
    }
}

void VRMouseController::Draw() {
    if (!vrMouse.isActive) return;

    DrawCursor();
    DrawRayToPanel();
}

bool VRMouseController::GetMouseData(Vector2& mouseUV, bool& isClicking, bool& isDragging) {
    if (!vrMouse.isActive) return false;

    mouseUV = vrMouse.panelUV;
    isClicking = vrMouse.isClicking;
    isDragging = vrMouse.isDragging;
    return true;
}

Vector2 VRMouseController::GetPanelUVFromWorldPos(const Vector3& worldPos) {
    Vector3 panelMin = {
        panelPosition.x - panelSize.x / 2.0f,
        panelPosition.y - panelSize.y / 2.0f,
        panelPosition.z - panelSize.z / 2.0f
    };

    Vector2 uv = {
        (worldPos.x - panelMin.x) / panelSize.x,
        1.0f - (worldPos.y - panelMin.y) / panelSize.y
    };

    uv.x = Clamp(uv.x, 0.0f, 1.0f);
    uv.y = Clamp(uv.y, 0.0f, 1.0f);

    return uv;
}

bool VRMouseController::IsPointingAtPanel(const HandLandmarks& hand) {
    if (!hand.active[8]) return false; // Index fingertip

    // Check if index finger is extended and pointing toward panel
    bool indexExtended = gestureRecognizer.IsIndexFingerExtended(hand);

    if (!indexExtended) return false;

    // Check if fingertip is within reasonable distance to panel
    Vector3 indexTip = hand.landmarks[8];
    float distanceToPanel = fabsf(indexTip.z - panelPosition.z);

    return distanceToPanel < 0.5f; // Within 50cm of panel
}

void VRMouseController::UpdateMousePosition(const HandLandmarks& hand) {
    if (!hand.active[8]) return;

    Vector3 indexTip = hand.landmarks[8];
    vrMouse.position = indexTip;
    vrMouse.panelUV = GetPanelUVFromWorldPos(indexTip);
}

void VRMouseController::UpdateClickState(const HandLandmarks& hand) {
    bool isPinching = gestureRecognizer.IsPinchGesture(hand, clickThreshold);

    if (isPinching && vrMouse.clickCooldown <= 0.0f && !vrMouse.isClicking) {
        vrMouse.isClicking = true;
        vrMouse.clickCooldown = 0.3f; // 300ms cooldown
        vrMouse.lastClickUV = vrMouse.panelUV;

        // Log click for debugging
        std::ofstream debug("vr_mouse_clicks.log", std::ios::app);
        debug << "VR Click at UV: " << vrMouse.panelUV.x << ", " << vrMouse.panelUV.y << std::endl;
    }
    else if (!isPinching) {
        vrMouse.isClicking = false;
    }
}

void VRMouseController::UpdateDragState(const HandLandmarks& hand) {
    if (vrMouse.isClicking) {
        Vector2 currentUV = vrMouse.panelUV;
        Vector2 clickUV = vrMouse.lastClickUV;

        float dragDistance = Vector2Distance(currentUV, clickUV);

        if (dragDistance > vrMouse.dragThreshold) {
            vrMouse.isDragging = true;
        }
    }
    else {
        vrMouse.isDragging = false;
    }
}

void VRMouseController::DrawCursor() {
    Color cursorColor = YELLOW;

    if (vrMouse.isClicking) {
        cursorColor = GREEN;
    }
    else if (vrMouse.isDragging) {
        cursorColor = ORANGE;
    }

    // Draw cursor at fingertip
    DrawSphere(vrMouse.position, 0.008f, cursorColor);

    // Draw cursor on panel
    Vector3 panelHitPoint = {
        panelPosition.x + (vrMouse.panelUV.x - 0.5f) * panelSize.x,
        panelPosition.y + (0.5f - vrMouse.panelUV.y) * panelSize.y,
        panelPosition.z
    };

    DrawSphere(panelHitPoint, 0.012f, cursorColor);
}

void VRMouseController::DrawRayToPanel() {
    Vector3 panelHitPoint = {
        panelPosition.x + (vrMouse.panelUV.x - 0.5f) * panelSize.x,
        panelPosition.y + (0.5f - vrMouse.panelUV.y) * panelSize.y,
        panelPosition.z
    };

    Color rayColor = vrMouse.isClicking ? GREEN : YELLOW;
    DrawLine3D(vrMouse.position, panelHitPoint, rayColor);
}
