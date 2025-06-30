#include "player.h"
#include <cmath>
#include <algorithm>
#include <fstream>

// Define missing functions
template<typename T>
T Clamp(T value, T minVal, T maxVal) {
    return (value < minVal) ? minVal : (value > maxVal) ? maxVal : value;
}

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

    // Initialize VR mouse system
    vrMouse.isActive = false;
    vrMouse.isClicking = false;
    vrMouse.leftClicking = false;
    vrMouse.isDragging = false;
    vrMouse.clickCooldown = 0.0f;
    vrMouse.position = { 0, 0, 0 };
    vrMouse.hitPoint = { 0, 0, 0 };
    vrMouse.panelUV = { 0.5f, 0.5f };
    vrMouse.cursorSize = 0.012f;
    vrMouse.clickThreshold = 0.03f;
    vrMouse.showLaser = true;
    vrMouse.laserIntensity = 1.0f;
    // Initialize panel info
    panelPosition = { 0.0f, 1.8f, 2.5f };
    panelSize = { 4.0f, 2.25f, 0.1f };
}

void Player::Update() {
    target.x = position.x + cosf(pitch) * sinf(yaw);
    target.y = position.y + sinf(pitch);
    target.z = position.z + cosf(pitch) * cosf(yaw);

    up.x = sinf(roll);
    up.y = cosf(roll);
    up.z = 0.0f;
    up = Vector3Normalize(up);
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
    Vector3 forward = GetForward();
    Vector3 right = GetRight();
    Vector3 up_vec = Getup();

    float vr_x = 1.0f - holisticCoords.x;
    float vr_y = holisticCoords.y-1.5f;
    float vr_z = holisticCoords.z;

    float hand_width = 0.8f;
    float hand_height = 0.6f;
    float hand_depth = 0.3f;

    Vector3 hand_offset = {
        (vr_x -1.0f) * hand_width,
        (vr_y - 0.5f) * hand_height,
        0.6f + vr_z * hand_depth
    };

    Vector3 hand_position = Vector3Add(position, Vector3Add(Vector3Add(
        Vector3Scale(right, hand_offset.x),
        Vector3Scale(up_vec, hand_offset.y)),
        Vector3Scale(forward, hand_offset.z)));

    return hand_position;
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

// VR Mouse functionality implementations
bool Player::IsIndexFingerExtended(const VRHand& hand) {
    if (!hand.is_tracked || hand.landmarks.size() < 21) return false;

    Vector3 mcp = hand.landmarks[5].position;
    Vector3 tip = hand.landmarks[8].position;
    Vector3 wrist = hand.landmarks[0].position;

    float indexLength = Vector3Distance(mcp, tip);
    float wristToTip = Vector3Distance(wrist, tip);

    return (indexLength > 0.07f) && (wristToTip > 0.12f);
}

bool Player::IsPinchGesture(const VRHand& hand) {
    if (!hand.is_tracked || hand.landmarks.size() < 21) return false;

    Vector3 thumbTip = hand.landmarks[4].position;
    Vector3 indexTip = hand.landmarks[8].position;

    float distance = Vector3Distance(thumbTip, indexTip);
    return distance < vrMouse.clickThreshold;
}

bool Player::IsScissorsGesture(const VRHand& hand) {
    if (!hand.is_tracked || hand.landmarks.size() < 21) return false;

    // Check if index finger is extended
    Vector3 indexMCP = hand.landmarks[5].position;
    Vector3 indexTip = hand.landmarks[8].position;
    float indexLength = Vector3Distance(indexMCP, indexTip);

    // Check if middle finger is extended
    Vector3 middleMCP = hand.landmarks[9].position;
    Vector3 middleTip = hand.landmarks[12].position;
    float middleLength = Vector3Distance(middleMCP, middleTip);

    // Check if ring finger is closed
    Vector3 ringMCP = hand.landmarks[13].position;
    Vector3 ringTip = hand.landmarks[16].position;
    float ringLength = Vector3Distance(ringMCP, ringTip);

    // Check if pinky is closed
    Vector3 pinkyMCP = hand.landmarks[17].position;
    Vector3 pinkyTip = hand.landmarks[20].position;
    float pinkyLength = Vector3Distance(pinkyMCP, pinkyTip);

    // Scissors gesture: index and middle extended, ring and pinky closed
    bool indexExtended = indexLength > 0.07f;
    bool middleExtended = middleLength > 0.07f;
    bool ringClosed = ringLength < 0.06f;
    bool pinkyClosed = pinkyLength < 0.05f;

    // Additional check: distance between index and middle fingertips
    float fingerDistance = Vector3Distance(indexTip, middleTip);
    bool fingersSpread = fingerDistance > 0.03f;

    return indexExtended && middleExtended && ringClosed && pinkyClosed && fingersSpread;
}

Vector3 Player::GetIndexFingerRayDirection(const VRHand& hand) {
    if (!hand.is_tracked || hand.landmarks.size() < 21) return { 0, 0, 0 };

    Vector3 indexMCP = hand.landmarks[5].position;
    Vector3 indexTip = hand.landmarks[8].position;

    Vector3 rayDirection = Vector3Subtract(indexTip, indexMCP);
    return Vector3Normalize(rayDirection);
}

bool Player::GetIndexFingerPanelIntersection(const VRHand& hand, Vector3& hitPoint, Vector2& hitUV) {
    if (!hand.is_tracked) return false;

    Vector3 rayOrigin = hand.landmarks[8].position;
    Vector3 rayDirection = GetIndexFingerRayDirection(hand);

    // Panel plane equation (assuming panel faces -Z direction)
    Vector3 panelNormal = { 0, 0, -1 };
    float panelDistance = panelPosition.z;

    // Ray-plane intersection
    float denominator = Vector3DotProduct(rayDirection, panelNormal);
    if (fabsf(denominator) < 0.0001f) return false;

    float t = (panelDistance - Vector3DotProduct(rayOrigin, panelNormal)) / denominator;
    if (t < 0) return false;

    hitPoint = Vector3Add(rayOrigin, Vector3Scale(rayDirection, t));

    // Check if hit point is within panel bounds
    Vector3 panelMin = {
        panelPosition.x - panelSize.x / 2.0f,
        panelPosition.y - panelSize.y / 2.0f,
        panelPosition.z - panelSize.z / 2.0f
    };
    Vector3 panelMax = {
        panelPosition.x + panelSize.x / 2.0f,
        panelPosition.y + panelSize.y / 2.0f,
        panelPosition.z + panelSize.z / 2.0f
    };

    if (hitPoint.x < panelMin.x || hitPoint.x > panelMax.x ||
        hitPoint.y < panelMin.y || hitPoint.y > panelMax.y) {
        return false;
    }

    hitUV.x = (hitPoint.x - panelMin.x) / panelSize.x;
    hitUV.y = 1.0f - (hitPoint.y - panelMin.y) / panelSize.y;

    return true;
}

Vector2 Player::GetPanelUVFromWorldPos(const Vector3& worldPos) {
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

// Replace this problematic code:
// std::ofstream debug("vr_mouse_clicks.log", std::ios::app);
// debug << "VR LEFT CLICK (Scissors) at UV: " << vrMouse.panelUV.x << ", " << vrMouse.panelUV.y << std::endl;

// With this safer version:
void Player::UpdateVRMouse(const VRHand& rightHand) {
    if (vrMouse.clickCooldown > 0.0f) {
        vrMouse.clickCooldown -= GetFrameTime();
    }

    if (!rightHand.is_tracked) {
        vrMouse.isActive = false;
        vrMouse.isClicking = false;
        vrMouse.leftClicking = false;
        return;
    }

    if (IsIndexFingerExtended(rightHand)) {
        Vector3 hitPoint;
        Vector2 hitUV;

        if (GetIndexFingerPanelIntersection(rightHand, hitPoint, hitUV)) {
            vrMouse.isActive = true;
            vrMouse.position = rightHand.landmarks[8].position;
            vrMouse.panelUV = hitUV;
            vrMouse.hitPoint = hitPoint;

            bool isPinching = IsPinchGesture(rightHand);
            bool isScissors = IsScissorsGesture(rightHand);

            if (isScissors && vrMouse.clickCooldown <= 0.0f) {
                vrMouse.leftClicking = true;
                vrMouse.clickCooldown = 0.3f;

                // Use printf instead of ofstream
                printf("VR LEFT CLICK (Scissors) at UV: %.3f, %.3f\n", vrMouse.panelUV.x, vrMouse.panelUV.y);
            }
            else {
                vrMouse.leftClicking = false;
            }

            if (isPinching && vrMouse.clickCooldown <= 0.0f) {
                vrMouse.isClicking = true;
                vrMouse.clickCooldown = 0.3f;

                printf("VR RIGHT CLICK (Pinch) at UV: %.3f, %.3f\n", vrMouse.panelUV.x, vrMouse.panelUV.y);
            }
            else if (!isScissors) {
                vrMouse.isClicking = false;
            }
        }
        else {
            vrMouse.isActive = false;
        }
    }
    else {
        vrMouse.isActive = false;
        vrMouse.isClicking = false;
        vrMouse.leftClicking = false;
    }
}


void Player::DrawVRMouse() {
    if (!vrMouse.isActive) return;

    Color cursorColor = YELLOW;
    Color laserColor = YELLOW;

    if (vrMouse.leftClicking) {
        cursorColor = GREEN;
        laserColor = GREEN;
    }
    else if (vrMouse.isClicking) {
        cursorColor = BLUE;
        laserColor = BLUE;
    }

    // Draw laser beam as a thick line
    DrawLine3D(vrMouse.position, vrMouse.hitPoint, laserColor);

    // Enhanced laser beam with multiple segments for better visibility
    Vector3 laserStart = vrMouse.position;
    Vector3 laserEnd = vrMouse.hitPoint;
    Vector3 laserDirection = Vector3Subtract(laserEnd, laserStart);
    float laserLength = Vector3Length(laserDirection);

    // Draw segmented laser beam for animated effect
    int segments = 10;
    for (int i = 0; i < segments; i++) {
        float t1 = (float)i / segments;
        float t2 = (float)(i + 1) / segments;

        Vector3 segmentStart = Vector3Add(laserStart, Vector3Scale(laserDirection, t1));
        Vector3 segmentEnd = Vector3Add(laserStart, Vector3Scale(laserDirection, t2));

        // Vary intensity for animated effect
        Color segmentColor = laserColor;
        segmentColor.a = (unsigned char)(255 * (1.0f - t1 * 0.3f)); // Fade towards end

        DrawLine3D(segmentStart, segmentEnd, segmentColor);
    }

    // Draw index fingertip origin point
    DrawSphere(vrMouse.position, 0.008f, cursorColor);

    // Draw target cursor at hit point
    DrawSphere(vrMouse.hitPoint, 0.015f, cursorColor);

    // Draw crosshair for precision
    Vector3 right = GetRight();
    Vector3 up_vec = Getup();
    float crosshairSize = 0.03f;

    Vector3 crosshairH1 = Vector3Add(vrMouse.hitPoint, Vector3Scale(right, -crosshairSize));
    Vector3 crosshairH2 = Vector3Add(vrMouse.hitPoint, Vector3Scale(right, crosshairSize));
    Vector3 crosshairV1 = Vector3Add(vrMouse.hitPoint, Vector3Scale(up_vec, -crosshairSize));
    Vector3 crosshairV2 = Vector3Add(vrMouse.hitPoint, Vector3Scale(up_vec, crosshairSize));

    DrawLine3D(crosshairH1, crosshairH2, cursorColor);
    DrawLine3D(crosshairV1, crosshairV2, cursorColor);

    // Draw circular cursor outline for better visibility
    DrawCircle3D(vrMouse.hitPoint, 0.025f, Vector3Normalize(Vector3Subtract(vrMouse.position, vrMouse.hitPoint)), 0.0f, cursorColor);
}

void Player::DrawHolisticHands(const std::vector<HolisticHandData>& holisticData) {
    for (const auto& holistic_hand : holisticData) {
        if (holistic_hand.handedness == "Left") {
            UpdateHolisticHand(leftHand, holistic_hand);
        }
        else if (holistic_hand.handedness == "Right") {
            UpdateHolisticHand(rightHand, holistic_hand);
        }
    }

    UpdateVRMouse(rightHand);

    if (leftHand.is_tracked) {
        for (size_t i = 0; i < leftHand.landmarks.size(); ++i) {
            if (leftHand.landmarks[i].active) {
                float pointSize = (i == 0) ? 0.015f : 0.01f;
                Color pointColor = RED;

                if (i == 4 || i == 8 || i == 12 || i == 16 || i == 20) {
                    pointSize = 0.012f;
                    pointColor = ORANGE;
                }

                DrawSphere(leftHand.landmarks[i].position, pointSize, pointColor);
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
            if (leftHand.landmarks[conn[0]].active && leftHand.landmarks[conn[1]].active) {
                DrawLine3D(leftHand.landmarks[conn[0]].position,
                    leftHand.landmarks[conn[1]].position, RED);
            }
        }
    }

    if (rightHand.is_tracked) {
        for (size_t i = 0; i < rightHand.landmarks.size(); ++i) {
            if (rightHand.landmarks[i].active) {
                float pointSize = (i == 0) ? 0.015f : 0.01f;
                Color pointColor = BLUE;

                if (i == 4 || i == 8 || i == 12 || i == 16 || i == 20) {
                    pointSize = 0.012f;
                    pointColor = SKYBLUE;
                }

                DrawSphere(rightHand.landmarks[i].position, pointSize, pointColor);
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

    DrawVRMouse();
}

void Player::DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData) {
    std::vector<HolisticHandData> holistic_data;
    for (const auto& hand : handData) {
        HolisticHandData holistic_hand;
        holistic_hand.handedness = hand.first;
        holistic_hand.landmarks = hand.second;
        holistic_hand.distance_factor = 1.0f;
        holistic_hand.depth_scale = 1.0f;
        holistic_hand.shoulder_calibrated = true;
        holistic_hand.confidence = 0.8f;
        holistic_data.push_back(holistic_hand);
    }
    DrawHolisticHands(holistic_data);
}

void Player::SetPanelInfo(const Vector3& position, const Vector3& size) {
    panelPosition = position;
    panelSize = size;
}

bool Player::GetVRMouseData(Vector2& mouseUV, bool& isLeftClicking, bool& isRightClicking, bool& isDragging) {
    if (!vrMouse.isActive) return false;

    mouseUV = vrMouse.panelUV;
    isLeftClicking = vrMouse.leftClicking;
    isRightClicking = vrMouse.isClicking;
    isDragging = vrMouse.isDragging;
    return true;
}

Camera Player::GetLeftEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, -eyeSeparation / 2.0f));
    camera.target = Vector3Add(target, Vector3Scale(right, -eyeSeparation / 2.0f));
    camera.up = up;
    camera.fovy = 90.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

Camera Player::GetRightEyeCamera(float eyeSeparation) const {
    Camera camera = {};
    Vector3 right = GetRight();
    camera.position = Vector3Add(position, Vector3Scale(right, eyeSeparation / 2.0f));
    camera.target = Vector3Add(target, Vector3Scale(right, eyeSeparation / 2.0f));
    camera.up = up;
    camera.fovy = 90.0f;
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
    pitch = -newPitch;
    roll = newRoll;

    if (isCalibrated) {
        yaw -= calibrationYaw;
        pitch -= calibrationPitch;
        roll -= calibrationRoll;
    }

    while (yaw > 2.0f * PI) yaw -= 2.0f * PI;
    while (yaw < 0.0f) yaw += 2.0f * PI;

    if (yaw > PI) yaw -= 2.0f * PI;

    pitch = Clamp(pitch, -PI / 2.0f, PI / 2.0f);
}

void Player::Calibrate() {
    calibrationYaw = yaw;
    calibrationPitch = pitch;
    calibrationRoll = roll;
    isCalibrated = true;
}

void Player::ResetVRDrift() {
    Calibrate();
}
void Player::DrawLaserPointer() {
    if (!vrMouse.isActive) return;

    Color laserColor = YELLOW;
    if (vrMouse.leftClicking) {
        laserColor = GREEN;
    }
    else if (vrMouse.isClicking) {
        laserColor = BLUE;
    }

    // Draw laser beam from finger to hit point
    Vector3 laserStart = vrMouse.position;
    Vector3 laserEnd = vrMouse.hitPoint;

    // Draw main laser line
    DrawLine3D(laserStart, laserEnd, laserColor);

    // Draw laser as cylinder for better visibility
    float laserRadius = 0.003f;
    DrawCylinderEx(laserStart, laserEnd, laserRadius, laserRadius, 8, laserColor);

    // Add glow effect
    Color glowColor = laserColor;
    glowColor.a = 80; // Semi-transparent
    DrawCylinderEx(laserStart, laserEnd, laserRadius * 2, laserRadius * 2, 8, glowColor);

    // Draw origin point at fingertip
    DrawSphere(laserStart, 0.008f, laserColor);

    // Draw target cursor at hit point
    DrawSphere(laserEnd, 0.015f, laserColor);

    // Draw crosshair at target
    Vector3 right = GetRight();
    Vector3 up_vec = Getup();
    float crossSize = 0.025f;

    Vector3 h1 = Vector3Add(laserEnd, Vector3Scale(right, -crossSize));
    Vector3 h2 = Vector3Add(laserEnd, Vector3Scale(right, crossSize));
    Vector3 v1 = Vector3Add(laserEnd, Vector3Scale(up_vec, -crossSize));
    Vector3 v2 = Vector3Add(laserEnd, Vector3Scale(up_vec, crossSize));

    DrawLine3D(h1, h2, laserColor);
    DrawLine3D(v1, v2, laserColor);
}
