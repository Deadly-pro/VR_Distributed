// player.cpp
#include "player.h"
#include <raymath.h>

Player::Player()
    : position({ 5.0f, 2.0f, 10.0f }),
    up({ 0.0f, 1.0f, 0.0f }),
    yaw(90.0f),
    pitch(0.0f),
    sensitivity(0.003f)
{
    target = Vector3Add(position, GetForward());

    // Preallocate hand points (e.g., 21 per hand x 2 = 42, using 30 for now)
    handPoints.resize(30);
    for (auto& hp : handPoints) {
        hp.position = { 0.0f, 0.0f, 0.0f };
        hp.active = false;
    }
}

void Player::Update(float deltaTime)
{
    target = Vector3Add(position, GetForward());
}

void Player::HandleMouseLook(Vector2 mouseDelta)
{
    yaw += mouseDelta.x * sensitivity;
    pitch -= mouseDelta.y * sensitivity;

    if (pitch > 1.5f) pitch = 1.5f;
    if (pitch < -1.5f) pitch = -1.5f;

    target = Vector3Add(position, GetForward());
}

void Player::SetYawPitch(float newYaw, float newPitch)
{
    yaw = newYaw;
    pitch = newPitch;

    if (pitch > 1.5f) pitch = 1.5f;
    if (pitch < -1.5f) pitch = -1.5f;

    target = Vector3Add(position, GetForward());
}

void Player::Move(Vector3 direction, float deltaTime)
{
    Vector3 forward = GetForward();
    Vector3 right = GetRight();

    Vector3 worldMovement = Vector3Add(
        Vector3Scale(forward, direction.z),
        Vector3Add(
            Vector3Scale(right, direction.x),
            Vector3Scale(up, direction.y)
        )
    );

    position = Vector3Add(position, Vector3Scale(worldMovement, deltaTime * 5.0f));
    target = Vector3Add(position, GetForward());
}

Camera Player::GetLeftEyeCamera(float eyeSeparation) const
{
    Vector3 forward = GetForward();
    Vector3 rightVec = Vector3Normalize(Vector3CrossProduct(forward, up));
    Vector3 leftPos = Vector3Subtract(position, Vector3Scale(rightVec, eyeSeparation));
    Vector3 leftTarget = Vector3Subtract(target, Vector3Scale(rightVec, eyeSeparation));

    Camera cam = { 0 };
    cam.position = leftPos;
    cam.target = leftTarget;
    cam.up = up;
    cam.fovy = 100.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

Camera Player::GetRightEyeCamera(float eyeSeparation) const
{
    Vector3 forward = GetForward();
    Vector3 rightVec = Vector3Normalize(Vector3CrossProduct(forward, up));
    Vector3 rightPos = Vector3Add(position, Vector3Scale(rightVec, eyeSeparation));
    Vector3 rightTarget = Vector3Add(target, Vector3Scale(rightVec, eyeSeparation));

    Camera cam = { 0 };
    cam.position = rightPos;
    cam.target = rightTarget;
    cam.up = up;
    cam.fovy = 100.0f;
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

Vector3 Player::GetForward() const
{
    return Vector3Normalize({
        cosf(pitch) * cosf(yaw),
        sinf(pitch),
        cosf(pitch) * sinf(yaw)
        });
}

Vector3 Player::GetRight() const
{
    return Vector3Normalize(Vector3CrossProduct(GetForward(), up));
}

Vector3 Player::GetPosition() const
{
    return position;
}

void Player::DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData)
{
    // Reset all hand points
    for (auto& hp : handPoints) hp.active = false;

    size_t i = 0;
    for (const auto& [handedness, landmarks] : handData)
    {
        Color color = (handedness == "Left") ? GREEN : BLUE;
        for (const auto& pt : landmarks)
        {
            if (i >= handPoints.size()) break;

            Vector3 localPt = {
                pt.x,
                pt.y,
                pt.z
            };

            // Position the hand in front of the camera (e.g., 0.3 units forward from player)
            Vector3 handOffset = Vector3Add(position, Vector3Add(GetForward(), localPt));
            handPoints[i].position = handOffset;
            handPoints[i].active = true;
            i++;
        }
    }

    // Draw only active points
    for (const auto& hp : handPoints)
    {
        if (hp.active)
            DrawSphere(hp.position, 0.035f, SKYBLUE);
    }
}
