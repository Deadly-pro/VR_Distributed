// player.h
#ifndef PLAYER_H
#define PLAYER_H

#include "raylib.h"
#include <vector>
#include <string>

class Player {
public:
    Player();
    void Update(float deltaTime);
    void HandleMouseLook(Vector2 mouseDelta);
    void Move(Vector3 direction, float deltaTime);
    void DrawHands(const std::vector<std::pair<std::string, std::vector<Vector3>>>& handData);

    Camera GetLeftEyeCamera(float eyeSeparation) const;
    Camera GetRightEyeCamera(float eyeSeparation) const;
    Vector3 GetForward() const;
    Vector3 GetRight() const;
    Vector3 GetPosition() const;
    void SetYawPitch(float newYaw, float newPitch);

private:
    Vector3 position;
    Vector3 target;
    Vector3 up;
    float yaw;
    float pitch;
    float sensitivity;

    // Preallocated hand spheres
    struct HandPoint {
        Vector3 position;
        bool active;
    };

    std::vector<HandPoint> handPoints;
};

#endif  // PLAYER_H
