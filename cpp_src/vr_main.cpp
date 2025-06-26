#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "player.h"
#include "vr_desktop_render.h"
#include <nlohmann/json.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

// Keep mapped memory persistent to avoid remapping every frame
static boost::interprocess::mapped_region* handRegion = nullptr;
static std::unique_ptr<boost::interprocess::file_mapping> handFile;

std::vector<std::pair<std::string, std::vector<Vector3>>> ReadHandDataFromSharedMemory(const std::string& filename) {
    namespace bip = boost::interprocess;
    std::vector<std::pair<std::string, std::vector<Vector3>>> handPoints;

    try {
        if (!handFile) {
            handFile = std::make_unique<bip::file_mapping>(filename.c_str(), bip::read_only);
            handRegion = new bip::mapped_region(*handFile, bip::read_only);
        }

        const char* mem = static_cast<const char*>(handRegion->get_address());

        uint32_t size;
        memcpy(&size, mem, sizeof(uint32_t));
        if (size == 0 || size > handRegion->get_size() - 4) return handPoints;

        std::string json_data(mem + 4, size);
        auto parsed = nlohmann::json::parse(json_data);

        for (const auto& hand : parsed) {
            std::string handType = hand["handedness"];
            std::vector<Vector3> points;

            for (const auto& lm : hand["landmarks"]) {
                // Keep MediaPipe normalized coordinates for proper transformation
                Vector3 pt = {
                    static_cast<float>(lm["x"].get<double>()),
                    static_cast<float>(lm["y"].get<double>()),
                    static_cast<float>(lm["z"].get<double>())
                };
                points.push_back(pt);
            }
            handPoints.emplace_back(handType, points);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Shared memory parse error: " << e.what() << "\n";
    }

    return handPoints;
}

bool ReadGyroData(const std::string& filename, float& yaw, float& pitch) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    try {
        nlohmann::json j;
        file >> j;
        yaw = DEG2RAD * j.value("alpha", 0.0f);
        pitch = DEG2RAD * j.value("beta", 0.0f);
        return true;
    }
    catch (...) {
        return false;
    }
}

int main(void) {
    const int screenWidth = 1800;
    const int screenHeight = 900;
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "VR Viewer with Hands");
    //SetTargetFPS(90); // VR-appropriate framerate

    VrDeviceInfo device = {
        .hResolution = screenWidth,
        .vResolution = screenHeight,
        .hScreenSize = 0.133793f,
        .vScreenSize = 0.0669f,
        .eyeToScreenDistance = 0.041f,
        .lensSeparationDistance = 0.07f,
        .interpupillaryDistance = 0.07f,
        .lensDistortionValues = { 1.0f, 0.22f, 0.24f, 0.0f },
        .chromaAbCorrection = { 0.996f, -0.004f, 1.014f, 0.0f }
    };

    VrStereoConfig config = LoadVrStereoConfig(device);
    Shader distortion = LoadShader(0, TextFormat("resources/distortion%i.fs", 330));

    SetShaderValue(distortion, GetShaderLocation(distortion, "leftLensCenter"), config.leftLensCenter, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "rightLensCenter"), config.rightLensCenter, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "leftScreenCenter"), config.leftScreenCenter, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "rightScreenCenter"), config.rightScreenCenter, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "scale"), config.scale, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "scaleIn"), config.scaleIn, SHADER_UNIFORM_VEC2);
    SetShaderValue(distortion, GetShaderLocation(distortion, "deviceWarpParam"), device.lensDistortionValues, SHADER_UNIFORM_VEC4);
    SetShaderValue(distortion, GetShaderLocation(distortion, "chromaAbParam"), device.chromaAbCorrection, SHADER_UNIFORM_VEC4);

    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);
    VRDesktopRenderer desktopRenderer;
    desktopRenderer.initialize();
    desktopRenderer.setMaxUpdateRate(120.0f);

    Player player;
    const float eyeSeparation = device.interpupillaryDistance / 2.0f;
    bool mouseDebugMode = true;
    Vector2 lastMousePos = { 0 };
    bool firstMouse = true;

    Vector3 panelPosition = { 0.0f, 3.0f, 4.0f };
    Vector3 panelSize = { 24.0f, 6.75f, 0.1f };

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();

        if (mouseDebugMode) {
            Vector2 mousePos = GetMousePosition();
            if (firstMouse) { lastMousePos = mousePos; firstMouse = false; }
            Vector2 delta = { mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y };
            lastMousePos = mousePos;
            player.HandleMouseLook(delta);
        }

        Vector3 move = { 0 };
        if (IsKeyDown(KEY_W)) move.z -= 1;
        if (IsKeyDown(KEY_S)) move.z += 1;
        if (IsKeyDown(KEY_A)) move.x -= 1;
        if (IsKeyDown(KEY_D)) move.x += 1;
        if (IsKeyDown(KEY_SPACE)) move.y += 1;
        if (IsKeyDown(KEY_LEFT_SHIFT)) move.y -= 1;
        player.Move(move, deltaTime);

        float gyroYaw, gyroPitch;
        if (ReadGyroData("../Shared/gyro.dat", gyroYaw, gyroPitch)) {
            player.SetYawPitch(gyroYaw, gyroPitch);
        }

        auto handData = ReadHandDataFromSharedMemory("../Shared/hands.dat");
        player.Update(deltaTime);
        desktopRenderer.update();

        BeginTextureMode(target);
        ClearBackground(RAYWHITE);

        // Left eye viewport
        rlViewport(0, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetLeftEyeCamera(eyeSeparation));
        DrawCube({ 0,0,0 }, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        // Right eye viewport
        rlViewport(screenWidth / 2, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetRightEyeCamera(eyeSeparation));
        DrawCube({ 0,0,0 }, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        rlViewport(0, 0, screenWidth, screenHeight);
        EndTextureMode();

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginShaderMode(distortion);
        DrawTexturePro(target.texture, { 0,0, (float)target.texture.width, -(float)target.texture.height },
            { 0,0,(float)GetScreenWidth(),(float)GetScreenHeight() }, { 0,0 }, 0.0f, WHITE);
        EndShaderMode();

        DrawFPS(10, 10);
        EndDrawing();
    }

    desktopRenderer.cleanup();
    UnloadVrStereoConfig(config);
    UnloadRenderTexture(target);
    UnloadShader(distortion);
    CloseWindow();
    return 0;
}
