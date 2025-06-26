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
#include <filesystem>
#include <iostream>
#include <io.h>
#include <fcntl.h>

namespace fs = std::filesystem;

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
    catch (...) {
        // Silent fail - just return empty vector
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

bool isStdoutPiped() {
    return !_isatty(_fileno(stdout));
}

int main(void) {
    // Check if stdout is NOT piped and show message
    if (!isStdoutPiped()) {
        std::cerr << "This program outputs frame data to stdout. Please run the Python file instead." << std::endl;
        return 1;
    }

    const int screenWidth = 1920;
    const int screenHeight = 1080;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_HIDDEN);
    InitWindow(screenWidth, screenHeight, "VR Viewer");
    SetTargetFPS(60);

    // Redirect stderr to null to prevent interference with binary stdout
    freopen_s((FILE**)stderr, "NUL", "w", stderr);
    // Ensure stdout is in binary mode and unbuffered
    _setmode(_fileno(stdout), _O_BINARY);
    setvbuf(stdout, nullptr, _IONBF, 0);

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
    desktopRenderer.setMaxUpdateRate(60.0f);

    Player player;
    const float eyeSeparation = device.interpupillaryDistance / 2.0f;

    Vector2 lastMousePos = { 0 };
    bool firstMouse = true;

    Vector3 panelPosition = { 0.0f, 3.0f, 4.0f };
    Vector3 panelSize = { 24.0f, 6.75f, 0.1f };

    // Path resolution
    fs::path exePath = fs::absolute(fs::path(__argv[0]));
    fs::path projectRoot = exePath.parent_path().parent_path().parent_path().parent_path();
    fs::path sharedDir = projectRoot / "Shared";
    std::string handFilePath = (sharedDir / "hands.dat").string();
    std::string gyroFilePath = (sharedDir / "gyro.dat").string();

    float timeSinceLastSend = 0.0f;
    const float sendInterval = 1.0f / 60.0f;

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();

        // Mouse input
        Vector2 mousePos = GetMousePosition();
        if (firstMouse) {
            lastMousePos = mousePos;
            firstMouse = false;
        }
        Vector2 delta = { mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y };
        lastMousePos = mousePos;
        player.HandleMouseLook(delta);

        // Keyboard input
        Vector3 move = { 0 };
        if (IsKeyDown(KEY_W)) move.z -= 1;
        if (IsKeyDown(KEY_S)) move.z += 1;
        if (IsKeyDown(KEY_A)) move.x -= 1;
        if (IsKeyDown(KEY_D)) move.x += 1;
        if (IsKeyDown(KEY_SPACE)) move.y += 1;
        if (IsKeyDown(KEY_LEFT_SHIFT)) move.y -= 1;
        player.Move(move, deltaTime);

        // Update gyro data
        float gyroYaw, gyroPitch;
        if (ReadGyroData(gyroFilePath, gyroYaw, gyroPitch)) {
            player.SetYawPitch(gyroYaw, gyroPitch);
        }

        // Update hand data
        auto handData = ReadHandDataFromSharedMemory(handFilePath);

        player.Update(deltaTime);
        desktopRenderer.update();

        // Render VR scene
        BeginTextureMode(target);
        ClearBackground(RAYWHITE);

        // Left eye
        rlViewport(0, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetLeftEyeCamera(eyeSeparation));
        DrawCube({ 0,0,0 }, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        // Right eye
        rlViewport(screenWidth / 2, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetRightEyeCamera(eyeSeparation));
        DrawCube({ 0,0,0 }, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        rlViewport(0, 0, screenWidth, screenHeight);
        EndTextureMode();

        // Need to call BeginDrawing/EndDrawing for Raylib to process the render
        BeginDrawing();
        EndDrawing();

        // Always send frame data to stdout (no screen display)
        timeSinceLastSend += deltaTime;
        if (timeSinceLastSend >= sendInterval) {
            timeSinceLastSend = 0.0f;

            Image frame = LoadImageFromTexture(target.texture);

            uint32_t width = static_cast<uint32_t>(frame.width);
            uint32_t height = static_cast<uint32_t>(frame.height);
            uint32_t magic = 0xDEADBEEF;

            // Send header
            std::cout.write(reinterpret_cast<const char*>(&magic), sizeof(uint32_t));
            std::cout.write(reinterpret_cast<const char*>(&width), sizeof(uint32_t));
            std::cout.write(reinterpret_cast<const char*>(&height), sizeof(uint32_t));

            // Send pixel data
            size_t pixel_data_size = width * height * 4;
            std::cout.write(reinterpret_cast<const char*>(frame.data), pixel_data_size);
            std::cout.flush();

            UnloadImage(frame);
        }
    }

    desktopRenderer.cleanup();
    UnloadVrStereoConfig(config);
    UnloadRenderTexture(target);
    UnloadShader(distortion);
    CloseWindow();
    return 0;
}