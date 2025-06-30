// vr_stream_sender.cpp
// This Raylib-based application renders a VR scene and streams raw RGBA frames via stdout with metadata headers.

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
#include <chrono>
#include <thread>
#include <memory>

namespace fs = std::filesystem;

// --- Shared Memory Management ---
static boost::interprocess::mapped_region* handRegion = nullptr;
static std::unique_ptr<boost::interprocess::file_mapping> handFile;

struct FrameData {
    uint32_t magic = 0xDEADBEEF;
    uint32_t timestamp_ms;
    uint32_t frame_size;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format; // 0=RGBA
};

std::vector<std::pair<std::string, std::vector<Vector3>>> ReadHandDataFromSharedMemory(const std::string& filename);
bool ReadGyroData(const std::string& filename, float& yaw, float& pitch);
bool isStdoutPiped();
uint32_t GetCurrentTimeMs();
bool SendRawFrame(const unsigned char* frameData, uint32_t dataSize, int width, int height, uint32_t pixel_format);

int main(void) {
    std::ofstream debugLog("debug.log", std::ios::app);
    debugLog << "[START] VR process launched\n";

    if (!isStdoutPiped()) {
        debugLog << "[ERROR] Stdout is not piped. Exiting.\n";
        return 1;
    }

    const int screenWidth = 1920;
    const int screenHeight = 1080;
    const float targetFPS = 120.0f;
    const float frameTime = 1.0f / targetFPS;

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_HIDDEN);
    InitWindow(screenWidth, screenHeight, "VR Viewer");
    SetTargetFPS(120);

    FILE* nullout;
    freopen_s(&nullout, "NUL", "w", stderr);
    _setmode(_fileno(stdout), _O_BINARY);
    setvbuf(stdout, nullptr, _IONBF, 128 * 1024);

    RenderTexture2D target = LoadRenderTexture(screenWidth, screenHeight);
    VRDesktopRenderer desktopRenderer;
    desktopRenderer.initialize();
    desktopRenderer.setMaxUpdateRate(120.0f);

    Player player;
    const float eyeSeparation = 0.035f; // Half of typical IPD
    Vector2 lastMousePos = { 0 };
    bool firstMouse = true;
    Vector3 panelPosition = { 0.0f, 3.0f, 4.0f };
    Vector3 panelSize = { 24.0f, 6.75f, 0.1f };

    fs::path exePath = fs::absolute(fs::path(__argv[0]));
    fs::path sharedDir = exePath.parent_path().parent_path().parent_path().parent_path() / "Shared";
    std::string handFilePath = (sharedDir / "hands.dat").string();
    std::string gyroFilePath = (sharedDir / "gyro.dat").string();

    auto lastFrameTime = std::chrono::high_resolution_clock::now();

    while (!WindowShouldClose()) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
        if (deltaTime < frameTime) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<int>((frameTime - deltaTime) * 1000000)));
            currentTime = std::chrono::high_resolution_clock::now();
        }
        lastFrameTime = currentTime;

        Vector2 mousePos = GetMousePosition();
        if (firstMouse) { lastMousePos = mousePos; firstMouse = false; }
        Vector2 delta = { mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y };
        lastMousePos = mousePos;
        player.HandleMouseLook(delta);

        float yaw, pitch;
        if (ReadGyroData(gyroFilePath, yaw, pitch)) player.SetYawPitch(yaw, pitch);
        auto handData = ReadHandDataFromSharedMemory(handFilePath);

        player.Update();
        desktopRenderer.update();

        BeginTextureMode(target);
        ClearBackground(RAYWHITE);

        rlViewport(0, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetLeftEyeCamera(eyeSeparation));
        DrawCube({0,0,0}, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        rlViewport(screenWidth / 2, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetRightEyeCamera(eyeSeparation));
        DrawCube({0,0,0}, 2, 2, 2, RED);
        DrawGrid(40, 1.0f);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHands(handData);
        EndMode3D();

        rlViewport(0, 0, screenWidth, screenHeight);
        EndTextureMode();

        BeginDrawing(); EndDrawing();

        Image frame = LoadImageFromTexture(target.texture);
        SendRawFrame((unsigned char*)frame.data, frame.width * frame.height * 4, frame.width, frame.height, 0);
        UnloadImage(frame);
    }

    desktopRenderer.cleanup();
    UnloadRenderTexture(target);
    CloseWindow();
    return 0;
}

// --- Helper Functions ---

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
                Vector3 pt = { lm["x"].get<float>(), lm["y"].get<float>(), lm["z"].get<float>() };
                points.push_back(pt);
            }
            handPoints.emplace_back(handType, points);
        }
    } catch (...) {}

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
    } catch (...) {
        return false;
    }
}

bool isStdoutPiped() {
    return !_isatty(_fileno(stdout));
}

uint32_t GetCurrentTimeMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

bool SendRawFrame(const unsigned char* frameData, uint32_t dataSize, int width, int height, uint32_t pixel_format) {
    FrameData header = { 0xDEADBEEF, GetCurrentTimeMs(), dataSize, (uint32_t)width, (uint32_t)height, pixel_format };
    std::cout.write(reinterpret_cast<const char*>(&header), sizeof(header));
    std::cout.write(reinterpret_cast<const char*>(frameData), dataSize);
    std::cout.flush();
    return true;
}