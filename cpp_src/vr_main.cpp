#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "player.h"
#include "vr_desktop_render.h"
#include "holistic_data.h"
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

static boost::interprocess::mapped_region* handRegion = nullptr;
static std::unique_ptr<boost::interprocess::file_mapping> handFile;

struct FrameData {
    uint32_t magic = 0xDEADBEEF;
    uint32_t timestamp_ms;
    uint32_t frame_size;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
};

// Remove the duplicate GyroData struct definition here

std::vector<HolisticHandData> ReadHolisticHandData(const std::string& filename);
bool ReadGyroData(const std::string& filename, float& yaw, float& pitch, float& roll);
bool isStdoutPiped();
uint32_t GetCurrentTimeMs();
bool SendRawFrame(const unsigned char* frameData, uint32_t dataSize, int width, int height, uint32_t pixel_format);

int main(void) {
    std::ofstream debugLog("debug.log", std::ios::app);
    debugLog << "[START] VR process launched with Holistic integration\n";

    if (!isStdoutPiped()) {
        debugLog << "[ERROR] Stdout is not piped. Exiting.\n";
        return 1;
    }

    const int screenWidth = 1920;
    const int screenHeight = 1080;

    SetTraceLogLevel(LOG_NONE);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_HIDDEN);
    InitWindow(screenWidth, screenHeight, "VR Holistic Viewer");
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
    const float eyeSeparation = 0.065f;
    Vector2 lastMousePos = { 0 };
    bool firstMouse = true;
    Vector3 panelPosition = { 0.0f, 1.8f, 2.5f };
    Vector3 panelSize = { 4.0f, 2.25f, 0.1f };

    fs::path exePath = fs::absolute(fs::path(__argv[0]));
    fs::path sharedDir = exePath.parent_path().parent_path().parent_path().parent_path() / "Shared";
    std::string handFilePath = (sharedDir / "hands.dat").string();
    std::string gyroFilePath = (sharedDir / "gyro.dat").string();

    while (!WindowShouldClose()) {
        Vector2 mousePos = GetMousePosition();
        if (firstMouse) { lastMousePos = mousePos; firstMouse = false; }
        Vector2 delta = { mousePos.x - lastMousePos.x, mousePos.y - lastMousePos.y };
        lastMousePos = mousePos;
        player.HandleMouseLook(delta);
        float yaw, pitch, roll;
        if (ReadGyroData(gyroFilePath, yaw, pitch, roll)) {
            player.SetYawPitchRoll(yaw, pitch, roll);
        }

        auto holisticHandData = ReadHolisticHandData(handFilePath);

        player.Update();
        desktopRenderer.update();

        BeginTextureMode(target);
        ClearBackground(BLACK);

        // Left eye
        rlViewport(0, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetLeftEyeCamera(eyeSeparation));
        DrawGrid(20, 1.0f);
        DrawCube({ 0, 0.5f, 0 }, 1, 1, 1, RED);
        DrawCube({ 3, 0.5f, 0 }, 1, 1, 1, GREEN);
        DrawCube({ -3, 0.5f, 0 }, 1, 1, 1, BLUE);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHolisticHands(holisticHandData);
        EndMode3D();

        // Right eye
        rlViewport(screenWidth / 2, 0, screenWidth / 2, screenHeight);
        BeginMode3D(player.GetRightEyeCamera(eyeSeparation));
        DrawGrid(20, 1.0f);
        DrawCube({ 0, 0.5f, 0 }, 1, 1, 1, RED);
        DrawCube({ 3, 0.5f, 0 }, 1, 1, 1, GREEN);
        DrawCube({ -3, 0.5f, 0 }, 1, 1, 1, BLUE);
        desktopRenderer.renderDesktopPanel(panelPosition, panelSize);
        player.DrawHolisticHands(holisticHandData);
        EndMode3D();

        rlViewport(0, 0, screenWidth, screenHeight);
        EndTextureMode();

        BeginDrawing(); EndDrawing();

        Image frame = LoadImageFromTexture(target.texture);
        ImageFlipVertical(&frame);
        SendRawFrame((unsigned char*)frame.data, frame.width * frame.height * 4, frame.width, frame.height, 0);
        UnloadImage(frame);
    }

    desktopRenderer.cleanup();
    UnloadRenderTexture(target);
    CloseWindow();
    return 0;
}

std::vector<HolisticHandData> ReadHolisticHandData(const std::string& filename) {
    namespace bip = boost::interprocess;
    std::vector<HolisticHandData> handData;

    try {
        if (!handFile) {
            handFile = std::make_unique<bip::file_mapping>(filename.c_str(), bip::read_only);
            handRegion = new bip::mapped_region(*handFile, bip::read_only);
        }
        const char* mem = static_cast<const char*>(handRegion->get_address());

        uint32_t size;
        memcpy(&size, mem, sizeof(uint32_t));
        if (size == 0 || size > handRegion->get_size() - 4) return handData;

        std::string json_data(mem + 4, size);
        auto parsed = nlohmann::json::parse(json_data);

        for (const auto& hand : parsed) {
            HolisticHandData holistic_hand;
            holistic_hand.handedness = hand["handedness"];
            holistic_hand.distance_factor = hand.value("distance_factor", 1.0f);
            holistic_hand.depth_scale = hand.value("depth_scale", 1.0f);
            holistic_hand.shoulder_calibrated = hand.value("shoulder_calibrated", false);
            holistic_hand.confidence = hand.value("confidence", 0.7f);

            for (const auto& lm : hand["landmarks"]) {
                Vector3 pt = {
                    lm["x"].get<float>(),
                    lm["y"].get<float>(),
                    lm["z"].get<float>()
                };
                holistic_hand.landmarks.push_back(pt);
            }

            handData.push_back(holistic_hand);
        }
    }
    catch (const std::exception& e) {
        std::ofstream errorLog("hand_error.log", std::ios::app);
        errorLog << "Error reading holistic hand data: " << e.what() << std::endl;
    }

    return handData;
}

bool ReadGyroData(const std::string& filename, float& yaw, float& pitch, float& roll) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    try {
        nlohmann::json j;
        file >> j;

        // Direct mapping - treat as absolute orientation angles, not velocities
        float alpha = j.value("alpha", 0.0f);   // Compass heading (0-360°)
        float beta = j.value("beta", 0.0f);     // Pitch (-180° to 180°)
		float gamma = j.value("gamma", 0.0f);   // Roll deals with up /down tilt (-90° to 90°)
		gamma += 45.0f; // dont ask why tf this works but it does
		//gamma = Clamp(beta, -90.0f, 90.0f); // Ensure beta is within valid range
        // Convert to radians and map directly
        yaw = DEG2RAD * alpha;
        pitch = DEG2RAD * (gamma);
        roll = DEG2RAD * (-beta);

        return true;
    }
    catch (const std::exception& e) {
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
