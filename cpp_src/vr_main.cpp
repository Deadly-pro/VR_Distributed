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

// For JPEG encoding - you'll need to install libjpeg-turbo
extern "C" {
#include <turbojpeg.h>
}

namespace fs = std::filesystem;

// Keep mapped memory persistent to avoid remapping every frame
static boost::interprocess::mapped_region* handRegion = nullptr;
static std::unique_ptr<boost::interprocess::file_mapping> handFile;

// JPEG encoder instance - reuse for performance
static tjhandle jpegCompressor = nullptr;

struct FrameData {
    uint32_t magic = 0xDEADBEEF;
    uint32_t timestamp_ms;
    uint32_t frame_size;
    uint32_t width;
    uint32_t height;
    uint32_t quality;
    // Followed by JPEG data
};

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

bool InitializeJPEGEncoder() {
    jpegCompressor = tjInitCompress();
    if (!jpegCompressor) {
        std::cerr << "Failed to initialize JPEG compressor: " << tjGetErrorStr() << std::endl;
        return false;
    }
    return true;
}

void CleanupJPEGEncoder() {
    if (jpegCompressor) {
        tjDestroy(jpegCompressor);
        jpegCompressor = nullptr;
    }
}

bool EncodeFrameToJPEG(const unsigned char* rgbData, int width, int height, int quality,
    unsigned char** jpegBuf, unsigned long* jpegSize) {
    if (!jpegCompressor) return false;

    // Convert RGBA to RGB (drop alpha channel)
    std::vector<unsigned char> rgbBuffer(width * height * 3);
    for (int i = 0; i < width * height; ++i) {
        rgbBuffer[i * 3 + 0] = rgbData[i * 4 + 0]; // R
        rgbBuffer[i * 3 + 1] = rgbData[i * 4 + 1]; // G
        rgbBuffer[i * 3 + 2] = rgbData[i * 4 + 2]; // B
    }

    int result = tjCompress2(jpegCompressor, rgbBuffer.data(), width, 0, height, TJPF_RGB,
        jpegBuf, jpegSize, TJSAMP_422, quality, TJFLAG_FASTDCT);

    return result == 0;
}

uint32_t GetCurrentTimeMs() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

bool SendEncodedFrame(const unsigned char* jpegData, unsigned long jpegSize,
    int width, int height, int quality) {
    FrameData header;
    header.magic = 0xDEADBEEF;
    header.timestamp_ms = GetCurrentTimeMs();
    header.frame_size = static_cast<uint32_t>(jpegSize);
    header.width = static_cast<uint32_t>(width);
    header.height = static_cast<uint32_t>(height);
    header.quality = static_cast<uint32_t>(quality);

    // Send header (with magic number at start)
    std::cout.write(reinterpret_cast<const char*>(&header), sizeof(FrameData));
    std::cout.write(reinterpret_cast<const char*>(jpegData), jpegSize);
    std::cout.flush();

    return true;
}

int main(void) {
    // Open debug log file for non-intrusive logging
    std::ofstream debugLog("debug.log", std::ios::app);
    debugLog << "[START] VR process launched\n";
    debugLog.flush();

    // Check if stdout is NOT piped and show message
    if (!isStdoutPiped()) {
        debugLog << "[ERROR] Stdout is not piped. Exiting.\n";
        debugLog.flush();
        return 1;
    }

    // Suppress all std::cout output
    // std::cout.rdbuf(nullptr);

    const int screenWidth = 1920;
    const int screenHeight = 1080;
    const int jpegQuality = 85; // Adjustable quality (1-100)
    const float targetFPS = 60.0f;
    const float frameTime = 1.0f / targetFPS;

    debugLog << "[INFO] Before JPEG encoder init\n";
    debugLog.flush();
    if (!InitializeJPEGEncoder()) {
        debugLog << "[ERROR] Failed to initialize JPEG encoder\n";
        debugLog.flush();
        return 1;
    }

    debugLog << "[INFO] Before disabling Raylib logs\n";
    debugLog.flush();
    // Disable all Raylib logging
    SetTraceLogLevel(LOG_NONE);

    debugLog << "[INFO] Before InitWindow\n";
    debugLog.flush();
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_HIDDEN);
    InitWindow(screenWidth, screenHeight, "VR Viewer");
    debugLog << "[INFO] Window initialized\n";
    debugLog.flush();
    SetTargetFPS(0); // Uncapped FPS - we'll control it manually

    // Redirect stderr and stdout to null to prevent interference with binary stdout
    // Remove this line:
    // freopen_s(&nullout, "NUL", "w", stdout);  // ALSO disables stdout fallback from libraries

    // Keep only:
    FILE* nullout;
    freopen_s(&nullout, "NUL", "w", stderr);  // Disables stderr only
    _setmode(_fileno(stdout), _O_BINARY);
    setvbuf(stdout, nullptr, _IONBF, 128 * 1024);

    debugLog << "[INFO] Entering main loop\n";
    debugLog.flush();

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

    // Timing variables
    auto lastFrameTime = std::chrono::high_resolution_clock::now();
    int frameCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    while (!WindowShouldClose()) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();

        // Frame rate limiting
        if (deltaTime < frameTime) {
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<int>((frameTime - deltaTime) * 1000000)
            ));
            currentTime = std::chrono::high_resolution_clock::now();
            deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
        }
        lastFrameTime = currentTime;

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

        // Encode and send frame
        Image frame = LoadImageFromTexture(target.texture);

        // Encode to JPEG
        unsigned char* jpegBuf = nullptr;
        unsigned long jpegSize = 0;

        if (EncodeFrameToJPEG((unsigned char*)frame.data, frame.width, frame.height,
            jpegQuality, &jpegBuf, &jpegSize)) {

            // Send encoded frame
            SendEncodedFrame(jpegBuf, jpegSize, frame.width, frame.height, jpegQuality);
            debugLog << "[INFO] Frame encoded and sent, size: " << jpegSize << "\n";
            debugLog.flush();

            // Free JPEG buffer
            tjFree(jpegBuf);
        }
        // else {
        //     std::cerr << "JPEG encoding failed: " << tjGetErrorStr() << std::endl;
        // }

        UnloadImage(frame);

        frameCount++;

        // Log performance stats every 5 seconds
        // auto elapsed = std::chrono::duration<float>(currentTime - startTime).count();
        // if (elapsed >= 5.0f) {
        //     float avgFPS = frameCount / elapsed;
        //     std::cerr << "Average FPS: " << avgFPS << ", Frame time: " << (deltaTime * 1000) << "ms" << std::endl;
        //     frameCount = 0;
        //     startTime = currentTime;
        // }
    }

    desktopRenderer.cleanup();
    UnloadVrStereoConfig(config);
    UnloadRenderTexture(target);
    UnloadShader(distortion);
    CleanupJPEGEncoder();
    CloseWindow();
    return 0;
}