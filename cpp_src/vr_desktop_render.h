#pragma once
#include "raylib.h"
#include "screen_capture.h"
#include "windows_input.h" // Include our wrapper instead
#include <chrono>

class VRDesktopRenderer {
private:
    Texture2D desktopTexture;
    bool textureInitialized;
    std::chrono::steady_clock::time_point lastUpdate;
    float maxUpdateRate;

public:
    VRDesktopRenderer();
    ~VRDesktopRenderer();

    void initialize();
    void cleanup();
    void update();
    void renderDesktopPanel(Vector3 panelPosition, Vector3 panelSize);
    void setMaxUpdateRate(float fps);
    bool isTextureReady() const;
    size_t getQueueSize() const;

    // VR Mouse interaction methods
    void sendLeftClick(int x, int y);
    void sendRightClick(int x, int y);
    void sendMouseMove(int x, int y);
    void sendMousePosition(int x, int y);
    void sendMouseDown(int x, int y);
    void sendMouseUp(int x, int y);
};
