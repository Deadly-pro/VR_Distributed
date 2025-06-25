#pragma once
#include "raylib.h"
#include "screen_capture.h"
#include <chrono>

class VRDesktopRenderer {
private:
    Texture2D desktopTexture;
    bool textureInitialized;
    std::chrono::steady_clock::time_point lastUpdate;
    float maxUpdateRate; // Maximum texture update rate 

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
};
