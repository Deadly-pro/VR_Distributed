#include "vr_desktop_render.h"
#include "rlgl.h"
#include <iostream>

VRDesktopRenderer::VRDesktopRenderer()
    : textureInitialized(false), maxUpdateRate(1.0f / 60.0f) {
    lastUpdate = std::chrono::steady_clock::now();
}

VRDesktopRenderer::~VRDesktopRenderer() {
    cleanup();
}

void VRDesktopRenderer::initialize() {
    //std::cout << "Initializing VR Desktop Renderer..." << std::endl;

    bool success = ScreenCapture::initialize();
    if (!success) {
        //std::cout << "Failed to initialize screen capture!" << std::endl;
        return;
    }

    ScreenCapture::setCaptureRate(60.0f); // Capture at 30 FPS
    textureInitialized = false;
    lastUpdate = std::chrono::steady_clock::now();

    //std::cout << "VR Desktop Renderer initialized successfully" << std::endl;
}

void VRDesktopRenderer::cleanup() {
    //std::cout << "Cleaning up VR Desktop Renderer..." << std::endl;
    if (textureInitialized) {
        UnloadTexture(desktopTexture);
        textureInitialized = false;
    }
    ScreenCapture::cleanup();
}

void VRDesktopRenderer::update() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<float>(now - lastUpdate).count();

    // Limit texture updates to prevent overwhelming the GPU
    if (elapsed < maxUpdateRate) {
        return;
    }

    auto frameOpt = ScreenCapture::getLatestFrame();
    if (frameOpt.has_value()) {
        const auto& frame = frameOpt.value();

        if (frame.isValid && !frame.pixels.empty()) {
            // Create raylib Image from captured data
            Image desktopImage = {
                .data = const_cast<void*>(static_cast<const void*>(frame.pixels.data())),
                .width = frame.width,
                .height = frame.height,
                .mipmaps = 1,
                .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
            };

            // Update or create texture (this must happen on main thread)
            if (textureInitialized) {
                UpdateTexture(desktopTexture, desktopImage.data);
            }
            else {
                desktopTexture = LoadTextureFromImage(desktopImage);
                textureInitialized = true;
                //std::cout << "Created desktop texture: " << frame.width << "x" << frame.height << std::endl;
            }

            lastUpdate = now;
        }
    }
}

void VRDesktopRenderer::renderDesktopPanel(Vector3 panelPosition, Vector3 panelSize) {
    if (!textureInitialized) {
        // Draw a placeholder rectangle when texture isn't ready
        DrawCube(panelPosition, panelSize.x, panelSize.y, 0.1f, GRAY);
        DrawCubeWires(panelPosition, panelSize.x, panelSize.y, 0.1f, RED);
        return;
    }

    // Calculate panel vertices for 3D quad
    Vector3 corners[4] = {
        {panelPosition.x - panelSize.x / 2, panelPosition.y + panelSize.y / 2, panelPosition.z},
        {panelPosition.x + panelSize.x / 2, panelPosition.y + panelSize.y / 2, panelPosition.z},
        {panelPosition.x + panelSize.x / 2, panelPosition.y - panelSize.y / 2, panelPosition.z},
        {panelPosition.x - panelSize.x / 2, panelPosition.y - panelSize.y / 2, panelPosition.z}
    };

    // Texture coordinates with flipping options
    float texCoords[8];

    // Standard texture coordinates (mirrored flipping)
     texCoords[0] = 0.0f; texCoords[1] = 0.0f; // Top-left
     texCoords[2] = 1.0f; texCoords[3] = 0.0f; // Top-right
     texCoords[4] = 1.0f; texCoords[5] = 1.0f; // Bottom-right
     texCoords[6] = 0.0f; texCoords[7] = 1.0f; // Bottom-left

    rlSetTexture(desktopTexture.id);
    rlBegin(RL_QUADS);
    rlColor4ub(255, 255, 255, 255);


    // Flipped horizontally by swapping U coordinates (1.0 <-> 0.0)
    rlTexCoord2f(1.0f, 0.0f); rlVertex3f(corners[0].x, corners[0].y, corners[0].z); // Top-left uses right texture
    rlTexCoord2f(0.0f, 0.0f); rlVertex3f(corners[1].x, corners[1].y, corners[1].z); // Top-right uses left texture
    rlTexCoord2f(0.0f, 1.0f); rlVertex3f(corners[2].x, corners[2].y, corners[2].z); // Bottom-right uses left texture
    rlTexCoord2f(1.0f, 1.0f); rlVertex3f(corners[3].x, corners[3].y, corners[3].z); // Bottom-left uses right texture

    rlEnd();
    rlSetTexture(0);
}

void VRDesktopRenderer::setMaxUpdateRate(float fps) {
    maxUpdateRate = 1.0f / fps;
    //std::cout << "Set max texture update rate to " << fps << " FPS" << std::endl;
}

bool VRDesktopRenderer::isTextureReady() const {
    return textureInitialized;
}

size_t VRDesktopRenderer::getQueueSize() const {
    return ScreenCapture::getQueueSize();
}
