#include "vr_desktop_render.h"
#include "rlgl.h"
// NO windows.h include here!

VRDesktopRenderer::VRDesktopRenderer()
    : textureInitialized(false), maxUpdateRate(1.0f / 60.0f) {
    lastUpdate = std::chrono::steady_clock::now();
}

VRDesktopRenderer::~VRDesktopRenderer() {
    cleanup();
}

void VRDesktopRenderer::initialize() {
    bool success = ScreenCapture::initialize();
    if (!success) {
        return;
    }

    ScreenCapture::setCaptureRate(60.0f);
    textureInitialized = false;
    lastUpdate = std::chrono::steady_clock::now();
}

void VRDesktopRenderer::cleanup() {
    if (textureInitialized) {
        UnloadTexture(desktopTexture);
        textureInitialized = false;
    }
    ScreenCapture::cleanup();
}

void VRDesktopRenderer::update() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<float>(now - lastUpdate).count();

    if (elapsed < maxUpdateRate) {
        return;
    }

    auto frameOpt = ScreenCapture::getLatestFrame();
    if (frameOpt.has_value()) {
        const auto& frame = frameOpt.value();

        if (frame.isValid && !frame.pixels.empty()) {
            Image desktopImage = {
                .data = const_cast<void*>(static_cast<const void*>(frame.pixels.data())),
                .width = frame.width,
                .height = frame.height,
                .mipmaps = 1,
                .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
            };

            if (textureInitialized) {
                UpdateTexture(desktopTexture, desktopImage.data);
            }
            else {
                desktopTexture = LoadTextureFromImage(desktopImage);
                textureInitialized = true;
            }

            lastUpdate = now;
        }
    }
}

void VRDesktopRenderer::renderDesktopPanel(Vector3 panelPosition, Vector3 panelSize) {
    if (!textureInitialized) {
        DrawCube(panelPosition, panelSize.x, panelSize.y, 0.1f, GRAY);
        DrawCubeWires(panelPosition, panelSize.x, panelSize.y, 0.1f, RED);
        return;
    }

    Vector3 corners[4] = {
        {panelPosition.x - panelSize.x / 2, panelPosition.y + panelSize.y / 2, panelPosition.z},
        {panelPosition.x + panelSize.x / 2, panelPosition.y + panelSize.y / 2, panelPosition.z},
        {panelPosition.x + panelSize.x / 2, panelPosition.y - panelSize.y / 2, panelPosition.z},
        {panelPosition.x - panelSize.x / 2, panelPosition.y - panelSize.y / 2, panelPosition.z}
    };

    rlSetTexture(desktopTexture.id);
    rlBegin(RL_QUADS);
    rlColor4ub(255, 255, 255, 255);

    rlTexCoord2f(1.0f, 0.0f); rlVertex3f(corners[0].x, corners[0].y, corners[0].z);
    rlTexCoord2f(0.0f, 0.0f); rlVertex3f(corners[1].x, corners[1].y, corners[1].z);
    rlTexCoord2f(0.0f, 1.0f); rlVertex3f(corners[2].x, corners[2].y, corners[2].z);
    rlTexCoord2f(1.0f, 1.0f); rlVertex3f(corners[3].x, corners[3].y, corners[3].z);

    rlEnd();
    rlSetTexture(0);
}

void VRDesktopRenderer::setMaxUpdateRate(float fps) {
    maxUpdateRate = 1.0f / fps;
}

bool VRDesktopRenderer::isTextureReady() const {
    return textureInitialized;
}

size_t VRDesktopRenderer::getQueueSize() const {
    return ScreenCapture::getQueueSize();
}

// VR Mouse interaction implementations - call our wrapper functions
void VRDesktopRenderer::sendLeftClick(int x, int y) {
    SendVRLeftClick(x, y);
}

void VRDesktopRenderer::sendRightClick(int x, int y) {
    SendVRRightClick(x, y);
}

void VRDesktopRenderer::sendMouseMove(int x, int y) {
    SendVRMouseMove(x, y);
}

void VRDesktopRenderer::sendMousePosition(int x, int y) {
    SendVRMousePosition(x, y);
}

void VRDesktopRenderer::sendMouseDown(int x, int y) {
    SendVRMouseDown(x, y);
}

void VRDesktopRenderer::sendMouseUp(int x, int y) {
    SendVRMouseUp(x, y);
}
