#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <wingdi.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "screen_capture.h"

// Static member definitions
std::unique_ptr<std::thread> ScreenCapture::captureThread = nullptr;
std::atomic<bool> ScreenCapture::shouldStop{ false };
std::atomic<bool> ScreenCapture::isRunning{ false };
ThreadSafeQueue<CapturedFrame> ScreenCapture::frameQueue;
std::atomic<float> ScreenCapture::captureRate{ 1.0f / 120.0f };

// Thread-local storage for Windows handles
thread_local HDC screenDC = nullptr;
thread_local HDC memoryDC = nullptr;
thread_local HBITMAP bitmap = nullptr;
thread_local BITMAPINFO bitmapInfo = {};
thread_local bool threadInitialized = false;
thread_local int lastWidth = 0;
thread_local int lastHeight = 0;

bool initializeCaptureThread() {
    if (threadInitialized) return true;

    screenDC = GetDC(NULL);
    if (!screenDC) {
        // // std::cout << "Failed to get screen DC" << std::endl;
        return false;
    }

    memoryDC = CreateCompatibleDC(screenDC);
    if (!memoryDC) {
        // // std::cout << "Failed to create memory DC" << std::endl;
        ReleaseDC(NULL, screenDC);
        return false;
    }

    threadInitialized = true;
    // // std::cout << "Capture thread initialized successfully" << std::endl;
    return true;
}

void cleanupCaptureThread() {
    if (bitmap) {
        DeleteObject(bitmap);
        bitmap = nullptr;
    }
    if (memoryDC) {
        DeleteDC(memoryDC);
        memoryDC = nullptr;
    }
    if (screenDC) {
        ReleaseDC(NULL, screenDC);
        screenDC = nullptr;
    }
    threadInitialized = false;
    lastWidth = 0;
    lastHeight = 0;
    // // std::cout << "Capture thread cleaned up" << std::endl;
}

CapturedFrame ScreenCapture::captureDesktopInternal() {
    CapturedFrame frame;

    if (!threadInitialized) {
        if (!initializeCaptureThread()) {
            // // std::cout << "Failed to initialize capture thread" << std::endl;
            return frame;
        }
    }

    int screenWidth = GetSystemMetrics(SM_CXSCREEN);
    int screenHeight = GetSystemMetrics(SM_CYSCREEN);

    if (screenWidth <= 0 || screenHeight <= 0) {
        // // std::cout << "Invalid screen dimensions: " << screenWidth << "x" << screenHeight << std::endl;
        return frame;
    }

    // Create or recreate bitmap if size changed
    if (!bitmap || screenWidth != lastWidth || screenHeight != lastHeight) {
        if (bitmap) {
            DeleteObject(bitmap);
        }

        bitmap = CreateCompatibleBitmap(screenDC, screenWidth, screenHeight);
        if (!bitmap) {
            // // std::cout << "Failed to create compatible bitmap" << std::endl;
            return frame;
        }

        // Setup bitmap info for pixel data extraction
        ZeroMemory(&bitmapInfo, sizeof(BITMAPINFO));
        bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
        bitmapInfo.bmiHeader.biWidth = screenWidth;
        bitmapInfo.bmiHeader.biHeight = -screenHeight; // Negative for top-down
        bitmapInfo.bmiHeader.biPlanes = 1;
        bitmapInfo.bmiHeader.biBitCount = 32;
        bitmapInfo.bmiHeader.biCompression = BI_RGB;

        lastWidth = screenWidth;
        lastHeight = screenHeight;
        // // std::cout << "Created new bitmap: " << screenWidth << "x" << screenHeight << std::endl;
    }

    HBITMAP oldBitmap = (HBITMAP)SelectObject(memoryDC, bitmap);

    // Capture screen
    BOOL blitResult = BitBlt(memoryDC, 0, 0, screenWidth, screenHeight, screenDC, 0, 0, SRCCOPY);
    if (!blitResult) {
        DWORD error = GetLastError();
        // // std::cout << "BitBlt failed with error: " << error << std::endl;
        SelectObject(memoryDC, oldBitmap);
        return frame;
    }

    // Extract pixel data
    frame.pixels.resize(screenWidth * screenHeight * 4);
    int result = GetDIBits(screenDC, bitmap, 0, screenHeight,
        frame.pixels.data(), &bitmapInfo, DIB_RGB_COLORS);

    SelectObject(memoryDC, oldBitmap);

    if (result == 0) {
        DWORD error = GetLastError();
        // // std::cout << "GetDIBits failed with error: " << error << std::endl;
        return frame;
    }

    frame.width = screenWidth;
    frame.height = screenHeight;
    frame.channels = 4;
    frame.isValid = true;
    frame.timestamp = std::chrono::steady_clock::now();

    // Convert BGRA to RGBA (Windows uses BGRA format)
    for (size_t i = 0; i < frame.pixels.size(); i += 4) {
        std::swap(frame.pixels[i], frame.pixels[i + 2]);
    }

    return frame;
}

void ScreenCapture::captureThreadFunction() {
    // // std::cout << "Capture thread started" << std::endl;
    isRunning = true;

    int frameCount = 0;
    auto lastLog = std::chrono::steady_clock::now();

    while (!shouldStop) {
        auto startTime = std::chrono::steady_clock::now();

        CapturedFrame frame = captureDesktopInternal();
        if (frame.isValid) {
            // Keep only the latest 3 frames to prevent memory buildup
            while (frameQueue.size() > 2) {
                frameQueue.tryPop();
            }
            frameQueue.push(std::move(frame));
            frameCount++;

            // Log progress every 5 seconds
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastLog).count() >= 5) {
                // // std::cout << "Captured " << frameCount << " frames in last 5 seconds, queue size: " << frameQueue.size() << std::endl;
                frameCount = 0;
                lastLog = now;
            }
        }
        else {
            // // // std::cout << "Failed to capture frame" << std::endl;
        }

        // Sleep for the remaining time to maintain capture rate
        auto endTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        auto targetDuration = std::chrono::milliseconds(static_cast<int>(captureRate.load() * 1000));

        if (elapsed < targetDuration) {
            std::this_thread::sleep_for(targetDuration - elapsed);
        }
    }

    cleanupCaptureThread();
    isRunning = false;
    // // // std::cout << "Capture thread stopped" << std::endl;
}

bool ScreenCapture::initialize() {
    if (captureThread && isRunning) {
        // // // std::cout << "Capture already initialized" << std::endl;
        return true;
    }

    // // // std::cout << "Initializing screen capture..." << std::endl;
    shouldStop = false;
    captureThread = std::make_unique<std::thread>(captureThreadFunction);

    // Wait for thread to start with timeout
    int timeout = 0;
    while (!isRunning && timeout < 1000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout += 10;
    }

    if (!isRunning) {
        // // // std::cout << "Failed to start capture thread within timeout" << std::endl;
        return false;
    }

    // // std::cout << "Screen capture initialized successfully" << std::endl;
    return true;
}

void ScreenCapture::cleanup() {
    // // std::cout << "Cleaning up screen capture..." << std::endl;
    if (captureThread) {
        shouldStop = true;
        if (captureThread->joinable()) {
            captureThread->join();
        }
        captureThread.reset();
    }

    // Clear remaining frames
    int clearedFrames = 0;
    while (!frameQueue.empty()) {
        frameQueue.tryPop();
        clearedFrames++;
    }
    // // std::cout << "Cleared " << clearedFrames << " remaining frames" << std::endl;
}

std::optional<CapturedFrame> ScreenCapture::getLatestFrame() {
    return frameQueue.tryPop();
}

void ScreenCapture::setCaptureRate(float fps) {
    captureRate = 1.0f / fps;
    // // std::cout << "Set capture rate to " << fps << " FPS" << std::endl;
}

bool ScreenCapture::isInitialized() {
    return isRunning;
}

size_t ScreenCapture::getQueueSize() {
    return frameQueue.size();
}
