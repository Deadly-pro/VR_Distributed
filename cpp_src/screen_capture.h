#pragma once
#include <vector>
#include <cstdint>
#include <thread>
#include <atomic>
#include <memory>
#include "thread_safe_queue.h"

struct CapturedFrame {
    std::vector<uint8_t> pixels;
    int width;
    int height;
    int channels;
    bool isValid;
    std::chrono::steady_clock::time_point timestamp;

    CapturedFrame() : width(0), height(0), channels(0), isValid(false) {}
};

class ScreenCapture {
private:
    static std::unique_ptr<std::thread> captureThread;
    static std::atomic<bool> shouldStop;
    static std::atomic<bool> isRunning;
    static ThreadSafeQueue<CapturedFrame> frameQueue;
    static std::atomic<float> captureRate;

    static void captureThreadFunction();
    static CapturedFrame captureDesktopInternal();

public:
    static bool initialize();
    static void cleanup();
    static std::optional<CapturedFrame> getLatestFrame();
    static void setCaptureRate(float fps);
    static bool isInitialized();
    static size_t getQueueSize();
};
