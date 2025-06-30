#ifndef HOLISTIC_DATA_H
#define HOLISTIC_DATA_H

#include "raylib.h"
#include <vector>
#include <string>

struct HolisticHandData {
    std::string handedness;
    std::vector<Vector3> landmarks;
    float distance_factor;
    float depth_scale;
    bool shoulder_calibrated;
    float confidence;

    // Default constructor
    HolisticHandData() : distance_factor(1.0f), depth_scale(1.0f),
        shoulder_calibrated(false), confidence(0.7f) {
    }
};

#endif // HOLISTIC_DATA_H
