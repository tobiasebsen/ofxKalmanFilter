//
//  ofxKalmanFilter.h
//
//  Created by Tobias Ebsen on 22/05/18.
//
//

#pragma once

#include "kalman.h"
#include <vector>

class ofxKalmanFilter : public KalmanFilter {
public:

    // Initialize elements. Vec2f: size = 2, Vec3f: size = 3
    void init(size_t elementSize, size_t count);

    // Keep only these labels. Delete all other.
    void keepLabels(const std::vector<unsigned int> & labels);

    // Correct an element
    void correct(unsigned int label, float * element);

    // Get element
    void get(unsigned int label, float * element);

protected:
    size_t elementSize;
    std::vector<unsigned int> aliveLabels;
};