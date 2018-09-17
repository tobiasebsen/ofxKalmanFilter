//
//  ofxKalmanFilter.cpp
//
//  Created by Tobias Ebsen on 16/09/18.
//
//

#include "ofxKalmanFilter.h"

#define DEAD_LABEL 0xFFFFFFFF

void ofxKalmanFilter::init(size_t elementSize, size_t count) {
    this->elementSize = elementSize;
    aliveLabels.resize(count, DEAD_LABEL);
    KalmanFilter::init(count*elementSize);
}

void ofxKalmanFilter::keepLabels(const std::vector<unsigned int> &labels) {
    for (int i=0; i<aliveLabels.size(); i++) {
        bool found = false;
        for (unsigned int label : labels) {
            if (label == aliveLabels[i]) {
                found = true;
                break;
            }
        }
        if (!found)
            aliveLabels[i] = DEAD_LABEL;
    }
}

void ofxKalmanFilter::correct(unsigned int label, float *element) {
    bool found = false;
    for (int i=0; i<aliveLabels.size(); i++) {
        if (label == aliveLabels[i]) {
            found = true;
            KalmanFilter::correct(element, elementSize, i*elementSize);
            break;
        }
    }
    if (!found) {
        for (int i=0; i<30; i++) {
            if (aliveLabels[i] == DEAD_LABEL) {
                aliveLabels[i] = label;
                KalmanFilter::reset(elementSize, i*elementSize);
                KalmanFilter::set(element, elementSize, i*elementSize);
                break;
            }
        }
    }
}

void ofxKalmanFilter::get(unsigned int label, float *element) {
    for (int i=0; i<aliveLabels.size(); i++) {
        if (label == aliveLabels[i]) {
            KalmanFilter::get(element, elementSize, i*elementSize);
            break;
        }
    }

}