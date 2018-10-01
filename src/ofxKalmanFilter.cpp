//
//  ofxKalmanFilter.cpp
//
//  Created by Tobias Ebsen on 16/09/18.
//
//

#include "ofxKalmanFilter.h"

#define DEAD_LABEL 0xFFFFFFFF

void ofxKalmanFilter::init(unsigned int elementSize, unsigned int count) {
    this->elementSize = elementSize;
    aliveLabels.resize(count, DEAD_LABEL);
	numAlive = 0;
    KalmanFilter::init(count*elementSize);
}

void ofxKalmanFilter::keepLabels(const std::vector<unsigned int> &labels) {
    numAlive = 0;
	for (int i=0; i<aliveLabels.size(); i++) {
        bool found = false;
        for (unsigned int label : labels) {
            if (label == aliveLabels[i]) {
                found = true;
				numAlive++;
                break;
            }
        }
        if (!found)
            aliveLabels[i] = DEAD_LABEL;
    }
}

unsigned int ofxKalmanFilter::getNumAlive() {
	return numAlive;
}

unsigned int ofxKalmanFilter::getAliveLabel(unsigned int index) {
	int j=0;
	for (int i=0; i<aliveLabels.size(); i++) {
		if (aliveLabels[i] != DEAD_LABEL) {
			if (j == index)
				return aliveLabels[i];
			j++;
		}
	}
	return DEAD_LABEL;
}

bool ofxKalmanFilter::hasLabel(unsigned int label) {
	for (int i=0; i<aliveLabels.size(); i++) {
		if (aliveLabels[i] == label)
			return true;
	}
	return false;
}

void ofxKalmanFilter::predict(float dt) {
	for (int i=0; i<aliveLabels.size(); i++) {
		if (aliveLabels[i] != DEAD_LABEL) {
			KalmanFilter::predict(dt, NULL, elementSize, i*elementSize);
		}
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
        for (int i=0; i<aliveLabels.size(); i++) {
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