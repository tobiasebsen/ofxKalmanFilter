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
    void init(unsigned int elementSize, unsigned int count);

    // Keep only these labels. Delete all other.
    void keepLabels(const std::vector<unsigned int> & labels);
	unsigned int getNumAlive();
	unsigned int getAliveLabel(unsigned int index);
	bool hasLabel(unsigned int label);

	// Predict alive models
	void predict(float dt);

    // Correct an element
    void correct(unsigned int label, float * element);

    // Get element
    void get(unsigned int label, float * element);

protected:
	unsigned int elementSize;
    std::vector<unsigned int> aliveLabels;
	unsigned int numAlive;
};

template <typename T>
class ofxKalmanFilterT : public ofxKalmanFilter {
public:
	void init(size_t count) {ofxKalmanFilter::init(elementSize(), count);}
	void correct(unsigned int label, T & element) {ofxKalmanFilter::correct(label, (float*)&element);}
	T get(unsigned int label) {
		T element;
		ofxKalmanFilter::get(label, (float*)&element);
		return element;
	}

protected:
	unsigned int elementSize() const {return sizeof(T)/sizeof(float);}
};