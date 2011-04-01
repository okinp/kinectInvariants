/*
 *  nJetOrder2.h
 *  kinectInvariants
 *
 *  Created by Nikolas Psaroudakis on 3/4/11.
 *  Copyright 2011 Addictive Labs. All rights reserved.
 *
 */

#pragma once 
#include "cv.h"
#include "gaussianDerivatives.h"
using namespace cv;

class nJetOrder2 {
public:
	nJetOrder2();
	void setParams(int ksize, double sigma, int ktype);
	void calculate(const Mat &input);
	Mat Gs,Gsx,Gsy,Gsxx,Gsxy,Gsyy;
private:
	int kernelSize, kernelType;
	double gaussSigma;
	gaussianDerivatives gDerivative;
	Mat K0,K1, K2;
	cv::Ptr<FilterEngine> Fs,Fsx,Fsy, Fsxx, Fsxy, Fsyy;
};