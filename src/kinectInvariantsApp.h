//
//  kinectInvariantsApp.h
//  kinectInvariants
//
//  Created by Nikolas Psaroudakis on 2/24/11.
//  Copyright Addictive Labs 2011. All rights reserved.
//

#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxAutoControlPanel.h"
#include "cv.h"
using namespace cv;
//========================================================================
class kinectInvariantsApp : public ofBaseApp {
	
public:
	void setup();
	void update();
	void draw();
	
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void exit();
	
	Mat getGaussianDerivativeKernel(int n, double sigma, int ktype, int order);
	void applyGaussianDerivative(const Mat& src, Mat& dst, cv::Size ksize, double sigma1, double sigma2, int order1, int order2, int borderType);
	void getDerivativeImage(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma, int orderX, int orderY);
	void getGaussianCurvature(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma);
	void getRidges(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma);
	void getCorners(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma);
	
	
	
	void applyFilter(ofImage &input, ofImage &output);
	
	
	ofxKinect kinect;
	ofxAutoControlPanel panel;
	ofImage videoColor;
	ofImage videoGrayscale;
	ofImage depth;
	
	int selectedFilter;
	
};
