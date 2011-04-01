//
//  kinectInvariantsApp.cpp
//  kinectInvariants
//
//  Created by Nikolas Psaroudakis on 2/24/11.
//  Copyright Addictive Labs 2011. All rights reserved.
//

#include "kinectInvariantsApp.h"


//--------------------------------------------------------------
void kinectInvariantsApp::setup() {
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();
	kinect.setCameraTiltAngle(60);
	selectedFilter = 0;
	
	videoColor.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
	videoGrayscale.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
	
	
	panel.setup("Control Panel", 5, 5, 300, 600);
	panel.addPanel("Threshold and Scale");
	panel.addPanel("Operations");
	
	panel.setWhichPanel("Threshold and Scale");
	panel.addSlider("near threshold", "nearThreshold", 255, 0, 255, true);
	panel.addSlider("far threshold", "farThreshold", 0, 0, 255, true);
	panel.addSlider("depth scale", "depthScale", 5, 1, 20);
	panel.addSlider("depth offset", "depthOffset", 128, 0, 255);
	panel.addSlider("step size", "stepSize", 2, 1, 4, true);
	panel.addSlider("point size", "pointSize", 1, 1, 10, true);
	panel.addToggle("draw zeros", "drawZeros", false);
	
	panel.setWhichPanel("Operations");
	panel.addToggle("Apply Operations", "applyOperations", false);
	panel.addSlider("Apply Threshold", "threshold", 0, -360, 360, false);	
	panel.addToggle(">=Threshold", "greaterThan", false);
	panel.addToggle("Apply on Depth Data", "onDepth", false);
	panel.addToggle("Threshold", "doThreshold", false);
	vector<string> names;
	names.push_back("G - 0");
	names.push_back("Gx - 1");
	names.push_back("Gy - 2");
	names.push_back("Gxx - 3");
	names.push_back("Gxy - 4");
	names.push_back("Gyy - 5");
	names.push_back("Gaussian Curvature - 6");
	names.push_back("Corners - 7");
	names.push_back("Edges - 8");
	names.push_back("Ridges - 9");
	panel.addMultiToggle("Operation", "operation", 0, names);
	
	panel.addSlider("Sigma", "sigma", 0.4f, 0.4f, 10.0f, false);
	panel.addSlider("KernelSize", "kernel", 5, 5, 41, true);
}

//--------------------------------------------------------------
void kinectInvariantsApp::update() {
	kinect.update();
	if (kinect.isFrameNew()){
		videoColor.setFromPixels(kinect.getPixels(), kinect.width, kinect.height, OF_IMAGE_COLOR);
		depth.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
		videoColor.setImageType(OF_IMAGE_GRAYSCALE);
		videoGrayscale = videoColor;
		videoGrayscale.update();
		depth.update();
		if (panel.getValueB("applyOperations", 0)) {
			
			if (!panel.getValueB("onDepth", 0)){
				applyFilter(videoGrayscale, videoGrayscale);
				//getDerivativeImage(videoGrayscale, videoGrayscale, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 1, 0);
			} else {
				applyFilter(depth, depth);
				//getDerivativeImage(depth, depth, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 1, 0);
			}
			
		//	cout << panel.getValueB("operation", 0) << endl;
			
//			getDerivativeImage(depth, depth, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 1, 0);
			
			
			
		}
		
	}
}

//--------------------------------------------------------------
void kinectInvariantsApp::draw() {
	//panel.draw();
	ofSetColor(255, 255, 255);
	//kinect.draw(420, 10, 400, 300);
	videoGrayscale.draw(0, 10, 400, 300);
	depth.draw(420, 10, 400,300);
}
void kinectInvariantsApp::applyFilter(ofImage &input, ofImage &output){
	
//	cout << panel.getValueB("operation", 0) << endl;
	//Get option
	if (selectedFilter == 0) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 0, 0);
	}
	else if (selectedFilter == 1) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 1, 0);
	} else if (selectedFilter == 2) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 0, 1);
	} else if (selectedFilter == 3) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 2, 0);
		
	} else if (selectedFilter == 4) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 1, 1);
		
	} else if (selectedFilter == 5) {
		getDerivativeImage(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0), 0, 2);
		
	} else if (selectedFilter == 6) {
		getGaussianCurvature(input, input, panel.getValueI("kernel", 0), panel.getValueF("sigma", 0));
		
	} else if (selectedFilter == 7) {
		
		
	} else if (selectedFilter == 8) {
		
		
	}
	
	
	
	
	
	
	
	
	
}




void kinectInvariantsApp::getDerivativeImage(ofImage &inputImage,ofImage &outputImage, int kernelSize, double sigma, int orderX, int orderY){
	inputImage.setImageType(OF_IMAGE_GRAYSCALE);
	unsigned char* pixels = inputImage.getPixels();
	//Allocate IplImages  
	IplImage *inputCV = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	IplImage *outputCV = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	IplImage *inputCV64 = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	IplImage *outputCV64 = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	
	IplImage *temp = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	//set inputCV to contain the pixels of inputImage
	cvSetData(inputCV, pixels, inputImage.width);
	//set inputCV32 to point to a float version of the input with range 0->1
	cvConvertScale( inputCV, inputCV64, 1/255., 0.0 );
	
	//Get cv::Mat of input
	
	Mat source = inputCV64;
	Mat destination = outputCV64;
	
	
	applyGaussianDerivative(source, destination,cv::Size(kernelSize,kernelSize), sigma, sigma, orderX,orderY,1);
	
	CvMat cvmat = destination;
	outputCV64 = cvGetImage( &cvmat, temp );
	double minVal, maxVal;
	
	cvMinMaxLoc( outputCV64, &minVal, &maxVal, NULL, NULL, 0);
	
	//cout << "max is: "<< maxVal << endl;
	//cout << "min is: "<< minVal << endl;
	//cvConvertScale( outputCV32, outputCV, 255, 0.0 );
	//cvConvertScale( outputCV32, outputCV, 255./(maxVal-minVal), -minVal*255./(maxVal-minVal) );
	cvConvertScale( outputCV64, outputCV64, 255./(maxVal-minVal), -minVal*255./(maxVal-minVal) );
	//cvConvertScale( outputCV32, outputCV32, -1.0, 255.0 );
	cvConvert(outputCV64, outputCV);
	cvMinMaxLoc( outputCV64, &minVal, &maxVal, NULL, NULL, 0);
	
	//cout << "max New is: "<< maxVal << endl;
	//cout << "min New is: "<< minVal << endl;
	
	
	//cvConvertScale( outputCV32, outputCV, 255, 0.0 );
	
	//Mat img2 = imread("Lenna.jpg");
	//cvSetData doesn't copy it just sets the pointer to pixels
	
	
	//Mat KernelX = getGaussianDerivativeKernel(5, 0.5, CV_64F, 2);
	outputImage.setFromPixels((unsigned char*) outputCV->imageData, inputImage.width, inputImage.height, OF_IMAGE_GRAYSCALE );
	outputImage.update();
	
	
	cvReleaseImage(&inputCV);
	cvReleaseImage(&inputCV64);
	cvReleaseImage(&outputCV);
	cvReleaseImage(&outputCV64);
	//cvReleaseImage(&temp);
	source.release();
	destination.release();
	//return outputImage;
	
	
}


void kinectInvariantsApp::applyGaussianDerivative(const Mat& src, Mat& dst, cv::Size ksize, double sigma1, double sigma2, int order1, int order2, int borderType){
	
	Mat kx = getGaussianDerivativeKernel(ksize.width, sigma1, CV_64F, order1);
	Mat ky = getGaussianDerivativeKernel(ksize.height, sigma2, CV_64F, order2);
	//TODO: Check if type is correct
	
	cv::Ptr<FilterEngine> f = createSeparableLinearFilter( CV_64F, CV_64F, kx, ky, cv::Point(-1,-1), 0, borderType );
	f->apply(src, dst);
	kx.release();
	ky.release();
}
void  kinectInvariantsApp::getGaussianCurvature(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma){
	inputImage.setImageType(OF_IMAGE_GRAYSCALE);
	unsigned char* pixels = inputImage.getPixels();
	//Allocate IplImages  
	IplImage *inputCV = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	IplImage *outputCV = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	IplImage *inputCV64 = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	IplImage *outputCV64 = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	IplImage *outputCV64Lxx = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	IplImage *outputCV64Lyy = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	IplImage *outputCV64Lxy = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	
//	IplImage *outputThresholded = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_64F, 1);
	
	
	
	IplImage *temp = cvCreateImage(cvSize(inputImage.width, inputImage.width), IPL_DEPTH_8U, 1);
	//set inputCV to contain the pixels of inputImage
	cvSetData(inputCV, pixels, inputImage.width);
	//set inputCV32 to point to a float version of the input with range 0->1
	cvConvertScale( inputCV, inputCV64, 1/255., 0.0 );
	
	//Get cv::Mat of input
	
	Mat source = inputCV64;
	Mat destination = outputCV64;
	Mat Lxx = outputCV64Lxx;
	Mat Lyy = outputCV64Lyy;
	Mat Lxy = outputCV64Lxy;
//	Mat thresholded = outputThresholded;
	
	applyGaussianDerivative(source, Lxx,cv::Size(kernelSize,kernelSize), sigma, sigma, 2,0,1);
	applyGaussianDerivative(source, Lyy,cv::Size(kernelSize,kernelSize), sigma, sigma, 0,2,1);
	applyGaussianDerivative(source, Lxy,cv::Size(kernelSize,kernelSize), sigma, sigma, 1,1,1);
	
	destination = Lxx.mul(Lyy) - Lxy.mul(Lxy);
	
//	if (panel.getValueB("doThreshold", 0)) {
//		if (panel.getValueB("greaterThan", 0)) {
//			//cout << "the threshold is: " << panel.getValueF("threshold",0) <<endl;	
//		threshold(destination, thresholded, (double) 0.0, 255.0, THRESH_BINARY);
//		} else {
//			//panel.getValueF("threshold",0)
//			threshold(destination, thresholded, (double) 0.0 , 255.0, THRESH_BINARY_INV);	
//		}
//	}
//	destination  = thresholded;
	
	
	CvMat cvmat = destination;
	outputCV64 = cvGetImage( &cvmat, temp );
	double minVal, maxVal;
	
	cvMinMaxLoc( outputCV64, &minVal, &maxVal, NULL, NULL, 0);
	
	//cout << "max is: "<< maxVal << endl;
	//cout << "min is: "<< minVal << endl;
	//cvConvertScale( outputCV32, outputCV, 255, 0.0 );
	//cvConvertScale( outputCV32, outputCV, 255./(maxVal-minVal), -minVal*255./(maxVal-minVal) );
	cvConvertScale( outputCV64, outputCV64, 255./(maxVal-minVal), -minVal*255./(maxVal-minVal) );
	//cvConvertScale( outputCV32, outputCV32, -1.0, 255.0 );
	cvConvert(outputCV64, outputCV);
	cvMinMaxLoc( outputCV64, &minVal, &maxVal, NULL, NULL, 0);
	
	//cout << "max New is: "<< maxVal << endl;
	//cout << "min New is: "<< minVal << endl;
	
	
	//cvConvertScale( outputCV32, outputCV, 255, 0.0 );
	
	//Mat img2 = imread("Lenna.jpg");
	//cvSetData doesn't copy it just sets the pointer to pixels
	
	
	//Mat KernelX = getGaussianDerivativeKernel(5, 0.5, CV_64F, 2);
	outputImage.setFromPixels((unsigned char*) outputCV->imageData, inputImage.width, inputImage.height, OF_IMAGE_GRAYSCALE );
	outputImage.update();
	
	
	cvReleaseImage(&inputCV);
	cvReleaseImage(&inputCV64);
	cvReleaseImage(&outputCV);
	cvReleaseImage(&outputCV64);
	cvReleaseImage(&outputCV64Lxx);
	cvReleaseImage(&outputCV64Lxy);
	cvReleaseImage(&outputCV64Lyy);
//	cvReleaseImage(&outputThresholded);
	//cvReleaseImage(&temp);
	source.release();
	destination.release();
	Lxx.release();
	Lyy.release();
	Lxy.release();
//	thresholded.release();
	//return outputImage;
	
	
	
}
void kinectInvariantsApp::getRidges(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma){
	
	
	
	
}
void kinectInvariantsApp::getCorners(ofImage &inputImage, ofImage &outputImage, int kernelSize, double sigma){
	
	
	
}



Mat kinectInvariantsApp::getGaussianDerivativeKernel(int n, double sigma, int ktype, int order){
	
	
    CV_Assert( ktype == CV_32F || ktype == CV_64F );
    Mat kernel(n, 1, ktype);
    float* cf = (float*)kernel.data;
    double* cd = (double*)kernel.data;
    double sigmaX = sigma > 0 ? sigma : ((n-1)*0.5 - 1)*0.3 + 0.8;
    double scale2X = -0.5/(sigmaX*sigmaX);
    double sum = 0;
	
    int i;
    for( i = 0; i < n; i++ )
    {
        double x = i - (n-1)*0.5;
        double t = std::exp(scale2X*x*x);
        if( ktype == CV_32F )
        {
            cf[i] = (float)t;
            sum += cf[i];
        }
        else
        {
            cd[i] = t;
            sum += cd[i];
        }
    }
	
    sum = 1./sum;
	//cout << "started 0" << endl;
    for( i = 0; i < n; i++ )
    {
        if( ktype == CV_32F )
            cf[i] = (float)(cf[i]*sum);
        else{
            cd[i] *= sum;
			//	cout << cd[i] << endl;
		}
    }
	/// Finished calculating Gaussian
	
	if (order==1) {
		//	cout << "started 1" << endl;
		for (i=0; i<n; i++) {
			double x = i - (n-1)*0.5;
			if( ktype == CV_32F ){
				cf[i]*= (float)(-x/(sigmaX*sigmaX));	
			}
			else {
				cd[i]*= (-x/(sigmaX*sigmaX));
				//			cout << cd[i] << endl;
			}
		}
	} else if (order==2) {
		//	cout << "started 2" << endl;
		for (i=0; i<n; i++) {
			
			double x = i - (n-1)*0.5;
			if( ktype == CV_32F ){
				cf[i]*= (float)(x*x-sigmaX*sigmaX)/(sigmaX*sigmaX*sigmaX*sigmaX);	
				
			}
			else {
				cd[i]*= (x*x-sigmaX*sigmaX)/(sigmaX*sigmaX*sigmaX*sigmaX);
				//			cout << cd[i] << endl;
			}
		}
	}
	return kernel;
}

void kinectInvariantsApp::exit(){
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}
//--------------------------------------------------------------
void kinectInvariantsApp::keyPressed(int key) {
	if (key=='0') selectedFilter =0;
	else if (key=='1') selectedFilter =1;
	else if (key=='2') selectedFilter =2;
	else if (key=='3') selectedFilter =3;
	else if (key=='4') selectedFilter =4;
	else if (key=='5') selectedFilter =5;
	else if (key=='6') selectedFilter =6;
}

//--------------------------------------------------------------
void kinectInvariantsApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void kinectInvariantsApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void kinectInvariantsApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void kinectInvariantsApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void kinectInvariantsApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void kinectInvariantsApp::windowResized(int w, int h) {

}

