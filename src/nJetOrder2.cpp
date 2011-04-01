/*
 *  nJetOrder2.cpp
 *  kinectInvariants
 *
 *  Created by Nikolas Psaroudakis on 3/4/11.
 *  Copyright 2011 Addictive Labs. All rights reserved.
 *
 */

#include "nJetOrder2.h"

nJetOrder2::nJetOrder2(){
	
	
}
void nJetOrder2::setParams(int ksize, double sigma, int ktype){
	kernelSize = ksize;
	kernelType = ktype;
	gaussSigma = sigma;
	gDerivative = gaussianDerivatives(kernelSize, gaussSigma, kernelType);
	K0 = gDerivative.getDerivativeKernel(0);
	K1 = gDerivative.getDerivativeKernel(1);
	K2 = gDerivative.getDerivativeKernel(2);
	Fs = createSeparableLinearFilter( ktype, ktype, K0, K0);
	Fsx = createSeparableLinearFilter( ktype, ktype, K1, K0);
	Fsy = createSeparableLinearFilter( ktype, ktype, K0, K1);
	Fsxx = createSeparableLinearFilter( ktype, ktype, K2, K0);
	Fsxy = createSeparableLinearFilter( ktype, ktype, K1, K1);
	Fsyy = createSeparableLinearFilter( ktype, ktype, K0, K2);
}
void nJetOrder2::calculate(const Mat &input){
	Fs->apply(input, Gs);
	Fsx->apply(input, Gsx);
	Fsy->apply(input, Gsy);
	Fsxx->apply(input, Gsxx);
	Fsxy->apply(input, Gsxy);
	Fsyy->apply(input, Gsyy);
}