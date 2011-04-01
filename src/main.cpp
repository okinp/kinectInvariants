//
//  main.cpp
//  kinectInvariants
//
//  Created by Nikolas Psaroudakis on 2/24/11.
//  Copyright Addictive Labs 2011. All rights reserved.
//

#include "ofMain.h"
#include "kinectInvariantsApp.h"
#include "ofAppGlutWindow.h"

//========================================================================
int main() {

    ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);

	ofRunApp(new kinectInvariantsApp());

}
