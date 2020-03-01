#pragma once


extern "C"
{
	enum ARUCO_DICTIONARY {
		ARUCO = 0,			//original aruco dictionary. By default
		ARUCO_MIP_25h7,
		ARUCO_MIP_16h3,
		ARUCO_MIP_36h12,	//*** recommended
		ARTAG,				//
		ARTOOLKITPLUS,		// ARToolKit
		ARTOOLKITPLUSBCH,	// ARToolKit
		TAG16h5,TAG25h7,TAG25h9,TAG36h11,TAG36h10 // april tags
	};
}