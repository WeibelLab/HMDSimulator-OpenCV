#include "pch.h"
#include "HMDSimOpenCV.h"

#include <opencv2/opencv.hpp>
#include "opencv2/aruco.hpp"



extern "C"
{
	DLL_EXPORT void Aruco_DrawMarker(int predefinedDict, int markerId, int markerSize, bool border, unsigned char* rgbOutput)
	{
		cv::Mat markerMtx;

		// gets a default dictionary
		auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

		// draws the marker
		cv::aruco::drawMarker(dict, markerId, markerSize, markerMtx, border ? 1 : 0);

		// copies RGB so that Unity can find
		cv::Mat destMatrix;
		cv::cvtColor(markerMtx, destMatrix, cv::COLOR_GRAY2RGB);

		// copies to unity (yes, I know, a lot of copies)
		memcpy(rgbOutput, destMatrix.data, (markerSize * markerSize) * sizeof(unsigned char) * 3);
	}
}
