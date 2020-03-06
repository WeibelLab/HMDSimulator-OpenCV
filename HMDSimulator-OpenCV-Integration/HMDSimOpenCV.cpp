#include "pch.h"
#include "HMDSimOpenCV.h"


#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>


//std::vector<

extern "C"
{

	DLL_EXPORT bool Aruco_DrawMarker(int predefinedDict, int markerId, int markerSize, bool border, unsigned char* rgbOutput)
	{
		try
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
			if (rgbOutput != nullptr)
			{
				memcpy(rgbOutput, destMatrix.data, (markerSize * markerSize) * sizeof(unsigned char) * 3);
				return true;
			}
		}
		catch (std::exception e)
		{
			// do nothing
		}

		return false;

	}

	DLL_EXPORT bool Aruco_DrawCharucoBoard(
		int predefinedDict, int squareWidth, int squareHeight, float squareLength, float markerLength, bool border,
		unsigned char* rgbOutput) {
		try
		{

			cv::Mat boardMat;
			// gets a default dictionary
			auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

			// create the charuco board
			cv::Ptr<cv::aruco::CharucoBoard> chBoard = cv::aruco::CharucoBoard::create(squareWidth, squareHeight, squareLength, markerLength, dict);

			// draw the charuco board
			cv::Size sz(int(squareWidth * squareLength), int(squareHeight * squareLength));
			chBoard->draw(sz, boardMat, 0, border ? 1 : 0);

			// copies RGB so that Unity can find
			cv::Mat destMatrix;
			cv::cvtColor(boardMat, destMatrix, cv::COLOR_GRAY2RGB);

			// copies to unity (yes, I know, a lot of copies)
			if (rgbOutput != nullptr)
			{
				memcpy(rgbOutput, destMatrix.data, (sz.width * sz.height) * sizeof(unsigned char) * 3);
				return true;
			}
		}
		catch (std::exception e)
		{
			// do nothing
		}

		return false;
	}


}

DLL_EXPORT bool Aruco_CameraCalibration(
  unsigned char* rgbInput, int width, int height, int predefinedDict, int squareWidth, int squareHeight, float squareLength,
  float markerLength, bool border) {

	cv::Mat image(height, width, CV_8UC3, rgbInput);

	// gets a default dictionary
	auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

	// create the charuco board
	cv::Ptr<cv::aruco::CharucoBoard> chBoard = cv::aruco::CharucoBoard::create(squareWidth, squareHeight, squareLength, markerLength, dict);

	// convert to black and white
	cv::Mat bwInput;
	cv::cvtColor(image, bwInput, cv::COLOR_RGB2GRAY);

	// Detect corners
	std::vector<std::vector<cv::Point2f>> corners, rejecteds;
	std::vector<int> ids;
	cv::aruco::detectMarkers(bwInput, dict, corners, ids, cv::aruco::DetectorParameters::create(), rejecteds);

	if(corners.size() >= 3) {

		//
		std::vector<cv::Point2f> charucoCorners;
		std::vector<int> ids;
		cv::aruco::interpolateCornersCharuco(corners, ids, bwInput, chBoard, charucoCorners, ids);
	}

	// 
	cv::Size sz(int(squareWidth * squareLength), int(squareHeight * squareLength));


}
