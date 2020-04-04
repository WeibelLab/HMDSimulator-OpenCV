#pragma once
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>

typedef void(__stdcall* DebugCallback) (const char* str);

void DebugLogInUnity(std::string message);

extern "C"
{
	#define DLL_EXPORT __declspec(dllexport) 


	// C# side should make use of this
	//
	//enum PREDEFINED_DICTIONARY_NAME {
	//	DICT_4X4_50 = 0,
	//	DICT_4X4_100,
	//	DICT_4X4_250,
	//	DICT_4X4_1000,
	//	DICT_5X5_50,
	//	DICT_5X5_100,
	//	DICT_5X5_250,
	//	DICT_5X5_1000,
	//	DICT_6X6_50,
	//	DICT_6X6_100,
	//	DICT_6X6_250,
	//	DICT_6X6_1000,
	//	DICT_7X7_50,
	//	DICT_7X7_100,
	//	DICT_7X7_250,
	//	DICT_7X7_1000,
	//	DICT_ARUCO_ORIGINAL,
	//	DICT_APRILTAG_16h5,     ///< 4x4 bits, minimum hamming distance between any two codes = 5, 30 codes
	//	DICT_APRILTAG_25h9,     ///< 5x5 bits, minimum hamming distance between any two codes = 9, 35 codes
	//	DICT_APRILTAG_36h10,    ///< 6x6 bits, minimum hamming distance between any two codes = 10, 2320 codes
	//	DICT_APRILTAG_36h11     ///< 6x6 bits, minimum hamming distance between any two codes = 11, 587 codes
	//};

	DLL_EXPORT void RegisterDebugCallback(DebugCallback callback);

	DLL_EXPORT bool Aruco_DrawMarker(int predefinedDict, int markerId, int markerSize, bool border, unsigned char* rgbOutput);

	DLL_EXPORT int Aruco_EstimateMarkersPoseWithDetector(
		unsigned char* rgbInput, int width, int height,
		int predefinedDict, float markerLength,
		int detectorHandle,
		int expectedMarkerCount,
		float outputMarkerPosVec3[], float outputMarkerRotVec3[], int outputMarkerIds[], unsigned char * rgbOutput);

	DLL_EXPORT int Aruco_EstimateMarkersPose(
		unsigned char* rgbInput, int width, int height,
		int predefinedDict, float markerLength,
		float cameraMatrix[],
		float distCoeffs[], int distCoeffLength,
		int expectedMarkerCount,
		float outputMarkerPosVec3[], float outputMarkerRotVec3[], int outputMarkerIds[], unsigned char* rgbOutput);

	DLL_EXPORT int Aruco_CreateDetector(int predefinedDict, int squareWidth, int squareHeight, float squareLength, float markerLength, bool border);

	DLL_EXPORT bool Aruco_DrawCharucoBoard(int detectorHandle, unsigned char* rgbOutput);

	DLL_EXPORT int Aruco_CollectCharucoCorners(int detectorHandle, unsigned char* rgbInput, int width, int height, unsigned char* rgbOutput = nullptr);

	DLL_EXPORT double Aruco_CalibrateCameraCharuco(int detectorHandle);

	DLL_EXPORT int Aruco_GetCalibrateResult(int detectorHandle, float * cameraMatrix, float * distCoeffs);
}


class ArucoDetector {
public:
	int imageWidth = -1;
	int imageHeight = -1;
	int detectorHandle;
	int predefinedDict;
	int squareWidth;
	int squareHeight;
  float squareLength;
	float markerLength;
	bool border;

	bool calibrated = false;

	cv::Ptr<cv::aruco::Dictionary> dict;
	cv::Ptr<cv::aruco::CharucoBoard> chBoard;

	std::vector<std::vector<cv::Point2f>> charucoCorners;
	std::vector<std::vector<int>> charucoIds;

	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
};