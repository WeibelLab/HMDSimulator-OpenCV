#pragma once


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

	DLL_EXPORT bool Aruco_DrawMarker(int predefinedDict, int markerId, int markerSize, bool border, unsigned char* rgbOutput);
}