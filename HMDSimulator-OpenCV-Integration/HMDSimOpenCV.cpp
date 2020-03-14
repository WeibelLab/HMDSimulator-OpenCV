#include "pch.h"
#include "HMDSimOpenCV.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

std::map<int, std::shared_ptr<ArucoDetector>> arucoDetectorMap;

DebugCallback gDebugCallback;

void DebugLogInUnity(std::string message) {
  if (gDebugCallback) {
    gDebugCallback(message.c_str());
  }
}

extern "C" {
DLL_EXPORT void RegisterDebugCallback(DebugCallback callback) {
  if (callback) {
    gDebugCallback = callback;
  }
}

DLL_EXPORT bool Aruco_DrawMarker(
  int predefinedDict, int markerId, int markerSize, bool border, unsigned char* rgbOutput) {
  try {
    cv::Mat markerMtx;

    // gets a default dictionary
    const auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

    // draws the marker
    cv::aruco::drawMarker(dict, markerId, markerSize, markerMtx, border ? 1 : 0);

    // copies RGB so that Unity can find
    cv::Mat destMatrix;
    cv::cvtColor(markerMtx, destMatrix, cv::COLOR_GRAY2RGB);

    // copies to unity (yes, I know, a lot of copies)
    if (rgbOutput != nullptr) {
      memcpy(rgbOutput, destMatrix.data, (markerSize * markerSize) * sizeof(unsigned char) * 3);
      return true;
    }
  }
  catch (std::exception& e) {
    // do nothing
  }

  return false;

}

DLL_EXPORT int Aruco_CreateDetector(
  int predefinedDict, int squareWidth, int squareHeight, float squareLength, float markerLength, bool border) {

  try {
    std::shared_ptr<ArucoDetector> arucoDetector = std::make_shared<ArucoDetector>();

    // Store parameters
    arucoDetector->predefinedDict = predefinedDict;
    arucoDetector->squareWidth = squareWidth;
    arucoDetector->squareHeight = squareHeight;
    arucoDetector->squareLength = squareLength;
    arucoDetector->markerLength = markerLength;
    arucoDetector->border = border;

    // gets a default dictionary
    arucoDetector->dict = cv::aruco::getPredefinedDictionary(predefinedDict);

    // create the charuco board
    arucoDetector->chBoard = cv::aruco::CharucoBoard::create(
      squareWidth, squareHeight, squareLength, markerLength, arucoDetector->dict);

    // Get a handle
    int availableHandle = arucoDetectorMap.size();
    arucoDetector->detectorHandle = availableHandle;
    arucoDetectorMap.insert({availableHandle, arucoDetector});

    return arucoDetector->detectorHandle;

  }
  catch (std::exception& e) {
    return -1;
  }
}

DLL_EXPORT bool Aruco_DrawCharucoBoard(int detectorHandle, unsigned char* rgbOutput) {
  try {

    cv::Mat boardMat;

    // try get arucoDetector object
    std::shared_ptr<ArucoDetector> arucoDetector = arucoDetectorMap[detectorHandle];

    // draw the charuco board
    cv::Size sz(
      int(arucoDetector->squareWidth * arucoDetector->squareLength), int(
        arucoDetector->squareHeight * arucoDetector->squareLength));
    arucoDetector->chBoard->draw(sz, boardMat, 0, arucoDetector->border ? 1 : 0);

    // copies RGB so that Unity can find
    cv::Mat destMatrix;
    cv::cvtColor(boardMat, destMatrix, cv::COLOR_GRAY2RGB);

    //DebugLogInUnity("successful");

    // copies to unity (yes, I know, a lot of copies)
    if (rgbOutput != nullptr) {
      memcpy(rgbOutput, destMatrix.data, (sz.width * sz.height) * sizeof(unsigned char) * 3);
      return true;
    }
  }
  catch (std::exception& e) {
    // do nothing
    DebugLogInUnity(e.what());
  }

  return false;
}

DLL_EXPORT int Aruco_CollectCharucoCorners(
  int detectorHandle,
  unsigned char* rgbInput, int width, int height, unsigned char* rgbOutput) {

  try {

    // try get arucoDetector object
    std::shared_ptr<ArucoDetector> arucoDetector = arucoDetectorMap[detectorHandle];

    if (arucoDetector->imageWidth < 0) {
      arucoDetector->imageWidth = width;
      arucoDetector->imageHeight = height;
    }
    else {
      if (arucoDetector->imageWidth != width || arucoDetector->imageHeight != height) {
        return -1;
      }
    }

    cv::Mat image(height, width, CV_8UC3, rgbInput);

    // gets a default dictionary
    auto dict = arucoDetector->dict;

    // create the charuco board
    cv::Ptr<cv::aruco::CharucoBoard> chBoard = arucoDetector->chBoard;

    // convert to black and white
    cv::Mat bwInput;
    cv::cvtColor(image, bwInput, cv::COLOR_RGB2GRAY);

    // Detect corners
    std::vector<std::vector<cv::Point2f>> corners, rejecteds;
    std::vector<int> ids;
    cv::aruco::detectMarkers(bwInput, dict, corners, ids, cv::aruco::DetectorParameters::create(), rejecteds);

    

    if (corners.size() >= 3) {

      // Transform corners
      std::vector<cv::Point2f> charucoCorners;
      std::vector<int> charucoIds;
      int numOfCorners = cv::aruco::interpolateCornersCharuco(
        corners, ids, bwInput, chBoard, charucoCorners, charucoIds);

      if (numOfCorners > 0) {

        // Collect corners
        arucoDetector->charucoCorners.push_back(charucoCorners);
        arucoDetector->charucoIds.push_back(charucoIds);

        if (rgbOutput) {
          cv::Mat output(image);
          cv::aruco::drawDetectedCornersCharuco(output, charucoCorners, charucoIds);

          memcpy(rgbOutput, output.data, (width * height) * sizeof(unsigned char) * 3);
        }

        return numOfCorners;
      }
    }

    return 0;
  }
  catch (std::exception e) {
    DebugLogInUnity(e.what());
    return -1;
    // do nothing
  }
}

DLL_EXPORT double Aruco_CalibrateCameraCharuco(int detectorHandle) {

  try {
    // try get arucoDetector object
    std::shared_ptr<ArucoDetector> arucoDetector = arucoDetectorMap[detectorHandle];

    cv::Size size(arucoDetector->imageWidth, arucoDetector->imageHeight);

    cv::Mat cameraMatrix, distCoeff;
    const double error = cv::aruco::calibrateCameraCharuco(
      arucoDetector->charucoCorners, arucoDetector->charucoIds, arucoDetector->chBoard, size, cameraMatrix, distCoeff);

    arucoDetector->cameraMatrix = cameraMatrix;
    arucoDetector->distCoeff = distCoeff;

    {
      std::stringstream buffer;
      buffer << cameraMatrix;
      DebugLogInUnity(buffer.str());
    }
    {
      std::stringstream buffer;
      buffer << distCoeff;
      DebugLogInUnity(buffer.str());
    }

    return error;
  }
  catch (std::exception& e) {
    DebugLogInUnity(e.what());
    return -1.0;
  }
}

DLL_EXPORT int Aruco_EstimateMarkersPose(
  unsigned char* rgbInput, int width, int height,
  int predefinedDict, float markerLength,
  float cameraMatrix[],
  float distCoeffs[], int distCoeffLength,
  int expectedMarkerCount,
  float outputMarkerPosVec3[], float outputMarkerRotVec3[], int outputMarkerIds[]) {
  if (rgbInput == nullptr || cameraMatrix == nullptr ||
    distCoeffs == nullptr ||
    outputMarkerPosVec3 == nullptr || outputMarkerRotVec3 == nullptr || outputMarkerIds == nullptr)
    return -1;

  try {
    cv::Mat image(height, width, CV_8UC3, rgbInput);

    // gets a default dictionary
    auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

    // convert to black and white
    cv::Mat bwInput;
    cv::cvtColor(image, bwInput, cv::COLOR_RGB2GRAY);

    // Detect corners
    std::vector<std::vector<cv::Point2f>> corners, rejecteds;
    std::vector<int> ids;
    cv::aruco::detectMarkers(bwInput, dict, corners, ids, cv::aruco::DetectorParameters::create(), rejecteds);

    // Did we find anything?
    if (ids.size() >= 0) {
      // if we did, then we now find their transformation to 3D
      cv::Mat cameraIntrinsics(3, 3, CV_32F, cameraMatrix);
      cv::Mat cameraDistortion(1, distCoeffLength, CV_32F, distCoeffs);

      // expected output
      std::vector<cv::Vec3d> rvecs, tvecs;

      // detect poses given camera parameters
      cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraIntrinsics, cameraDistortion, rvecs, tvecs);

      // now serialize it back for unity to use it

      int positionsToCopyBack = std::min(expectedMarkerCount, (int)ids.size());
      for (unsigned int i = 0; i < positionsToCopyBack; ++i) {
        outputMarkerIds[i] = ids[i];
        const int i3 = i * 3;

        // copy rotation
        outputMarkerRotVec3[i3] = (float)rvecs[i][0];
        outputMarkerRotVec3[i3 + 1] = (float)rvecs[i][1];
        outputMarkerRotVec3[i3 + 2] = (float)rvecs[i][2];

        // copy translation
        outputMarkerPosVec3[i3] = (float)tvecs[i][0];
        outputMarkerPosVec3[i3 + 1] = (float)tvecs[i][1];
        outputMarkerPosVec3[i3 + 2] = (float)tvecs[i][2];
      }

      return positionsToCopyBack;
    }
    else {
      return 0; // we did not find anything
    }

  }
  catch (std::exception e) {
    DebugLogInUnity(e.what());
    return -1; // if something goes wrong, we just return -1 to let the user know
  }
}
}
