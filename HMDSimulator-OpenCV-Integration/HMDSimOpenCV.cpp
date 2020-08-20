#include "pch.h"
#include "HMDSimOpenCV.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// ARUCO is included locally
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"

std::map<int, std::shared_ptr<ArucoDetector>> arucoDetectorMap;

DebugCallback gDebugCallback;

void DebugLogInUnity(std::string message) {
  if (gDebugCallback) {
    gDebugCallback(message.c_str());
  }
}

template<typename _Ty>
std::vector<_Ty> mat2vector(const cv::Mat & mat) {
  return std::vector<_Ty>(mat.reshape(1, 1));
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
    arucoDetector->calibrated = true;
    {
      std::stringstream buffer;
      buffer << arucoDetector->cameraMatrix;
      DebugLogInUnity(buffer.str());
    }
    {
      std::stringstream buffer;
      buffer << arucoDetector->distCoeff;
      DebugLogInUnity(buffer.str());
    }

    return error;
  }
  catch (std::exception& e) {
    DebugLogInUnity(e.what());
    return -1.0;
  }
}

DLL_EXPORT int Aruco_GetCalibrateResult(int detectorHandle, float* cameraMatrix, float* distCoeffs) {
  try {
    // try get arucoDetector object
    std::shared_ptr<ArucoDetector> arucoDetector = arucoDetectorMap[detectorHandle];

    if (!arucoDetector->calibrated) {
      return -1;
    }

    if (cameraMatrix != nullptr) {
      memcpy(cameraMatrix, arucoDetector->cameraMatrix.data, 9 * sizeof(float));
    }

    int size = arucoDetector->distCoeff.total();
    if (distCoeffs != nullptr) {
      memcpy(distCoeffs, arucoDetector->distCoeff.data, size * sizeof(float));
    }

    return size;
  }
  catch (std::exception& e) {
    DebugLogInUnity(e.what());
    return -2;
  }
}

DLL_EXPORT int Aruco_EstimateMarkersPoseWithDetector(
  unsigned char* rgbInput, int width, int height, int predefinedDict, float markerLength, int detectorHandle,
  int expectedMarkerCount, float outputMarkerPosVec3[], float outputMarkerRotVec3[], int outputMarkerIds[], unsigned char* rgbOutput) {

  try {
    // try get arucoDetector object
    std::shared_ptr<ArucoDetector> arucoDetector = arucoDetectorMap[detectorHandle];

    if (!arucoDetector->calibrated) {
      return -1;
    }

    int distCoeffLength = arucoDetector->distCoeff.total();

    /*{
      std::stringstream buffer;
      buffer << arucoDetector->cameraMatrix;
      DebugLogInUnity(buffer.str());
    }

    {
      std::stringstream buffer;
      buffer << arucoDetector->distCoeff;
      DebugLogInUnity(buffer.str());
    }*/

    std::vector<float>cameraMatrix = mat2vector<float>(arucoDetector->cameraMatrix);
    std::vector<float>distCoeff = mat2vector<float>(arucoDetector->distCoeff);
    int ret = Aruco_EstimateMarkersPose(
      rgbInput, width, height, predefinedDict, markerLength, cameraMatrix.data(),
      distCoeff.data(), distCoeffLength, expectedMarkerCount, outputMarkerPosVec3,
      outputMarkerRotVec3, outputMarkerIds, rgbOutput);
    return ret;
  }
  catch (std::exception& e) {
    DebugLogInUnity(e.what());
    return -2;
  }
}

DLL_EXPORT int Aruco_EstimateMarkersPose(
  unsigned char* rgbInput, int width, int height,
  int predefinedDict, float markerLength,
  float cameraMatrix[],
  float distCoeffs[], int distCoeffLength,
  int expectedMarkerCount,
  float outputMarkerPosVec3[], float outputMarkerRotVec3[], int outputMarkerIds[], unsigned char * rgbOutput) {
  if (rgbInput == nullptr || cameraMatrix == nullptr ||
    distCoeffs == nullptr ||
    outputMarkerPosVec3 == nullptr || outputMarkerRotVec3 == nullptr || outputMarkerIds == nullptr)
    return -1;

  try {
    cv::Mat image(height, width, CV_8UC3, rgbInput);

    // DebugLogInUnity("parameter:" + std::to_string(predefinedDict) + "," + std::to_string(markerLength));

    // gets a default dictionary
    auto dict = cv::aruco::getPredefinedDictionary(predefinedDict);

    // convert to black and white
    cv::Mat bwInput;
    cv::cvtColor(image, bwInput, cv::COLOR_RGB2GRAY);

    // Detect corners
    std::vector<std::vector<cv::Point2f>> corners, rejecteds;
    std::vector<int> ids;
    auto param = cv::aruco::DetectorParameters::create();
    param->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(bwInput, dict, corners, ids, param, rejecteds);

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
        const int i9 = i * 9;

        // https://github.com/opencv/opencv/issues/8813#issuecomment-390462446
        cv::Mat rot;
        cv::Rodrigues(rvecs[i], rot);

        // Y-up
        static double switchAxisData[9] = {
          1,0,0,
          0,0,-1,
          0,1,0 };
        static cv::Mat switchAxis = cv::Mat(3, 3, CV_64F, switchAxisData);
        //rot *= switchAxis;

        //if (rot.at<double>(1,1) < 1 && rot.at<double>(1, 1) > 0) {
        //  // flip pose

        //  // flip axes
        //  static double flipAxisData[9] = {
        //    1,-1,1,
        //    1,-1,1,
        //    -1,1,-1};
        //  static cv::Mat flipAxis = cv::Mat(3, 3, CV_64F, flipAxisData);
        //  rot *= flipAxis;

        //  // Fixup
        //  cv::Vec3d T = tvecs[i];
        //  cv::Vec3d forward(0,0,1);
        //  cv::Vec3d tnorm = cv::normalize(forward);

        //  cv::Vec3d axis = tnorm.cross(forward);

        //  double angle = -2.0 * std::acos(tnorm.dot(forward));
        //  cv::Mat fixup;
        //  cv::Rodrigues(angle * axis, fixup);
        //  rot = fixup * rot;
        //}

        cv::Vec3d fixedRot;
        cv::Rodrigues(rot, fixedRot);

        // copy rotation
        /*outputMarkerRotVec3[i3] = (float)fixedRot[0];
        outputMarkerRotVec3[i3 + 1] = (float)fixedRot[1];
        outputMarkerRotVec3[i3 + 2] = (float)fixedRot[2];*/

        outputMarkerRotVec3[i9] = (float)rot.at<double>(0, 0);
        outputMarkerRotVec3[i9 + 1] = (float)rot.at<double>(0, 1);
        outputMarkerRotVec3[i9 + 2] = (float)rot.at<double>(0, 2);

        outputMarkerRotVec3[i9 + 3] = (float)rot.at<double>(1, 0);
        outputMarkerRotVec3[i9 + 4] = (float)rot.at<double>(1, 1);
        outputMarkerRotVec3[i9 + 5] = (float)rot.at<double>(1, 2);

        outputMarkerRotVec3[i9 + 6] = (float)rot.at<double>(2, 0);
        outputMarkerRotVec3[i9 + 7] = (float)rot.at<double>(2, 1);
        outputMarkerRotVec3[i9 + 8] = (float)rot.at<double>(2, 2);

        // copy translation
        outputMarkerPosVec3[i3] = (float)tvecs[i][0];
        outputMarkerPosVec3[i3 + 1] = (float)tvecs[i][1];
        outputMarkerPosVec3[i3 + 2] = (float)tvecs[i][2];


      }

      /*if (rgbOutput) {
        cv::Mat output(image);
        cv::drawFrameAxes(output, cameraIntrinsics, cameraDistortion, rvecs, tvecs, markerLength * 0.5f);

        memcpy(rgbOutput, output.data, (width * height) * sizeof(unsigned char) * 3);
      }*/

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

DLL_EXPORT float SPAAM_Solve(float* alignments, int alignmentCount, float* resultMatrix, bool affine, bool is3Dto2D, bool getError) {

  try {

    cv::Mat1f A;
    cv::Mat1f B;
    if (!is3Dto2D) {
      if (affine) {
        SPAAM_CreateAffineEquation(alignments, alignmentCount, A, B);
      }
      else {
        SPAAM_CreatePerspectiveEquation(alignments, alignmentCount, A, B);
      }
    }else {
      SPAAM_Create3x4Equation(alignments, alignmentCount, A, B);
    }

    // Solve Ax=B

    cv::Mat w, u, vt;
    cv::Mat result;
    if (affine && !is3Dto2D) {
      cv::solve(A, B, result, cv::DECOMP_SVD);
    }else {
      cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
      /*{
        std::stringstream buffer;
        buffer << w;
        DebugLogInUnity(buffer.str());
      }
      {
        std::stringstream buffer;
        buffer << vt;
        DebugLogInUnity(buffer.str());
      }*/
      result = vt.row(vt.rows - 1);
      result = result.t();
    }

    float error = -1;
    if(getError) {
      error = cv::norm(A * result - B);
    }

    std::vector<float>resultVector = mat2vector<float>(result);
    if(resultMatrix!=nullptr) {
      memcpy(resultMatrix, resultVector.data(), resultVector.size() * sizeof(float));

      if(is3Dto2D) {
        resultMatrix[12] = w.at<float>(0);
        resultMatrix[13] = w.at<float>(10);
        cv::Mat mat = result.reshape(1, 3);
        {
          std::stringstream buffer;
          buffer << mat;
          DebugLogInUnity(buffer.str());
        }
        cv::Mat cam, rot, trans;
        cv::decomposeProjectionMatrix(mat, cam, rot, trans);
        {
          std::stringstream buffer;
          buffer << cam;
          DebugLogInUnity(buffer.str());
        }
        {
          std::stringstream buffer;
          buffer << rot;
          DebugLogInUnity(buffer.str());
        }
        resultMatrix[14] = trans.at<float>(0);
        resultMatrix[15] = trans.at<float>(1);
        resultMatrix[16] = trans.at<float>(2);
        resultMatrix[17] = trans.at<float>(3);
      }
    }
    return error;
  }
  catch (std::exception & e) {
    DebugLogInUnity(e.what());
    return -2;
  }
}
}

bool SPAAM_CreateAffineEquation(float* alignments, int alignmentCount, cv::Mat1f& A, cv::Mat1f& B) {
  A = cv::Mat1f::zeros(3 * alignmentCount, 12);
  B = cv::Mat1f::zeros(3 * alignmentCount, 1);
  for(int i = 0; i < alignmentCount; i ++) {
    int pairStep = 6 * i;
    int step = 3 * i;

    // Equation 1
    A[step][0] = alignments[pairStep];
    A[step][1] = alignments[pairStep + 1];
    A[step][2] = alignments[pairStep + 2];
    A[step][3] = 1;
    B[step][0] = alignments[pairStep + 3];

    // Equation 2
    A[step + 1][4] = alignments[pairStep];
    A[step + 1][5] = alignments[pairStep + 1];
    A[step + 1][6] = alignments[pairStep + 2];
    A[step + 1][7] = 1;
    B[step + 1][0] = alignments[pairStep + 4];

    // Equation 3
    A[step + 2][8] = alignments[pairStep];
    A[step + 2][9] = alignments[pairStep + 1];
    A[step + 2][10] = alignments[pairStep + 2];
    A[step + 2][11] = 1;
    B[step + 2][0] = alignments[pairStep + 5];
  }

  return true;
}

bool SPAAM_CreatePerspectiveEquation(float* alignments, int alignmentCount, cv::Mat1f & A, cv::Mat1f & B) {
  A = cv::Mat1f::zeros(3 * alignmentCount, 16);
  B = cv::Mat1f::zeros(3 * alignmentCount, 1);
  for (int i = 0; i < alignmentCount; i++) {
    int pairStep = 6 * i;
    int step = 3 * i;

    // Equation 1
    A[step][0] = alignments[pairStep];
    A[step][1] = alignments[pairStep + 1];
    A[step][2] = alignments[pairStep + 2];
    A[step][3] = 1;
    A[step][12] = -alignments[pairStep + 3] * alignments[pairStep];
    A[step][13] = -alignments[pairStep + 3] * alignments[pairStep + 1];
    A[step][14] = -alignments[pairStep + 3] * alignments[pairStep + 2];
    A[step][15] = -alignments[pairStep + 3];

    // Equation 2
    A[step + 1][4] = alignments[pairStep];
    A[step + 1][5] = alignments[pairStep + 1];
    A[step + 1][6] = alignments[pairStep + 2];
    A[step + 1][7] = 1;
    A[step + 1][12] = -alignments[pairStep + 4] * alignments[pairStep];
    A[step + 1][13] = -alignments[pairStep + 4] * alignments[pairStep + 1];
    A[step + 1][14] = -alignments[pairStep + 4] * alignments[pairStep + 2];
    A[step + 1][15] = -alignments[pairStep + 4];

    // Equation 3
    A[step + 2][8] = alignments[pairStep];
    A[step + 2][9] = alignments[pairStep + 1];
    A[step + 2][10] = alignments[pairStep + 2];
    A[step + 2][11] = 1;
    A[step + 2][12] = -alignments[pairStep + 5] * alignments[pairStep];
    A[step + 2][13] = -alignments[pairStep + 5] * alignments[pairStep + 1];
    A[step + 2][14] = -alignments[pairStep + 5] * alignments[pairStep + 2];
    A[step + 2][15] = -alignments[pairStep + 5];
  }

  return true;
}

bool SPAAM_Create3x4Equation(float* alignments, int alignmentCount, cv::Mat1f& A, cv::Mat1f& B) {
  A = cv::Mat1f::zeros(2 * alignmentCount, 12);
  B = cv::Mat1f::zeros(2 * alignmentCount, 1);
  for (int i = 0; i < alignmentCount; i++) {
    int pairStep = 5 * i;
    int step = 2 * i;

    // Equation 1
    A[step][0] = alignments[pairStep];
    A[step][1] = alignments[pairStep + 1];
    A[step][2] = alignments[pairStep + 2];
    A[step][3] = 1;
    A[step][8] = -alignments[pairStep + 3] * alignments[pairStep];
    A[step][9] = -alignments[pairStep + 3] * alignments[pairStep + 1];
    A[step][10] = -alignments[pairStep + 3] * alignments[pairStep + 2];
    A[step][11] = -alignments[pairStep + 3];

    // Equation 2
    A[step + 1][4] = alignments[pairStep];
    A[step + 1][5] = alignments[pairStep + 1];
    A[step + 1][6] = alignments[pairStep + 2];
    A[step + 1][7] = 1;
    A[step + 1][8] = -alignments[pairStep + 4] * alignments[pairStep];
    A[step + 1][9] = -alignments[pairStep + 4] * alignments[pairStep + 1];
    A[step + 1][10] = -alignments[pairStep + 4] * alignments[pairStep + 2];
    A[step + 1][11] = -alignments[pairStep + 4];
  }

  return true;
}
