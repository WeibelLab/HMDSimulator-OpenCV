#ifndef SPACECALIBRATOR_H
#define SPACECALIBRATOR_H

// eigen3
#include <eigen3/Eigen/Eigen>

struct Pose
{
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;

  Pose() { }

  Pose(float* hmdMatrix);

  Pose(double x, double y, double z) : trans(Eigen::Vector3d(x, y, z)) { }
};

struct Sample
{
  Pose ref, target;
  bool valid;
  Sample() : valid(false) { }
  Sample(Pose ref, Pose target) : ref(ref), target(target), valid(true) { }
};

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

class Calibrator {

public:
  int handle;
  std::vector<Sample> samples;

  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;
};

extern "C"
{
#define DLL_EXPORT __declspec(dllexport) 
  DLL_EXPORT int CreateCalibrator();
  DLL_EXPORT int AddSamples(int handle, float* refMatrix4x4, float* targetMatrix4x4);
  DLL_EXPORT int GetSamples(int handle, int index, int isRef, float* trans, float* rot);
  DLL_EXPORT int ClearSamples(int handle);

  DLL_EXPORT int PerformCalibration(int handle);

  DLL_EXPORT int GetRotationQuat(int handle, float* rot);
  DLL_EXPORT int GetTranslationVec(int handle, float* trans);
}

Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot);

double AngleFromRotationMatrix3(Eigen::Matrix3d rot);

DSample DeltaRotationSamples(Sample s1, Sample s2);

Eigen::Matrix3d CalibrateRotation(const std::vector<Sample>& samples);

Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d rot, const std::vector<Sample>& samples);

#endif
