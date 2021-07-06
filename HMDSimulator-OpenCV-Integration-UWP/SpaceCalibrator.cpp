#include "pch.h"
#include "SpaceCalibrator.h"
#define AT(arr,x,y) (arr)[(x) + (y)*4]

std::map<int, std::shared_ptr<Calibrator>> CalibratorMap;

Pose::Pose(float* hmdMatrix) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rot(i, j) = AT(hmdMatrix, i, j);
    }
  }
  trans = Eigen::Vector3d(AT(hmdMatrix, 0, 3), AT(hmdMatrix, 1, 3), AT(hmdMatrix, 2, 3));
}

int CreateCalibrator() {
  try {
    std::shared_ptr<Calibrator> spaceCalibrator = std::make_shared<Calibrator>();

    // Get a handle
    int availableHandle = CalibratorMap.size();
    spaceCalibrator->handle = availableHandle;
    CalibratorMap.insert({ availableHandle, spaceCalibrator });

    return spaceCalibrator->handle;
  }
  catch (std::exception & e) {
    return -1;
  }
}


int AddSamples(int handle, float* refMatrix4x4, float* targetMatrix4x4) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];

    spaceCalibrator->samples.push_back(Sample(
      Pose(refMatrix4x4),
      Pose(targetMatrix4x4)
    ));

    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }
}

int ClearSamples(int handle) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];
    spaceCalibrator->samples.clear();
    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }
}

static std::string toString(const Eigen::MatrixXd& mat) {
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

void ApplyRotation(std::shared_ptr<Calibrator> cal) {
  // apply rotation to samples
  for(int i = 0; i < cal->samples.size(); i ++) {
    auto & curr = cal->samples[i];
    curr.target.trans = cal->rot * curr.target.trans;
    curr.target.rot = cal->rot * curr.target.rot;
  }
}

int PerformCalibration(int handle) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];

    spaceCalibrator->rot = CalibrateRotation(spaceCalibrator->samples);
    //OutputDebugStringA(toString(spaceCalibrator->rot).c_str());
    //OutputDebugStringA("\n");

    //OutputDebugStringA(toString(spaceCalibrator->samples[0].ref.rot).c_str());
    //ApplyRotation(spaceCalibrator);
    //OutputDebugStringA(toString(spaceCalibrator->samples[0].ref.rot).c_str());

    spaceCalibrator->trans = CalibrateTranslation(spaceCalibrator->rot, spaceCalibrator->samples);

    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }
}


int GetRotationQuat(int handle, float* rot) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];

    Eigen::Quaterniond q(spaceCalibrator->rot);

    rot[0] = q.x();
    rot[1] = q.y();
    rot[2] = q.z();
    rot[3] = q.w();

    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }

}

int GetTranslationVec(int handle, float* trans) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];

    trans[0] = spaceCalibrator->trans[0];
    trans[1] = spaceCalibrator->trans[1];
    trans[2] = spaceCalibrator->trans[2];

    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }

}

Eigen::Vector3d AxisFromRotationMatrix3(Eigen::Matrix3d rot) {
  return Eigen::Vector3d(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
}

double AngleFromRotationMatrix3(Eigen::Matrix3d rot) {
  return acos((rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0);
}

DSample DeltaRotationSamples(Sample s1, Sample s2) {
  // Difference in rotation between samples.
  auto dref = s1.ref.rot * s2.ref.rot.transpose();
  auto dtarget = s1.target.rot * s2.target.rot.transpose();

  // When stuck together, the two tracked objects rotate as a pair,
  // therefore their axes of rotation must be equal between any given pair of samples.
  DSample ds;
  ds.ref = AxisFromRotationMatrix3(dref);
  ds.target = AxisFromRotationMatrix3(dtarget);

  // Reject samples that were too close to each other.
  auto refA = AngleFromRotationMatrix3(dref);
  auto targetA = AngleFromRotationMatrix3(dtarget);
  ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

  ds.ref.normalize();
  ds.target.normalize();
  return ds;
}

Eigen::Matrix3d CalibrateRotation(const std::vector<Sample>& samples) {
  std::vector<DSample> deltas;

  for (size_t i = 0; i < samples.size(); i++) {
    for (size_t j = 0; j < i; j++) {
      auto delta = DeltaRotationSamples(samples[i], samples[j]);
      if (delta.valid)
        deltas.push_back(delta);
    }
  }
  //char buf[256];
  //snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
  //CalCtx.Log(buf);

  // Kabsch algorithm
  Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
  Eigen::Vector3d refCentroid(0, 0, 0), targetCentroid(0, 0, 0);

  for (size_t i = 0; i < deltas.size(); i++) {
    refPoints.row(i) = deltas[i].ref;
    refCentroid += deltas[i].ref;

    targetPoints.row(i) = deltas[i].target;
    targetCentroid += deltas[i].target;
  }

  refCentroid /= (double)deltas.size();
  targetCentroid /= (double)deltas.size();

  for (size_t i = 0; i < deltas.size(); i++) {
    refPoints.row(i) -= refCentroid;
    targetPoints.row(i) -= targetCentroid;
  }

  auto crossCV = refPoints.transpose() * targetPoints;

  Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
  auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
    i(2, 2) = -1;
  }

  Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
  rot.transposeInPlace();

  //Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

  //snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
  //CalCtx.Log(buf);
  //return euler;

  return rot;
}

Eigen::Vector3d CalibrateTranslation(const Eigen::Matrix3d rot, const std::vector<Sample>& samples) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

  for (size_t i = 0; i < samples.size(); i++) {
    for (size_t j = 0; j < i; j++) {
      auto QAi = (samples[i].ref.rot).transpose();
      auto QAj = (samples[j].ref.rot).transpose();
      auto dQA = QAj - QAi;
      auto CA = QAj * (samples[j].ref.trans - rot * samples[j].target.trans) - QAi * (samples[i].ref.trans - rot * samples[i].target
        .trans);
      deltas.push_back(std::make_pair(CA, dQA));

      auto QBi = (rot * samples[i].target.rot).transpose();
      auto QBj = (rot * samples[j].target.rot).transpose();
      auto dQB = QBj - QBi;
      auto CB = QBj * (samples[j].ref.trans - rot * samples[j].target.trans) - QBi * (samples[i].ref.trans - rot * samples[i].target
        .trans);
      deltas.push_back(std::make_pair(CB, dQB));
    }
  }

  Eigen::VectorXd constants(deltas.size() * 3);
  Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

  for (size_t i = 0; i < deltas.size(); i++) {
    for (int axis = 0; axis < 3; axis++) {
      constants(i * 3 + axis) = deltas[i].first(axis);
      coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
    }
  }

  Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
  //auto transcm = trans * 100.0;

  //char buf[256];
  //snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
  //CalCtx.Log(buf);
  return trans;
}

int GetSamples(int handle, int index, int isRef, float* trans, float* rot) {
  try {

    // try get Calibrator object
    std::shared_ptr<Calibrator> spaceCalibrator = CalibratorMap[handle];

    if(index >= spaceCalibrator->samples.size()) {
      return -1;
    }

    Sample curr = spaceCalibrator->samples[index];
    if(isRef) {
      trans[0] = curr.ref.trans.x();
      trans[1] = curr.ref.trans.y();
      trans[2] = curr.ref.trans.z();

    }else {
      trans[0] = curr.target.trans.x();
      trans[1] = curr.target.trans.y();
      trans[2] = curr.target.trans.z();
    }

    return spaceCalibrator->samples.size();
  }
  catch (std::exception & e) {
    // do nothing
    //DebugLogInUnity(e.what());
    return -1;
  }
}
