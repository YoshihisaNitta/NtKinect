#pragma once
#include "NtKinect.h"

class KinectTetris : public NtKinect {
#define LEN_HOLI        160
#define LEN_VERT        120
#define LEN_NEAR        200
#define LEN_MATCH       120

 private:
  cv::Vec3f hipL, hipR, shoulderL, shoulderR, handL, handR, elbowL, elbowR, spineM;
  double baseLen;
  bool debug;
  cv::Vec3f p2v(CameraSpacePoint& sp) {
    ColorSpacePoint cp;
    coordinateMapper->MapCameraPointToColorSpace(sp,&cp);
    cv::Vec3f v(cp.X, cp.Y, 0.0f);
    return v;
  }

 public:
 KinectTetris() : debug() {}
  ~KinectTetris() {}
  void setDebug() { debug = true; }
  bool tracked() {
    if (skeleton.size() == 0) return false;

    int js[] = {
      JointType_HipLeft,
      JointType_HipRight,
      JointType_ShoulderLeft,
      JointType_ShoulderRight,
      JointType_ElbowLeft,
      JointType_ElbowRight,
      JointType_HandLeft,
      JointType_HandRight,
      JointType_SpineMid,
    };
    int njs = sizeof(js)/sizeof(int);
    for (int i=0; i<njs; i++) {
      if (skeleton[0][js[i]].TrackingState == TrackingState_NotTracked) return false;
    }

    hipL = p2v(skeleton[0][JointType_HipLeft].Position);
    hipR = p2v(skeleton[0][JointType_HipRight].Position);
    shoulderL = p2v(skeleton[0][JointType_ShoulderLeft].Position);
    shoulderR = p2v(skeleton[0][JointType_ShoulderRight].Position);
    elbowL = p2v(skeleton[0][JointType_ElbowLeft].Position);
    elbowR = p2v(skeleton[0][JointType_ElbowRight].Position);
    handL = p2v(skeleton[0][JointType_HandLeft].Position);
    handR = p2v(skeleton[0][JointType_HandRight].Position);
    spineM = p2v(skeleton[0][JointType_SpineMid].Position);

    float lenL = cv::norm(handL - elbowL) + cv::norm(elbowL - shoulderL);
    float lenR = cv::norm(handR - elbowR) + cv::norm(elbowR - shoulderR);
    baseLen = max(lenL, lenR);

    return true;
  }
  bool nearShoulderL(cv::Vec3f p) {
    double len = cv::norm(shoulderL - p);
    if (debug) cout << "nearShoulderL: " << len << endl;
    return (len < baseLen*0.2);
  }
  bool nearShoulderR(cv::Vec3f p) {
    double len = cv::norm(shoulderR - p);
    if (debug) cout << "nearShoulderR: " << len << endl;
    return (len < baseLen*0.2);
  }
  bool nearHipL(cv::Vec3f p) {
    double len = cv::norm(hipL - p);
    if (debug) cout << "nearHipL: " << len << endl;
    return (len < baseLen*0.2);
  }
  bool nearHipR(cv::Vec3f p) {
    double len = cv::norm(hipR - p);
    if (debug) cout << "nearHipR: " << len << endl;
    return (len < baseLen*0.2);
  }
  bool overCenter(cv::Vec3f p) {
    return p[1] < spineM[1];
  }
  bool overShoulder(cv::Vec3f p) {
    return p[1] <  min(shoulderL[1], shoulderR[1]);
  }
  bool leftOfShoulderL(cv::Vec3f p) {
    cv::Vec3f q = p - shoulderL;
    if (debug) cout << "leftOfShoulderL: " << q << endl;
    return q[0] < - 0.5 * baseLen;
  }

  bool rightOfShoulderR(cv::Vec3f p) {
    cv::Vec3f q = p - shoulderR;
    if (debug) cout << "rightOfShoulderR: " << q << endl;
    return q[0] > 0.5 * baseLen;
  }
  bool wristMatch() {
    double len = cv::norm(handL, handR);
    return len < baseLen * 0.1;
  }
  void check() {
    cerr << baseLen << "  " << overCenter(handL) << " " << overCenter(handR) << endl;
    return;
    cerr << baseLen << " nearSL " << nearShoulderL(handL) << " nearSR " << nearShoulderR(handR)
	 << " OCL " << overCenter(handL) << " OCR " << overCenter(handR)
	 << " LSL " << leftOfShoulderL(handL) << " RSR " << rightOfShoulderR(handR)
	 << endl;
  }
  bool moveR() {
    return rightOfShoulderR(handR) && overCenter(handR);
  }
  bool moveL() {
    return leftOfShoulderL(handL) && overCenter(handL);
  }
  bool rotR() {
    return nearShoulderR(handR);
  }
  bool rotL() {
    return nearShoulderL(handL);
  }
  bool drop() {
    return wristMatch();
  }
};
