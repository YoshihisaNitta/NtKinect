/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#define USE_FACE
#include "NtKinect.h"

using namespace std;

string faceProp[] = {
  "happy", "engaed", "glass", "leftEyeClosed",
  "rightEyeClosed", "mouseOpen", "mouseMoved", "lookingAway"
};
string dstate[] = { "unknown", "no", "maybe", "yes" };

void doJob() {
  NtKinect kinect;
  cv::Mat img8;
  cv::Mat img;
  while (1) {
    kinect.setDepth();
    kinect.depthImage.convertTo(img8,CV_8UC1,255.0/4500);
    cv::cvtColor(img8,img,cv::COLOR_GRAY2BGR);
    kinect.setSkeleton();
    for (auto person : kinect.skeleton) {
      for (auto joint : person) {
        if (joint.TrackingState == TrackingState_NotTracked) continue;
        DepthSpacePoint dp;
        kinect.coordinateMapper->MapCameraPointToDepthSpace(joint.Position,&dp);
        cv::rectangle(img, cv::Rect((int)dp.X-5, (int)dp.Y-5,10,10), cv::Scalar(0,0,255),2);
      }
    }
    kinect.setFace(false);
    for (cv::Rect r : kinect.faceRect) {
      cv::rectangle(img, r, cv::Scalar(255, 255, 0), 2);
    }

    for (vector<PointF> vf : kinect.facePoint) {
      for (PointF p : vf) {
        cv::rectangle(img, cv::Rect((int)p.X-3, (int)p.Y-3, 6, 6), cv::Scalar(0, 255, 255), 2);
      }
    }
    for (int p = 0; p < kinect.faceDirection.size(); p++) {
      cv::Vec3f dir = kinect.faceDirection[p];
      cv::putText(img, "pitch : " + to_string(dir[0]), cv::Point(200 * p + 50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
      cv::putText(img, "yaw : " + to_string(dir[1]), cv::Point(200 * p + 50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
      cv::putText(img, "roll : " + to_string(dir[2]), cv::Point(200 * p + 50, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);
    }
    for (int p = 0; p < kinect.faceProperty.size(); p++) {
      for (int k = 0; k < FaceProperty_Count; k++) {
        int v = kinect.faceProperty[p][k];
        cv::putText(img, faceProp[k] +" : "+ dstate[v], cv::Point(200 * p + 50, 30 * k + 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1, cv::LINE_AA);

      }
    }
    cv::imshow("depth", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }
  cv::destroyAllWindows();
}

int main(int argc, char** argv) {
  try {
    doJob();
  } catch (exception &ex) {
    cout << ex.what() << endl;
    string s;
    cin >> s;
  }
  return 0;
}
