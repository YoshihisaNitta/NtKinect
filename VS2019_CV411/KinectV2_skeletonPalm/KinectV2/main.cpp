/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#include "NtKinect.h"

using namespace std;

void doJob() {
  NtKinect kinect;
  cv::Scalar colors[] = {
    cv::Scalar(255,0,0),  // HandState_Unknown
    cv::Scalar(0,255,0),  // HandState_NotTracked
    cv::Scalar(255,255,0), // HandState_Open
    cv::Scalar(255,0,255), // HandState_Closed
    cv::Scalar(0,255,255),  // HandState_Lass
  };
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    for (int i = 0; i < kinect.skeleton.size(); i++) {
      auto person = kinect.skeleton[i];
      for (int j = 0; j < person.size(); j++) {
        Joint joint = person[j];
        if (joint.TrackingState == TrackingState_NotTracked) continue;
        ColorSpacePoint cp;
        kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position,&cp);
        cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X-5, (int)cp.Y-5,10,10), cv::Scalar(0,0,255),2);
        if (j == JointType_HandLeft || j == JointType_HandRight) {
          pair<int, int> handState = kinect.handState(i, j == JointType_HandLeft);
          cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X - 8, (int)cp.Y - 8, 16, 16), colors[handState.first], 4);
        }
      }
    }
    cv::imshow("rgb", kinect.rgbImage);
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
