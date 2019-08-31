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
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    for (auto person : kinect.skeleton) {
      for (auto joint : person) {
        if (joint.TrackingState == TrackingState_NotTracked) continue;
        ColorSpacePoint cp;
        kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position,&cp);
        cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X-5, (int)cp.Y-5,10,10), cv::Scalar(0,0,255),2);
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
