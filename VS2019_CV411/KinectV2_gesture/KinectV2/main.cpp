/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#define USE_GESTURE
#include "NtKinect.h"

using namespace std;

void putText(cv::Mat& img,string s,cv::Point p) {
  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2.0, cv::LINE_AA);
}

void doJob() {
  NtKinect kinect;
  kinect.setGestureFile(L"SampleDatabase.gbd");
  //kinect.setGestureFile(L"Seated.gbd");
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    for (auto person: kinect.skeleton) {
      for (auto joint: person) {
	if (joint.TrackingState == TrackingState_NotTracked) continue;
        ColorSpacePoint cp;
        kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position,&cp);
        cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X-5, (int)cp.Y-5,10,10), cv::Scalar(0,0,255),2);
      }
    }
    kinect.setGesture();
    for (int i=0; i<kinect.discreteGesture.size(); i++) {
      auto g = kinect.discreteGesture[i];
      putText(kinect.rgbImage,kinect.gesture2string(g.first)+" "+to_string(g.second), cv::Point(50,30+30*i));
    }
    for (int i=0; i<kinect.continuousGesture.size(); i++) {
      auto g = kinect.continuousGesture[i];
      putText(kinect.rgbImage,kinect.gesture2string(g.first)+" "+to_string(g.second), cv::Point(500,30+30*i));
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
