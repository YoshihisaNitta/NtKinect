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

void putText(cv::Mat& img,string s,cv::Point p) {
  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 1, cv::LINE_AA);
}

string hexString(int n) { stringstream ss; ss << hex << n; return ss.str(); }

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
    putText(kinect.rgbImage, "TrackingId", cv::Point(0, 30));
    putText(kinect.rgbImage, "Collection", cv::Point(0, 60));
    putText(kinect.rgbImage, "Capture", cv::Point(0, 90));
    putText(kinect.rgbImage, "Collection", cv::Point(0, 120));
    putText(kinect.rgbImage, "Capture", cv::Point(0, 150));
    kinect.setHDFace();
    for (int i=0; i<kinect.hdfaceVertices.size(); i++) {
      for (CameraSpacePoint sp : kinect.hdfaceVertices[i]) {
	ColorSpacePoint cp;
	kinect.coordinateMapper->MapCameraPointToColorSpace(sp,&cp);
	cv::rectangle(kinect.rgbImage, cv::Rect((int)cp.X-1, (int)cp.Y-1, 2, 2), cv::Scalar(0,192, 0), 1);
      }
      int x = 200 * i + 150;
      auto status = kinect.hdfaceStatus[i];
      auto statusS = kinect.hdfaceStatusToString(status);
      putText(kinect.rgbImage, hexString(kinect.hdfaceTrackingId[i]), cv::Point(x, 30));
      putText(kinect.rgbImage, hexString(status.first), cv::Point(x, 60));
      putText(kinect.rgbImage, hexString(status.second), cv::Point(x, 90));
      putText(kinect.rgbImage, statusS.first, cv::Point(x, 120));
      putText(kinect.rgbImage, statusS.second, cv::Point(x, 150));
    }
    cv::imshow("hdface", kinect.rgbImage);
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
