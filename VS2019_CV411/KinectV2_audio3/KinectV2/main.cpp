/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#define USE_AUDIO
#include "NtKinect.h"

using namespace std;

void doJob() {
  NtKinect kinect;
  cv::Mat beam;
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    kinect.setAudio(true);
    for (int i=0; i<kinect.skeleton.size(); i++) {
      int w = 10;
      cv::Scalar color = cv::Scalar(0,0,255);
      if (kinect.audioTrackingId == kinect.skeletonTrackingId[i]) {
        w = 20;
        color = cv::Scalar(0,255,0);
      }
      for (auto joint: kinect.skeleton[i]) {
        if (joint.TrackingState == TrackingState_NotTracked) continue;
        ColorSpacePoint cp;
        kinect.coordinateMapper->MapCameraPointToColorSpace(joint.Position,&cp);
        cv::rectangle(kinect.rgbImage,cv::Rect((int)cp.X-w/2,(int)cp.Y-w/2,w,w),color, 2);
      }
    }
    cv::imshow("rgb", kinect.rgbImage);
    kinect.drawAudioDirection(beam);
    cv::imshow("beam", beam);
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
