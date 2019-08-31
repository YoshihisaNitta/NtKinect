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
    kinect.setAudio();
    cv::putText(kinect.rgbImage, "beamAngle: "+to_string(kinect.beamAngle),
                cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::putText(kinect.rgbImage, "beamAngleConfidence: "+to_string(kinect.beamAngleConfidence),
                cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::imshow("rgb", kinect.rgbImage);
    kinect.drawAudioDirection(beam);
    cv::imshow("beam",beam);
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
