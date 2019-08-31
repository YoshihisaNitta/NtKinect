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

#include <time.h>
string now() {
  char s[1024];
  time_t t = time(NULL);
  struct tm lnow;
  localtime_s(&lnow, &t);
  sprintf_s(s, "%04d-%02d-%02d_%02d-%02d-%02d", lnow.tm_year + 1900, lnow.tm_mon + 1, lnow.tm_mday, 
	    lnow.tm_hour, lnow.tm_min, lnow.tm_sec);
  return string(s);
}

void doJob() {
  NtKinect kinect;
  bool flag = false;
  while (1) {
    kinect.setRGB();
    if (flag) kinect.setAudio();
    cv::putText(kinect.rgbImage, flag ? "Recording" : "Stopped", cv::Point(50, 50),
		cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::imshow("rgb", kinect.rgbImage);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    else if (key == 'r') flag = true;
    else if (key == 's') flag = false;

    if (flag && !kinect.isOpenedAudio()) kinect.openAudio(now() + ".wav");
    else if (!flag && kinect.isOpenedAudio()) kinect.closeAudio();
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
