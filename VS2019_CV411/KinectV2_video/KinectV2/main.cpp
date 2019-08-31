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

#include <time.h>
string now() {
  char s[1024];
  time_t t = time(NULL);
  struct tm lnow;
  localtime_s(&lnow, &t);
  sprintf_s(s, "%04d-%02d-%02d_%02d-%02d-%02d", lnow.tm_year + 1900, lnow.tm_mon + 1, lnow.tm_mday, lnow.tm_hour, lnow.tm_min, lnow.tm_sec);
  return string(s);
}

void doJob() {
  NtKinect kinect;
  cv::VideoWriter vw;
  int scale = 1;
  cv::Size sz(1920/scale,1080/scale);
  bool onSave = false;
  cv::Mat img;
  while (1) {
    kinect.setRGB();
    if (onSave) {
      cv::resize(kinect.rgbImage, img, sz, 0, 0);
      cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);
      vw << img;
    }
    cv::imshow("rgb", kinect.rgbImage);
    auto key = cv::waitKey(33);
    if (key == 'q') break;
    else if (key == 'r' && !onSave) {
      vw = cv::VideoWriter(now()+".avi",cv::VideoWriter::fourcc('X','V','I','D'), 30.0, sz);
      //vw = cv::VideoWriter(now()+".mp4",cv::VideoWriter::fourcc('F','M','P','4'), 30.0, sz);
      //vw = cv::VideoWriter(now()+".mp4",cv::VideoWriter::fourcc('M','P','4','V'), 30.0, sz);
      if (!vw.isOpened()) throw runtime_error("cannot create video file");
      onSave = true;
    } else if (key == 's' && onSave) {
      vw.release();
      onSave = false;
    }
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
