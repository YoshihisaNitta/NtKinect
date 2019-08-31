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
  string path = "";
  string cascadeName = "haarcascade_frontalface_alt.xml";
  string cascadeName2 = "haarcascade_eye.xml";
  string cascadeName3 = "haarcascade_smile.xml";
  cv::CascadeClassifier cascade, cascade2, cascade3;
  if (!cascade.load(path + cascadeName)) throw runtime_error(cascadeName + " not found");
  if (!cascade2.load(path + cascadeName2)) throw runtime_error(cascadeName2 + " not found");
  if (!cascade3.load(path + cascadeName3)) throw runtime_error(cascadeName3 + " not found");

  cv::VideoCapture cap(0);
  if (!cap.isOpened()) throw runtime_error("VideoCapture open failed");  
  cv::Mat image;
  cv::Mat gray;  
  while (1) {
    kinect.setRGB(image);
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    equalizeHist(gray, gray);
    vector<cv::Rect> founds, founds2, founds3;
    cascade.detectMultiScale(gray, founds, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
    for (auto faceRect: founds) {
      cv::rectangle(image, faceRect, cv::Scalar(0, 0, 255), 2);
      cv::Mat roi = gray(faceRect);
      cascade2.detectMultiScale(roi, founds2, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
      for (auto eyeRect: founds2) {
        cv::Rect rect(faceRect.x + eyeRect.x, faceRect.y + eyeRect.y, eyeRect.width, eyeRect.height);
        cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
      }
      cv::Rect halfRect = faceRect;
      halfRect.y += faceRect.height/2;
      halfRect.height = faceRect.height/2 - 1;  // under half of face
      cv::Mat roi2 = gray(halfRect);
      cascade3.detectMultiScale(roi2, founds3, 1.1, 0, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
      const int smile_neighbors = (int)founds3.size();
      static int max_neighbors=-1;
      static int min_neighbors=-1;
      if (min_neighbors == -1) min_neighbors = smile_neighbors;
      max_neighbors = MAX(max_neighbors, smile_neighbors);
      float intensityZeroOne = ((float)smile_neighbors - min_neighbors) / (max_neighbors - min_neighbors + 1);
      cv::Rect meter(faceRect.x, faceRect.y-20, (int)100*intensityZeroOne, 20);
      cv::rectangle(image, meter, cv::Scalar(255, 0, 0), -1);
      cv::Rect meterFull(faceRect.x, faceRect.y-20, 100, 20);
      cv::rectangle(image, meterFull, cv::Scalar(255, 0, 0), 1);
    }
    cv::imshow("video", image);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
   }
  cv::destroyAllWindows();
}

int main(int argc, char** argv) {
  try {
    doJob();
  }
  catch (exception& ex) {
    cout << ex.what() << endl;
    string s;
    cin >> s;
  }
  return 0;
}
