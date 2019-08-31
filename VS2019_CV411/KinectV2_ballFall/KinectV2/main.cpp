/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#include "NtKinect.h"
#include "BallFall.h"

const int segment[] = {
  JointType_SpineBase, JointType_SpineMid,           // spine
  JointType_SpineMid, JointType_SpineShoulder,
  JointType_SpineShoulder, JointType_Neck,           // head
  JointType_Neck, JointType_Head,

  JointType_SpineShoulder, JointType_ShoulderLeft,   // left hand
  JointType_ShoulderLeft, JointType_ElbowLeft,
  JointType_ElbowLeft, JointType_WristLeft,
  JointType_WristLeft, JointType_HandLeft,
  JointType_HandLeft, JointType_HandTipLeft,
  JointType_HandLeft, JointType_ThumbLeft,

  JointType_SpineShoulder, JointType_ShoulderRight,  // right hand
  JointType_ShoulderRight, JointType_ElbowRight,
  JointType_ElbowRight, JointType_WristRight,
  JointType_WristRight, JointType_HandRight,
  JointType_HandRight, JointType_HandTipRight,
  JointType_HandRight, JointType_ThumbRight,
  
  JointType_SpineBase,  JointType_HipLeft,           // left leg
  JointType_HipLeft,  JointType_KneeLeft,
  JointType_KneeLeft,  JointType_AnkleLeft,
  JointType_AnkleLeft,  JointType_FootLeft,

  JointType_SpineBase,  JointType_HipRight,          // right leg
  JointType_HipRight,  JointType_KneeRight,
  JointType_KneeRight,  JointType_AnkleRight,
  JointType_AnkleRight,  JointType_FootRight,
};
const int segmentSize = sizeof(segment) / sizeof(int);

using namespace std;

void doJob() {
  NtKinect kinect;
  BallFall bf;
  bool flag = false;
  kinect.setRGB();
  bf.start(kinect.rgbImage.cols, kinect.rgbImage.rows);
  vector<pair<cv::Vec3f, cv::Vec3f>> skel;
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    skel.clear();
    for (int i = 0; i < kinect.skeleton.size(); i++) {
      for (int j = 0; j < segmentSize/2; j++) {
	Joint joint1 = kinect.skeleton[i][segment[j * 2]];
	Joint joint2 = kinect.skeleton[i][segment[j * 2 + 1]];
	if (joint1.TrackingState != TrackingState_NotTracked
	    && joint2.TrackingState != TrackingState_NotTracked) {
	  ColorSpacePoint cp;
	  kinect.coordinateMapper->MapCameraPointToColorSpace(joint1.Position, &cp);
	  cv::Vec3f p1 = cv::Vec3f(cp.X, cp.Y, 0.0f);
	  kinect.coordinateMapper->MapCameraPointToColorSpace(joint2.Position, &cp);
	  cv::Vec3f p2 = cv::Vec3f(cp.X, cp.Y, 0.0f);
	  skel.push_back(pair<cv::Vec3f, cv::Vec3f>(p1,p2));
	}
      }
    }
    bf.step(skel);
    cv::Mat image;
    if (flag) image = kinect.rgbImage.clone();
    else {
      image = cv::Mat(kinect.rgbImage.rows,kinect.rgbImage.cols,CV_8UC3);
      cv::rectangle(image,cv::Rect(0,0,image.cols,image.rows),cv::Scalar(0,0,0), -1);
    }
    bf.draw(skel, image);
    cv::imshow("ball fall", image);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'v') flag = !flag;
  }
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
