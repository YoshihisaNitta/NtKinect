/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#include <Windows.h>

#include "KinectTetris.h"
#include "Tetris.h"

using namespace std;

void doJob() {
  KinectTetris kinect;
  Tetris tetris;
  tetris.start();
  DWORD t1 = GetTickCount();
  int cmd=0;
  while (1) {
    if (!tetris.getState()) break;

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
    cv::Mat shrink = kinect.rgbImage;
    cv::resize(shrink,shrink,cv::Size(shrink.cols/2,shrink.rows/2),0,0);
    cv::imshow("rgb", shrink);

    if (kinect.tracked()) {
      //kinect.check();
      if (kinect.moveL()) cmd = 1;
      else if (kinect.moveR()) cmd = 2;
      else if (kinect.rotL()) cmd = 3;
      else if (kinect.rotR()) cmd = 4;
      else if (kinect.drop()) cmd = 5;
    }
    
    int key = cv::waitKey(50);
    if (key == 'q') break;
    switch (key) {
    case 'h': cmd = 1; break;
    case 'j': cmd = 3; break;
    case 'k': cmd = 4; break;
    case 'l': cmd = 2; break;
    case ' ': cmd = 5; break;
    }

      DWORD t2 = GetTickCount();
    if (t2 - t1 > 300) {

      switch (cmd) {
      case 1: tetris.moveL(); break;
      case 2: tetris.moveR(); break;
      case 3: tetris.rotL(); break;
      case 4: tetris.rotR(); break;
      case 5:  tetris.drop();
      }

      cmd = 0;
      t1 = t2;

      tetris.step();
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
