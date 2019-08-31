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

void copyRect(cv::Mat& src, cv::Mat& dst, int sx, int sy, int w, int h, int dx, int dy) {
  if (sx+w < 0 || sx >= src.cols || sy+h < 0 || sy >= src.rows) return;
  if (sx < 0) { w += sx; dx -= sx; sx=0; }
  if (sx+w > src.cols) w = src.cols - sx; 
  if (sy < 0) { h += sy; dy -= sy; sy=0; }
  if (sy+h > src.rows) h = src.rows - sy;

  if (dx+w < 0 || dx >= dst.cols || dy+h < 0 || dy >= dst.rows) return;
  if (dx < 0) { w += dx; sx -= dx; dx = 0; }
  if (dx+w > dst.cols) w = dst.cols - dx;
  if (dy < 0) { h += dy; sy -= dy; dy = 0; }
  if (dy+h > dst.rows) h = dst.rows - dy;

  cv::Mat roiSrc(src,cv::Rect(sx,sy,w,h));
  cv::Mat roiDst(dst,cv::Rect(dx,dy,w,h));
  roiSrc.copyTo(roiDst);
}

void doJob() {
  NtKinect kinect;
  cv::Mat cat = cv::imread("cat.jpg");
  cv::Mat bgImg;
  cv::Mat fgImg;
  while (1) {
    kinect.setRGB();
    cv::cvtColor(kinect.rgbImage,fgImg,cv::COLOR_BGRA2BGR);
    bgImg = cat.clone();
    kinect.setDepth();
    kinect.setBodyIndex();
    for (int y=0; y<kinect.bodyIndexImage.rows; y++) {
      for (int x=0; x<kinect.bodyIndexImage.cols; x++) {
        UINT16 d = kinect.depthImage.at<UINT16>(y,x);
        uchar bi = kinect.bodyIndexImage.at<uchar>(y,x);
        if (bi == 255) continue;
        ColorSpacePoint cp;
        DepthSpacePoint dp; dp.X = x; dp.Y = y;
        kinect.coordinateMapper->MapDepthPointToColorSpace(dp, d, &cp);
        int cx = (int) cp.X, cy = (int) cp.Y;
        copyRect(fgImg,bgImg,cx-2,cy-2,4,4,cx-2,cy-2);
      }
    }
    cv::imshow("cat", bgImg);
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
