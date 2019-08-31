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

int getFaceIndex(NtKinect& kinect, UINT64 trackingId) {
  for (int i=0; i< kinect.faceTrackingId.size(); i++) {
    if (kinect.faceTrackingId[i] == trackingId) return i;
  }
  return -1;
}
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
void bigEye(NtKinect& kinect,cv::Mat& result,cv::Mat& work,vector<CameraSpacePoint>& hdEye,PointF& fEye) {
  cv::Rect rect = kinect.boundingBoxInColorSpace(hdEye);
  double cx = rect.x + rect.width/2, cy = rect.y + rect.height/2;
  double dx = fEye.X - cx, dy = fEye.Y - cy;
  cv::Rect rect2((int)(rect.x+dx), (int)(rect.y+dy), rect.width, rect.height);
  double margin = 0.5, mw = rect2.width * margin, mh = rect2.height * margin;
  cv::Rect rect3 ((int)(rect2.x-mw/2), (int)(rect2.y-mh/2), (int)(rect2.width+mw), (int)(rect2.height+mh));
  if (rect3.x < 0 || rect3.y < 0 || rect3.x+rect3.width >= kinect.rgbImage.cols || rect3.y+rect3.height >= kinect.rgbImage.rows) {
    cerr << "rect3: " << rect3 << endl;
    return;
  }
  cv::Mat eyeImg(kinect.rgbImage, rect3);
  double scale = 2.0;
  cv::resize(eyeImg,eyeImg,cv::Size((int)(eyeImg.cols*scale), (int)(eyeImg.rows*scale)));
  copyRect(eyeImg, result, 0, 0, eyeImg.cols, eyeImg.rows, (int)(rect3.x-(scale-1)*rect3.width/2), (int)(rect3.y-(scale-1)*rect3.height/2));
  cv::rectangle(work, rect, cv::Scalar(0,255,0), 2);
  cv::rectangle(work, rect2, cv::Scalar(0,0,255), 2);
  cv::rectangle(work, rect3, cv::Scalar(255,0,0), 2);
  cv::rectangle(work, cv::Rect((int)(fEye.X-2), (int)(fEye.Y-2), 4, 4), cv::Scalar(0,255,255), -1);
  cv::rectangle(work, cv::Rect((int)(cx-2), (int)(cy-2), 4, 4), cv::Scalar(255,0,255), -1);
}
void doJob() {
  NtKinect kinect;
  while (1) {
    kinect.setRGB();
    kinect.setSkeleton();
    kinect.setFace();
    kinect.setHDFace();
    cv::Mat result = kinect.rgbImage.clone();
    cv::Mat work = kinect.rgbImage.clone();
    for (int i=0; i<kinect.hdfaceTrackingId.size(); i++) {
      int idx = getFaceIndex(kinect,kinect.hdfaceTrackingId[i]);
      if (idx < 0) continue;
      auto& hdFace = kinect.hdfaceVertices[i];
      vector<CameraSpacePoint> hdLeft({
        hdFace[HighDetailFacePoints_LefteyeInnercorner],
        hdFace[HighDetailFacePoints_LefteyeOutercorner],
        hdFace[HighDetailFacePoints_LefteyeMidtop],
        hdFace[HighDetailFacePoints_LefteyeMidbottom]
      });
      vector<CameraSpacePoint> hdRight({
        hdFace[HighDetailFacePoints_RighteyeInnercorner],
        hdFace[HighDetailFacePoints_RighteyeOutercorner],
        hdFace[HighDetailFacePoints_RighteyeMidtop],
        hdFace[HighDetailFacePoints_RighteyeMidbottom]
      });
      bigEye(kinect,result,work,hdLeft,kinect.facePoint[idx][0]); // left eye
      bigEye(kinect,result,work,hdRight,kinect.facePoint[idx][1]); // right eye
    }
    for (int i=0; i<kinect.hdfaceVertices.size(); i++) {
      for (CameraSpacePoint sp : kinect.hdfaceVertices[i]) {
        ColorSpacePoint cp;
        kinect.coordinateMapper->MapCameraPointToColorSpace(sp,&cp);
        cv::rectangle(work, cv::Rect((int)cp.X-1, (int)cp.Y-1, 2, 2), cv::Scalar(0,192, 0), 1);
      }
    }
    cv::resize(work,work,cv::Size(work.cols/2,work.rows/2));
    cv::imshow("work", work);
    cv::imshow("result", result);
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
