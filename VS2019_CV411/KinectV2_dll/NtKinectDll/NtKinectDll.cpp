#include "pch.h"
#include "framework.h"
#include "NtKinectDll.h"

#include "NtKinect.h"

using namespace std;

NTKINECTDLL_API void* getKinect(void) {
  NtKinect* kinect = new NtKinect;
  return static_cast<void*>(kinect);
}

NTKINECTDLL_API int rightHandState(void* ptr) {
  NtKinect *kinect = static_cast<NtKinect*>(ptr);
  (*kinect).setRGB();
  (*kinect).setSkeleton();
  int scale = 4;
  cv::Mat img((*kinect).rgbImage);
  cv::resize(img,img,cv::Size(img.cols/scale,img.rows/scale),0,0);
  for (auto person: (*kinect).skeleton) {
    for (auto joint: person) {
      if (joint.TrackingState == TrackingState_NotTracked) continue;
      ColorSpacePoint cp;
      (*kinect).coordinateMapper->MapCameraPointToColorSpace(joint.Position,&cp);
      cv::rectangle(img, cv::Rect((int)cp.X/scale-2, (int)cp.Y/scale-2,4,4), cv::Scalar(0,0,255),2);
    }
  }
  cv::imshow("rgb",img);
  cv::waitKey(1);
  for (int i=0; i<(*kinect).skeleton.size(); i++) {
    Joint right = (*kinect).skeleton[i][JointType_HandRight];
    if (right.TrackingState == TrackingState_NotTracked) continue;
    auto state = (*kinect).handState(i,false);
    if (state.first == HandState_Open
	|| state.first == HandState_Closed
	|| state.first == HandState_Lasso ) {
      return state.first;
    }
  }
  return HandState_Unknown;
}
