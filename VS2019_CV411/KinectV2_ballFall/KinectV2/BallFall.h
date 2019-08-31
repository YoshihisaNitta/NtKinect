#pragma once
#include <time.h>
#include <opencv2/opencv.hpp>
#include "Ball.h"
using namespace std;

class BallFall
{
#define N_BALL	8

 private:
  cv::Mat image;
  vector<Ball> balls;
  cv::Vec3f mouse;
 public:
  BallFall() {
    srand((unsigned int)time(NULL));
  }
  ~BallFall() {}
  void start(int w = 640, int h = 480) {
    image = cv::Mat(h, w, CV_8UC3);
    for (int i = 0; i < N_BALL; i++) {
      Ball ball;
      randomBall(ball);
      balls.push_back(ball);
    }
    cv::namedWindow("ball fall");
    mouse[0] = -1;
  }
  void randomBall(Ball &b) {
    cv::Vec3f pos, v, accel;
    pos[0] = rand() % image.cols;
    pos[1] = 0; pos[2] = 0;
    v[0] = rand() % 4 - 2; v[1] = rand() % 1; v[2] = 0;
    accel[0] = 0; accel[1] = 1.0; accel[2] = 0;
    b.setPos(pos); b.setV(v); b.setAccel(accel); b.setR(30);
  }
  void step(vector<pair<cv::Vec3f, cv::Vec3f>>& segments) {
    for (int i = 0; i < balls.size(); i++) balls[i].step();
    for (int i = 0; i < balls.size(); i++) {
      for (int j = i + 1; j < balls.size(); j++)
	balls[i].bounce(balls[j]);
    }
    for (int i = 0; i < balls.size(); i++) {
      for (int j = 0; j < segments.size(); j++)
	balls[i].bounce(segments[j]);
      if (!balls[i].in(image)) randomBall(balls[i]);
    }
  }
  void draw(vector<pair<cv::Vec3f, cv::Vec3f>>& segments) {
    cv::rectangle(image, cv::Rect(0, 0, image.cols, image.rows), cv::Scalar(50, 50, 50), -1);
    draw(segments, image);
    cv::imshow("ball fall", image);
  }

  void draw(vector<pair<cv::Vec3f, cv::Vec3f>>& segments, cv::Mat& image) {
    for (int i = 0; i < balls.size(); i++) {
      balls[i].draw(image);
    }
    for (int i = 0; i < segments.size(); i++) {
      cv::Point p((int)segments[i].first[0], (int)segments[i].first[1]);
      cv::Point q((int)segments[i].second[0], (int)segments[i].second[1]);
      cv::line(image, p, q, cv::Scalar(255, 255, 0), 8);
    }
  }
};
