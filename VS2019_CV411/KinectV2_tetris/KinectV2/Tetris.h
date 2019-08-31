#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>
#include "Block.h"
using namespace std;

class Tetris
{
#define BL	16
 private:
  Field field;
  Block block;
  cv::Mat image;
  bool state;
 public:
  Tetris() {
    srand((unsigned int)time(NULL));
  }
  ~Tetris() {}
  void setBlock() {
    block.setType(rand() % block.typeSize());
    block.setDir(rand() % block.dirSize());
    block.setX(field.width() / 2);
    block.setY(-block.minY());
  }
  void start() {
    image = cv::Mat::zeros(field.height()*BL, field.width()*BL, CV_8UC3);
    field.clear();
    cout << field.width() << " " << field.height() << endl;
    setBlock();
    field.draw(image, BL, BL);
    block.draw(image, BL, BL);
    state = true;
  }
  bool getState() { return state; }
  bool step() {
    if (block.canMove(field, 1, 0)) {
      block.draw(image, BL, BL, false);
      block.move(field, 1, 0);
      block.draw(image, BL, BL);
    }
    else {
      block.pile(field);
      field.draw(image, BL, BL);
      int count = field.eraseLine();
      if (count > 0) field.draw(image, BL, BL);
      setBlock();
      if (!block.canMove(field, 0, 0)) {
	state = false;
      }
      block.draw(image, BL, BL);
    }
    cv::imshow("tetris", image);
    return true;
  }
  bool drop() {
    while (block.canMove(field, 1, 0)) {
      block.draw(image, BL, BL, false);
      block.move(field, 1, 0);
      block.draw(image, BL, BL);
      cv::imshow("tetris", image);
      cv::waitKey(30);
    }
    return true;
  }
  bool moveL() {
    block.draw(image, BL, BL, false);
    bool flag = block.move(field, 0, -1);
    block.draw(image, BL, BL);
    return flag;
  }
  bool moveR() {
    block.draw(image, BL, BL, false);
    bool flag = block.move(field, 0, 1);
    block.draw(image, BL, BL);
    return flag;
  }
  bool rotR() {
    block.draw(image, BL, BL, false);
    bool flag = block.rotR(field);
    block.draw(image, BL, BL);
    return flag;
  }
  bool rotL() {
    block.draw(image, BL, BL, false);
    bool flag = block.rotL(field);
    block.draw(image, BL, BL);
    return flag;
  }
};
