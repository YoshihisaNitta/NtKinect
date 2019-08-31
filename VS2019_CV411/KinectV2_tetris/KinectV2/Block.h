#pragma once
#include <opencv2/opencv.hpp>
#include "Field.h"

class Block
{
#define NTYPE	7
#define NDIR	4
#define NBLOCK	4

 private:
  int type;
  int dir;
  int x;
  int y;

 public:
 Block() : type(0), dir(0), x(0), y(0) {}
  ~Block() {}

  static const int blocks[NTYPE][NDIR][NBLOCK][2];

  static int typeSize() { return NTYPE; }
  static int dirSize() { return NDIR; }
  void setType(int t) { type = t; }
  int getType() { return type; }
  void setDir(int d) { dir = d; }
  int getDir() { return dir; }
  void setX(int x) { this->x = x; }
  int getX() { return x; }
  void setY(int y) { this->y = y; }
  int getY() { return y; }
  bool isOk(Field &f, int y, int x, int type, int dir) {
    for (int i = 0; i<NBLOCK; ++i) {
      int nx = x + blocks[type][dir][i][0];
      int ny = y + blocks[type][dir][i][1];
      if (!f.in(ny, nx) || !f.isEmpty(ny, nx)) return(false);
    }
    return(true);
  }
  bool canRotR(Field &f) {
    return isOk(f, y, x, type, (dir + 1) % NDIR);
  }
  bool rotR(Field &f) {
    if (!canRotR(f)) return false;
    dir = (dir + 1) % NDIR;
    return true;
  }
  bool canRotL(Field &f) {
    int ndir = (dir + NDIR - 1) % NDIR;
    return isOk(f, y, x, type, (dir + NDIR - 1) % NDIR);
  }

  bool rotL(Field &f) {
    if (!canRotL(f)) return false;
    dir = (dir + NDIR - 1) % NDIR;
    return true;
  }
  bool canMove(Field &f, int dy, int dx) {
    return isOk(f, y + dy, x + dx, type, dir);
  }
  bool move(Field &f, int dy, int dx) {
    if (!canMove(f, dy, dx)) return false;
    y += dy;
    x += dx;
    return true;

  }
  void pile(Field &f) {
    for (int i = 0; i < NBLOCK; i++) {
      int tx = x + blocks[type][dir][i][0];
      int ty = y + blocks[type][dir][i][1];
      f.set(ty, tx, 1);
    }
  }
  void draw(cv::Mat &image, int bHeight, int bWidth, bool flag = true) {
    for (int i = 0; i < NBLOCK; i++) {
      int tx = x + blocks[type][dir][i][0];
      int ty = y + blocks[type][dir][i][1];
      cv::Rect rect(tx*bWidth, ty*bHeight, bWidth, bHeight);
      cv::Scalar color(0, 80, 0);
      if (flag) { color[0] = 255; color[1] = 255; color[2] = 0; }
      cv::rectangle(image, rect, color, -1);
    }
  }
  int minY() {
    int n = 0;
    for (int i = 0; i<NBLOCK; ++i) {
      if (blocks[type][dir][i][1] < n) n = blocks[type][dir][i][1];
    }
    return(n);
  }
};
