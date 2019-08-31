#pragma once
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;

class Field
{
#define NWIDTH	12
#define NHEIGHT	24

 private:
  vector<vector<int>> field;
  void initialize(int h, int w) {
    field.clear();
    field.resize(h);
    for (int i = 0; i < h; i++) {
      field[i].resize(w);
      for (int j = 0; j < w; j++)
	field[i][j] = 0;
    }
  }
 public:

  Field()	{
    initialize(NHEIGHT, NWIDTH);
  }
  Field(int h, int w) { initialize(h, w); }
  ~Field() {}
  int width() { return field[0].size(); }
  int height() { return field.size(); }
  int in(int y, int x) { return y >= 0 && y < height() && x >= 0 && x < width(); }
  int set(int y, int x, int val) {
    if (!in(y, x)) {
      stringstream ss;
      ss << "out of range (" << y << ", " << x << ")";
      throw runtime_error(ss.str().c_str());
    }
    int old = field[y][x];
    field[y][x] = val;
    return old;
  }
  int get(int y, int x) {
    if (!in(y, x)) {
      stringstream ss;
      ss << "out of range (" << y << ", " << x << ")";
      throw runtime_error(ss.str().c_str());
    }
    return field[y][x];
  }
  bool isEmpty(int y, int x) { return get(y, x) == 0; }
  void clear() {
    for (int i = 0; i < field.size(); i++)
      for (int j = 0; j < field[i].size(); j++)
	field[i][j] = 0;
  }
  int eraseLine() {
    int count = 0;
    for (int i = height() - 1; i >= 0;) {
      bool flag = true;
      for (int j = 0; j < width(); j++) {
	if (isEmpty(i, j)) { flag = false; break; }
      }
      if (flag) {
	count++;
	for (int d = i, s = i - 1; d >= 0; --d, --s)
	  for (int j = 0; j < width(); j++)
	    set(d, j, (s<0) ? 0 : get(s, j));
      }
      else --i;
    }
    return count;
  }
  void draw(cv::Mat &image, int bHeight, int bWidth) {
    for (int i = 0; i < height(); i++) {
      for (int j = 0; j < width(); j++) {
	cv::Rect rect(j*bWidth, i*bHeight, bWidth, bHeight);
	cv::Scalar color(0, 80, 0);
	if (!isEmpty(i, j)) { color[1] = 255; }
	cv::rectangle(image, rect, color, -1);
      }
    }
  }
};
