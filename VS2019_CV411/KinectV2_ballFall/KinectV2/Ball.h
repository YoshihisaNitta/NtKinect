#pragma once
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
using namespace std;

class Ball
{
#define EPS 1.0e-6
 private:
  cv::Vec3f pos;
  cv::Vec3f vel;
  cv::Vec3f accel;
  double r;
 public:
 Ball() : pos(0, 0, 0), vel(0, 0, 0), accel(0, 0, 0), r(1.0) {}
  Ball(cv::Vec3f &pos, cv::Vec3f &v, cv::Vec3f &accel, double r) {
    this->pos = pos;
    this->vel = v;
    this->accel = accel;
    this->r = r;
  }
  ~Ball() {}
  void setPos(cv::Vec3f &pos) { this->pos = pos; }
  cv::Vec3f getPos() { return pos; }
  void setV(cv::Vec3f &v) { this->vel = v; }
  cv::Vec3f getV() { return vel; }
  void setAccel(cv::Vec3f &a) { accel = a; }
  cv::Vec3f getAccel() { return accel; }
  void setR(double r) { this->r = r; }
  double getR() { return r; }
  void step() {
    vel += accel;
    pos += vel;
  }
  bool bounce(pair<cv::Vec3f, cv::Vec3f> &seg) {
    cv::Vec3f nearPt = nearest(seg, pos);
    double len = cv::norm(pos, nearPt);

    cv::Vec3f oldPos = pos - vel;
    pair<cv::Vec3f, cv::Vec3f> trail(oldPos, pos);
    pair<cv::Vec3f, cv::Vec3f> pr = nearest(seg, trail);
    double len2 = cv::norm(pr.first, pr.second);

    if (len2 < EPS) {
      cv::Vec3f pt = oldPos - pr.second;
      cv::Vec3f par = seg.second - seg.first;
      cv::Vec3f normal = par.cross(cv::Vec3f(0, 0, 1));
      normal /= cv::norm(normal);
      double cs = pt.dot(normal);
      if (cs < 0){
	normal *= -1.0; cs *= -1.0;
      }
      vel = 2 * cs * normal - pt;
      pos += vel;
      return true;
    }
    else if (len < r) {
      reflect(seg, nearPt);
      return true;
    }
    return false;
  }
  bool bounce(Ball &ball) {
    cv::Vec3f normal = pos - ball.pos;
    double len = cv::norm(normal);
    if (len >= r + ball.r) return false;
    if (len < 1.0) {
      pos[0] = -1;
      normal = pos - ball.pos;
      len = cv::norm(normal);
    }
    normal /= len;
    cv::Vec3f u = -vel;
    double cs = normal.dot(u);
    if (cs > 0) vel = 2 * cs * normal - u;
    pos += (r + ball.r - len) / 2 * normal;

    normal *= -1.0;
    u = -ball.vel;
    cs = normal.dot(u);
    if (cs > 0) ball.vel = 2 * cs * normal - u;
    ball.pos += (r + ball.r - len) / 2 * normal;

    return true;
  }
  pair<cv::Vec3f, cv::Vec3f> nearest(pair<cv::Vec3f, cv::Vec3f> &seg1, pair<cv::Vec3f, cv::Vec3f> &seg2) {
    pair<cv::Vec3f, cv::Vec3f> ans;
    cv::Vec3f ab = seg1.second - seg1.first, cd = seg2.second - seg2.first, ac = seg2.first - seg1.first;
    cv::Vec3f v = ab.cross(cd);
    if (v[2] > EPS) {
      cv::Vec3f u = ac.cross(cd), w = ac.cross(ab);
      double t = u[2] / v[2], s = w[2] / v[2];
      if (t >= 0 - EPS && t <= 1.0 + EPS && s >= 0 - EPS && s <= 1.0 + EPS) {
	if (abs(t) < EPS) { ans.first = ans.second = seg1.first; }
	else if (abs(t - 1) < EPS) { ans.first = ans.second = seg1.second; }
	else if (abs(s) < EPS) { ans.first = ans.second = seg2.first; }
	else if (abs(s - 1) < EPS) { ans.first = ans.second = seg2.second; }
	else {
	  ab[2] = 0.0;
	  ans.first = ans.second = ab*t + seg1.first;
	}
	return ans;
      }
    }
    ans.first = seg1.first;
    ans.second = nearest(seg2, seg1.first);
    double minlen = cv::norm(ans.second, seg1.first);
    v = nearest(seg2, seg1.second);
    double len = cv::norm(v, seg1.second);
    if (len < minlen) { ans.first = seg1.first; ans.second = v; minlen = len; }
    v = nearest(seg1, seg2.first);
    len = cv::norm(v, seg2.first);
    if (len < minlen) { ans.first = v; ans.second = seg2.first; minlen = len; }
    v = nearest(seg1, seg2.second);
    len = cv::norm(v, seg2.second);
    if (len < minlen) { ans.first = v; ans.second = seg2.second; minlen = len; }
    return ans;
  }

  double minDistance(pair<cv::Vec3f, cv::Vec3f> &seg, cv::Vec3f &pt) {
    cv::Vec3f nearPt = nearest(seg, pt);
    return cv::norm(nearPt, pt);
  }
  cv::Vec3f nearest(pair<cv::Vec3f, cv::Vec3f> &seg, cv::Vec3f &pt) {
    cv::Vec3f p(seg.second - seg.first), q(pt - seg.first);
    double t = p.dot(q) / p.dot(p);
    if (t <= 0.0 + EPS) return seg.first;
    if (t >= 1.0 - EPS) return seg.second;
    p = p*t + seg.first;
    return p;
  }
  void reflect(pair<cv::Vec3f, cv::Vec3f> &seg, cv::Vec3f &nearPt) {
    cv::Vec3f normal = pos - nearPt;
    double len = cv::norm(normal);
    if (len < 1.0) { // if ball's center is on the segment, add 2 to ball.y
      pos[1] -= 2;
      normal = pos - nearPt;
      len = cv::norm(normal);
    }
    normal /= len;
    if (len<r) pos += (r - len) * normal;
    if (normal.dot(vel) > 0) { // same direction
      vel += 5 * normal;
    }
    else {
      cv::Vec3f u = -vel;
      vel = 2 * normal.dot(u) * normal - u; // reflect
    }
  }
  bool in(cv::Mat &image) {
    return pos[0] >= 0 && pos[0] < image.cols && pos[1] >= -30 && pos[1] < image.rows;
  }
  void draw(cv::Mat &image, int id = -1) {
    cv::Point pt((int)pos[0], (int)pos[1]);
    cv::circle(image, pt, r, cv::Scalar(255, 0, 255), -1);
    stringstream ss; ss << id;
    if (id >= 0) cv::putText(image, ss.str(), pt, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
  }
  string tostring() {
    stringstream ss;
    ss << "ball[" << pos << " " << vel << " " << accel << " " << r << "]";
    return ss.str();
  }
};
