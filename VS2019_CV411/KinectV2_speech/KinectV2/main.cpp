/*
 * Copyright (c) 2016-2019 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */
/* http://nw.tsuda.ac.jp/lec/kinect2/ */

#include <iostream>
#include <sstream>

#include <conio.h>

#define USE_SPEECH
#include "NtKinect.h"

using namespace std;

void doJob() {
  NtKinect kinect;
  kinect.startSpeech();
  std::wcout.imbue(std::locale(""));
  while (1) {
    kinect.setSpeech();
    if (kinect.recognizedSpeech) {
      wcout << kinect.speechTag << L" " << kinect.speechItem << endl;
    }
    if (kinect.speechTag == L"EXIT") break;
    if (_kbhit() && _getch() == 'q') break;
  }
  kinect.stopSpeech();
}

int main(int argc, char** argv) {
  try {
    ERROR_CHECK(CoInitializeEx(NULL, COINIT_MULTITHREADED));
    doJob();
    CoUninitialize();
  } catch (exception &ex) {
    cout << ex.what() << endl;
    string s;
    cin >> s;
  }
  return 0;
}
