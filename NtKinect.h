/*
 * Copyright (c) 2016 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */

/* version 1.1: 2016/07/18 */

#pragma once

#ifdef USE_SPEECH
#include <sapi.h>
#pragma warning(disable: 4996) // for error GetVersionExW() of sphelper.h
#include <sphelper.h> // for SpFindBestToken()
#include <locale.h>
#endif /* USE_SPEECH */

// If you define USE_FACE, please link "Kinect20.Face.lib" 
// and specify "Configuration Property" -> "Build Event" -> "Post Build Event" -> "Command Line" --> 
//     xcopy "$(KINECTSDK20_DIR)\Redist\Face\x64" "$(OutDir)" /e /y /i /r

#ifdef USE_FACE
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#endif /* USE_FACE */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <atlbase.h>

#include <Kinect.h>

#ifdef USE_FACE
#include <Kinect.Face.h>
#endif /* USE_FACE */

#ifdef USE_SPEECH
// Quote from Kinect for Windows SDK v2.0 - Sample/Native/SpeechBasics-D2D
// KinectAudioStream.h and .cpp : Copyright (c) Microsoft Corporation.  All rights reserved.
#include "KinectAudioStream.h"

#define INITGUID
#include <guiddef.h>
// This is the class ID we expect for the Microsoft Speech recognizer.
// Other values indicate that we're using a version of sapi.h that is
// incompatible with this sample.
DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);
#endif /* USE_SPEECH */

#ifdef USE_AUDIO
#include "WaveFile.h"
#endif /* USE_AUDIO */

using namespace std;

#define ERROR_CHECK(ret)                                \
  if ((ret) != S_OK) {                                  \
    stringstream ss;                                    \
    ss << "failed " #ret " " << hex << ret << endl;     \
    throw runtime_error(ss.str().c_str());              \
  }
#define ERROR_CHECK2( ret )                                     \
  if( FAILED( ret ) ){                                          \
    std::stringstream ss;                                       \
    ss << "failed " #ret " " << std::hex << ret << std::endl;   \
    throw std::runtime_error( ss.str().c_str() );               \
  }

#define BODY_MAX        6


class NtKinect {

  // ******* kinect ********
 private:
  CComPtr<IKinectSensor> kinect = nullptr;
 public:
  CComPtr<ICoordinateMapper> coordinateMapper = nullptr;

 private:
  void initialize() {
    ERROR_CHECK(GetDefaultKinectSensor(&kinect));
    ERROR_CHECK(kinect->Open());
    BOOLEAN isOpen;
    ERROR_CHECK(kinect->get_IsOpen(&isOpen));
    if (!isOpen)throw runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");

    kinect->get_CoordinateMapper(&coordinateMapper);
  }

#ifdef USE_AUDIO
  // ******** audio *******
 private:
  CComPtr<IAudioBeamFrameReader> audioBeamFrameReader = nullptr;
  BOOLEAN audio_initialized = false;
  vector<BYTE> audioBuffer;
  void initializeAudio() {
    CComPtr<IAudioSource > audioSource = nullptr;
    ERROR_CHECK(kinect->get_AudioSource(&audioSource));
    ERROR_CHECK(audioSource->OpenReader(&audioBeamFrameReader));
    UINT subFrameLength = 0;
    ERROR_CHECK(audioSource->get_SubFrameLengthInBytes(&subFrameLength));
    audioBuffer.resize(subFrameLength);
    audio_initialized = true;
  }
  // call setSkeleton(), to set flag and get "audioTrackingID" and "audioTrackingIndex".
  void updateAudioFrame(bool flag = false) {
    if (!audio_initialized) initializeAudio();
    CComPtr<IAudioBeamFrameList> audioBeamFrameList;
    auto ret = audioBeamFrameReader->AcquireLatestBeamFrames(&audioBeamFrameList);
    if (ret != S_OK) return;
    UINT beamCount = 0;
    ERROR_CHECK(audioBeamFrameList->get_BeamCount(&beamCount));
    for (int i = 0; i < (int)beamCount; i++) {
      CComPtr<IAudioBeamFrame> audioBeamFrame;
      ERROR_CHECK(audioBeamFrameList->OpenAudioBeamFrame(i, &audioBeamFrame));
      UINT subFrameCount = 0;
      ERROR_CHECK(audioBeamFrame->get_SubFrameCount(&subFrameCount));
      for (int j = 0; j < (int)subFrameCount; j++) {
        CComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
        ERROR_CHECK(audioBeamFrame->GetSubFrame(j, &audioBeamSubFrame));
        ERROR_CHECK(audioBeamSubFrame->get_BeamAngle(&beamAngle));
        ERROR_CHECK(audioBeamSubFrame->get_BeamAngleConfidence(&beamAngleConfidence));
        if (flag) {
          if (!body_initialized) throw runtime_error("You must call setSkeleton() before setAudio()");
          UINT32 count = 0;
          ERROR_CHECK(audioBeamSubFrame->get_AudioBodyCorrelationCount(&count));
          if (count == 0) { audioTrackingId = (UINT64)-1; return; }
          CComPtr<IAudioBodyCorrelation> audioBodyCorrelation;
          ERROR_CHECK(audioBeamSubFrame->GetAudioBodyCorrelation(0, &audioBodyCorrelation));
          ERROR_CHECK(audioBodyCorrelation->get_BodyTrackingId(&audioTrackingId));
        }
        if (audioFile.audioFile.is_open()) {
          audioBeamSubFrame->CopyFrameDataToArray((UINT)audioBuffer.size(), &audioBuffer[0]);
          audioFile.Write(&audioBuffer[0], (UINT)audioBuffer.size());
        }
      }
    }
  }
 public:
  float beamAngle;
  float beamAngleConfidence;
  UINT64 audioTrackingId = (UINT64)-1;
  WaveFile audioFile;
  void setAudio(bool flag = false) { updateAudioFrame(flag); }
  void openAudio(string fname) { audioFile.Open(fname); }
  void closeAudio() { audioFile.Close(); }
  bool isOpenedAudio() { return audioFile.audioFile.is_open(); }
  void drawAudioDirection(cv::Mat& image) {
    const int w = 640, h = 480, len = h * 2 / 3;
    image = cv::Mat::zeros(h, w, CV_8UC4);
    float theta = -beamAngle;
    auto dx = 0 * cos(theta) - len * sin(theta);
    auto dy = 0 * sin(theta) + len * cos(theta);
    if (beamAngleConfidence > 0.5)
      cv::line(image, cv::Point(w / 2, 0), cv::Point((int)dx + w / 2, (int)dy), cv::Scalar(255, 255, 255), 10);
    //cerr << "beamAngle = " << beamAngle << "  beamAngleConfidense = " << beamAngleConfidence << endl;
  }
#endif /* USE_AUDIO */

#ifdef USE_SPEECH
  // ******** speech ********
 private:
  bool speech_initialized = false;
  CComPtr<IAudioBeam> audioBeam;
  CComPtr<IStream> inputStream;
  CComPtr<KinectAudioStream> audioStream;
  CComPtr<ISpStream> speechStream;
  CComPtr<ISpRecognizer> speechRecognizer;
  CComPtr<ISpRecoContext> speechContext;
  std::vector<CComPtr<ISpRecoGrammar>> speechGrammar;
  HANDLE speechEvent;
  bool startedSpeech = false;
  string speechLang = "ja-JP";
  wstring speechGRXML = L"Grammar_jaJP.grxml";
  void initializeSpeech() {
    if (speech_initialized) return;
    speech_initialized = true;
    if (CLSID_ExpectedRecognizer != CLSID_SpInprocRecognizer)
      {
        stringstream ss;
        ss << "This sample was compiled against an incompatible version of sapi.h.\n"
           << "Please ensure that Microsoft Speech SDK and other sample requirements are installed and then rebuild application.";
        throw runtime_error(ss.str());
      }
    CComPtr<IAudioSource> audioSource;
    ERROR_CHECK(kinect->get_AudioSource(&audioSource));

    CComPtr<IAudioBeamList> audioBeamList;
    ERROR_CHECK(audioSource->get_AudioBeams(&audioBeamList));
    ERROR_CHECK(audioBeamList->OpenAudioBeam(0, &audioBeam));

    ERROR_CHECK(audioBeam->OpenInputStream(&inputStream));
    audioStream = new KinectAudioStream(inputStream);

    initializeSpeechStream();

    // Speech Recognizer
    // "en-US" ... English, "ja-JP" ... Japanese
    //createSpeechRecognizer("ja-JP");
    createSpeechRecognizer(speechLang);

    // Speech Recognition Grammar (*.grxml)
    // Grammar ID, Grammar File Name
    //loadSpeechGrammar(0, L"Grammar_jaJP.grxml");
    loadSpeechGrammar(0, speechGRXML);
    /*loadSpeechGrammar( 1, L"Grammar_Additional.grxml" );*/
  }
  inline void initializeSpeechStream() {
    ERROR_CHECK(CoCreateInstance(CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpStream), reinterpret_cast<void**>(&speechStream)));

    // Wave Format of Microphone
    WORD AudioFormat = WAVE_FORMAT_PCM;
    WORD AudioChannels = 1;
    DWORD AudioSamplesPerSecond = 16000;
    DWORD AudioAverageBytesPerSecond = 32000;
    WORD AudioBlockAlign = 2;
    WORD AudioBitsPerSample = 16;

    WAVEFORMATEX waveFormat = { AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0 };

    // Initialize Speech Stream
    ERROR_CHECK(speechStream->SetBaseStream(audioStream, SPDFID_WaveFormatEx, &waveFormat));
  }

  inline void createSpeechRecognizer(const std::string& language = "en-US") {
    // Instance of Speech Recognizer
    ERROR_CHECK(CoCreateInstance(CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpRecognizer), reinterpret_cast<void**>(&speechRecognizer)));

    // Input Stream of Speech Recognizer
    ERROR_CHECK(speechRecognizer->SetInput(speechStream, TRUE));

    // Recognizer Engine Language
    // Kinect for Windows SDK 2.0 Language Packs http://www.microsoft.com/en-us/download/details.aspx?id=43662
    // L"Language=409;Kinect=True" ... English | United States (MSKinectLangPack_enUS.msi)
    // L"Language=411;Kinect=True" ... Japanese | Japan (MSKinectLangPack_jaJP.msi)
    // Other Languages Hexadecimal Value, Please see here https://msdn.microsoft.com/en-us/library/hh378476(v=office.14).aspx
    std::wstring attribute;
    if (language == "de-DE") {
      attribute = L"Language=C07;Kinect=True";
    } else if (language == "en-AU") {
      attribute = L"Language=C09;Kinect=True";
    } else if (language == "en-CA") {
      attribute = L"Language=1009;Kinect=True";
    } else if (language == "en-GB") {
      attribute = L"Language=809;Kinect=True";
    } else if (language == "en-IE") {
      attribute = L"Language=1809;Kinect=True";
    } else if (language == "en-NZ") {
      attribute = L"Language=1409;Kinect=True";
    } else if (language == "en-US") {
      attribute = L"Language=409;Kinect=True";
    } else if (language == "es-ES") {
      attribute = L"Language=2C0A;Kinect=True";
    } else if (language == "es-MX") {
      attribute = L"Language=80A;Kinect=True";
    } else if (language == "fr-CA") {
      attribute = L"Language=C0C;Kinect=True";
    } else if (language == "fr-FR") {
      attribute = L"Language=40c;Kinect=True";
    } else if (language == "it-IT") {
      attribute = L"Language=410;Kinect=True";
    } else if (language == "ja-JP") {
      attribute = L"Language=411;Kinect=True";
    } else {
      throw std::runtime_error("failed " __FUNCTION__);
    }

    setlocale(LC_CTYPE, language.c_str());

    // Speech Recognizer Engine
    CComPtr<ISpObjectToken> engineToken = nullptr;
    ERROR_CHECK(SpFindBestToken(SPCAT_RECOGNIZERS, attribute.c_str(), NULL, &engineToken));
    ERROR_CHECK(speechRecognizer->SetRecognizer(engineToken));

    // Speech Recognizer Context
    ERROR_CHECK(speechRecognizer->CreateRecoContext(&speechContext));

    // Set Adaptation to "Off(0)"
    // (For Long Time (few hours~) Running Program of Speech Recognition)
    ERROR_CHECK(speechRecognizer->SetPropertyNum(L"AdaptationOn", 0));
  }

  inline void loadSpeechGrammar(const ULONGLONG id, const std::wstring& grammar) {
    // Speech Recognition Grammar (*.grxml)
    speechGrammar.push_back(nullptr);
    ERROR_CHECK(speechContext->CreateGrammar(id, &speechGrammar.back()));
    ERROR_CHECK(speechGrammar.back()->LoadCmdFromFile(grammar.c_str(), SPLOADOPTIONS::SPLO_STATIC));
  }

  void speechResult() {
    SPEVENT eventStatus;
    ULONG eventFetch;
    ERROR_CHECK2(speechContext->GetEvents(1, &eventStatus, &eventFetch));
    while (eventFetch > 0) {
      switch (eventStatus.eEventId) {
      case SPEVENTENUM::SPEI_RECOGNITION: // Recognition
        if (eventStatus.elParamType == SPET_LPARAM_IS_OBJECT) {
          CComPtr<ISpRecoResult> speechRes = reinterpret_cast<ISpRecoResult*>(eventStatus.lParam);
          // <tag>PHRASE</tag>
          SPPHRASE* phrase;
          ERROR_CHECK2(speechRes->GetPhrase(&phrase));
          const SPPHRASEPROPERTY* semantic = phrase->pProperties->pFirstChild;
          speechConfidence = semantic->SREngineConfidence;
          if (semantic->SREngineConfidence > confidenceThreshold) {
            std::wstring tag = semantic->pszValue;
            //std::wcout << L"tag : " << tag << std::endl;
            recognizedSpeech = true;
            speechTag = wstring(tag);
          }
          CoTaskMemFree(phrase);
          // <item>Text</item>
          wchar_t* text;
          ERROR_CHECK2(speechRes->GetText(SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, FALSE, &text, NULL));
          //std::wcout << L"\tText : " << text << std::endl;
          speechItem = wstring(text);
          CoTaskMemFree(text);
        }
        break;
      default:
        break;
      }
      ERROR_CHECK2(speechContext->GetEvents(1, &eventStatus, &eventFetch));
    }
  }
 public:
  bool recognizedSpeech = false;
  wstring speechTag;
  wstring speechItem;
  float speechConfidence;
  float confidenceThreshold = 0.3f;
  void setSpeechLang(string &lang, wstring& grxml) {
    speechLang = lang;
    speechGRXML = grxml;
  }
  void startSpeech() {
    if (!speech_initialized) initializeSpeech();
    if (startedSpeech) return;

    //std::cout << "speech recognition starts" << std::endl;

    // Audio Input Stream Starts
    audioStream->SetSpeechState(true);

    // Activate Speech Recognition Grammar
    for (const CComPtr<ISpRecoGrammar> grammar : speechGrammar) {
      ERROR_CHECK(grammar->SetRuleState(NULL, NULL, SPRULESTATE::SPRS_ACTIVE));
    }

    // Set Recognition Status to Active
    ERROR_CHECK(speechRecognizer->SetRecoState(SPRECOSTATE::SPRST_ACTIVE_ALWAYS));

    // Event Timing (Speech Recognition completed)
    ERROR_CHECK(speechContext->SetInterest(SPFEI(SPEVENTENUM::SPEI_RECOGNITION), SPFEI(SPEVENTENUM::SPEI_RECOGNITION)));

    // Speech Recognition starts
    ERROR_CHECK2(speechContext->Resume(0));

    // Speech Recognition Event Handle
    speechEvent = speechContext->GetNotifyEventHandle();

    startedSpeech = true;
    recognizedSpeech = false;
  }
  void stopSpeech() {
    if (!startedSpeech) return;
    startedSpeech = false;
    recognizedSpeech = false;
    audioStream->SetSpeechState(false);
    ERROR_CHECK2(speechRecognizer->SetRecoState(SPRECOSTATE::SPRST_INACTIVE_WITH_PURGE));
    ERROR_CHECK2(speechContext->Pause(0));
    CloseHandle(speechEvent);
  }
  bool setSpeech() {
    if (!startedSpeech) return false;
    recognizedSpeech = false;
    speechTag = wstring();
    speechItem = wstring();
    ResetEvent(speechEvent);
    HANDLE events[1] = { speechEvent };
    DWORD objects = MsgWaitForMultipleObjectsEx(ARRAYSIZE(events), events, 50, QS_ALLINPUT, MWMO_INPUTAVAILABLE);
    switch (objects) {
    case WAIT_OBJECT_0:
      speechResult();
      break;
    default:
      break;
    }
    return recognizedSpeech;
  }
#endif /* USE_SPEECH */

  // ******** color ********
 private:
  CComPtr<IColorFrameReader> colorFrameReader = nullptr;
  BOOLEAN color_initialized = false;
  vector<BYTE> colorBuffer;
 public:
  int colorWidth;
  int colorHeight;
 private:
  unsigned int colorBytesPerPixel;
  void initializeColorFrame() {
    CComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
    ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));
    CComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
    colorFrameDescription->get_Width(&colorWidth);
    colorFrameDescription->get_Height(&colorHeight);
    colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel);
    colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
    color_initialized = true;
  }
  void updateColorFrame()
  {
    if (!color_initialized) initializeColorFrame();
    CComPtr<IColorFrame> colorFrame;
    auto ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
    if (FAILED(ret))return;
    ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray((UINT)colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
  }
 public:
  cv::Mat rgbImage;
  void setRGB() { setRGB(rgbImage); }
  void setRGB(cv::Mat& image) {
    updateColorFrame();
    image = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
    // cv::cvtColor(image,image,CV_BGRA2BGR); // when you save with VideoWriter
  }

  // ******** depth *******
 private:
  CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
  BOOLEAN depth_initialized = false;
  vector<UINT16> depthBuffer;
  int depthWidth;
  int depthHeight;
  UINT16 maxDepth;
  UINT16 minDepth;
  void initializeDepthFrame()
  {
    CComPtr<IDepthFrameSource> depthFrameSource;
    ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
    ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));
    CComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK(depthFrameSource->get_FrameDescription(
                                                       &depthFrameDescription));
    ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
    ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));
    ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(&maxDepth));
    ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(&minDepth));
    depthBuffer.resize(depthWidth * depthHeight);
    depth_initialized = true;
  }
  void updateDepthFrame()
  {
    if (!depth_initialized) initializeDepthFrame();
    CComPtr<IDepthFrame> depthFrame;
    auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
    if (FAILED(ret))return;
    ERROR_CHECK(depthFrame->CopyFrameDataToArray((UINT)depthBuffer.size(), &depthBuffer[0]));
  }
 public:
  cv::Mat depthImage;
  void setDepth(bool raw = true) { setDepth(depthImage, raw); }
  void setDepth(cv::Mat& depthImage, bool raw = true) {
    updateDepthFrame();
    depthImage = cv::Mat(depthHeight, depthWidth, CV_16UC1);
    for (int i = 0; i < depthImage.total(); i++) {
      double rate = depthBuffer[i] / (double)maxDepth;
      depthImage.at<UINT16>(i) = (raw) ? depthBuffer[i] : (UINT16)(255 * 255 * rate); // [0, 65535]
    }
  }

  // ******* infrared ********
 private:
  CComPtr<IInfraredFrameReader> infraredFrameReader = nullptr;
  BOOLEAN infrared_initialized = false;
  int infraredWidth;
  int infraredHeight;
  vector<UINT16> infraredBuffer;
  void initializeInfraredFrame() {
    CComPtr<IInfraredFrameSource> infraredFrameSource;
    ERROR_CHECK(kinect->get_InfraredFrameSource(&infraredFrameSource));
    ERROR_CHECK(infraredFrameSource->OpenReader(&infraredFrameReader));
    CComPtr<IFrameDescription> infraredFrameDescription;
    ERROR_CHECK(infraredFrameSource->get_FrameDescription(&infraredFrameDescription));
    ERROR_CHECK(infraredFrameDescription->get_Width(&infraredWidth));
    ERROR_CHECK(infraredFrameDescription->get_Height(&infraredHeight));
    //cout << "infrared: " << infraredWidth << " " << infraredHeight << endl;
    infraredBuffer.resize(infraredWidth * infraredHeight);
    infrared_initialized = true;
  }
  void updateInfraredFrame() {
    if (!infrared_initialized) initializeInfraredFrame();
    CComPtr<IInfraredFrame> infraredFrame;
    auto ret = infraredFrameReader->AcquireLatestFrame(&infraredFrame);
    if (FAILED(ret)) return;
    ERROR_CHECK(infraredFrame->CopyFrameDataToArray((UINT)infraredBuffer.size(), &infraredBuffer[0]));
  }
 public:
  cv::Mat infraredImage;
  void setInfrared() { setInfrared(infraredImage); }
  void setInfrared(cv::Mat& infraredImage) {
    updateInfraredFrame();
    infraredImage = cv::Mat(infraredHeight, infraredWidth, CV_16UC1, &infraredBuffer[0]);
  }

  // ******** bodyIndex *******
 private:
  CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
  BOOLEAN bodyIndex_initialized = false;
  vector<BYTE> bodyIndexBuffer;
  int bodyIndexWidth;
  int bodyIndexHeight;
  cv::Vec3b   colors[8];
  void initializeBodyIndexFrame()
  {
    CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
    ERROR_CHECK(kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource));
    ERROR_CHECK(bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader));
    CComPtr<IFrameDescription> bodyIndexFrameDescription;
    ERROR_CHECK(bodyIndexFrameSource->get_FrameDescription(&bodyIndexFrameDescription));
    bodyIndexFrameDescription->get_Width(&bodyIndexWidth);
    bodyIndexFrameDescription->get_Height(&bodyIndexHeight);
    bodyIndexBuffer.resize(depthWidth * depthHeight);

    colors[0] = cv::Vec3b(0, 0, 0);
    colors[1] = cv::Vec3b(255, 0, 0);
    colors[2] = cv::Vec3b(0, 255, 0);
    colors[3] = cv::Vec3b(0, 0, 255);
    colors[4] = cv::Vec3b(255, 255, 0);
    colors[5] = cv::Vec3b(255, 0, 255);
    colors[6] = cv::Vec3b(0, 255, 255);
    colors[7] = cv::Vec3b(255, 255, 255);

    bodyIndex_initialized = true;
  }
  void updateBodyIndexFrame()
  {
    if (!bodyIndex_initialized) initializeBodyIndexFrame();
    CComPtr<IBodyIndexFrame> bodyIndexFrame;
    auto ret = bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame);
    if (FAILED(ret)) return;
    ERROR_CHECK(bodyIndexFrame->CopyFrameDataToArray((UINT)bodyIndexBuffer.size(), &bodyIndexBuffer[0]));
  }
 public:
  cv::Mat bodyIndexImage;
  void setBodyIndex() { setBodyIndex(bodyIndexImage); }
  void setBodyIndex(cv::Mat& bodyIndexImage) {
    updateBodyIndexFrame();
    bodyIndexImage = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC3);
    for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
      int y = i / bodyIndexWidth;
      int x = i % bodyIndexWidth;
      int c = (bodyIndexBuffer[i] != 255) ? bodyIndexBuffer[i] + 1 : 0;
      bodyIndexImage.at<cv::Vec3b>(y, x) = colors[c];
    }
  }

  // ******** body ********
  CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
  BOOLEAN body_initialized = false;
  IBody* bodies[BODY_MAX];
  void initializeBodyFrame() {
    CComPtr<IBodyFrameSource> bodyFrameSource;
    ERROR_CHECK(kinect->get_BodyFrameSource(&bodyFrameSource));
    ERROR_CHECK(bodyFrameSource->OpenReader(&bodyFrameReader));
    CComPtr<IFrameDescription> bodyFrameDescription;
    for (auto& body : bodies) {
      body = nullptr;
    }
    body_initialized = true;
  }
  void updateBodyFrame() {
    if (!body_initialized) initializeBodyFrame();
    CComPtr<IBodyFrame> bodyFrame;
    auto ret = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
    if (FAILED(ret)) return;
    for (auto& body : bodies) {
      if (body != nullptr) {
        body->Release();
        body = nullptr;
      }
    }
    ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(BODY_MAX, &bodies[0]));
  }
 public:
  // Joint.Position.{X,Y,Z}  // CameraSpacePoint
  //    DepthSpacePoint dp;
  //    coordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dp);
  //    ColorSpacePoint cp;
  //    coordinateMapper->MapCameraPointToColorSpace(joint.Position, &cp);
  // Joint.TrackingState  == TrackingState::TrackingState_{Tracked,Inferred}
  vector<int> skeletonId;
  vector<UINT64> skeletonTrackingId;
  vector<vector<Joint> > skeleton;
  void setSkeleton() { setSkeleton(skeleton); }
  void setSkeleton(vector<vector<Joint> >& skeleton) {
    updateBodyFrame();
    skeleton.clear();
    skeletonId.clear();
    skeletonTrackingId.clear();
    for (int i = 0; i < BODY_MAX; i++) {
      auto body = bodies[i];
      if (body == nullptr) continue;
      BOOLEAN isTracked = false;
      ERROR_CHECK(body->get_IsTracked(&isTracked));
      if (!isTracked) continue;
      vector<Joint> skel;
      Joint joints[JointType::JointType_Count];
      body->GetJoints(JointType::JointType_Count, joints);
      for (auto joint : joints) skel.push_back(joint);
      skeleton.push_back(skel);
      skeletonId.push_back(i);
      UINT64 trackingId;
      ERROR_CHECK(body->get_TrackingId(&trackingId));
      skeletonTrackingId.push_back(trackingId);
    }
  }
  // HandState::HandState_{Unknown,NotTracked,Open,Closed,Lasso}
  // TrackingConfidence::TrackingConfidence_{Low,Hight}
  pair<int, int> handState(int id = 0, bool isLeft = true) {
    if (!body_initialized) throw runtime_error("body not initialized");
    if (id < 0 || id >= BODY_MAX) throw runtime_error("handstate: bad id " + id);
    pair<int, int> ans(HandState::HandState_Unknown, TrackingConfidence::TrackingConfidence_Low);
    //pair<int, int> ans(HandState::HandState_Unknown, 2);
    if (id >= skeletonId.size()) return ans;
    HRESULT hResult;
    auto body = bodies[skeletonId[id]];
    if (body == nullptr) return ans;
    BOOLEAN isTracked = false;
    ERROR_CHECK(body->get_IsTracked(&isTracked));
    if (!isTracked) return ans;
    HandState handState;
    TrackingConfidence handConfidence;
    if (isLeft) {
      hResult = body->get_HandLeftState(&handState);
      if (!SUCCEEDED(hResult)) return ans;
      hResult = body->get_HandLeftConfidence(&handConfidence);
      if (!SUCCEEDED(hResult)) return ans;
    }
    else {
      hResult = body->get_HandRightState(&handState);
      if (!SUCCEEDED(hResult)) return ans;
      hResult = body->get_HandRightConfidence(&handConfidence);
      if (!SUCCEEDED(hResult)) return ans;
    }
    ans.first = handState; ans.second = handConfidence;
    return ans;
  }

#ifdef USE_FACE
  // ******** face ********
 private:
  array<CComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader;
  BOOLEAN face_initialized = false;
  //array<cv::Scalar, BODY_COUNT> colors;
  //array<string, FaceProperty::FaceProperty_Count> labels;
  void initializeFace()
  {
    if (!body_initialized) throw runtime_error("You must call setSkeleton() before calling setFace().");
    DWORD features =
      FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
      | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
      | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
      | FaceFrameFeatures::FaceFrameFeatures_Happy
      | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
      | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
      | FaceFrameFeatures::FaceFrameFeatures_MouthOpen
      | FaceFrameFeatures::FaceFrameFeatures_MouthMoved
      | FaceFrameFeatures::FaceFrameFeatures_LookingAway
      | FaceFrameFeatures::FaceFrameFeatures_Glasses
      | FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

    for (int count = 0; count < BODY_COUNT; count++) {
      CComPtr<IFaceFrameSource> faceFrameSource;
      ERROR_CHECK(CreateFaceFrameSource(kinect, 0, features, &faceFrameSource));
      ERROR_CHECK(faceFrameSource->OpenReader(&faceFrameReader[count]));
    }
    face_initialized = true;
  }
  inline void quaternion2degree(const Vector4* quaternion, float* pitch, float* yaw, float* roll)
  {
    double x = quaternion->x;
    double y = quaternion->y;
    double z = quaternion->z;
    double w = quaternion->w;

    *pitch = static_cast<float>(std::atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0f);
    *yaw = static_cast<float>(std::asin(2 * (w * y - x * z)) / M_PI * 180.0f);
    *roll = static_cast<float>(std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0f);
  }
 public:
  vector<vector<PointF> > facePoint;
  vector<cv::Rect> faceRect;
  vector<cv::Vec3f> faceDirection;
  vector<vector<DetectionResult>> faceProperty;
  void setFace()
  {
    if (!face_initialized) initializeFace();
    facePoint.clear();
    faceRect.clear();
    faceDirection.clear();
    faceProperty.clear();
    for (int i = 0; i < skeleton.size(); i++) {
      int bid = skeletonId[i];
      UINT64 trackingId = skeletonTrackingId[i];
      CComPtr<IFaceFrameSource> faceFrameSource;
      ERROR_CHECK(faceFrameReader[bid]->get_FaceFrameSource(&faceFrameSource));
      ERROR_CHECK(faceFrameSource->put_TrackingId(trackingId));
      CComPtr<IFaceFrame> faceFrame;
      HRESULT ret = faceFrameReader[bid]->AcquireLatestFrame(&faceFrame);
      if (FAILED(ret)) continue;
      BOOLEAN tracked;
      ERROR_CHECK(faceFrame->get_IsTrackingIdValid(&tracked));
      if (!tracked) continue;
      CComPtr<IFaceFrameResult> faceResult;
      ERROR_CHECK(faceFrame->get_FaceFrameResult(&faceResult));
      if (faceResult != nullptr) doFaceResult(faceResult, bid);
    }
  }
  void doFaceResult(const CComPtr<IFaceFrameResult>& result, const int bid) {
    vector<PointF> points;
    points.resize(FacePointType::FacePointType_Count);
    ERROR_CHECK(result->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, &points[0]));
    facePoint.push_back(points);

    RectI box;
    ERROR_CHECK(result->get_FaceBoundingBoxInColorSpace(&box));
    cv::Rect rect((int)box.Left, (int)box.Top, (int)(box.Bottom - box.Top), (int)(box.Right - box.Left));
    faceRect.push_back(rect);

    Vector4 quaternion;
    float pitch, yaw, roll;
    ERROR_CHECK(result->get_FaceRotationQuaternion(&quaternion));
    quaternion2degree(&quaternion, &pitch, &yaw, &roll);
    cv::Vec3f dir(pitch, yaw, roll);
    faceDirection.push_back(dir);

    vector<DetectionResult> property;
    property.resize(FaceProperty::FaceProperty_Count);
    ERROR_CHECK(result->GetFaceProperties(FaceProperty::FaceProperty_Count, &property[0]));
    faceProperty.push_back(property);
  }
#endif /* USE_FACE */

 public:
  NtKinect() { initialize();  }
  ~NtKinect() {
    if (kinect != nullptr) kinect->Close();
  }
};
