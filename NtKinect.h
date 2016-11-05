/*
 * Copyright (c) 2016 Yoshihisa Nitta
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */

/* version 1.8: 2016/11/04 */

/* http://nw.tsuda.ac.jp/lec/kinect2/ */

/* [Definable Macros]
 *   USE_FACE
 *     Library: Kinect20.Face.lib
 *     Post Build Event:
 *            xcopy "$(KINECTSDK20_DIR)\Redist\Face\x64" "$(OutDir)" /e /y /i /r
 *   USE_SPEECH
 *     Library: sapi.lib
 *   USE_GESTURE
 *     Library: Kinect20.VisualGestureBuilder.lib
 *     Post Build Event:
 *            xcopy "$(KINECTSDK20_DIR)\Redist\VGB\x64" "$(OutDir)" /e /y /i /r
 *            if exist "$(ProjectDir)\*.gbd" ( copy "$(ProjectDir)\*.gbd" "$(OutDir)" /y )
 *   USE_AUDIO
 *   USE_THREAD
 *
 *
 *  [to add "Post Build Event"]
 *    "Configuration Property" -> "Build Event" -> "Post Build Event" -> "Command Line" --> add
 */

#pragma once

#if defined(USE_THREAD)
#include <windows.h>
#include <process.h>
#endif /* USE_THREAD */

#if defined(USE_SPEECH)
#include <sapi.h>
#pragma warning(disable: 4996) // for error GetVersionExW() of sphelper.h
#include <sphelper.h> // for SpFindBestToken()
#include <locale.h>
#endif /* USE_SPEECH */

#if defined(USE_FACE) || defined(USE_GESTURE)
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#endif /* USE_FACE || USE_GESTURE */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <atlbase.h>

#include <Kinect.h>

#if defined(USE_FACE)
#include <Kinect.Face.h>
#endif /* USE_FACE */

#if defined(USE_GESTURE)
#include <Kinect.VisualGestureBuilder.h>
#endif

#if defined(USE_SPEECH)
// Quote from Kinect for Windows SDK v2.0 - Sample/Native/SpeechBasics-D2D
// KinectAudioStream.h and .cpp : by Microsoft Corporation
#include "KinectAudioStream.h"

#define INITGUID
#include <guiddef.h>
// This is the class ID we expect for the Microsoft Speech recognizer.
// Other values indicate that we're using a version of sapi.h that is
// incompatible with this sample.
DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);
#endif /* USE_SPEECH */

#if defined(USE_AUDIO)
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

#if defined(USE_THREAD)
  // ******* multi thread ********
 private:
  HANDLE mutex;
  BOOLEAN thread_initialized = false;
  void initThread() {
    if (thread_initialized) return;
    mutex = CreateMutex(NULL, FALSE, NULL);
    if (GetLastError() == ERROR_ALREADY_EXISTS) throw runtime_error("cannot create mutex");
    thread_initialized = true;
  }
 public:
  void acquire() {
    if (!thread_initialized) initThread();
    DWORD ret = WaitForSingleObject(mutex, INFINITE);
    if (ret == WAIT_FAILED) throw runtime_error("error in wait for mutex");
  }
  void release() {
    if (!thread_initialized) throw runtime_error("You must call acquire() begore calling release()");
    ReleaseMutex(mutex);
  }
  HRESULT _MapCameraPointToColorSpace(CameraSpacePoint sp, ColorSpacePoint* cp) {
    acquire();
    HRESULT h = coordinateMapper->MapCameraPointToColorSpace(sp,cp);
    release();
    return h;
  }
  HRESULT _MapCameraPointToDepthSpace(CameraSpacePoint sp, DepthSpacePoint* dp) {
    acquire();
    HRESULT h = coordinateMapper->MapCameraPointToDepthSpace(sp,dp);
    release();
    return h;
  }
  HRESULT _MapDepthPointToColorSpace(DepthSpacePoint dp, UINT16 depth, ColorSpacePoint* cp) {
    acquire();
    HRESULT h = coordinateMapper->MapDepthPointToColorSpace(dp,depth,cp);
    release();
    return h;
  }
  HRESULT _MapDepthPointToCameraSpace(DepthSpacePoint dp, UINT16 depth, CameraSpacePoint* sp) {
    acquire();
    HRESULT h = coordinateMapper->MapDepthPointToCameraSpace(dp,depth,sp);
    release();
    return h;
  }
#endif /* USE_THREAD */

#if defined(USE_AUDIO)
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
# if defined(USE_THREAD)
  void _setAudio(float& beamAngle, float& beamAngleConfidence, UINT64& audioTrackingId, bool flag = false) {
    acquire();
    setAudio(flag);
    beamAngle = this->beamAngle;
    beamAngleConfidence = this->beamAngleConfidence;
    audioTrackingId = this->audioTrackingId;
    release();
  }
# endif /* USE_THREAD */
#endif /* USE_AUDIO */

#if defined(USE_SPEECH)
  // ******** speech ********
 private:
  bool speech_initialized = false;
  CComPtr<IAudioBeam> audioBeam;
  CComPtr<IStream> inputStream;
  CComPtr<KinectAudioStream> audioStream;
  CComPtr<ISpStream> speechStream;
  CComPtr<ISpRecognizer> speechRecognizer;
  CComPtr<ISpRecoContext> speechContext;
  std::vector<CComPtr<ISpRecoGrammar> > speechGrammar;
  HANDLE speechEvent;
  bool startedSpeech = false;
  string speechLang = "ja-JP";
  wstring speechGRXML = L"Grammar_jaJP.grxml";
  void initializeSpeech() {
    if (speech_initialized) return;
    speech_initialized = true;
    if (CLSID_ExpectedRecognizer != CLSID_SpInprocRecognizer) {
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
# if defined(USE_THREAD)
  bool _setSpeech(pair<wstring,wstring>& p) {
    acquire();
    bool ret = setSpeech();
    p = pair<wstring,wstring>(wstring(speechTag),wstring(speechItem));
    release();
    return ret;
  }
# endif /* USE_THREAD */
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
  void updateColorFrame() {
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
#if defined(USE_THREAD)
  void _setRGB(cv::Mat& image) {
    acquire();
    setRGB();
    image = rgbImage.clone();
    release();
  }
#endif /* USE_THREAD */

  // ******** depth *******
 private:
  CComPtr<IDepthFrameReader> depthFrameReader = nullptr;
  BOOLEAN depth_initialized = false;
  vector<UINT16> depthBuffer;
  int depthWidth;
  int depthHeight;
  UINT16 maxDepth;
  UINT16 minDepth;
  void initializeDepthFrame() {
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
  void updateDepthFrame() {
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
#if defined(USE_THREAD)
  void _setDepth(cv::Mat& image, bool raw = true) {
    acquire();
    setDepth(raw);
    image = depthImage.clone();
    release();
  }
#endif /* USE_THREAD */

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
#if defined(USE_THREAD)
  void _setInfrared(cv::Mat& image) {
    acquire();
    setInfrared();
    image = infraredImage.clone();
    release();
  }
#endif /* USE_THREAD */

  // ******** bodyIndex *******
 private:
  CComPtr<IBodyIndexFrameReader> bodyIndexFrameReader = nullptr;
  BOOLEAN bodyIndex_initialized = false;
  vector<BYTE> bodyIndexBuffer;
  int bodyIndexWidth;
  int bodyIndexHeight;
  cv::Vec3b   colors[8];
  void initializeBodyIndexFrame() {
    CComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
    ERROR_CHECK(kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource));
    ERROR_CHECK(bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader));
    CComPtr<IFrameDescription> bodyIndexFrameDescription;
    ERROR_CHECK(bodyIndexFrameSource->get_FrameDescription(&bodyIndexFrameDescription));
    bodyIndexFrameDescription->get_Width(&bodyIndexWidth);
    bodyIndexFrameDescription->get_Height(&bodyIndexHeight);
    bodyIndexBuffer.resize(bodyIndexWidth * bodyIndexHeight);

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
  void updateBodyIndexFrame() {
    if (!bodyIndex_initialized) initializeBodyIndexFrame();
    CComPtr<IBodyIndexFrame> bodyIndexFrame;
    auto ret = bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame);
    if (FAILED(ret)) return;
    ERROR_CHECK(bodyIndexFrame->CopyFrameDataToArray((UINT)bodyIndexBuffer.size(), &bodyIndexBuffer[0]));
  }
 public:
  cv::Mat bodyIndexImage;
  void setBodyIndex(bool raw = true) { setBodyIndex(bodyIndexImage, raw); }
  void setBodyIndex(cv::Mat& bodyIndexImage, bool raw = true) {
    updateBodyIndexFrame();
    if (raw) {
      bodyIndexImage = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC1);
      for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
        int y = i / bodyIndexWidth;
        int x = i % bodyIndexWidth;
        bodyIndexImage.at<uchar>(y, x) = bodyIndexBuffer[i];
      }
    } else {
      bodyIndexImage = cv::Mat(bodyIndexHeight, bodyIndexWidth, CV_8UC3);
      for (int i = 0; i < bodyIndexHeight*bodyIndexWidth; i++) {
        int y = i / bodyIndexWidth;
        int x = i % bodyIndexWidth;
        int c = (bodyIndexBuffer[i] != 255) ? bodyIndexBuffer[i] + 1 : 0;
        bodyIndexImage.at<cv::Vec3b>(y, x) = colors[c];
      }
    }
  }
#if defined(USE_THREAD)
  void _setBodyIndex(cv::Mat& image, bool raw = true) {
    acquire();
    setBodyIndex(raw);
    image = bodyIndexImage.clone();
    release();
  }
#endif /* USE_THREAD */

  // ******** body ********
  CComPtr<IBodyFrameReader> bodyFrameReader = nullptr;
  BOOLEAN body_initialized = false;
  IBody* bodies[BODY_COUNT];
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
    ERROR_CHECK(bodyFrame->GetAndRefreshBodyData(BODY_COUNT, &bodies[0]));
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
    for (int i = 0; i < BODY_COUNT; i++) {
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
    if (id < 0 || id >= BODY_COUNT) throw runtime_error("handstate: bad id " + id);
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
#if defined(USE_THREAD)
  void _setSkeleton(vector<vector<Joint> >& skel, vector<int>& skelId, vector<UINT64>& skelTrackingId) {
    acquire();
    skel.clear();
    skelId.clear();
    skelTrackingId.clear();
    setSkeleton(skel);
    for (auto id: skeletonId) skelId.push_back(id);
    for (auto tid: skeletonTrackingId) skelTrackingId.push_back(tid);
    release();
  }
  pair<int, int> _handState(int id = 0, bool isLeft = true) {
    acquire();
    pair<int,int> state = handState(id,isLeft);
    release();
    return state;
  }
#endif /* USE_THREAD */

#if defined(USE_FACE)
  // ******** face ********
 private:
  array<CComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader; // color
  BOOLEAN face_initialized = false;
  array<CComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader2; // infrared
  BOOLEAN face_initialized2 = false;
  //array<cv::Scalar, BODY_COUNT> colors;
  //array<string, FaceProperty::FaceProperty_Count> labels;
  void initializeFace() {
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
  void initializeFace2() {
    if (!body_initialized) throw runtime_error("You must call setSkeleton() before calling setFace().");
    DWORD features =
      FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInInfraredSpace
      | FaceFrameFeatures::FaceFrameFeatures_PointsInInfraredSpace
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
      ERROR_CHECK(faceFrameSource->OpenReader(&faceFrameReader2[count]));
    }
    face_initialized2 = true;
  }
  inline void quaternion2degree(const Vector4* quaternion, float* pitch, float* yaw, float* roll) {
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
  vector<vector<DetectionResult> > faceProperty;
  vector<UINT64> faceTrackingId;
  void setFace(bool isColorSpace=true) {
    if (isColorSpace && !face_initialized) initializeFace();
    else if (!isColorSpace && !face_initialized2) initializeFace2();
    facePoint.clear();
    faceRect.clear();
    faceDirection.clear();
    faceProperty.clear();
    faceTrackingId.clear();
    for (int i = 0; i < skeleton.size(); i++) {
      int bid = skeletonId[i];
      UINT64 trackingId = skeletonTrackingId[i];
      CComPtr<IFaceFrameSource> faceFrameSource;
      if (isColorSpace) {
        ERROR_CHECK(faceFrameReader[bid]->get_FaceFrameSource(&faceFrameSource));
      } else {
        ERROR_CHECK(faceFrameReader2[bid]->get_FaceFrameSource(&faceFrameSource));
      }
      ERROR_CHECK(faceFrameSource->put_TrackingId(trackingId));
      CComPtr<IFaceFrame> faceFrame;
      if (isColorSpace) {
        HRESULT ret = faceFrameReader[bid]->AcquireLatestFrame(&faceFrame);
        if (FAILED(ret)) continue;
      } else {
        HRESULT ret = faceFrameReader2[bid]->AcquireLatestFrame(&faceFrame);
        if (FAILED(ret)) continue;
      }
      BOOLEAN tracked;
      ERROR_CHECK(faceFrame->get_IsTrackingIdValid(&tracked));
      if (!tracked) continue;
      CComPtr<IFaceFrameResult> faceResult;
      ERROR_CHECK(faceFrame->get_FaceFrameResult(&faceResult));
      if (faceResult != nullptr) doFaceResult(faceResult, bid, trackingId, isColorSpace);
    }
  }
  void doFaceResult(const CComPtr<IFaceFrameResult>& result, const int bid, const UINT64 trackingId, const bool isColorSpace=true) {
    vector<PointF> points;
    points.resize(FacePointType::FacePointType_Count);
    if (isColorSpace) {
      ERROR_CHECK(result->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, &points[0]));
    } else {
      ERROR_CHECK(result->GetFacePointsInInfraredSpace(FacePointType::FacePointType_Count, &points[0]));
    }
    facePoint.push_back(points);

    RectI box;
    if (isColorSpace) {
      ERROR_CHECK(result->get_FaceBoundingBoxInColorSpace(&box));
    } else {
      ERROR_CHECK(result->get_FaceBoundingBoxInInfraredSpace(&box));
    }
    cv::Rect rect((int)box.Left, (int)box.Top, (int)(box.Bottom - box.Top), (int)(box.Right - box.Left));
    faceRect.push_back(rect);
    faceTrackingId.push_back(trackingId);

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
# if defined(USE_THREAD)
  void _setFace(vector<vector<PointF> >& facePoint, vector<cv::Rect>& faceRect, vector<cv::Vec3f>& faceDirection, vector<vector<DetectionResult> >& faceProperty, vector<UINT64>& faceTrackingId, bool isColorSpace=true) {
    acquire();
    facePoint.clear();
    faceRect.clear();
    faceDirection.clear();
    faceProperty.clear();
    faceTrackingId.clear();
    setFace(isColorSpace);
    for (auto p: this->facePoint) facePoint.push_back(p);
    for (auto p: this->faceRect) faceRect.push_back(p);
    for (auto p: this->faceDirection) faceDirection.push_back(p);
    for (auto p: this->faceProperty) {
      vector<DetectionResult> v;
      for (auto q: p) v.push_back(q);
      facePoint.push_back(v);
    }
    for (auto p: this->faceTrackingId) faceTrackingId.push_back(p);
    release();
  }
# endif /* USE_THREAD */
#endif /* USE_FACE */

#if defined(USE_FACE)
  // ******** hdface ********
 private:
  BOOLEAN hdface_initialized = false;
  array<CComPtr<IHighDefinitionFaceFrameSource>,BODY_COUNT> hdfaceFrameSource;
  array<CComPtr<IHighDefinitionFaceFrameReader>,BODY_COUNT> hdfaceFrameReader;
  array<CComPtr<IFaceModelBuilder>,BODY_COUNT> faceModelBuilder;
  array<bool,BODY_COUNT> hdfaceModelValid;
  array<CComPtr<IFaceAlignment>,BODY_COUNT> faceAlignment;
  array<CComPtr<IFaceModel>,BODY_COUNT>  faceModel;
  array<array<float,FaceShapeDeformations::FaceShapeDeformations_Count>,BODY_COUNT> shapeUnits;
  array<CComPtr<IFaceModelData>,BODY_COUNT> faceModelData;
  array<UINT32,BODY_COUNT> hdfaceVertexCount;
  array<UINT64,BODY_COUNT> _hdfaceTrackingId;
  bool hdfaceModelFlag = false;
  void initializeHDFace() {
    if (!body_initialized) throw runtime_error("You must call setSkeleton() before calling setHDFace().");
    for (int count = 0; count < BODY_COUNT; count++) {
      ERROR_CHECK(CreateHighDefinitionFaceFrameSource(kinect,&hdfaceFrameSource[count]));
      ERROR_CHECK(hdfaceFrameSource[count]->OpenReader(&hdfaceFrameReader[count]));
      ERROR_CHECK(hdfaceFrameSource[count]->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None,&faceModelBuilder[count]));
      ERROR_CHECK(faceModelBuilder[count]->BeginFaceDataCollection());
      ERROR_CHECK(CreateFaceAlignment(&faceAlignment[count]));
      ERROR_CHECK(CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count,&shapeUnits[count][0],&faceModel[count]));
      hdfaceVertexCount[count]=0;
      ERROR_CHECK(GetFaceModelVertexCount(&hdfaceVertexCount[count]));
      _hdfaceTrackingId[count] = -1;
      hdfaceModelValid[count] = false;
    }
    hdface_initialized = true;
  }
  void updateHDFaceFrame(int idx) {
    CComPtr<IHighDefinitionFaceFrame> hdfaceFrame = nullptr;
    HRESULT ret = hdfaceFrameReader[idx]->AcquireLatestFrame(&hdfaceFrame);
    if (!SUCCEEDED(ret) || hdfaceFrame == nullptr) return;
    BOOLEAN tracked = false;
    ret = hdfaceFrame->get_IsFaceTracked(&tracked);
    if (!SUCCEEDED(ret) || !tracked) return;
    ret = hdfaceFrame->GetAndRefreshFaceAlignmentResult(faceAlignment[idx]);
    if (!SUCCEEDED(ret) || faceAlignment[idx] == nullptr) return;
    buildFaceModel(idx);
    vector<CameraSpacePoint> vertices;
    vertices.resize(hdfaceVertexCount[idx]);
    ERROR_CHECK(faceModel[idx]->CalculateVerticesForAlignment(faceAlignment[idx],hdfaceVertexCount[idx],&vertices[0]));
    hdfaceVertices.push_back(vertices);
    hdfaceTrackingId.push_back(_hdfaceTrackingId[idx]);
    hdfaceStatus.push_back(hdfaceGetStatus(idx));
  }
  void buildFaceModel(int idx) {
    if (hdfaceModelValid[idx]) return;
    if (hdfaceCollectionStatus(idx)) return;
    faceModelData[idx] = nullptr;
    HRESULT ret = faceModelBuilder[idx]->GetFaceData(&faceModelData[idx]);
    if (SUCCEEDED(ret) && faceModelData[idx] != nullptr) {
      if (hdfaceModelFlag) {
	faceModel[idx] = nullptr;
	ret = faceModelData[idx]->ProduceFaceModel(&faceModel[idx]);
	if (SUCCEEDED(ret) && faceModel[idx] != nullptr) {
	  hdfaceModelValid[idx] = true;
	}
      } else {
	hdfaceModelValid[idx] = true;
      }
    }
  }
 public:
  vector<vector<CameraSpacePoint> > hdfaceVertices;
  vector<UINT64> hdfaceTrackingId;
  vector<pair<int,int> > hdfaceStatus;
  bool setHDFaceModelFlag(bool flag=false) {
    bool old = hdfaceModelFlag;
    hdfaceModelFlag = flag;
    return old;
  }
  void setHDFace() {
    if (!hdface_initialized) initializeHDFace();
    hdfaceVertices.clear();
    hdfaceTrackingId.clear();
    hdfaceStatus.clear();
    for (int i = 0; i < skeleton.size(); i++) {
      int bid = skeletonId[i];
      UINT64 trackingId = skeletonTrackingId[i];
      if (trackingId != _hdfaceTrackingId[bid]) {
	ERROR_CHECK(hdfaceFrameSource[bid]->put_TrackingId(trackingId));
	_hdfaceTrackingId[bid] = trackingId;
	hdfaceModelValid[bid] = false;
      }
      updateHDFaceFrame(bid);
    }
  }
  int hdfaceCollectionStatus(int idx) {
    FaceModelBuilderCollectionStatus collection;
    ERROR_CHECK(faceModelBuilder[idx]->get_CollectionStatus(&collection));
    return collection;
  }
  int hdfaceCaptureStatus(int idx) {
    FaceModelBuilderCaptureStatus capture;
    ERROR_CHECK(faceModelBuilder[idx]->get_CaptureStatus(&capture));
    return capture;
  }
  pair<int,int> hdfaceGetStatus(int idx) {
    return pair<int,int>(hdfaceCollectionStatus(idx),hdfaceCaptureStatus(idx));
  }
  string hdfaceCollectionStatusToString(int st) {
    string s = "";
    if (st & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_MoreFramesNeeded) {
      s += "MoreFrameNeeded,";
    }
    if (st & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded) {
      s += "FrontViewFramesNeeded,";
    }
    if (st & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded) {
      s += "LeftViewsNeeded,";
    }
    if (st & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded) {
      s += "RightViewsNeeded,";
    }
    if (st & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded) {
      s += "TiltedUpViewsNeeded,";
    }
    return s;
  }
  string hdfaceCaptureStatusToString(int st) {
    string s = "";
    if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_GoodFrameCapture) {
      s = "GoodFrameCapture";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_OtherViewsNeeded) {
      s = "OtherViewsNeeded";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_LostFaceTrack) {
      s = "LostFaceTrack";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar) {
      s = "FaceTooFar";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear) {
      s = "FaceTooNear";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_MovingTooFast) {
      s = "MovingTooFast";
    } else if (st == FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_SystemError) {
      s = "SystemError";
    }
    return s;
  };
  pair<string,string> hdfaceStatusToString(pair<int,int>& status) {
    return pair<string,string>(hdfaceCollectionStatusToString(status.first),hdfaceCaptureStatusToString(status.second));
  }
# if defined(USE_THREAD)
  void _setHDFace(vector<vector<CameraSpacePoint> >& hdfaceVertices, vector<UINT64>& hdfaceTrackingId, vector<pair<int,int> >& hdfaceStatus) {
    acquire();
    hdfaceVertices.clear();
    hdfaceTrackingId.clear();
    hdfaceStatus.clear();
    setHdFace();
    for (auto p: this->hdfaceVertices) hdfaceVertices.push_back(p);
    for (auto p: this->hdfaceTrackingId) hdfaceTrackingId.push_back(p);
    for (auto p: this->hdfaceStatus) hdfaceStatus.push_back(p);
    release();
  }
  pair<string,string> _hdfaceStatusToString(pair<int,int>& status) {
    acquire();
    pair<string,string> p = hdfaceStatusToString(status);
    release();
    return p;
  }
# endif /* USE_THREAD*/
#endif /* USE_FACE */

#if defined(USE_GESTURE)
  // ******** gesture ********
 private:
  std::array<CComPtr<IVisualGestureBuilderFrameReader>, BODY_COUNT> gestureFrameReader;
  BOOLEAN gesture_initialized = false;
  std::vector<CComPtr<IGesture> > gestures;
  wstring gestureFile =  L"SampleDatabase.gbd";
  void initializeGesture() {
    if (!body_initialized) throw runtime_error("You must call setSkeleton() before calling setGesture().");
    for (int count = 0; count < BODY_COUNT; count++) {
      CComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
      ERROR_CHECK(CreateVisualGestureBuilderFrameSource(kinect, 0, &gestureFrameSource));
      ERROR_CHECK(gestureFrameSource->OpenReader(&gestureFrameReader[count]));
    }
    CComPtr<IVisualGestureBuilderDatabase> gestureDatabase;
    ERROR_CHECK(CreateVisualGestureBuilderDatabaseInstanceFromFile(gestureFile.c_str(), &gestureDatabase))
      UINT gestureCount;
    ERROR_CHECK(gestureDatabase->get_AvailableGesturesCount(&gestureCount));
    gestures.resize(gestureCount);
    ERROR_CHECK(gestureDatabase->get_AvailableGestures(gestureCount, &gestures[0]));
    for( int count = 0; count < BODY_COUNT; count++ ){
      CComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
      ERROR_CHECK(gestureFrameReader[count]->get_VisualGestureBuilderFrameSource(&gestureFrameSource));
      ERROR_CHECK(gestureFrameSource->AddGestures(gestureCount, &gestures[0].p));
      for (const CComPtr<IGesture> g : gestures){
        ERROR_CHECK(gestureFrameSource->SetIsEnabled(g, TRUE));
      }
    }
    gesture_initialized = true;
  }
  void gestureResult(const CComPtr<IVisualGestureBuilderFrame>& gestureFrame, const CComPtr<IGesture>& gesture, const UINT64 trackingId) {
    GestureType gestureType;
    ERROR_CHECK( gesture->get_GestureType( &gestureType ) );
    switch( gestureType ) {
    case GestureType::GestureType_Discrete: {
      CComPtr<IDiscreteGestureResult> gestureResult;
      ERROR_CHECK( gestureFrame->get_DiscreteGestureResult( gesture, &gestureResult ) );
      BOOLEAN detected;
      ERROR_CHECK( gestureResult->get_Detected( &detected ) );
      if(!detected) break;
      float confidence;
      ERROR_CHECK( gestureResult->get_Confidence( &confidence ) );
      pair<CComPtr<IGesture>,float> p(gesture,confidence);
      discreteGesture.push_back(p);
      discreteGestureTrackingId.push_back(trackingId);
      break;
    }
    case GestureType::GestureType_Continuous: {
      CComPtr<IContinuousGestureResult> gestureResult;
      ERROR_CHECK(gestureFrame->get_ContinuousGestureResult(gesture, &gestureResult));
      float progress;
      ERROR_CHECK(gestureResult->get_Progress(&progress));
      pair<CComPtr<IGesture>,float> p(gesture,progress);
      continuousGesture.push_back(p);
      continuousGestureTrackingId.push_back(trackingId);
      break;
    }
    default:
      break;
    }
  }
  wstring trim(const wstring& str) {
    const wstring::size_type last = str.find_last_not_of(L" ");
    if(last == wstring::npos) throw runtime_error("failed " __FUNCTION__);
    return str.substr(0,last+1);
  }
 public:
  vector<pair<CComPtr<IGesture>,float> > discreteGesture;
  vector<pair<CComPtr<IGesture>,float> > continuousGesture;
  vector<UINT64> discreteGestureTrackingId;
  vector<INT64> continuousGestureTrackingId;
  void setGesture() {
    if (!gesture_initialized) initializeGesture();
    discreteGesture.clear();
    continuousGesture.clear();
    discreteGestureTrackingId.clear();
    continuousGestureTrackingId.clear();
    for (int i=0; i<skeleton.size(); i++) {
      int bid = skeletonId[i];
      UINT64 trackingId = skeletonTrackingId[i];
      CComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
      ERROR_CHECK(gestureFrameReader[bid]->get_VisualGestureBuilderFrameSource(&gestureFrameSource));
      gestureFrameSource->put_TrackingId(trackingId);
      CComPtr<IVisualGestureBuilderFrame> gestureFrame;
      HRESULT ret = gestureFrameReader[bid]->CalculateAndAcquireLatestFrame(&gestureFrame);
      if (FAILED(ret)) continue;
      BOOLEAN tracked;
      ERROR_CHECK(gestureFrame->get_IsTrackingIdValid(&tracked));
      if(!tracked) continue;
      for(const CComPtr<IGesture> g : gestures) {
        gestureResult(gestureFrame, g, trackingId);
      }
    }
  }
  void setGestureFile(wstring name) {
    gestureFile  = name;
  }
  string gesture2string(const CComPtr<IGesture>& gesture) {
    wstring buffer(BUFSIZ, L'\0');
    ERROR_CHECK( gesture->get_Name(BUFSIZ, &buffer[0]));
    const wstring temp = trim(&buffer[0]);
    const string name(temp.begin(), temp.end());
    return name;
  }
# if defined(USE_THREAD)
  void _setGesture(vector<pair<CComPtr<IGesture>,float> >& discreteGesture,
                   vector<pair<CComPtr<IGesture>,float> >& continuousGesture,
                   vector<UINT64>& discreteGestureTrackingId,
                   vector<INT64>& continuousGestureTrackingId) {
    acquire();
    discreteGesture.clear();
    continuousGesture.clear();
    discreteGestureTrackingId.clear();
    continuousGestureTrackingId.clear();
    setGesture();
    for (auto p: this->discreteGesture) discreteGesture.push_back(p);
    for (auto p: this->continuousGesture) continuousGesture.push_back(p);
    for (auto p: this->discreteGestureTrackingId) discreteGestureTrackingId.push_back(p);
    for (auto p: this->continuousGestureTrackingId) continuousGestureTrackingId.push_back(p);
    release();
  }
  string _gesture2string(const CComPtr<IGesture>& gesture) {
    acquire();
    string name = getsutre2string(gesture);
    release();
    return name;
  }
# endif /* USE_THREAD */
#endif

 public:
  NtKinect() { initialize();  }
  ~NtKinect() {
    if (kinect != nullptr) kinect->Close();
  }
  cv::Rect boundingBoxInColorSpace(vector<CameraSpacePoint>& v) {
    int minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN;
    for (CameraSpacePoint sp: v) {
      ColorSpacePoint cp;
      coordinateMapper->MapCameraPointToColorSpace(sp,&cp);
      if (minX > (int)cp.X) minX = (int)cp.X;
      if (maxX < (int)cp.X) maxX = (int)cp.X;
      if (minY > (int)cp.Y) minY = (int)cp.Y;
      if (maxY < (int)cp.Y) maxY = (int)cp.Y;
    }
    if (maxX<=minX || maxY<=minY) return cv::Rect(0,0,0,0);
    return cv::Rect(minX,minY,maxX-minX,maxY-minY);
  }
};
