#ifdef NTKINECTDLL_EXPORTS
#define NTKINECTDLL_API __declspec(dllexport)
#else
#define NTKINECTDLL_API __declspec(dllimport)
#endif

extern "C" {
  NTKINECTDLL_API void* getKinect(void);
  NTKINECTDLL_API int faceDirection(void* ptr, float* dir);
}
