#ifndef FILTER
#define FILTER



#ifdef __cplusplus
extern "C" {
#endif

void* LowPassFilter_Create_A(float a);
float LowPassFilter_Process(void* handle, float value);
typedef void * Filter;

#ifdef __cplusplus 
}
#endif


#endif
