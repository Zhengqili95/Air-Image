#pragma once

#include <stdint.h>

#if defined(_WIN32)
#  if defined(MEDIAPIPE_HAND_BRIDGE_EXPORTS)
#    define MPHB_API __declspec(dllexport)
#  else
#    define MPHB_API __declspec(dllimport)
#  endif
#else
#  define MPHB_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mphb_landmark {
    float x;
    float y;
    float z;
} mphb_landmark;

typedef struct mphb_result {
    int detected;
    int landmark_count;
    mphb_landmark landmarks[21];
} mphb_result;

MPHB_API void* mphb_create(const char* model_path, char* error_message, int error_capacity);
/* Same as mphb_create, with Hand Landmarker's three confidence thresholds (each in (0, 1]).
   mphb_create uses MediaPipe's defaults of 0.5. Callers look this export up optionally, so an
   older DLL without it still loads. */
MPHB_API void* mphb_create_with_options(const char* model_path,
                                        float min_hand_detection_confidence,
                                        float min_hand_presence_confidence,
                                        float min_tracking_confidence,
                                        char* error_message,
                                        int error_capacity);
MPHB_API int mphb_detect_rgb(void* handle,
                             const unsigned char* rgb,
                             int width,
                             int height,
                             int stride_bytes,
                             int64_t timestamp_ms,
                             mphb_result* result,
                             char* error_message,
                             int error_capacity);
MPHB_API void mphb_destroy(void* handle);

#ifdef __cplusplus
}
#endif
