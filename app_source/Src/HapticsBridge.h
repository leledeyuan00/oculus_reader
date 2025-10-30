#pragma once
#include "VrApi_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct HapticSimple {
    float amp_l;      // 0..1
    float amp_r;
};

HapticSimple Haptics_Query();

#ifdef __cplusplus
}
#endif
