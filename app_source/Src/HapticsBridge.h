#pragma once
#include "VrApi_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct HapticSimple {
    float amp;      // 0..1
    int durMs;      // >=10
};
// 每帧由 VR 线程调用（ovr 为有效的 ovrMobile*）
//void Haptics_Tick(ovrMobile* ovr, int frameDeltaMs);

// 在你知道当前左右手的 ovrDeviceID 时调用（可随时更新）
//void Haptics_SetDeviceIds(ovrDeviceID leftDev, ovrDeviceID rightDev);

HapticSimple Haptics_Queue_Left();
HapticSimple Haptics_Queue_Right();

#ifdef __cplusplus
}
#endif
