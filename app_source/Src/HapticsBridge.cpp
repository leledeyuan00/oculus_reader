//
// Created by leled on 2025-10-28.
//

#include "HapticsBridge.h"
#include <jni.h>
#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>
#include <android/log.h>
// ---------------- state ----------------

static std::mutex               g_mtx;

HapticSimple g_haptics = {0.0, 0.0};

extern "C"
JNIEXPORT void JNICALL
Java_com_rail_oculus_teleop_HapticBridge_nativePostSimple(
        JNIEnv*, jclass, jfloat amplitude_l, jfloat amplitude_r)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_haptics = { (float)amplitude_l, (float)amplitude_r };
}


HapticSimple Haptics_Query(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_haptics;
}

