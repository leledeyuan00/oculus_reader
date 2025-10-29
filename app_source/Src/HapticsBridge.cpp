//
// Created by leled on 2025-10-28.
//

#include "HapticsBridge.h"
#include <jni.h>
#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>

// ---------------- state ----------------

static std::mutex               g_mtx;
static std::queue<HapticSimple> g_queue_l;
static std::queue<HapticSimple> g_queue_r;

HapticSimple g_haptic_l = {0.5, 0};
HapticSimple g_haptic_r = {0.5, 0};

// JNI：Java -> 入队
extern "C"
JNIEXPORT void JNICALL
Java_com_rail_oculus_teleop_HapticBridge_nativePostSimple(
        JNIEnv*, jclass, jint handId, jfloat amplitude, jint durationMs)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    if ((int)handId == 0) // left_hand
    {
        g_queue_l.push({ (float)amplitude, (int)durationMs });
        g_haptic_l = { (float)amplitude, (int)durationMs };
    }
    else
    {
        g_queue_r.push({ (float)amplitude, (int)durationMs });
        g_haptic_r = { (float)amplitude, (int)durationMs };
    }
}



//extern "C"
//void Haptics_Tick(ovrMobile* ovr, int frameDeltaMs)
//{
//    // 取出所有待执行命令
//    std::vector<HapticSimple> cmds;
//    {
//        std::lock_guard<std::mutex> lk(g_mtx);
//        while(!g_queue.empty()) { cmds.push_back(g_queue.front()); g_queue.pop(); }
//    }
//    if (cmds.empty()) return;
//
//    // 本帧执行：每条命令打一记（simple 本质就是短脉冲）
//    const int stepMs = std::max(16, frameDeltaMs);
//    for (auto &c : cmds) {
//        const ovrDeviceID dev = pickDev(c.hand);
//        applySimpleBurst(ovr, dev, c.amp, c.durMs, stepMs);
//    }
//}

HapticSimple Haptics_Queue_Left(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_haptic_l;
}

HapticSimple Haptics_Queue_Right(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_haptic_r;
}
