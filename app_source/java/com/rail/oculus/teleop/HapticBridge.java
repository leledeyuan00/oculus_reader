package com.rail.oculus.teleop;

public final class HapticBridge {
    static {
        try { System.loadLibrary("vrinputstandard"); } catch (Throwable ignored) {}
    }
    public static native void nativePostSimple( float amplitude_l, float amplitude_r);
}
