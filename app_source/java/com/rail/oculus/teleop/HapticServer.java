package com.rail.oculus.teleop;

import android.util.Log;
import org.json.JSONObject;
import java.io.*;
import java.net.*;

public final class HapticServer {
    private static final String TAG = "HAPTIC";
    private static final int PORT = 7777;
    private static volatile boolean running = false;
    private static ServerSocket server = null;

    static void start() {
        if (running) {
            Log.i(TAG, "Server already running");
            return;
        }
        running = true;
        new Thread(new Runnable() {
            @Override public void run() {
                try {
                    // 绑定到 127.0.0.1，配合 adb reverse 使用
                    server = new ServerSocket(PORT, 0, InetAddress.getByName("127.0.0.1"));
                    Log.i(TAG, "Server starting on 127.0.0.1:" + PORT);
                    while (running) {
                        final Socket s = server.accept();
                        Log.i(TAG, "Client connected: " + s);
                        handleClient(s);
                    }
                } catch (Throwable e) {
                    Log.e(TAG, "Server start error", e);
                } finally {
                    Log.i(TAG, "Server thread exit");
                }
            }
        }, "HapticServer-Accept").start();
    }

    static void stop() {
        running = false;
        try {
            if (server != null) {
                server.close();
                server = null;
            }
        } catch (Throwable ignored) {}
        Log.i(TAG, "Server stopped");
    }

    private static void handleClient(final Socket s) {
        new Thread(new Runnable() {
            @Override public void run() {
                try {
                    Socket sock = s;
                    BufferedReader r = new BufferedReader(new InputStreamReader(sock.getInputStream()));
                    PrintWriter  w = new PrintWriter(sock.getOutputStream(), true);
                    String line;
                    while ((line = r.readLine()) != null) {
//                        Log.i(TAG, "RX: " + line);
                        try {
                            JSONObject j = new JSONObject(line);
                            String type = j.optString("type", "simple");

                            // check
                            if ("ping".equals(type)) {
                                w.println("{\"ok\":true,\"pong\":1}");
                                continue;
                            }

                            if ("simple".equals(type)) {
                                float amp_l   = (float) j.optDouble("amp_l", 0.0);
                                float amp_r   = (float) j.optDouble("amp_r", 0.0);
                                // clip
                                if (amp_l < 0f) amp_l = 0f; else if (amp_l > 1f) amp_l = 1f;
                                if (amp_r < 0f) amp_r = 0f; else if (amp_r > 1f) amp_r = 1f;

                                // JNI inject C++
                                HapticBridge.nativePostSimple(amp_l, amp_r);
                            }
                        } catch (Throwable e) {
                            Log.e(TAG, "Parse/Handle error", e);
                            w.println("{\"ok\":false,\"err\":\""+e.getMessage()+"\"}");
                        }
                    }
                } catch (Throwable e) {
                    Log.e(TAG, "Client error", e);
                } finally {
                    try { s.close(); } catch (Throwable ignored) {}
                    Log.i(TAG, "Client disconnected");
                }
            }
        }, "HapticServer-Client").start();
    }
}
