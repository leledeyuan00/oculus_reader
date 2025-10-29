package com.rail.oculus.teleop;

import org.json.JSONObject;
import java.io.*;
import java.net.*;
import java.util.concurrent.*;

public final class HapticServer {
    private static final int PORT = 7777;
    private static volatile boolean running = false;
    private static ServerSocket server;

    static void start() {
        if (running) return;
        running = true;

        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    server = new ServerSocket(PORT, 0, InetAddress.getByName("127.0.0.1"));
                    while (running) {
                        Socket s = server.accept();
                        handleClient(s);
                    }
                } catch (Throwable ignored) {}
            }
        }, "HapticServer-Accept").start();
    }

    static void stop() {
        running = false;
        try {
            if (server != null) server.close();
        } catch (Throwable ignored) {}
    }

    private static void handleClient(final Socket s){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try (Socket sock = s) {
                    BufferedReader r = new BufferedReader(new InputStreamReader(sock.getInputStream()));
                    PrintWriter  w = new PrintWriter(sock.getOutputStream(), true);
                    String line;
                    while ((line = r.readLine()) != null) {
                        try {
                            JSONObject j = new JSONObject(line);
                            String type = j.optString("type","simple");
                            if ("simple".equals(type)) {
                                String hand = j.optString("hand","right");
                                float amp   = (float) j.optDouble("intensity", 0.6);
                                int durMs   = j.optInt("duration_ms", 80);
                                HapticBridge.nativePostSimple(
                                        "left".equalsIgnoreCase(hand) ? 0 : 1,
                                        Math.max(0f, Math.min(1f, amp)),
                                        Math.max(10, durMs)
                                );
                                w.println("{\"ok\":true}");
                            } else {
                                // 预留 buffer 模式：后续再加
                                w.println("{\"ok\":false,\"err\":\"unsupported_type\"}");
                            }
                        } catch (Throwable e) {
                            w.println("{\"ok\":false,\"err\":\""+e.getMessage()+"\"}");
                        }
                    }
                } catch (Throwable ignored) {}
            }
        }, "HapticServer-Client").start();
    }
}
