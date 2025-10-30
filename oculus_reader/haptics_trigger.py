# haptics_client.py
import json, errno
import socket
import subprocess
import sys
import time
from typing import Optional

class HapticClient:
    """
    PC ->(adb forward)-> Device:7777 -> App(HapticServer)
    """
    def __init__(self, pc_port: int = 45043, device_port: int = 7777,
                 adb_serial: Optional[str] = None, connect_timeout=3.0):
        self.addr = ("127.0.0.1", pc_port)
        self.timeout = float(connect_timeout)
        self.pc_port = pc_port
        self.device_port = device_port
        self.adb_serial = adb_serial
        self.sock: Optional[socket.socket] = None

    # ---------- ADB helpers ----------
    def _adb(self, *args, check=True, capture_output=True, text=True):
        cmd = ["adb"]
        if self.adb_serial:
            cmd += ["-s", self.adb_serial]
        cmd += list(args)
        try:
            return subprocess.run(cmd, check=check, capture_output=capture_output, text=text)
        except FileNotFoundError:
            raise RuntimeError("adb not found. Please install Android Platform Tools and ensure 'adb' is in your PATH.")

    def ensure_forward(self) -> int:
        # remove existing forward if any
        try:
            self._adb("forward", "--remove", f"tcp:{self.pc_port}", check=False)
        except Exception:
            pass
        # create forward
        self._adb("forward", f"tcp:{self.pc_port}", f"tcp:{self.device_port}")
        # verify
        out = self._adb("forward", "--list").stdout.strip()
        if f"tcp:{self.pc_port}\t tcp:{self.device_port}" not in out.replace(" ", ""):
            if f"tcp:{self.pc_port}" not in out:
                raise RuntimeError(f"adb forward failed: current list: \n{out}")
        return self.pc_port

    # ---------- TCP connection ----------
    def _connect(self):
        if self.sock:
            return
        # Ensure adb forward is set up
        self.ensure_forward()
        # Create socket
        s = socket.create_connection(self.addr, timeout=self.timeout)
        s.settimeout(self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # TCP keepalive（Linux）
        s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "TCP_KEEPIDLE"):
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        self.sock = s

    def _safe_sendall(self, data: bytes):
        try:
            self._connect()
            self.sock.sendall(data)
        except OSError as e:
            if e.errno in (errno.EPIPE, errno.ECONNRESET, errno.ETIMEDOUT) or isinstance(e, BrokenPipeError):
                self.close()
                self._connect()
                self.sock.sendall(data)
            else:
                raise

    def close(self):
        if self.sock:
            try: self.sock.shutdown(socket.SHUT_RDWR)
            except Exception: pass
            try: self.sock.close()
            except Exception: pass
            self.sock = None

    # ---------- Protocol ----------

    def ping(self) -> bool:
        try:
            resp = self._send_jsonl({"type": "ping"})
            return '"ok":true' in resp
        except Exception:
            return False

    
    def send_haptics(self, amp_l: float = 0.0, amp_r: float = 0.0):
        amp_l = max(0.0, min(1.0, float(amp_l)))
        amp_r = max(0.0, min(1.0, float(amp_r)))
        payload = {
            "type": "simple",
            "amp_l": amp_l,
            "amp_r": amp_r,
        }
        line = (json.dumps(payload) + "\n").encode("utf-8")
        self._safe_sendall(line)
    
    # ---------- Context manager ----------

    def __exit__(self, exc_type, exc, tb):
        self.close()