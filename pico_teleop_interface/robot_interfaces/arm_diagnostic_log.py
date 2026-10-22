"""双臂链路专用 JSONL 日志；只依赖标准库，可随桥接脚本上传 HDU。"""

import json
import logging
import math
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import sys
import threading
import time
import uuid


def _json_values(value):
    # NumPy 数组和标量先转成标准 Python 类型；本模块仍不依赖 NumPy。
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _json_values(tolist())
    # 部分驱动用 NaN 表示无效反馈，保留该帧并用 JSON null 标记这些字段。
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (list, tuple)):
        return [_json_values(item) for item in value]
    if isinstance(value, dict):
        return {key: _json_values(item) for key, item in value.items()}
    return value


class ArmDiagnosticLog:
    def __init__(self, role, directory=None):
        directory = Path(directory) if directory else (
            Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
            / "pico_teleop/arm_diagnostics")
        directory.mkdir(parents=True, exist_ok=True)
        self.session = f"{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}_{uuid.uuid4().hex[:6]}"
        self.path = directory / f"{role}_{self.session}.jsonl"
        self.role = role
        self._queue = queue.Queue(maxsize=4096)
        self.dropped = 0
        self._closed = False
        self._handler = RotatingFileHandler(
            self.path, maxBytes=40 * 1024 * 1024, backupCount=4, encoding="utf-8")
        self._thread = threading.Thread(target=self._drain, name="arm-diagnostics", daemon=True)
        self._thread.start()
        self.write("program_started")
        print(f"Arm diagnostic log: {self.path}", flush=True)

    def write(self, event, **fields):
        if self._closed:
            return
        row = dict(role=self.role, session=self.session, event=event,
                   wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(),
                   dropped_total=self.dropped, **fields)
        try:
            if event == "program_stopped":
                # 仅退出阶段允许短暂等待，尽量保留最终停止记录。
                self._queue.put(row, timeout=1.)
            else:
                self._queue.put_nowait(row)
        except queue.Full:
            # 不阻塞控制线程；后续记录中的累计值明确标记日志丢帧。
            self.dropped += 1

    def _drain(self):
        try:
            while True:
                row = self._queue.get()
                if row is None:
                    break
                try:
                    message = json.dumps(_json_values(row), ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                    self._handler.handle(logging.LogRecord(
                        "arm_diagnostics", logging.INFO, "", 0, message, (), None))
                except Exception as exc:
                    print(f"Arm diagnostic log write failed: {exc}", file=sys.stderr)
        finally:
            self._handler.close()

    def close(self):
        if self._closed:
            return
        self.write("program_stopped", dropped_total_at_stop=self.dropped)
        self._closed = True
        try:
            self._queue.put(None, timeout=1.)
        except queue.Full:
            print("Arm diagnostic log could not drain before shutdown", file=sys.stderr)
            return
        self._thread.join(timeout=5.)
        if self._thread.is_alive():
            print("Arm diagnostic log is still draining at shutdown", file=sys.stderr)
