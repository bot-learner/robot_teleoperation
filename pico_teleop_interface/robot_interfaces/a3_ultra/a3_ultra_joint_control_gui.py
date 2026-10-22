"""A3 Ultra 双臂单关节实机测试界面。

运行：
  conda activate a3_teleop
  python pico_teleop_interface/robot_interfaces/a3_ultra/a3_ultra_joint_control_gui.py

启动前必须确认机器人已可靠吊装、急停可用、官方 motion_control 已停止，
并保持 usage/start_a3_ultra_state_bridge.sh 正在运行。
连接成功后，程序会规划避障路径并限速归位。滑条松开时规划并执行新目标。
"""

from __future__ import annotations

from pathlib import Path
import queue
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

if __package__:
    from .a3_ultra_real_robot import (
        A3UltraRealRobot,
        ARM_JOINT_LIMITS,
        ARM_JOINT_NAMES,
        DEFAULT_ARM_JOINT_POSITIONS,
    )
else:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from pico_teleop_interface.robot_interfaces.a3_ultra.a3_ultra_real_robot import (
        A3UltraRealRobot,
        ARM_JOINT_LIMITS,
        ARM_JOINT_NAMES,
        DEFAULT_ARM_JOINT_POSITIONS,
    )


class A3UltraJointControlGui:
    """设置单关节目标，双臂联合避障；绕行过程中其他关节也可能运动。"""

    UPDATE_INTERVAL_MS = 20
    TEST_MAX_SPEED_RAD_S = 0.2

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("A3 Ultra Joint Control Test")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        dds_profile = Path(__file__).resolve().parents[3] / "usage/ros_dds_configuration.xml"
        self.robot = A3UltraRealRobot(dds_profile=dds_profile)
        self.robot.max_joint_speed = self.TEST_MAX_SPEED_RAD_S
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.feedback: dict[str, float] = {}
        self.targets: dict[str, float] = {}
        self.connected = False
        self.enabled = False
        self.holding = False
        self.closing = False
        self.planning = False
        self.moving = False

        self.joint = tk.StringVar(value=ARM_JOINT_NAMES[3])
        self.angle = tk.DoubleVar(value=0.0)
        self.current_text = tk.StringVar(value="--")
        self.target_text = tk.StringVar(value="--")
        lower, upper = ARM_JOINT_LIMITS[self.joint.get()]
        self.minimum_text = tk.StringVar(value=f"{lower:+.3f} rad")
        self.maximum_text = tk.StringVar(value=f"{upper:+.3f} rad")
        self.status_text = tk.StringVar(value="Connecting to robot feedback...")
        self._build_widgets()

        threading.Thread(target=self._connect, daemon=True).start()
        self.root.after(100, self._process_events)
        self.root.after(self.UPDATE_INTERVAL_MS, self._update)

    def _build_widgets(self) -> None:
        frame = ttk.Frame(self.root, padding=16)
        frame.grid()

        ttk.Label(frame, text="Joint").grid(row=0, column=0, sticky="w")
        self.joint_box = ttk.Combobox(
            frame,
            textvariable=self.joint,
            values=ARM_JOINT_NAMES,
            state="readonly",
            width=34,
        )
        self.joint_box.grid(row=0, column=1, columnspan=2, padx=(12, 0), sticky="ew")
        self.joint_box.bind("<<ComboboxSelected>>", self._joint_changed)

        ttk.Label(frame, text="Current").grid(row=1, column=0, pady=(14, 0), sticky="w")
        ttk.Label(frame, textvariable=self.current_text).grid(
            row=1, column=1, columnspan=2, pady=(14, 0), sticky="w"
        )
        ttk.Label(frame, text="Target").grid(row=2, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.target_text).grid(
            row=2, column=1, columnspan=2, sticky="w"
        )

        lower, upper = ARM_JOINT_LIMITS[self.joint.get()]
        ttk.Label(frame, textvariable=self.minimum_text).grid(
            row=3, column=0, pady=(14, 0)
        )
        self.scale = ttk.Scale(
            frame,
            from_=lower,
            to=upper,
            variable=self.angle,
            command=self._slider_changed,
            length=360,
            state="disabled",
        )
        self.scale.grid(row=3, column=1, pady=(14, 0), padx=8)
        self.scale.bind("<ButtonRelease-1>", self._send_target)
        self.scale.bind("<KeyRelease>", self._send_target)
        ttk.Label(frame, textvariable=self.maximum_text).grid(
            row=3, column=2, pady=(14, 0)
        )

        self.enable_button = ttk.Button(
            frame, text="Enable control", command=self._enable, state="disabled"
        )
        self.enable_button.grid(row=4, column=0, columnspan=2, pady=(18, 0), sticky="ew")
        self.disable_button = ttk.Button(
            frame, text="Stop / hold", command=self._disable, state="disabled"
        )
        self.disable_button.grid(row=4, column=2, pady=(18, 0), padx=(8, 0))

        self.status_label = tk.Label(
            frame, textvariable=self.status_text, anchor="w", fg="#9a6700",
            wraplength=560, justify="left",
        )
        self.status_label.grid(row=5, column=0, columnspan=3, pady=(14, 0), sticky="ew")

    def _connect(self) -> None:
        try:
            self.robot.connect()
            if self.closing:
                self.robot.close()
                return
            feedback = self.robot.read_all_arm_joint_positions()
            self.events.put(("planning_home", None))
            self.robot.move_to_default_pose()
            self.events.put(("connected", feedback))
        except Exception as exc:
            print(f"Startup failed: {exc}", file=sys.stderr, flush=True)
            self.robot.close()
            self.events.put(("error", exc))

    def _process_events(self) -> None:
        if self.closing:
            return
        try:
            event, value = self.events.get_nowait()
        except queue.Empty:
            event, value = "", None

        if event == "planning_home":
            self.status_text.set("PLANNING — collision-free return to default pose")
        elif event == "connected":
            self.feedback = dict(value)  # type: ignore[arg-type]
            self.targets = dict(zip(
                ARM_JOINT_NAMES, DEFAULT_ARM_JOINT_POSITIONS, strict=True
            ))
            self.connected = True
            self.holding = True
            self.moving = True
            self._joint_changed()
            self.disable_button.configure(state="normal")
            self.status_text.set(
                "FOLLOWING PLANNED PATH — returning to default pose"
            )
            self.status_label.configure(fg="#1a7f37")
        elif event == "motion_started":
            self.planning = False
            self.moving = True
            self.disable_button.configure(state="normal")
            self.status_text.set("FOLLOWING PLANNED PATH — keep program running")
        elif event == "motion_error":
            self.planning = self.moving = False
            self.scale.configure(state="normal" if self.enabled else "disabled")
            self.disable_button.configure(state="normal" if self.enabled else "disabled")
            self.status_text.set(f"PLANNING ERROR: {value}")
            self.status_label.configure(fg="#cf222e")
        elif event == "error":
            self.status_text.set(f"CONNECTION ERROR: {value}")
            self.status_label.configure(fg="#cf222e")
        self.root.after(100, self._process_events)

    def _joint_changed(self, _event: object = None) -> None:
        name = self.joint.get()
        lower, upper = ARM_JOINT_LIMITS[name]
        self.scale.configure(from_=lower, to=upper)
        self.minimum_text.set(f"{lower:+.3f} rad")
        self.maximum_text.set(f"{upper:+.3f} rad")
        if name in self.feedback:
            current = self.feedback[name]
            target = self.targets.get(name, current)
            self.angle.set(target)
            self.current_text.set(f"{current:+.4f} rad")
            self.target_text.set(f"{target:+.4f} rad")

    def _slider_changed(self, _value: object = None) -> None:
        if not self.enabled:
            return
        name = self.joint.get()
        lower, upper = ARM_JOINT_LIMITS[name]
        self.targets[name] = min(upper, max(lower, self.angle.get()))
        self.target_text.set(f"{self.targets[name]:+.4f} rad")

    def _send_target(self, _event: object = None) -> None:
        """松开滑条后只提交一次目标，后台规划期间不阻塞 Tk 界面。"""

        if not self.enabled or self.planning or self.moving or self.closing:
            return
        positions = tuple(self.targets[name] for name in ARM_JOINT_NAMES)
        self.planning = True
        self.scale.configure(state="disabled")
        self.disable_button.configure(state="disabled")
        self.status_text.set("PLANNING — holding current pose")
        self.status_label.configure(fg="#9a6700")

        def plan() -> None:
            try:
                self.robot.send_joint_positions(ARM_JOINT_NAMES, positions)
                self.events.put(("motion_started", None))
            except Exception as exc:
                self.events.put(("motion_error", exc))

        threading.Thread(target=plan, daemon=True).start()

    def _enable(self) -> None:
        if not self.connected or not self.feedback or self.moving or self.planning:
            return
        if not messagebox.askyesno(
            "Enable real robot control",
            "Confirm reliable suspension, available emergency stop, stopped "
            "motion_control, and an active body-drive controller.\n\n"
            "Enable planned joint-target control? Other joints may move during detours.",
            parent=self.root,
        ):
            return

        self.targets = dict(self.feedback)
        self.angle.set(self.feedback[self.joint.get()])
        self.enabled = True
        self.holding = False
        self.joint_box.configure(state="disabled")
        self.scale.configure(state="normal")
        self.enable_button.configure(state="disabled")
        self.disable_button.configure(state="normal")
        self.status_text.set("CONTROL ENABLED — release slider to plan and move")
        self.status_label.configure(fg="#cf222e")
        self._slider_changed()

    def _disable(self) -> None:
        if self.planning:
            return
        if self.enabled or self.moving:
            self.robot.stop()
            self.holding = True
        self.moving = False
        self.enabled = False
        self.targets = dict(self.feedback)
        self._joint_changed()
        self.joint_box.configure(state="readonly")
        self.scale.configure(state="disabled")
        self.enable_button.configure(state="normal" if self.connected else "disabled")
        self.disable_button.configure(state="disabled")
        self.status_text.set("STOPPED — holding last command; keep program running")
        self.status_label.configure(fg="#1a7f37")

    def _update(self) -> None:
        if self.closing:
            return
        if self.connected:
            try:
                self.feedback = self.robot.read_all_arm_joint_positions()
                current = self.feedback[self.joint.get()]
                self.current_text.set(f"{current:+.4f} rad")
                if self.moving and not self.robot.is_moving:
                    self.moving = False
                    self.holding = True
                    self.scale.configure(state="normal" if self.enabled else "disabled")
                    self.disable_button.configure(state="normal" if self.enabled else "disabled")
                    self.enable_button.configure(state="disabled" if self.enabled else "normal")
                    self.status_text.set("HOLDING TARGET — release slider to plan next move")
                    self.status_label.configure(fg="#1a7f37")
                if not self.enabled and not self.holding:
                    self.angle.set(current)
                    self.target_text.set(f"{current:+.4f} rad")
            except Exception as exc:
                self.enabled = False
                self.planning = self.moving = False
                self.scale.configure(state="disabled")
                self.enable_button.configure(state="disabled")
                self.disable_button.configure(state="disabled")
                self.status_text.set(f"CONTROL ERROR: {exc}")
                self.status_label.configure(fg="#cf222e")
        self.root.after(self.UPDATE_INTERVAL_MS, self._update)

    def _close(self) -> None:
        if self.connected and (self.enabled or self.holding or self.planning or self.moving) and not messagebox.askyesno(
            "Stop real robot command stream",
            "Closing this program stops the 100 Hz command stream. The robot may "
            "return to its controller's default pose.\n\nClose anyway?",
            parent=self.root,
        ):
            return
        self.closing = True
        self.robot.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    A3UltraJointControlGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
