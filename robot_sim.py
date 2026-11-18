#!/usr/bin/env python3
# 需要 Python 3.8 以上，只用標準函式庫
"""QuadDrop 機器狗端模擬器：假裝自己是一台真機，示範機器狗要怎麼呼叫後端。

只用 Python 標準函式庫，給機器狗團隊當參考實作；實際的狗把「假裝移動」換成
真正的導航（例：ROS 2 Nav2），其他流程照抄即可。說明文件：docs/robot_api.md

每秒做一輪：
  1. POST /robot/robots/{id}/state       回報位置與電量（也是心跳，超過 15 秒沒回報會被當成離線）
  2. GET  /robot/robots/{id}/tasks/current 看有沒有要執行的導航任務
  3. 有任務就沿 route 一站一站走：
       - 開始時   POST .../tasks/{taskId}/status  {"status": "in_progress"}
       - 每過一站 POST .../tasks/{taskId}/status  {"status": "in_progress", "reachedSeq": n}
       - 到終點   POST .../tasks/{taskId}/status  {"status": "done"}
     任務不見了（App 取消）或回 409 TASK_CANCELLED：立刻停下
  4. 失敗（--fail-at-seq）：POST .../status {"status": "failed", "errorCode": ...}，並上傳錯誤紀錄

用法：
  python3 robot_sim.py --base-url https://api.example.com/api/v1 \\
      --robot-id 5 --token <這台狗的 ROBOT_DEVICE_TOKENS> --start 4f-08

  --start 可以是點位 ID（從 GET /robot/points 查座標）或 "x,y"（地圖 1 的座標，公尺）。
"""

from __future__ import annotations  # 讓 Python 3.8／3.9 也能跑

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from typing import Any

ARRIVE_RADIUS_M = 0.3  # 離一站多近算走到


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str) -> None:
        super().__init__(f"{status} {code}: {message}")
        self.status, self.code, self.message = status, code, message


class RobotApi:
    """機器狗端 API 的最小客戶端：所有請求都帶 Authorization: Bearer <這台狗的 token>。"""

    def __init__(self, base_url: str, robot_id: int, token: str | None, timeout: float = 5.0) -> None:
        self.base = base_url.rstrip("/") + "/robot"
        self.robot_id = robot_id
        self.token = token
        self.timeout = timeout

    def _request(self, method: str, path: str, body: dict[str, Any] | None = None) -> Any:
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method=method)
        req.add_header("Accept", "application/json")
        if data is not None:
            req.add_header("Content-Type", "application/json")
        if self.token:
            req.add_header("Authorization", f"Bearer {self.token}")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as res:
                text = res.read().decode()
                return json.loads(text) if text else None
        except urllib.error.HTTPError as exc:
            # 錯誤格式：{"error": {"code": "...", "message": "..."}}
            try:
                err = json.loads(exc.read().decode())["error"]
                raise ApiError(exc.code, err.get("code", "?"), err.get("message", "")) from None
            except (ValueError, KeyError, TypeError):
                raise ApiError(exc.code, "HTTP_ERROR", str(exc)) from None

    def me(self, path: str = "") -> str:
        return f"/robots/{self.robot_id}{path}"

    # ---- 機器狗會用到的 API ----
    def points(self) -> list[dict[str, Any]]:
        return self._request("GET", "/points")

    def report_state(self, map_id: int, x: float, y: float, theta: float, battery: float | None) -> dict[str, Any]:
        body: dict[str, Any] = {
            "mapId": map_id,
            "pose": {"x": round(x, 3), "y": round(y, 3), "theta": round(theta, 4)},
            "observedAt": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        }
        if battery is not None:
            body["batteryPercent"] = round(battery, 1)
        return self._request("POST", self.me("/state"), body)

    def current_task(self) -> dict[str, Any] | None:
        return self._request("GET", self.me("/tasks/current"))["task"]

    def task_status(self, task_id: int, status: str, **extra: Any) -> dict[str, Any]:
        body = {"status": status, **{k: v for k, v in extra.items() if v is not None}}
        return self._request("POST", self.me(f"/tasks/{task_id}/status"), body)

    def upload_error_logs(self, logs: list[dict[str, Any]]) -> dict[str, Any]:
        return self._request("POST", self.me("/error-logs"), {"logs": logs})


class SimRobot:
    def __init__(self, api: RobotApi, args: argparse.Namespace) -> None:
        self.api, self.args = api, args
        self.map_id, self.x, self.y, self.theta = 1, 0.0, 0.0, 0.0
        self.battery = args.battery
        self.task: dict[str, Any] | None = None
        self.next_seq = 1  # route[0] 是起點
        self.boot_count = int(time.time()) % 100000  # 範例：真機用開機次數
        self.error_seq = 0

    def log(self, msg: str) -> None:
        print(f"{datetime.now().strftime('%H:%M:%S')} [狗 {self.api.robot_id}] {msg}", flush=True)

    def place(self, start: str) -> None:
        if "," in start:
            self.x, self.y = (float(v) for v in start.split(","))
        else:
            point = next((p for p in self.api.points() if p["pointId"] == start), None)
            if point is None:
                sys.exit(f"找不到點位 {start}")
            self.map_id, self.x, self.y = point["mapId"], point["x"], point["y"]
        self.log(f"起始位置 地圖 {self.map_id} ({self.x:.2f}, {self.y:.2f})")

    # ---- 一輪 ----
    def tick(self, dt: float) -> None:
        self.api.report_state(self.map_id, self.x, self.y, self.theta, self.battery)
        if self.task is None:
            self.pick_up_task()
        else:
            self.follow_route(dt)
        if self.battery is not None:
            self.battery = max(0.0, self.battery - (0.02 if self.task else 0.005) * dt)

    def pick_up_task(self) -> None:
        task = self.api.current_task()
        if task is None:
            return
        self.task, self.next_seq = task, 1
        stops = " → ".join(s["checkpointId"] or "起點" for s in task["route"])
        self.log(f"收到任務 {task['taskId']}（{task['targetType']} {task['pointId'] or task['stationId']}）：{stops}，"
                 f"約 {task['routeLengthM'] or 0:.0f} 公尺")
        if task["status"] == "pending":
            self.api.task_status(task["taskId"], "in_progress")

    def follow_route(self, dt: float) -> None:
        assert self.task is not None
        task_id = self.task["taskId"]
        # 每輪確認任務還在（App 取消、釋放時任務會消失）
        current = self.api.current_task()
        if current is None or current["taskId"] != task_id:
            self.log(f"任務 {task_id} 已取消或結束，停下")
            self.task = None
            return
        route = self.task["route"]
        if self.next_seq >= len(route):
            return
        stop = route[self.next_seq]
        if self.args.fail_at_seq is not None and self.next_seq == self.args.fail_at_seq:
            self.fail(task_id, "NAVIGATION_BLOCKED", f"前往第 {self.next_seq} 站時通道被擋住")
            return
        dx, dy = stop["x"] - self.x, stop["y"] - self.y
        dist = math.hypot(dx, dy)
        step = self.args.speed * dt
        if dist > 1e-6:
            self.theta = math.atan2(dy, dx)
        if dist <= max(step, ARRIVE_RADIUS_M):
            self.x, self.y, self.map_id = stop["x"], stop["y"], stop["mapId"]
            if stop.get("theta") is not None:
                self.theta = stop["theta"]
            self.arrive(task_id, self.next_seq, last=self.next_seq == len(route) - 1)
            self.next_seq += 1
        else:
            self.x += dx / dist * step
            self.y += dy / dist * step

    def arrive(self, task_id: int, seq: int, last: bool) -> None:
        try:
            if last:
                self.api.report_state(self.map_id, self.x, self.y, self.theta, self.battery)
                self.api.task_status(task_id, "done")
                self.log(f"到達終點，回報 done（任務 {task_id}）")
                self.task = None
            else:
                self.api.task_status(task_id, "in_progress", reachedSeq=seq)
                self.log(f"走過第 {seq} 站 {self.task['route'][seq]['checkpointId'] if self.task else ''}")
        except ApiError as exc:
            if exc.code == "TASK_CANCELLED":
                self.log("任務被取消，停下")
                self.task = None
            else:
                raise

    def fail(self, task_id: int, code: str, message: str) -> None:
        self.api.task_status(task_id, "failed", errorCode=code, message=message)
        self.error_seq += 1
        self.api.upload_error_logs([{
            "bootCount": self.boot_count, "errorSeq": self.error_seq, "stage": "navigation",
            "errorCode": code, "message": message, "taskId": task_id,
            "detail": {"x": round(self.x, 2), "y": round(self.y, 2)}, "shutdown": False,
            "occurredAt": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        }])
        self.log(f"回報失敗 {code} 並上傳錯誤紀錄（任務 {task_id}）")
        self.task = None
        self.args.fail_at_seq = None  # 只示範一次


def main() -> None:
    parser = argparse.ArgumentParser(description="QuadDrop 機器狗端模擬器（參考實作）")
    parser.add_argument("--base-url", required=True, help="例：https://api.quaddrop.fclinlab.com/api/v1")
    parser.add_argument("--robot-id", type=int, required=True)
    parser.add_argument("--token", help="這台狗的 token（後端 ROBOT_DEVICE_TOKENS）；本機開發沒設 token 時可省略")
    parser.add_argument("--start", default="4f-08", help="起始點位 ID，或 x,y（公尺）")
    parser.add_argument("--speed", type=float, default=1.0, help="移動速度，公尺／秒")
    parser.add_argument("--interval", type=float, default=1.0, help="每幾秒一輪（回報間隔）")
    parser.add_argument("--battery", type=float, default=90.0, help="起始電量；給 -1 表示不回報電量")
    parser.add_argument("--fail-at-seq", type=int, help="示範失敗：走到這一站時回報 NAVIGATION_BLOCKED")
    args = parser.parse_args()
    if args.battery < 0:
        args.battery = None

    robot = SimRobot(RobotApi(args.base_url, args.robot_id, args.token), args)
    robot.place(args.start)
    last = time.monotonic()
    while True:
        now = time.monotonic()
        try:
            robot.tick(now - last)
        except ApiError as exc:
            robot.log(f"API 錯誤：{exc}")
        except (urllib.error.URLError, TimeoutError) as exc:
            robot.log(f"連不上後端，稍後重試：{exc}")
        last = now
        time.sleep(args.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
