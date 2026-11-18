# QuadDropRobotSim

QuadDrop 機器狗端模擬器：假裝自己是一台真機，照真機的流程呼叫後端，用來測試派送流程，也給機器狗團隊當呼叫 API 的參考實作。

- `robot_sim.py`：模擬器（Python 3.8 以上，只用標準函式庫，不用安裝任何套件）
- [`docs/robot_api.md`](docs/robot_api.md)：機器狗要用的 API、輸入輸出、使用場景（交接文件）

## 執行

```bash
python3 robot_sim.py --base-url https://api.quaddrop.fclinlab.com/api/v1 --robot-id 5 --token <這台狗的 token> --start 4f-08
```

用 uv 也可以：`uv run robot_sim.py ...`。

| 參數 | 說明 | 預設 |
|---|---|---|
| `--base-url` | 後端 API 位址（到 `/api/v1`） | 必填 |
| `--robot-id` | 機器狗 ID（與 App 的 `robots.id` 相同） | 必填 |
| `--token` | 這台狗的 token（後端 `ROBOT_DEVICE_TOKENS`）；本機開發後端沒設 token 時可省略 | |
| `--start` | 起始位置：點位 ID（如 `4f-08`）或 `x,y`（公尺） | `4f-08` |
| `--speed` | 移動速度（公尺／秒） | `1.0` |
| `--interval` | 每幾秒回報一次 | `1.0` |
| `--battery` | 起始電量（`-1` 不回報電量） | `90` |
| `--fail-at-seq` | 走到第 N 站時示範失敗：回報 `failed` 並上傳錯誤紀錄 | |

## 後端要先設定

1. 後端 `.env` 的 `ROBOT_DEVICE_TOKENS` 加上這台狗：`5=<至少 16 字元的 token>`，重啟後端。
2. App 資料庫的 `robots` 表要有同一個 id（例：`QD-05`），機器狗端的 `robot.robots` 要是 `control_mode = 'external'`。
3. 之後建立任務，派工器就會派給這台狗；模擬器會自動接任務、沿路線走、回報到達。

## 輸出範例

```text
21:48:24 [狗 5] 起始位置 地圖 1 (85.50, 8.55)
21:48:26 [狗 5] 收到任務 3（point 4f-02）：起點 → 4f-c13 → 4f-c12 → 4f-c11 → 4f-04 → 4f-c10 → 4f-c09 → 4f-03 → 4f-c08 → 4f-02，約 82 公尺
21:48:28 [狗 5] 走過第 1 站 4f-c13
...
21:48:57 [狗 5] 到達終點，回報 done（任務 3）
21:48:59 [狗 5] 收到任務 4（point 4f-03）：起點 → 4f-c08 → 4f-03，約 24 公尺
21:49:08 [狗 5] 到達終點，回報 done（任務 4）
```

## 換成真機

把 `SimRobot.follow_route()` 裡「每秒往下一站前進一步」換成真正的導航（例：ROS 2 Nav2 依序前往 `route` 的每一站），位置改用定位結果；其他流程（回報、讀任務、回報每一站與到達、取消時停下）照抄即可。
