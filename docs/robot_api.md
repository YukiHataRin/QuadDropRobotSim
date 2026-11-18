# QuadDrop 機器狗端 API 交接文件

給機器狗（真機）開發者：狗要呼叫哪些 API、送什麼、拿到什麼、什麼時候用。
`robot_sim.py` 是照這份文件寫的參考實作，可以直接對照。

## 1. 整體流程

機器狗**不自己挑任務、也不自己規劃大方向的路線**：App 的派工器指派任務，後端在 checkpoint 圖上算好路線，狗依序走過每一站。

```text
           ┌──────────── 後端（QuadDrop App + 機器狗 API）────────────┐
 使用者 ──▶│ 建立任務 → 派工器指派狗 → 算路線（起點 → checkpoint → 目標） │
           └───────▲──────────────────────────────┬────────────────────┘
                   │ 每秒：回報位置、電量         │ 任務與路線
                   │ 走過一站：reachedSeq         │
                   │ 到達：done／失敗：failed     ▼
                 ┌─┴────────────────────────────────┐
                 │ 機器狗：依序走到每一站，兩站之間 │
                 │ 自己導航、避開人與障礙           │
                 └──────────────────────────────────┘
```

一筆配送會有**兩個導航任務**：先到「發貨處室」（等人放物品、按出發），再到「目的處室」。狗不用知道是哪一段，照任務走就好。

## 2. 共通規則

| 項目 | 規則 |
|---|---|
| 網址 | `{BASE}/robot/...`，`BASE` 例：`https://api.quaddrop.fclinlab.com/api/v1` |
| 認證 | 每個請求帶 `Authorization: Bearer <這台狗的 token>`（後端設定 `ROBOT_DEVICE_TOKENS`，每台一組）。狗的 token 只能存取自己那台的 API |
| 格式 | JSON，欄位 camelCase（`mapId`、`taskId`…） |
| 座標 | 地圖座標，單位公尺，原點同 ROS 地圖（`4F.yaml`）；`theta` 弧度，逆時針為正 |
| 時間 | ISO 8601、一定要帶時區（`2026-11-18T08:30:00Z`）；請開 NTP，**不能比伺服器快超過 5 秒** |
| 錯誤 | HTTP 狀態碼 + `{"error": {"code": "TASK_CANCELLED", "message": "..."}}`，程式請看 `code` |
| 重送 | 下面所有寫入的 API 都可以安全重送（逾時就重送，不會重複執行） |

認證錯誤：`401 UNAUTHENTICATED`（沒帶或帶錯 token）、`403`（用狗的 token 呼叫別台或非機器狗端的 API）。

## 3. API

### 3.1 回報狀態：`POST /robot/robots/{id}/state`

**什麼時候用**：開機後就開始，**每 1～2 秒一次**，不管有沒有任務。這也是心跳：超過 15 秒沒回報，後端把狗標成離線，不會派任務給它。

請求：

```json
{"mapId": 1, "pose": {"x": 85.5, "y": 8.55, "theta": 1.57}, "batteryPercent": 85, "observedAt": "2026-11-18T08:30:00Z"}
```

| 欄位 | 必填 | 說明 |
|---|---|---|
| `mapId` | ✅ | 目前在哪張地圖（目前只有 1 = 4F） |
| `pose.x`、`pose.y`、`pose.theta` | ✅ | 目前位置與朝向 |
| `batteryPercent` | | 0～100；**低於 30% 不會被派任務** |
| `observedAt` | | 量到這個位置的時間；省略時用伺服器收到的時間。晚到的舊回報不會蓋掉新的位置 |

回應：這台狗的狀態，重要欄位：

```json
{"robotId": 5, "status": "busy", "missionId": 17, "available": false, "unavailableReason": "BUSY",
 "position": {"floor": "4F", "x": 85.5, "y": 8.55, "pointId": "4f-08", "mapId": 1, "theta": 1.57},
 "batteryPercent": 85, "currentTaskId": 3,
 "navigation": {"commandId": "mission-17-origin", "missionId": 17, "remainingM": 69.7, "etaAt": "2026-11-18T13:49:07Z"}}
```

- 離點位 0.5 公尺內時 `position.pointId` 會填上點位 ID。
- `navigation` 是後端依狗回報的位置算出的剩餘距離與預計到達時間（顯示在 App 上），狗不用處理。

錯誤：`400 POSE_OUT_OF_MAP`（座標在地圖外）、`400 CLOCK_AHEAD`（時鐘太快）、`404 MAP_NOT_FOUND`。

### 3.2 讀目前任務：`GET /robot/robots/{id}/tasks/current`

**什麼時候用**：每次回報狀態後讀一次（約每秒）。沒任務時 `task` 是 `null`；**執行中任務消失了就代表被取消，要立刻停下**。

回應：

```json
{"task": {
  "taskId": 3, "robotId": 5, "targetType": "point", "missionId": 17, "commandId": "mission-17-origin",
  "pointId": "4f-02", "stationId": null, "mapId": 1, "floor": "4F",
  "checkpointPose": {"x": 17.65, "y": 25.75, "theta": null},
  "status": "pending", "createdAt": "...", "startedAt": null, "finishedAt": null, "error": null,
  "routeLengthM": 82.1,
  "route": [
    {"seq": 0, "checkpointId": null,     "kind": "start",    "mapId": 1, "x": 85.5,  "y": 8.55,  "theta": null, "reachedAt": null},
    {"seq": 1, "checkpointId": "4f-c13", "kind": "corner",   "mapId": 1, "x": 79.98, "y": 9.67,  "theta": null, "reachedAt": null},
    {"seq": 2, "checkpointId": "4f-c12", "kind": "corner",   "mapId": 1, "x": 78.25, "y": 24.61, "theta": null, "reachedAt": null},
    {"seq": 3, "checkpointId": "4f-c11", "kind": "junction", "mapId": 1, "x": 75.96, "y": 25.57, "theta": null, "reachedAt": null},
    {"seq": 9, "checkpointId": "4f-02",  "kind": "point",    "mapId": 1, "x": 17.65, "y": 25.75, "theta": null, "reachedAt": null}
  ]}}
```

（`route` 中間幾站省略）

- `route[0]` 是建立任務時狗所在的位置（起點），不用走。
- 依 `seq` 順序走到每一站的 `x`、`y`；最後一站是目標，有 `theta` 時到達後轉到這個朝向。
- `kind`：`corner` 轉角、`junction` 路口、`door` 門口、`stairs` 樓梯口、`elevator` 電梯口、`point` 任務點位、`station` 換電站。
- 兩站之間怎麼走、怎麼避障由狗自己處理（例：把每一站當成 Nav2 的 waypoint）。
- 任務建立後路線不會變。
- `targetType` 為 `station` 時是前往換電站（見 3.4）。

### 3.3 回報任務進度：`POST /robot/robots/{id}/tasks/{taskId}/status`

**什麼時候用**：開始走、每走過一站、到達、失敗時。

```json
{"status": "in_progress"}
{"status": "in_progress", "reachedSeq": 3}
{"status": "done"}
{"status": "failed", "errorCode": "NAVIGATION_BLOCKED", "message": "走廊被擋住"}
```

| 情境 | 送什麼 | 後端做什麼 |
|---|---|---|
| 開始走 | `in_progress` | 任務變成進行中 |
| 走過第 n 站 | `in_progress` + `reachedSeq: n` | 記錄第 0～n 站的走過時間（用來算剩餘距離）。`pending` 的任務會順便變成進行中 |
| 到達終點 | `done` | 狗的位置設成目標點、發「已到達」給 App（App 通知使用者） |
| 走不過去 | `failed` + `errorCode`（必填）+ `message` | 通知 App；狗仍屬於這筆任務，等後端決定 |

- 狀態跟現在相同時直接回 200（可重送）；`reachedSeq` 回報舊的站也沒關係。
- 錯誤：`404 TASK_NOT_FOUND`、`409 TASK_CANCELLED`（任務已被取消，**停下**並重新讀 `tasks/current`）、`409 INVALID_TASK_TRANSITION`（已經 done／failed 又改別的）、`400 VALIDATION_ERROR`（`reachedSeq` 超出路線）、`422`（`reachedSeq` 搭配 `in_progress` 以外的狀態）。

回應：更新後的任務（格式同 3.2 的 `task`）。

### 3.4 申請去換電：`POST /robot/robots/{id}/tasks`

**什麼時候用**：電量低、狗決定去換電時（目前門檻 30%）。條件：在線上、**沒有被任務占用**、沒有未完成的任務。

```json
{"targetType": "station", "stationId": 1}
```

- 回應 201：新的換電任務（有 `route`，照 3.2、3.3 走）。重送同一個申請回 200 與原任務。
- 換電中 App 不會派任務給這台狗。
- 錯誤：`404 STATION_NOT_FOUND`、`409 ROBOT_BUSY`（正在執行配送）、`409 TASK_ALREADY_OPEN`、`503 ROBOT_OFFLINE`（先回報狀態）。
- 換電站清單：`GET /robot/stations`（目前 4F 還沒有登記換電站）。

### 3.5 上傳錯誤紀錄：`POST /robot/robots/{id}/error-logs`

**什麼時候用**：發生錯誤時；開機時把離線期間累積的紀錄補傳。一次 1～100 筆。

```json
{"logs": [{
  "bootCount": 12, "errorSeq": 1, "stage": "navigation",
  "errorCode": "NAVIGATION_BLOCKED", "message": "走廊被擋住",
  "taskId": 3, "detail": {"x": 40.1, "y": 25.0}, "shutdown": false,
  "occurredAt": "2026-11-18T08:30:00Z"
}]}
```

| 欄位 | 說明 |
|---|---|
| `bootCount` + `errorSeq` | 第幾次開機的第幾筆錯誤；同一組重送不會存兩次（不同內容回 `409 ERROR_LOG_CONFLICT`） |
| `stage` | `self_check` 自檢、`localization` 定位、`navigation` 導航、`docking` 換電、`other` |
| `taskId` | 跟哪個任務有關（可省略） |
| `detail` | 任意 JSON，自由放除錯資訊 |
| `shutdown` | `true`＝因為這個錯誤安全停機：進行中的任務變成失敗、狗標成離線，直到下一次回報狀態 |

回應 201：`{"inserted": 1, "duplicates": 0, "logs": [...]}`。查詢：`GET /robot/robots/{id}/error-logs?limit=50`。

### 3.6 唯讀資料（需要時才讀）

| API | 用途 |
|---|---|
| `GET /robot/maps` | 地圖解析度、原點（公尺座標 ↔ 地圖像素） |
| `GET /robot/points` | 任務點位座標 |
| `GET /robot/stations` | 換電站與停靠姿態 |
| `GET /robot/checkpoints`、`/robot/checkpoint-links` | 整張 checkpoint 圖 |
| `GET /robot/routes/preview?fromPointId=4f-01&toPointId=4f-08` | 兩點間的路線（不建立任務），測試用 |
| `GET /robot/robots/{id}` | 自己的狀態（同 3.1 的回應） |

## 4. 使用場景

| 場景 | 狗要做的事 |
|---|---|
| 開機 | 定位完成後開始每秒 `state`；補傳離線期間的 `error-logs` |
| 待命 | 持續 `state` + `tasks/current`，`task` 為 `null` 就原地等 |
| 收到任務 | 讀 `route`，送 `in_progress`，開始走 `route[1]` |
| 途中 | 持續 `state`；每走過一站送 `reachedSeq`；每輪確認 `tasks/current` 還是同一個 `taskId` |
| 到達發貨處室 | 送 `done`，原地等待；人放好物品按出發後會收到下一個任務（前往目的處室） |
| 到達目的處室 | 送 `done`，原地等收貨；收貨後後端釋放這台狗，回到待命 |
| 任務被取消 | `tasks/current` 變成 `null` 或回 `409 TASK_CANCELLED` → 立刻停下，回到待命 |
| 走不過去 | 送 `failed` + `errorCode`，上傳 `error-logs` |
| 電量低 | 沒有任務時 `POST tasks` 申請換電，照路線走到換電站 |
| 嚴重錯誤 | `error-logs` 帶 `shutdown: true` 後停機 |
| 斷線 | 恢復後照常回報；寫入的 API 都可以重送 |

## 5. 用模擬器測試

```bash
python3 robot_sim.py --base-url https://api.quaddrop.fclinlab.com/api/v1 --robot-id 5 --token <token> --start 4f-08
```

- 模擬器就是一台「真機」：照上面流程每秒回報、讀任務、沿路線移動、回報每一站與到達。
- `--speed` 移動速度（公尺／秒）、`--battery` 起始電量、`--fail-at-seq N` 走到第 N 站時示範回報失敗與上傳錯誤紀錄。
- 後端要先在 `ROBOT_DEVICE_TOKENS` 登記這台狗的 token，App 的 `robots` 表也要有同一個 id，才會被派任務。

## 6. 待確認（硬體團隊）

- 狗能不能照一串 checkpoint 走、兩站之間自己避障。
- 到達一站的容許誤差（模擬器用 0.3 公尺）。
- 座標單位與 `theta` 方向是否與 ROS 地圖一致。
- 錯誤碼清單（`errorCode`）。
- 樓梯／電梯怎麼走（目前只有 4F）。
