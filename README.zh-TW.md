# Team 23083Z Reaper — 2025 PAS-VEX Signature Event

[English README](README.md)

這是 **Team 23083Z Reaper** 參加 **2025–2026 PAS-VEX V5RC Signature Event: High School** 時，為 VEX V5 Robotics Competition *Push Back* 使用的比賽程式。

賽事於 2025 年 12 月 18–20 日在新竹 Pacific American School 舉行。23083Z 在資格賽取得 **5 勝 3 敗**、晉級 16 強，並獲得 **Create Award（發明獎）**。可參考[官方賽事頁面](https://events.vex.com/robot-competitions/vex-robotics-competition/RE-V5RC-25-0121.html)與[賽事結果](https://events.vex.wang/event.php?id=60121&team=128435)。

此 repository 是賽事版本的歷史快照，保留當時使用的程式、測試功能及針對該台機器人調整的參數。

![Push Back 場地上的 autonomous 路徑記錄](path/VEX_RED_SOLO.png)

## 主要功能

- 以 [JAR-Template](https://github.com/JacksonAreaRobotics/JAR-Template) 為基礎的六馬達坦克底盤
- Brain 觸控螢幕自動程式選擇器與即時狀態頁面
- 結合底盤編碼器與 IMU 的里程計定位
- `driveToXY()` 與 `turnToXY()` 座標導航
- 使用 cosine velocity profile 平滑加減速
- 手動路徑記錄與 waypoint 終端輸出
- 透過光學感測器辨識聯盟顏色並排出對手方 block
- 多組比賽、AWP 與 Skills 自動程式

## 程式架構

1. `pre_auton()` 初始化硬體、校正 IMU，並顯示 Brain 觸控自動程式選擇器。
2. `autonomous()` 啟動里程計背景任務、設定聯盟顏色，並執行選定的 autonomous routine。
3. `usercontrol()` 切換為坦克駕駛，並啟動 intake、光學感測器與狀態背景任務。

主要檔案：

| 檔案 | 內容 |
| --- | --- |
| `src/main.cpp` | 比賽生命週期、觸控介面、駕駛控制與背景任務 |
| `src/autons.cpp` | 比賽與 Skills 自動路線 |
| `src/odometry.cpp` | 位姿追蹤、座標移動、平滑速度曲線與路徑記錄 |
| `src/note.cpp` | 聯盟顏色辨識與對手 block 排出邏輯 |
| `src/robot-config.cpp` | 馬達、感測器及氣動元件連接埠 |
| `src/JAR-Template/` | JAR-Template 底盤、PID 與里程計實作 |

## 自動程式

| 選項 | 用途 |
| --- | --- |
| `Right_43` | 右側 4+3 block 比賽路線 |
| `Left_43` | 左側 4+3 block 比賽路線 |
| `Right_7` | 右側 7 block 路線 |
| `Left_7` | 左側 7 block 路線 |
| `Solo` | Solo／AWP 路線 |
| `Skills` | Robot Skills 路線 |
| `Record` | 手動 waypoint 記錄模式 |
| `blank1`–`blank3` | 測試或預留位置 |

自動程式中的距離、角度、速度、延遲與氣動時序都針對當時的 23083Z 機器人調整；若移植到其他機器人，必須重新測試與調參。

## 里程計與座標移動

- `set_robot_pose(x, y, heading)`：重設場地座標與 IMU heading offset。
- `get_robot_pose()`：取得目前 X、Y 與朝向。
- `odometry_task()`：持續讀取左右底盤行程與 IMU，更新機器人位姿。
- `driveToXY(targetX, targetY, maxV, turnMaxV)`：計算目標距離與方位，同時控制前進與轉向修正。
- `turnToXY(targetX, targetY, turnMaxV)`：讓底盤面向指定座標。
- `recordPath()`：駕駛時按 A 記錄 waypoint、按 B 結束，並將可複製到 autonomous routine 的座標輸出至終端。

本程式的角度慣例為 `0°` 朝設定的正前方 Y 軸、`90°` 朝正 X 軸；除非函式另有註明，距離以英吋計算。

## 硬體連接埠

### Smart Port

| 裝置 | Port |
| --- | ---: |
| 左底盤 `L1`, `L2`, `L3` | 18, 17, 19 |
| 右底盤 `R1`, `R2`, `R3` | 7, 9, 8 |
| Intake | 6 |
| Intake-down | 5 |
| IMU | 20 |
| 光學感測器 | 11 |
| 第二顆光學感測器 | 15 |
| 後／左／右／前距離感測器 | 12 / 1 / 2 / 3 |

### Three-wire Port

| 裝置 | Port |
| --- | ---: |
| Shooter | A |
| Push cylinder | B |
| Wing | C |
| Intake cylinder | D |
| 白色指示燈 | G |
| 紅色指示燈 | H |

## 駕駛操作

| 輸入 | 動作 |
| --- | --- |
| 左／右搖桿 | 坦克底盤控制 |
| `L1` 按住 | Intake 正轉 |
| `R1` 按住 | Intake 反轉 |
| `L2` | 切換 wing |
| `R2` | 切換 intake cylinder |
| `B` | 切換 push cylinder |
| `Right` | 切換 shooter |
| `Y` | 同時切換 shooter 與 push cylinder |

## 建置與下載

1. 安裝 VEXcode V5 或 Visual Studio Code 的 VEX Robotics extension。
2. Clone 此 repository。
3. 開啟 `Jar-Template.v5code`。
4. 確認所有馬達方向、Smart Port 與 Three-wire Port 和實際機器人一致。
5. Build 專案，使用 USB 連接 VEX V5 Brain 後下載程式。
6. 第一次測試時請先將底盤架空，逐項確認馬達方向、IMU 校正、氣動初始狀態與緊急停止方式。

## 調參提醒

- PID、settling condition 與最大輸出都與機器人的重量、齒比和輪徑有關。
- 使用座標移動前，必須先正確設定起始位姿。
- 輪胎打滑或碰撞會造成平移位置誤差；IMU 能修正 heading，但無法消除所有 X/Y 漂移。
- 修改移動控制前，請先閱讀 `PID_TUNING_GUIDE.md` 與 `PATH_RECORDING_GUIDE.md`。

## 致謝與第三方程式

隊伍程式及比賽整合由 **Team 23083Z Reaper / Wego Robotics** 開發。

底盤控制使用 Jackson Area Robotics 的 JAR-Template，原專案採 MIT License。完整出處與授權說明請見 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 授權

隊伍原創程式採 [MIT License](LICENSE)。JAR-Template 與其他第三方元件仍依 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 所列的原始授權使用。

## 免責聲明

本程式是為特定實體機器人撰寫的比賽程式。任何硬體、韌體、齒比或控制參數變更後，都應重新安全測試。本 repository 僅供教育與歷史紀錄，並非 VEX Robotics 官方產品。
