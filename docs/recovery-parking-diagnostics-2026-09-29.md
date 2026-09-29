# 9/29 定位、待機及資料管線修正

## 行為變更

- 狀態離開／重入時清除有效觀察時鐘。第二次 SUSPECT 不再把中間正常導航時間加成約 205 秒。
- 初始驗證通常 25 秒；僅資料新鮮、靜止、位置一致且已有健康樣本或 covariance 確實改善時，最多等 40 秒。
- 局部靜止確認及掃描後確認通常 12 秒；同樣有改善才可延長至最多 30 秒。仍需原本健康门檻與 6 筆独立樣本，不因圖形吻合就放行。
- 座位燈待機點由 `(0.7,-5,0)` 改為 `(1.2,-5,0)`，CYCU EE 與服務座標不變。9/29 記錄中 x=1.0 的 footprint 仍含代價 99，x=1.1 不含 99/100；x=1.2 保留既有 0.10 m 到點容許誤差。這是舊 bag 的靜態核對，仍需要實際停靠驗證。
- 同一待機目標最多 3 次導航嘗試，失敗後等待 10、20 秒；3 次失敗後停止自動重送。不宣稱已停妥，不影響新訂單。新服務任務、成功停靠、選到另一待機點或重啟配送程式可重置這一輪預算。新訂單/定位中斷不算導航失敗。
- 速度、避障區、footprint、里程計校正、INA 模型、取還確認筆數與 0.7 排程不變。

## 被動診斷（1 Hz，不記錄每幀影像）

- `/localization/state` 新增 `tick_max_gap_sec`、`tick_max_duration_sec`（約 5 秒的區間峰值）與 `pipeline_receipts`：raw/filtered scan、odom、AMCL 的接收數、來源時間差、最長接收間隔及峰值距今時間。
- `/chassis/feedback_health`：原始 STM32 字串接收數/間隔、完整四輪樣本數/距今時間、odom timer 間隔。可分辨原始資料沒收到、組包失敗或整個 executor 停頓；不改 0.8 秒 watchdog。
- `/vision/pipeline_health`：相機 callback 間隔、來源影像時間差、預處理/Hailo 推論/後處理/語意與預覽發布耗時、總 callback 耗時、sequence/FPS。
- `/semantic/pipeline_health`：fusion 收到語意與 scan 的間隔、來源時間差、watchdog callback 間隔與實際安全倍率。
- `/delivery/standby_state`：待機目標、`approaching/arrived/retry_wait/blocked/interrupted_by_*`、失敗次數與剩餘等待。只有 `arrived` 的 `parked=true`。
- 沒人訂閱預覽影像時，不再繪圖與發布；每幀語意仍發布。有訂閱時維持原來每 3 幀預覽與現有 QoS。

診斷是為了追查停頓，尚不能宣稱 DDS、TF 或 Hailo 停頓根因已修好。來源與消費者應一同對照，不能只看 bag 缺包。既有語意 0.8 秒半速、2 秒零速及雷達/里程計安全策略保持。

## 下一輪實車測試

1. 同一次啟動連續觸發兩次定位疑慮，確認第二次有效等待從 0 開始，不立即升級。等待期間車輛應停下。
2. 人工初始位置緩慢收斂，25 秒時若已有 4～5/6 樣本，應繼續確認；40 秒未達條件才升級，過期資料或嚴重不確定度不可借用寬限。
3. 左樓梯口取還後觀察 AMCL 各向 covariance；正常資料改善時先靜態確認，資料中斷不可旋轉盲校。
4. 座位燈 `(1.2,-5)` 實際停靠，核對後方空間、車頭方向及 10 公分到點精度；無障礙時應成功，而不是停在舊位置反覆啟動。
5. 以安全方式製造待機不可達，檢查嘗試上限/退避；途中下新訂單仍應立即中斷或繞過待機等待，不需人工標記待機。
6. 對照 producer/fusion 健康 topic、原始 STM32/odom 與 localization timer，追查下一次停頓；保留語意逾時煞停保護。

前景錄製（不會自行啟動導航／車輪）：

```bash
bash /home/kj0921/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

以 Ctrl+C 正常結束。`recording_qos.yaml` 將 initialpose reader 設為 volatile，能收未來 RViz 和 manager 发布，但不補錄錄製前的初始位置。腳本只錄明列的 topic，不包括 raw 影像。

## 給網頁隊友

本次沒有資料庫 migration、租借歸還 API 或結果 ACK 格式改動。若網頁保存或繪製座位燈待機座標，請同步為 x=1.2、y=-5.0、yaw=0；CYCU EE 及服務點不變。`/delivery/standby_state` 為新增 ROS 診斷，若未來要在網頁顯示待機受阻，須由 bridge/後端另行接入；本版未悄悄改既有 API schema。
