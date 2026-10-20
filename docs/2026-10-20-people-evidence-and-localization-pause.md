# 人流證據、短暫定位暫停與雷達時序診斷

基準版本：`5bd4035`。本次不變更地圖、服務／待機座標、導航速度、AMCL alpha／covariance 門檻、原有定位恢復採樣數、INA 模型、API 或前端。

## 1. 人流證據不再被中位數全部抹成零

- 原本將每個 15°方向箱的人數逐幀取中位數；遠處人員間歇被辨識時，正值常被零值壓掉。
- 改為有效觀察幀中的穩健平均：保留零值作為出現比例的分母，以正值中位數裁切偶發的大人數值。
- 每箱至少 3 個獨立正向影像且正向比例至少 15%；辨識信心仍為 0.65、角度限 ±34°、完整方向覆蓋至少 75%。相同／倒退時間戳不能累加證據。
- 對已通過證據門檻的角箱做 0.25/0.5/0.25 鄰角平滑，權重總和為 1，不將同一人當成三人。
- 計分是相對方向需求，不是去重後的實際人數；不累加跨影像的人次當作人口。
- `/smart_carrier/optional_idle_state.people_demand` 與完成日誌會區分 `no_person_detected`、`insufficient_positive_evidence`、`insufficient_heading_coverage`、`direction_selected`，並列出每方向證據數與待機點分數。
- 成功後 600 秒冷卻、穩定待機 60 秒、既有安全旋轉與精確停靠／有界補停規則不變。

## 2. 短暫 SUSPECT：先停止，再有限等待

- 僅在定位相位是新鮮的 SUSPECT（或 ready／相位回呼交接中的 LOCALIZED）、感測安全來源有效、未 motion-inhibited 時，允許有限等待。
- 當前導航／自旋先取消；必須取得原 action 終態、獨立的停穩證據，不能只收到取消 ACK 就送下一個 goal。
- 每次最多等待 20 秒，每次人流流程最多 2 次；已開始的自旋仍受原 90 秒總會話限制。前往分析點的導航仍沿用原導航截止時間，不因暫停而不斷延長。
- 只有原定位管理器重新允許 dispatch、定位相位新鮮、里程計／雷達／TF 安全、車輛停妥，才可重新驗證旋轉權限。原本 3 個獨立 scan、連續安全 0.75 秒、停穩 1 秒與 15°未知角區限制不變。
- 比較 map→odom 在同一車體 odom 座標上的修正，而非車身停止尾段的物理轉角或地圖原點平移。若修正超過 0.10m 或 0.10rad，丟棄舊方向資料並重新收集，最多重建 1 次，仍受原會話／續轉額度限制。新的位置仍需在分析點 0.10m 內；若不符，安全退出，不在錯誤位置硬轉。
- 等待期間的影像不加入舊方向計分；odom 仍追蹤取消後尾段，避免續轉多算。
- 若升級為全域／人工恢復、資料失效、等待逾時或一再修正，就退出人流，不強行救回。新訂單／電池優先級可隨時接管；舊人流流程不在服務結束後直接續跑。
- 在其他等待旋轉階段遇全域恢復或感測失效，也不保留舊人流證據穿過另一輪定位自旋。

## 3. 定位品質診斷，不降低安全門檻

- `/localization/state.quality_history` 記錄獨立 AMCL source stamp、最近 3 秒 critical／healthy 數量、標準差峰值、短暫／持續 critical burst 與姿態步幅。
- 只有時間覆蓋足夠才分類 burst；中間缺更新不會被推測成持續失準。所有欄位標記 `diagnostic_only=true`，不宣稱粒子一定是離群值或真實車位一定正確。
- 不增加運行中的粒子雲處理，也不使用主粒子群覆蓋原 covariance 安全判斷。
- 本次粒子雲雖呈少量遠端粒子，但停止前後仍有位置／角度修正，因此不直接減少原定位恢復 6 筆確認或放寬 0.5m critical 門檻。電機工程路段的非預期 SUSPECT 是否可再降低，須用新診斷與實測確認。

## 4. 獨立雷達觀察者

- `my_lidar.launch.py` 新增 `scan_pipeline_monitor`，獨立程序、淺層 BEST_EFFORT 訂閱，只輸出每秒 1 筆 `/lidar/pipeline_health`，不轉發／過濾 scan、不發布控制指令、不改 Nav2。
- 同時保存 scan header、DDS writer source、RMW receive、Python callback 四個時序；Jazzy MessageInfo 字典與 attribute 形式都支援。不支援的時間戳明確為 unavailable，不用 header 偽裝成 DDS 時間。
- 相同 scan stamp 可比對 raw→filtered 的 writer／RMW／callback 時差；另記錄原遮罩 frame 的零等待 TF 可用性及觀察者自身 callback 耗時。
- `writer_pipeline_sec` 包含輸入通訊、TF／濾波路徑，不是濾波 CPU 時間或 publish 函式阻塞時間。此版本沒有修改官方 laser_filters 或其 QoS；若新資料顯示 publisher 路徑或接收排程積壓，再針對來源修正。
- 收到延遲舊幀不會只因 callback 新就標成 healthy；分開標示接收缺口、來源過期與時鐘異常。
- 新監測節點本身不接管安全，原遮罩及過期保護保留。

## 錄製

指令相同，保留全部舊 topics，只增加 `/lidar/pipeline_health`：

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

下次正常重新啟動 `start_all`／雷達 launch 後，監測節點才會隨之運行。Ctrl+C 結束錄製，不用 Ctrl+Z。

## 驗證與限制

- 功能／安全 mock 回歸共 527 項通過：涵蓋間歇正值、零人流、單幀誤報、重複 stamp、SUSPECT 取消排空、等待上限、原截止時間、方向資料重建、人工／全域升級、新訂單搶占及無效資料禁止續轉。此計數不包含既有 ament copyright／flake8／PEP257 檢查，不代表全庫 lint 已清零。
- 晚間 bag `20261020_183548_full_system_SXrScw` 離線對照：原先 3 輪完整觀察的所有分箱中位數為零；新計分三輪均選「座位」。未完成的一輪仍因方向覆蓋不足而不選點。這是算法離線對照，不是新版本實車導航成功率。
- 另用隔離 localhost ROS domain 201、假 scan 與靜態 TF，確認 Jazzy 實際 DDS 時間戳與診斷心跳可用；沒有連正式 domain 30 或發布底盤控制。
- 尚須實車：人流選點→精確停靠／有界補停；短暫 SUSPECT 後有界續轉／重建；新診斷定位約 2 秒 filtered 延遲的來源。不能宣稱雷達停頓或路段定位問題已根治。
- 不需要前端／API migration 或隊友修改。
