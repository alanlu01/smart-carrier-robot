# 2026-11-03：與昨日版本差異、SD 清理候選及測試資料保存

本次全程唯讀樹莓派與 GitHub 權限；沒有刪除、卸載、修改車載程式或上傳資料。以下容量為盤點當時的近似值，GiB/MiB 使用 1024 進位。

## 1. 先更正 CPU 比較基準

昨天指 **11/2 晚間 19:45 測試**，不是今天 11/3 下午的全獨立 Nav2 測試。兩份紀錄都以 /proc/stat 差分、將 iowait 與實際 CPU 忙碌分開計算。

| 指標 | 昨天 11/2 晚間 | 目前 11/3 20:19～35 |
|---|---:|---:|
| 錄製當時版本 | 3303695 | 82f7bbd |
| 整體 CPU 忙碌平均 | 85.797% | 84.510% |
| I/O wait 平均 | 0.412% | 3.782% |
| I/O wait 最高 | 5.500% | 20.999% |
| Nav2 容器 top CPU 平均（單核心100%口徑） | 101.702% | 96.781% |
| /stm32_data 接收間隔 >0.8s | 0段 | 15段 |
| 昨日 odom feedback 健康紀錄 | 908筆全部healthy，最大完整組年齡0.522s | 多次stale，完整組來源最長中斷6.953s |
| SD可用空間 | 8.5GiB（85%使用） | 目前1.7GiB（97%使用） |

因此**目前並非整體 CPU 較昨天上升，但資料來源停頓、I/O等待與恢復行為確實惡化**。場景、網路與行駛/等待比例不同，這不是受控 A/B 因果實驗。
今天下午全獨立 Nav2 約97.65%的數字是另一組比較，不能當成昨天的基準。
昨天測試後還有97bbce1/d716b9e的供電處理提交；版本差異同時以昨天最終d716b9e及實測3303695作核對。

## 2. 今天實際新增或修改了什麼

### 上午／下午 e3fa7cc、29de853

1. **串口重連與新鮮回授閘門**：RX/TX保留兩個程序，新增session、重試、20Hz檢查、零速維持、停車證據及新鮮度驗證。
2. **新增可靠發布的RX/TX健康topic**：RX以5Hz定時並在完整組到達時加發，TX以5Hz發布；RX原始文字與健康發布仍在串口讀取的同一執行鏈。
3. **新增 navigation_health 節點**：每秒詢問六個Nav2 GetState服務，回應等待約0.75秒，最近狀態超過3秒就判不可用。
4. **把輪速／導航健康缺失接到正常導航的定位恢復分支**：舊版LOCALIZED沒有這一段。現在完整輪速超過0.8秒、或navigation_health不可用，會立即_begin_recovery、取消導航、清map-match歷史，再依後續條件升級全域定位。這是「逾時現在更常顯示成定位問題」的明確程式原因。
5. **一度改為全獨立Nav2+async DDS profile**：下午CPU確實較高，晚間已撤回這個預設組合。
6. 新增程序診斷輸出及錄製項目；不是新增模型訓練或提高MPPI/相機頻率。

### 晚間 82f7bbd（現版）

1. **恢復組合Nav2與current傳輸**：執行架構大致回到昨天，保留診斷；同時把下午的async profile換回current。不能把現在仍稱作「全部Nav2獨立」。
2. **原生組合插件生命週期修正**：只對Nav2容器保留其元件/MPPI程式庫至退出，修正gdb確認的退出崩潰；本次實車容器退出碼0。沒有證據顯示這是主要持續CPU來源。
3. **新增完整四輪topic**：/chassis/wheel_sample約2Hz，odom/定位/fusion改用它取代各自接四行。這能避免訂閱端分組差異，但**原始文字、健康與完整組仍由同一RX程序發布**，沒有真正隔離阻塞。
4. 近距離STOP區、0.75秒停止保持、障礙最小距離0.15m；它解釋上一輪近物時無法動，不能解釋本輪來源端6秒不發布。
5. 輕微失配的有界低速續行、人流暫停額度分離、同輪TF快取與idle決策限頻；沒有提高導航速度、controller頻率或視覺目標FPS。
6. URDF只改四輪顯示半徑；軸高/雷達TF、0.000389、yaw_ratio與STM32韌體未改。

### 局部新增負載的量級

以下是top程序百分比，100%代表一個CPU核心；不能直接與四核心總體百分比相加比較。

| 程序 | 昨天平均 | 目前平均 |
|---|---:|---:|
| serial_to_ros | 1.582% | 2.936% |
| cmd_vel_to_serial | 1.211% | 4.470% |
| navigation_health | 未執行 | 1.979% |
| mecanum_odom_real | 5.090% | 4.260% |

新增監測與保護有局部CPU開銷，但不能據此說整體CPU大幅上升。今日三個新增/加重的監測路徑相對昨日約多6.6個單核心百分點，約等於四核心總量1.65個百分點的量級；兩次場景不同，只作量級參考。

### 為何仍懷疑通訊／發布阻塞

- current與昨天都使用原有SHM+UDP設定，沒有下午async profile的非同步與non_blocking_send配置。這不是相對昨天全新引入的模式，但今天增加了同鏈可靠發布與健康判斷。
- rmw_fastrtps官方說明：預設同步發布可能讓write中的阻塞連帶阻塞呼叫執行緒；改成async也仍需要排查佇列與資源限制，不能保證永不阻塞。
- 本輪RX原始文字、完整組及健康來源時間同時停頓，後來又集中送出多組資料；AMCL bond一端來源也停頓、另一端持續。與弱網時未充分隔離的發布路徑相符。
- 但目前沒有逐步read/publish執行時間，仍不能聲稱已證明某一個publish、SD操作或某個更新單獨就是全部根因。弱網狀態與SD壓力也與昨天不同。

**已確定需修的實作問題**：來源讀取與可靠發布未隔離，以及「資料管線暫停」過快變成「重設定位」。新增安全檢查合理，但應先安全停止、等待新鮮資料及驗證目前位姿，而非把每次來源短缺都放大成粒子重設。

## 3. SD滿載與CPU的關係

- 目前根分割區57GiB、已用約53GiB、可用1.7GiB（97%）；inode只用11%，不是小檔inode耗盡。
- 接近滿載可能增加配置/寫回壓力，錄製、日誌與系統共同寫SD時有卡頓風險；本輪I/O wait確實平均3.78%、最高21%。
- **I/O等待不等於CPU在運算**。程序可能卡在I/O而CPU忙碌比例沒有上升；load/延遲也不能直接當成CPU使用率。
- 不能由97%單獨斷言SD已損壞，亦未證明清空間就能解決全部DDS問題。先整理空間，再同場景重測並保留分段時間量測。
- 以目前一輪可產生1～2.6GiB的紀錄量，建議車載預留至少8～10GiB作操作餘量；這是本專案建議，不是通用的效能保證門檻。

## 4. 清理候選：尚未刪除

### A. 可重建／快取，優先處理

| 路徑 | 盤點容量 | 條件與影響 |
|---|---:|---|
| /home/robot/.cache/epiphany/ | 約853MiB | 關閉該瀏覽器後清快取；不動瀏覽器帳號設定。 |
| /home/robot/.cache/pip/ | 約168MiB | 下載快取，不是已安裝的Python/Hailo驅動；重要離線安裝包另保存。 |
| /var/cache/apt/ | 約129MiB | 下載快取；清理不等於卸載套件或升級系統。 |
| /home/robot/dev_ws/log/ | 約542MiB | 舊colcon建置/測試輸出，先保留最近部署驗證。不是dev_ws/install或src。 |
| /home/robot/.ros/log/ | 約166MiB | 先備份近期完整log與容器stderr，再清較舊日誌。 |
| /var/log/journal/ | 約447.5MiB（journal總量） | 依時間/容量保留近期；不手動刪正在寫入的單一journal檔。實際可釋放量小於總量。 |
| /home/robot/test_logs/validation_*/ | 總約464MiB | 候選src副本、build/install、重複bundle可重建；先保存驗證log、checksum、gdb根因與結果，不整包盲刪。 |

### B. 垃圾桶中的大檔，先移出電腦確認保存

垃圾桶總約1.63GiB，其中也有舊地圖、設定、編譯資料與程式備份，**不建議未核對就整桶清空**。下列不是正在使用的/home/robot/libcamera、rpicam-apps或hailort-drivers：

| 垃圾桶內路徑（完整前綴 /home/robot/.local/share/Trash/files/） | 佔用 |
|---|---:|
| hailo_dataflow_compiler-3.27.0-py3-none-linux_x86_64/ | 約735.5MiB |
| hailo_dataflow_compiler-3.27.0-py3-none-linux_x86_64.whl | 約563.2MiB |
| rpicam-apps.tar.xz | 約253.8MiB |
| hailo_compile/ | 約55.1MiB |
| dev_ws.tar.xz、dev_ws.2.tar.xz | 合計約30.9MiB |

以上安裝包、解開副本及歷史備份適合移到PC作離線還原，不必長駐車載SD。Trash裡的config、地圖及task內容仍需比對再處理。

### C. 原始rosbag是最大來源：備份驗證後移出Pi

/home/robot/test_logs 總35.47GiB（du顯示約36G）。今天5包實測合計5.645GiB；今天的組合模式驗證資料夾僅約34.4MiB，沒有突然產生數十GB程式副本。

優先保存以下對照資料：

- 20261102_194540_full_system_mK0obd：昨日基準。
- 20261103_122855_full_system_EB3Qmp：今日中午基準，單一mcap約2.55GiB。
- 20261103_151013_full_system_mFxsQA：start_nav問題。
- 20261103_153414_full_system_tfZVZN：全獨立模式CPU/椅腳/定位對照。
- 20261103_194631_full_system_ENlh0R：NearFootprintStop持續攔截證據。
- 20261103_201845_full_system_aiK4cq：本輪回授與AMCL心跳連鎖故障，修復前應保留可重現資料。
- 9/22後作為專題量化基準、校正、成功案例與已標註搬車/斷網事件的資料，不應只因較舊就刪除。

原始bag先完整複製到PC/外接儲存，保存metadata、設定快照、環境版本、事件及所有分片；用SHA-256校驗並離線開啟確認，再從Pi移除。不要只搬mcap而遺失metadata/事件時間線。目前PC C槽約98.5GiB可用，可容納這35.5GiB，但仍須安排與驗證，這次尚未進行大檔備份。

### 不列入本次清理

- dev_ws/src、dev_ws/install、正在使用的build及非Git本機設定。
- libcamera、rpicam-apps、hailort-drivers、/usr/local中的手動安裝依賴。
- 1060相關boot備份、最近車載源碼備份與目前尚未同步的設定。
- .ssh、robot.env、訂單journal/outbox、定位快取等必要運行狀態。

## 5. 重要測試資料能否放GitHub

可以。我已唯讀確認目前帳號對alanlu01/smart-carrier-robot有WRITE權限，main為預設分支；**該repo是公開的**。

推薦分層：

1. **程式repo直接保存**：分析報告、去識別化量化CSV/JSON、圖表、測試情境、環境/版本/參數、分析腳本與原始檔校驗清單。這些小檔方便引用與重現。
2. **精選原始bag作Release附件**：不進Git提交歷史。官方目前要求每個asset小於2GiB；最新1.04GiB的mcap可作附件，今日中午2.55GiB、10/6約2.04GiB的單檔需要分卷或重打包。建議在PC進行，保留還原說明與SHA-256，不增加Pi在測試中的壓縮負擔。
3. **完整原始實驗集合**：優先PC+外接/私有異地備份；GitHub只放索引與公開適合分享的精選資料。LFS可處理大檔，但有儲存與下載額度，也會增加日常部署下載負擔，未必適合整套35GiB資料。

普通GitHub Git單檔上限100MiB，不能直接把這些0.4～2.6GiB的bag git add/push。Releases則可分離大型附件與程式clone。任何公開資料仍需檢查訂單/使用者欄位與秘密；robot.env、金鑰與整包含本機設定的src備份不應跟著公開。

此處只確認可行性與保存方案，尚未建立Release、上傳、刪除或變更權限。

## 官方參考

- [ROS 2 rmw_fastrtps publication mode](https://github.com/ros2/rmw_fastrtps#change-publication-mode)
- [Linux PSI：I/O與CPU等待](https://docs.kernel.org/accounting/psi.html)
- [GitHub Git大檔限制](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)
- [GitHub Release附件限制](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)

## 附錄：原始實測/驗證資料容量清單

下列是移出候選，不代表已完成備份或可立即刪除。容量由唯讀du取得。

| 目錄（前綴 /home/robot/test_logs/） | GiB | 處理原則 |
|---|---:|---|
| `20261103_122855_full_system_EB3Qmp` | 2.568 | 先完整備份校驗，再移出Pi |
| `20261006_190906_full_system_yNptN5` | 2.051 | 先完整備份校驗，再移出Pi |
| `20261020_183548_full_system_SXrScw` | 1.880 | 先完整備份校驗，再移出Pi |
| `20261021_163451_full_system_6UzlQF` | 1.775 | 先完整備份校驗，再移出Pi |
| `20261103_153414_full_system_tfZVZN` | 1.739 | 先完整備份校驗，再移出Pi |
| `20261008_134637_full_system_vbs6Rn` | 1.512 | 先完整備份校驗，再移出Pi |
| `20261003_121329_full_system_YdTwqN` | 1.382 | 先完整備份校驗，再移出Pi |
| `20260929_152029_full_system` | 1.365 | 先完整備份校驗，再移出Pi |
| `20261029_132749_full_system_DI484z` | 1.336 | 先完整備份校驗，再移出Pi |
| `20261027_163238_full_system_GkoJKQ` | 1.315 | 先完整備份校驗，再移出Pi |
| `20260929_203819_full_system` | 1.178 | 先完整備份校驗，再移出Pi |
| `20260924_135009_full_system` | 1.166 | 先完整備份校驗，再移出Pi |
| `20261030_110140_full_system_g50udd` | 1.164 | 先完整備份校驗，再移出Pi |
| `20260923_172438_full_system` | 1.126 | 先完整備份校驗，再移出Pi |
| `20261001_135504_full_system_RmLFyB` | 1.053 | 先完整備份校驗，再移出Pi |
| `20261103_201845_full_system_aiK4cq` | 1.044 | 先完整備份校驗，再移出Pi |
| `20260930_202404_full_system_MwTQe0` | 0.995 | 先完整備份校驗，再移出Pi |
| `20261020_142844_full_system_r16gdG` | 0.991 | 先完整備份校驗，再移出Pi |
| `20261015_134206_full_system_UMFpwI` | 0.987 | 先完整備份校驗，再移出Pi |
| `20261102_194540_full_system_mK0obd` | 0.937 | 先完整備份校驗，再移出Pi |
| `20261007_163855_full_system_UwYjGV` | 0.913 | 先完整備份校驗，再移出Pi |
| `20261008_110935_full_system_hta7WD` | 0.847 | 先完整備份校驗，再移出Pi |
| `20260929_145348_full_system` | 0.815 | 先完整備份校驗，再移出Pi |
| `20261028_194505_full_system_B2CPyL` | 0.790 | 先完整備份校驗，再移出Pi |
| `20261013_192251_full_system_PmVUGd` | 0.668 | 先完整備份校驗，再移出Pi |
| `20260923_123933_full_system` | 0.638 | 先完整備份校驗，再移出Pi |
| `20261014_190847_full_system_ppkeMt` | 0.618 | 先完整備份校驗，再移出Pi |
| `20261013_153655_full_system_dxstKV` | 0.607 | 先完整備份校驗，再移出Pi |
| `20260922_154239_full_system` | 0.422 | 先完整備份校驗，再移出Pi |
| `20261019_164715_full_system_RCGkec` | 0.400 | 先完整備份校驗，再移出Pi |
| `20261103_151013_full_system_mFxsQA` | 0.181 | 先完整備份校驗，再移出Pi |
| `20261019_191342_full_system_idp3MV` | 0.156 | 先完整備份校驗，再移出Pi |
| `20261007_161317_full_system_1YSvcJ` | 0.124 | 先完整備份校驗，再移出Pi |
| `20261103_194631_full_system_ENlh0R` | 0.114 | 先完整備份校驗，再移出Pi |
| `validation_20261030_stationary_fusion_gAHb55` | 0.043 | 保留結果/日誌；可重建副本候選 |
| `20261019_190956_full_system_H7P6nR` | 0.039 | 先完整備份校驗，再移出Pi |
| `validation_20261103_composed_kGsS5F` | 0.034 | 保留結果/日誌；可重建副本候選 |
| `20261019_190636_full_system_KL7gLd` | 0.031 | 先完整備份校驗，再移出Pi |
| `validation_20261103_serial_recovery_FbbXOn` | 0.030 | 保留結果/日誌；可重建副本候選 |
| `validation_20261102_supply_visibility_oQ6wsz` | 0.027 | 保留結果/日誌；可重建副本候選 |
| `validation_20261029_global_stop_idle_Aalsuy` | 0.027 | 保留結果/日誌；可重建副本候選 |
| `validation_20261028_delivery_io_braking_UJX3DT` | 0.027 | 保留結果/日誌；可重建副本候選 |
| `validation_20261027_recovery_console_QdQ1aD` | 0.027 | 保留結果/日誌；可重建副本候選 |
| `validation_20261021_navigation_parking_LBdwmX` | 0.026 | 保留結果/日誌；可重建副本候選 |
| `validation_20260929_parking_clock` | 0.026 | 保留結果/日誌；可重建副本候選 |
| `validation_20261020_people_quality_mhL7K3` | 0.026 | 保留結果/日誌；可重建副本候選 |
| `validation_20261020_parking` | 0.026 | 保留結果/日誌；可重建副本候選 |
| `validation_20261019_people` | 0.026 | 保留結果/日誌；可重建副本候選 |
| `validation_20261015` | 0.014 | 保留結果/日誌；可重建副本候選 |
| `validation_20261014_delivery_idle` | 0.014 | 保留結果/日誌；可重建副本候選 |
| `validation_20261014_recovery` | 0.014 | 保留結果/日誌；可重建副本候選 |
| `validation_20261013_people` | 0.014 | 保留結果/日誌；可重建副本候選 |
| `validation_20261012_features` | 0.014 | 保留結果/日誌；可重建副本候選 |
| `validation_20261008_recovery` | 0.013 | 保留結果/日誌；可重建副本候選 |
| `validation_20261007_optional_idle` | 0.013 | 保留結果/日誌；可重建副本候選 |
| `validation_20261007_tf_ina_dispatch` | 0.012 | 保留結果/日誌；可重建副本候選 |
| `static_verification_20261007` | 0.002 | 先完整備份校驗，再移出Pi |
| `validation_20260929_recovery_scheduling` | 0.001 | 保留結果/日誌；可重建副本候選 |
| `stabilize_1060_20261007` | 0.000 | 先完整備份校驗，再移出Pi |
