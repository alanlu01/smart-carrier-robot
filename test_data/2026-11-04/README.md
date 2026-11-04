# FlowCharge 實車測試資料封存（2026-11-04）

本次只新增測試資料與分析文件，未修改車載程式、韌體、導航參數或錄製項目。近期測試使用的程式版本見各場環境快照；11/3 晚間為 `82f7bbd`。

## 取得資料

- `quantitative_20260922_20261003/`：九場歷史測試的 CSV、JSON、Excel 與方法說明。
- `reviews/`：各次測試的分析與人工操作註記。歷史文件中的「目前／今天」指該文件日期，不代表所有問題均已修復。
- `analysis_scripts/`：分析腳本及去識別化中間資料。四份 `inputs/*.jsonl` 可離線重算九場彙整結果。
- `release_index.json`：六包近期對照資料的訊息數、時間範圍、原件與公開副本校驗資訊。
- `SHA256SUMS.txt`：Release 附件的 SHA-256。

六包公開 rosbag 存於 [測試資料 Release](https://github.com/alanlu01/smart-carrier-robot/releases/tag/test-data-2026-11-04)，不加入 Git 歷史。下載後先核對 SHA-256，再解開 `.public.tar.zst`。Linux 可使用：

```bash
sha256sum -c SHA256SUMS.txt
tar --zstd -xf 20261103_201845_full_system_aiK4cq.public.tar.zst
```

每包含完整的原有 topic、metadata、參數快照、效能紀錄與校驗結果。原始 `rosbag_info.txt` 的檔案大小描述原件；重新封裝後的大小請以 `metadata.yaml`、`validation.json` 為準。

## 六包對照資料

| 錄製時間（臺灣時間） | 用途 |
|---|---|
| 11/2 19:45 | 修改前基準，原始輪速未出現超過 0.8 秒的接收間隔 |
| 11/3 12:28 | 當日較早版本對照 |
| 11/3 15:10 | start_nav 啟動異常 |
| 11/3 15:34 | 獨立 Nav2 架構、CPU 與導航對照 |
| 11/3 19:46 | 近距離停止區攔截，使用者確認可能擺得太近 |
| 11/3 20:18 | 輪速來源停頓、AMCL 心跳逾時及反覆定位 |

六包是不同版本、操作與網路條件下的實測，不是受控 A/B 實驗。實際錄製起迄、程式提交與訊息筆數以索引及各包環境快照為準。

## 去識別化及完整性

完整原件另外保存於本機私有備份。公開副本並非原件逐位元複製：

1. 業務 UUID 使用一致的替代 ID，保留跨表、跨場次關聯。移除可辨識的帳密、聯絡及網路識別資訊。
2. 修改限於文字型訊息、ROS 日誌、診斷文字與外部文字檔。非文字的感測／導航 payload 以逐 topic 雜湊確認與原件一致。
3. 保留所有 topic、每個 topic 的訊息數、記錄／發布時間戳、序號與時間範圍。公開版不含影像或音訊 topic。
4. MCAP 重寫後重新檢查 CRC 及修改訊息的 CDR 可讀性。原件與公開副本各有不同的 SHA-256，不能混用。

資料用於離線分析。請勿將速度、訂單或 action topic 重播到正在運行的實車。

## 重算與限制

具備 Python、NumPy 後，可從任意目錄執行：

```bash
python analysis_scripts/aggregate.py /tmp/flowcharge_reproduced
```

已核對重算後 `analysis_data.json` 與發布版完全一致，包含 72 筆去重任務、69 筆 done、3 筆取消。這些完成結果含人工協助與重啟，不能宣稱為 69 次全自主配送成功；軟體位置自洽也不是實體停靠精度真值。

Excel 驗證可執行 `python analysis_scripts/validate_saved.py`，需 openpyxl；該腳本只讀取公開表格。其他抽取及建表腳本保留原始研究流程，重新抽取須具備原始九場 bag、ROS 2 Jazzy 及所用套件，並依本地路徑調整。Release 的六包近期資料與九場歷史量化來源不是同一批。

延遲欄位須分開解讀：

- `first_ack_delay_s`：相同結果 event_id 的第一筆 `/smart_carrier/task_result_ack` 接收延遲。早期訊息缺少 `ack_stage`，不能只憑這個欄位就判定本機持久化或雲端已接受。
- `cloud_ack_delay_s`：以 API bridge 明確記錄「雲端已確認任務結果」的時間減去車端結果時間。不是前端畫面延遲，也不是純網路 RTT。
- 新版訊息另有 `local_durable`、`cloud_confirmed`、`cloud_reconciled` 等階段，應依階段分析。不能把數十毫秒與數十秒的兩欄直接當成同一種通訊延遲。

歷史量化範圍與確認方式保持原樣；本次沒有擴大樣本、補造真值或重新宣稱成功率。
