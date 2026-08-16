# eNB 侧与 UE 侧三大 Manager 的对偶互逆关系分析

> 项目：`ul_mac` (LTE 上行 MAC 教学仿真)
> 对应真实协议：3GPP TS 36.321 (MAC)、TS 36.213 (物理层调度)
> 本文按 **BSR Manager / SR Manager / HARQ Manager** 三个模块，每个模块分
> **对偶关系概述 / 数据结构差异 / 关键函数实现差异** 三部分展开。

---

## 0. 全局视角：对偶关系的本质

在 LTE 上行链路中，**UE 是请求方 / 发送方，eNB 是决策方 / 接收方**。三大 Manager
在两侧形成"镜像互逆"：

| 模块 | UE 侧角色 | eNB 侧角色 |
|------|-----------|------------|
| BSR | 编码器（触发 + 组包 + 上报） | 解码器（解析 + 维护缓冲区视图估计） |
| SR | 发送方（触发 + 周期 + prohibit + RA 回退） | 检测器（仅置 `sr_pending` 标志，无独立 Manager） |
| HARQ | 发送方（NDI/RV/新传重传、软缓冲、早期终止） | 接收方（软合并 + CRC + PHICH + 重传/释放决策） |

**一个重要事实**：本项目**没有独立的 eNB SR Manager**。eNB 侧的 SR 处理退化成
`ul_scheduler::handle_sr()` 一个置位函数 + `ue_sched_context::sr_pending` 一个布尔位。
这与真实协议一致——SR 在 PUCCH 上是一个"1 bit"资源，基站侧不需要像 BSR/HARQ 那样
维护复杂状态机，只需在 MAC 层记录"该 UE 请求了授权"即可。

---

## 1. BSR Manager 对比

### 1.1 对偶关系概述

UE 侧 `bsr_manager` 与 eNB 侧 `enb_bsr_manager` 是**最完整的一对镜像**：

```
UE: lcg_buffer_manager(真实缓冲区) ──触发判断──> generate_bsr() ──组包──> bsr_ce
                                                              │
                                                              │ (MAC CE 经 PUSCH 传输)
                                                              ▼
eNB: receive_bsr(bsr_ce) ──解码──> ue_bsr_entity.lcg_buffer(缓冲区估计视图)
                                                              │
                                                              ▼
                                  供 ul_scheduler 的 ul_buffer[NOF_LCGS] 调度使用
```

- **UE 是信息源**：拥有 RLC 的真实缓冲区（`lcg_buffer_manager`），负责决定"何时上报
  (Regular/Periodic/Padding BSR)"以及"用哪种格式上报 (Short/Truncated/Long)"。
- **eNB 是信息汇**：不拥有真实缓冲区，只维护一个**保守下界估计**。解码语义刻意镜像
  UE 的编码语义（见 1.3），保证"eNB 看到的缓冲区 ≦ UE 真实缓冲区"，从而避免过度授权。

### 1.2 数据结构差异

**UE 侧（`bsr_manager` 内部 + `lcg_buffer_manager`）**

```cpp
// 真实缓冲区：new_buffer(实时快照) / old_buffer(上一次 step 的快照)
struct lcg_buffer {
    uint32_t new_buffer[NOF_LCGS];
    uint32_t old_buffer[NOF_LCGS];
    // 派生状态：pending_bsr、nof_with_data 等
};

// BSR 管理器状态
class bsr_manager {
    bsr_trigger_type triggered_type_;      // NONE/REGULAR/PERIODIC/PADDING
    bsr_config       bsr_cfg_;             // periodic_timer, retx_timer
    int32_t  periodic_timer_counter_;      // 周期定时器(倒计数)
    int32_t  retx_timer_counter_;          // 重传定时器(倒计数)
    bool     periodic_timer_running_;
    bool     retx_timer_running_;
    sr_manager* sr_proc_;                  // 与 SR 联动：Regular BSR 触发 SR
};
```

要点：
- UE 侧**同时持有真实缓冲区 + 触发状态机 + 定时器**，所有 BSR 均按 3GPP 标准完整上报。

**eNB 侧（`enb_bsr_manager`）**

```cpp
struct ue_bsr_entity {
    std::array<uint32_t, NOF_LCGS> lcg_buffer; // 各 LCG 的缓冲区估计(下界)
    enb_bsr_stats stats;                        // 收发计数
};

class enb_bsr_manager {
    std::unordered_map<uint16_t, ue_bsr_entity> ue_db_; // rnti -> 实体
    enb_bsr_stats total_stats_;
};

// 解析用的中间结构(UE 编码产物的镜像)
struct bsr_report  { uint8_t lcg_id; uint8_t buffer_size; }; // buffer_size 是索引(0-63)
struct bsr_ce     { bsr_format format; std::vector<bsr_report> reports; };
```

要点：
- eNB 侧**只有估计视图，没有真实缓冲区、没有定时器、没有触发状态机**。
- 没有 `periodic/retx` 定时器——eNB 不需要"触发 BSR"，它只"消费"UE 送来的 BSR。
- `lcg_buffer` 存的是**字节数下界**（由 `bsr_index_to_bytes()` 由索引还原）。

**LCG 映射表的共性**

两侧通过 `NOF_LCGS`（=4）和同一张 `BSR_INDEX_TABLE` 对齐映射语义。UE 把字节数
`bytes_to_bsr_index()` 压缩成 6-bit 索引；eNB 用 `bsr_index_to_bytes()` 解压回字节数。
这个 **(字节数 ⇄ 索引) 的编解码对** 就是两侧 LCG 映射表互逆的核心。

### 1.3 关键函数实现差异

| 维度 | UE 侧 `bsr_manager` | eNB 侧 `enb_bsr_manager` |
|------|---------------------|--------------------------|
| 入口 | `step(tti)`（每个 TTI 驱动） | `receive_bsr(rnti, bsr)`（被动接收） |
| 触发判断 | `check_regular_bsr_trigger()`：条件1 空 LCG 来新数据；条件2 高优先级 LCG 来数据；`retxBSR-Timer` 超时 | 无（eNB 不触发） |
| 定时器 | `periodic_timer_counter_`/`retx_timer_counter_` 倒计数 + `handle_timer_expiry()` | 无 |
| 格式选择 | `select_bsr_format()`：依据 `pdu_space` 与 `nof_lcg_with_data` 选 Short/Truncated/Long | 无（格式由 `bsr.format` 携带） |
| 组包/解码 | `generate_bsr()`：把 `lcg_sizes` 编成 `bsr.reports`（重启周期定时器） | `receive_bsr()`：`if LONG_BSR: view.fill(0);` 再填报告项（**镜像编码语义**） |
| 与 SR 联动 | `set_trigger(REGULAR)` 内调用 `sr_proc_->start()` | 无 |

**最关键的语义镜像**——`enb_bsr_manager::receive_bsr()` 的 Long BSR 处理：

```cpp
// enb_bsr_manager.cpp:46-48  —— 镜像 ue_bsr_manager::generate_bsr() 的 Long BSR 编码
if (bsr.format == bsr_format::LONG_BSR) {
    view.fill(0);   // UE 端只编码 buffer>0 的 LCG，eNB 端先清零再填充
}
```

而 `generate_bsr()`（ue_bsr_manager.cpp:293-301）确实只推送 `lcg_sizes[i] > 0` 的项，
因此 Long BSR 必须整体清零+重填，否则残留旧值会让 eNB 高估缓冲区。

**解码下界保守性**（`enb_bsr_manager.cpp:57-59`）：

```cpp
// TS 36.321 §6.1.3.1: 索引 i 对应区间下界, eNB 按"缓冲区 >= table[i]"解读
view[r.lcg_id] = bsr_index_to_bytes(r.buffer_size);
```

UE 上报的是区间索引，eNB 取区间下界作为估计——这是"宁低勿高"的设计，防止过度授权。

---

## 2. SR Manager 对比

### 2.1 对偶关系概述

**SR 是三者中对称性最弱的一对**。真实协议中 SR 只有 1 bit 物理资源（PUCCH 上的
专用 SR 时机），基站侧**不做复杂状态管理**，只需知道"该 UE 是否在请求上行授权"。

```
UE: 数据到达(空缓冲) ──> Regular BSR 触发 ──> sr_manager::start() ──> PUCCH 上发 1-bit SR
                                                            │
                                                            │ (PUCCH 1-bit 指示)
                                                            ▼
eNB: ul_scheduler::handle_sr(rnti) ──> ue_sched_context.sr_pending = true
                                                            │
                                                            ▼
        调度器在下一 TTI 给该 UE 分配 UL Grant ──> UE 的 notify_ul_grant_received() 清 SR
```

- **UE 侧**：完整过程（`sr_manager`，基于 TS 36.321 §5.4.4）——触发、周期约束、
  `dsr-TransMax` 重传、PUCCH 未配置时回退 RA。
- **eNB 侧**：**退化**为 `ul_scheduler::handle_sr()` 一个置位函数 + `sr_pending` 标志。

### 2.2 数据结构差异

**UE 侧（`sr_manager`）—— 完整的 SR 过程状态机**

```cpp
enum class sr_state { IDLE, PENDING, FAILED };  // TRANSMITTING 瞬态已删除

struct sr_config {
    bool     enabled;        // PUCCH 上的 SR 是否配置
    uint32_t dsr_transmax;   // SR 最大传输次数 (对应 dsr-TransMax)
    uint32_t sr_period;      // SR 周期(ms) —— 对应 sr-ConfigIndex 推导出的周期
};

class sr_manager {
    sr_state state_;
    int      sr_counter_;          // 已发送次数, 与 dsr_transmax 比较
    uint32_t last_sr_tx_tti_;      // 上次发送 TTI, 用于周期约束
    int      sr_prohibit_counter_; // sr-ProhibitTimer 计数器(本工程保留字段, 未启用动态)
    uint32_t adaptive_sr_period_;  // 增强: 自适应 SR 周期
    double   avg_traffic_rate_;    // 增强: EMA 平滑流量速率
    sr_config sr_cfg_;
};
```

要点：
- `sr_period` 即真实协议中 **`sr-ConfigIndex` → SR 周期** 的"翻译结果"。本项目把
  3GPP 中查表得到的 `SR periodicity` 直接存成毫秒数（简化，真实协议存 index 然后查表）。
- `sr_prohibit_counter_` 字段存在但**本工程未实施动态 prohibit**（注释保留），对应真实
  协议 `sr-ProhibitTimer`。真实协议里 SR 受 prohibit timer 约束：在 timer 运行期间不重复发 SR。
- `dsr_transmax` 对应 `dsr-TransMax`（SR 传输上限），达到后回退到随机接入。

**eNB 侧（`ue_sched_context`）—— 只有一个布尔标志**

```cpp
struct ue_sched_context {
    bool sr_pending;   // SR 待处理标志, 由 handle_sr() 置 true
    // ... 其余是 BSR 视图 / HARQ 回放 / CQI / PHR 等
};
```

要点：
- eNB 侧**不存储** `sr-ConfigIndex`、`sr-ProhibitTimer`、`dsr-TransMax` 等任何 SR 配置。
- eNB 不需要"分配 SR 资源"——SR 资源（PUCCH 上的 SR 时机）是 RRC 在连接建立时通过
  `sr-ConfigIndex` 半静态配置的，PHY 层按配置检测，MAC 层只消费检测结果。
- eNB 侧 SR"冲突"概念不存在：不同 UE 的 SR 资源由 `sr-ConfigIndex`/`SchedulingRequestResourceConfig`
  区分在不同 PUCCH 时机，PHY 检测天然分离，MAC 层见到的已经是"哪个 rnti 发了 SR"。

**SR 配置数据结构的存储形式对比**

| 配置项 (真实协议) | UE 侧存储 | eNB 侧存储 |
|-------------------|-----------|------------|
| `sr-ConfigIndex`（→SR 周期/偏移） | `sr_cfg_.sr_period`（已翻译为 ms） | 无（PHY 层持有，MAC 不使用） |
| `sr-ProhibitTimer` | `sr_prohibit_counter_`（保留，未启用） | 无 |
| `dsr-TransMax` | `sr_cfg_.dsr_transmax` + `sr_counter_` | 无 |

### 2.3 关键函数实现差异

**UE 侧发送逻辑（`sr_manager::step()` 核心）**

```cpp
// ue_sr_manager.cpp:125-145 —— 完整的 SR 发送状态机
if (sr_cfg_.enabled) {
    if (sr_counter_ < dsr_transmax) {
        if (sr_counter_ == 0 || can_send_sr(tti)) {  // 周期约束
            sr_counter_++;
            state_ = sr_state::PENDING;    // 保持 PENDING (TRANSMITTING 瞬态已删除)
            last_sr_tx_tti_ = tti;
            sr_transmitted_flag_ = true;   // 一次性通知: 本 TTI 实际发送过
            do_send_sr = true;
        }
    } else {
        state_ = sr_state::FAILED;        // 达上限 -> 回退 RA
        do_fail = true;
    }
} else {
    // PUCCH 未配置 -> 启动随机接入
    state_ = sr_state::FAILED;
    do_fail = true;
}
```

- `can_send_sr(tti)`：用 `last_sr_tx_tti_` 与 `adaptive_sr_period_`/`sr_period` 比较，
  实现 SR 周期约束（本工程的"轻量 prohibit"）。
- `notify_ul_grant_received()`：收到 UL Grant 后清 `sr_pending` 并计 SR 成功——这是
  UE 侧 SR 过程的**闭合点**（与 eNB 侧 `sr_pending = false` 呼应）。

**eNB 侧检测逻辑（`ul_scheduler::handle_sr()`）**

```cpp
// enb_ul_scheduler.cpp:42-48 —— 整个 eNB SR "Manager"
void ul_scheduler::handle_sr(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.sr_pending = true;   // 仅置位, 无状态机
}
```

- SR 被消费的位置在调度循环里：调度器给 UE 分配授权后 `ctx.sr_pending = false;`
  （enb_ul_scheduler.cpp:396 / 445 / 494）。
- **对比结论**：UE 侧 SR Manager 是"主动过程机"（触发→周期约束→重传→回退），
  eNB 侧 SR 处理是"被动标志位"（置位→被调度消费→清除）。这正是 SR 作为 1-bit 物理
  指示的本质——基站不需要也不应该维护与 UE 对称的复杂 SR 状态。

---

## 3. HARQ Manager 对比

### 3.1 对偶关系概述

HARQ 是三者中**算法最重、对称性最强但方向相反**的一对。UE 负责"发"，eNB 负责"收+判"：

```
UE (发送方)                         eNB (接收方)
─────────────                       ─────────────
new_grant_ul(grant)                 receive_tb(rnti, grant)
  ├─ 处理 PHICH 反馈                  ├─ 判定新 TB / 重传(NDI)
  ├─ 判定新传/重传(NDI)              ├─ 软合并(IR: combined_count++)
  ├─ generate_new_tx / retx          ├─ CRC 解码 (eff_snr vs threshold)
  ├─ RV 序列 {0,2,3,1}               ├─ 生成 PHICH: last_crc_ok
  └─ 早期终止(连续 NACK>=3)          └─ ACK→release / NACK→RetxPending
                                        └─ get_phich() 供调度器下发 PHICH
```

- **UE 持有"发送视角"**：NDI 翻转识别、RV  puncture 序列、最大重传、早期终止。
- **eNB 持有"接收视角"**：软缓冲合并（IR 增益）、CRC 判决、PHICH 1-bit 反馈生成、
  重传/释放状态机。两侧通过 **NDI（新传标识）** 与 **PID（进程号）** 对齐。

### 3.2 数据结构差异

**UE 侧（`ul_harq_process` + `ul_harq_manager`）**

```cpp
class ul_harq_process {
    uint32_t pid_;
    std::atomic<uint32_t> current_tx_nb_; // 已传输次数(含新传)
    std::atomic<uint32_t> current_irv_;   // IRV 计数器, 推导 RV {0,2,3,1}
    bool     harq_feedback_;   // 最近一次反馈(ACK=true/NACK=false)
    bool     is_grant_configured_;
    bool     cur_ndi_;         // 当前 NDI
    uint32_t cur_tbs_;         // 锁定 TBS(重传不变)
    int32_t  cur_rv_;          // 自适应重传时 eNB 指定的 RV
    uint32_t consecutive_nack_;// 增强: 连续 NACK 计数(早期终止用)
    bool     early_termination_enabled_;
};

class ul_harq_manager {
    std::unique_ptr<ul_harq_process[]> processes_; // MAX_HARQ_PROCESSES 个
    ul_harq_config harq_cfg_;  // max_harq_tx, max_harq_msg3_tx
};
```

要点：
- UE 侧有 **RV/IRV** 字段（生成不同 puncture 版本的冗余比特）、**`harq_feedback_`**
  （UE 自己记录最近反馈，用于状态机）、**`consecutive_nack_`**（早期终止）。
- UE 侧**不持有 `ul_snr`**（SNR 是 eNB 测的）、**不持有 `combined_count`**（软合并在
  eNB 做，UE 只发不同 RV 版本）。

**eNB 侧（`enb_rx_process` + `enb_ul_harq_manager`）**

```cpp
struct enb_rx_process {
    harq_state state;            // INACTIVE / WAITING_FB / RETX_PENDING
    bool   ndi_known;            // 是否已见过 NDI
    bool   cur_ndi;              // 当前 NDI(判定新 TB)
    uint32_t tx_count;           // 该进程总传输次数(含重传)
    uint32_t combined_count;     // 软合并次数(=1 新传, >1 重传合并)
    uint8_t  cur_mcs;            // 锁定 MCS(计算解码阈值)
    uint32_t cur_tbs;            // 锁定 TBS
    bool   last_crc_ok;          // 最近 CRC 结果 -> 直接映射为 PHICH 值
    uint32_t last_tti;
};

struct ue_rx_entity {
    enb_rx_process procs[MAX_HARQ_PROCESSES]; // 每 UE 独立 0~7
    int32_t ul_snr_x100;  // eNB 测量到的上行 SNR(固定点 x100)
    enb_harq_stats stats;
};

class enb_ul_harq_manager {
    std::unordered_map<uint16_t, ue_rx_entity> ue_db_;
    uint32_t max_harq_tx_;   // 最大重传次数(达到则 Discard)
};
```

要点：
- eNB 侧有 **`combined_count`**（软合并次数，驱动 IR 增益）、**`ul_snr_x100`**
  （eNB 独有，UE 没有）、**`last_crc_ok`**（直接作为 PHICH 输出）、**`cur_mcs`**
  （解码阈值用，UE 侧不需要，因为 UE 不解码自己发的包）。
- 两侧**都没有软比特缓冲的真实数组**——本项目用 `effective_snr = ul_snr + (combined_count-1)*IR_GAIN`
  把"软合并"建模成"SNR 增益"（见 `effective_snr_x100()`），即把 IR 的多次合并等效为
  信噪比累加，避免了真实实现里巨大的软缓冲内存。
- 相同点：两侧都按 `(rnti, pid)` 组织，**每 UE 独立 0~7 共 8 个进程**（与之前讨论一致）。

**HARQ 进程表 / 软缓冲字段对照表**

| 字段 | UE 侧 `ul_harq_process` | eNB 侧 `enb_rx_process` | 互逆含义 |
|------|-------------------------|-------------------------|----------|
| 进程标识 | `pid_` | 数组下标 (0~7) | 同一 PID 空间 |
| 新传判断 | `cur_ndi_` | `cur_ndi_` / `ndi_known` | NDI 翻转 = 新 TB |
| 传输计数 | `current_tx_nb_` | `tx_count` | 重传次数一致 |
| 软合并 | （无，靠发不同 RV） | `combined_count` | eNB 侧建模 IR 增益 |
| 信道质量 | （无） | `ul_snr_x100` | eNB 独有测量 |
| 解码/反馈 | `harq_feedback_` | `last_crc_ok` | 决策结果镜像 |
| 冗余版本 | `current_irv_` / `cur_rv_` | （无，eNB 不需要发 RV） | UE 生成 puncture |
| 早期终止 | `consecutive_nack_` + 开关 | （无） | UE 侧增强 |

### 3.3 关键函数实现差异

**① 新传/重传判定（两侧都基于 NDI，但方向相反）**

UE 侧（`ue_ul_harq_manager.cpp:195-207`）——以"NDI 与本地 `cur_ndi_` 不同"判新传：

```cpp
bool is_new_tx = (!is_temp_rnti && grant.ndi != cur_ndi_) ||
                 (!is_grant_configured_ && !is_temp_rnti) ||
                 grant.is_rar;
```

eNB 侧（`enb_ul_harq_manager.cpp:93-100`）——以"NDI 翻转或未见过 NDI"判新 TB：

```cpp
if (grant.ndi_present) {
    if (!p.ndi_known || grant.ndi != p.cur_ndi) is_new_tb = true;
} else {
    is_new_tb = (p.state == harq_state::INACTIVE); // 非自适应重传
}
```

两侧判定逻辑**互为印证**：NDI 是 UE 与 eNB 之间的"契约"——UE 翻转 NDI 表示新 TB，
eNB 检测 NDI 翻转表示新 TB。这是 HARQ 对偶关系的枢纽。

**② 软合并 / CRC 判决（仅 eNB 侧有）**

```cpp
// enb_ul_harq_manager.cpp:119-122 —— eNB 独有的接收判决
int32_t eff = effective_snr_x100(ue.ul_snr_x100, p.combined_count);
int32_t threshold = decode_threshold_x100(p.cur_mcs);
bool crc_ok = (eff >= threshold);
```

UE 侧没有对应函数——UE 不解码自己发出的包，它只根据 eNB 经**独立 PHICH 信道**下发
的反馈（由 `apply_phich_feedback(ack)` 落地）来更新 `harq_feedback_`。**PHICH 是两侧
HARQ 状态机的耦合点**：eNB 的 `last_crc_ok` → `get_phich()` → 调度器经 `phich_ch`
（+4 TTI 延时）下发 PHICH → UE 的 `handle_harq_feedback(pid, ack)` → UE 更新
`harq_feedback_`。反馈不再随 `ul_grant` 携带（`phich_available`/`hi_value` 字段已移除）。

**③ 重传管理**

- **UE 侧重传**：`generate_retx()`（ue_ul_harq_manager.cpp:244-267）锁定 `cur_tbs_`，
  若 eNB 指定 `grant.rv>=0` 则采用自适应 RV（`current_irv_ = irv_of_rv(grant.rv)`），
  否则按本地 IRV 序列 `{0,2,3,1}` 推进。还含**早期终止**增强
  （连续 NACK≥3 且已重传≥2 次，主动 `reset_unlocked()` 丢弃 TB）。
- **eNB 侧重传决策**：在 `receive_tb()` 内（`enb_ul_harq_manager.cpp:149-173`）：
  - CRC 失败且 `tx_count < max_harq_tx_` → `state = RETX_PENDING`，等调度器重传；
  - CRC 失败且 `tx_count >= max_harq_tx_` → `discarded = true` + `release_process_unlocked()`；
  - CRC 成功 → 直接 `release_process_unlocked()`。
  eNB 侧**不设早期终止**，因为"是否丢弃"由 eNB 通过 PHICH+重传授权控制，UE 的早期终止
  只是本地优化（节省 UE 自身重传尝试）。

**④ ACK/NACK 反馈路径（方向相反）**

```cpp
// eNB: 生成 PHICH (enb_ul_harq_manager.cpp:178-183)
bool enb_ul_harq_manager::get_phich(uint16_t rnti, uint32_t pid) const {
    return uit->second.procs[pid].last_crc_ok; // true = ACK
}

// UE: 消费 PHICH (ue_ul_harq_manager.cpp, apply_phich_feedback)
// 反馈经独立 PHICH 信道到达, 不再从 grant 读取 hi_value
void ul_harq_process::apply_phich_feedback(bool ack) {
    harq_feedback_ = ack;  // ack 来自 eNB 的 PHICH (经 timed_channel +4 TTI)
    feedback_received_ = true;
}
```

UE 侧额外有 `handle_harq_feedback(pid, feedback)` 接口（供外部直接喂 ACK/NACK 统计），
而 eNB 侧通过 `get_phich()` 把 CRC 结果暴露给调度器——两条路径在真实系统中被 PHICH
信道隔开，本项目用 `main.cpp` 里的 `grant.hi_value = rx.crc_ok` 模拟这个 1-bit 传输。

---

## 4. 总结：对偶性的强弱光谱

```
强对偶(双向完整, 语义镜像)   中对称(单向驱动)        弱对偶(严重不对称)
────────────────────────     ──────────────────      ──────────────────
HARQ (UE发 / eNB收+判)  ≈     BSR (UE编 / eNB解)  >    SR (UE过程机 / eNB标志位)
  - NDI 契约两侧一致             - 编解码函数互逆           - eNB 无独立 SR Manager
  - PHICH 耦合两侧               - LCG 映射表对齐           - eNB 仅 sr_pending 置位
  - RV/软合并方向分离            - 格式语义镜像             - PUCCH 资源由 PHY 持有
```

**核心结论**：
1. **BSR** 两侧最"教科书式对称"——UE 编码、eNB 解码，函数与结构一一呼应。
2. **HARQ** 两侧对称但方向相反——共享 NDI/PID 契约，算法最重，且本项目用
   `effective_snr` 模型把"软缓冲合并"转成"SNR 增益"，规避了真实软缓冲内存。
3. **SR** 不对称是**协议使然**而非简化——SR 本质是 1-bit PUCCH 指示，eNB 侧无需
   维护与 UE 对称的状态机，只需 `sr_pending` 标志位被调度消费即可。UE 侧 `sr_manager`
   的复杂度（周期/prohibit/重传上限/RA 回退）完全来自"UE 如何可靠地把这 1 bit 送到基站"。
```
