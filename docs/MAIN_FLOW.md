# main.cpp 调用流程与函数作用说明

> 项目：`ul_mac` (LTE 上行 MAC 教学仿真)
> 文件：`src/main.cpp`
> 定位：eNB 侧上行 MAC **接收链路**演示程序。UE 侧原模块 (`bsr_manager` / `sr_manager` /
> `ul_harq_manager`) 被保留为"发送桩"，仅用于驱动数据到达、SR/BSR 编码、TB 发送；
> eNB 侧由三个核心模块构成接收链路：`enb_bsr_manager` + `ul_scheduler` + `enb_ul_harq_manager`。

---

## 1. 整体架构：发送桩 ↔ 接收链路（方案 B + 方案 C）

```
                  多线程 + 中央 TTI 时钟 + 四信道时间解耦 (各 +4 TTI 可见时延)
 ┌──────────────────────── UE 线程 (每UE一个) ────────────────┐   ┌──── eNB 调度线程 ────┐
 │ data_arrived()                                              │   │                     │
 │ run_tti() ── 触发SR(PUCCH) + 编码BSR CE                      │   │                     │
 │   │ SR ──enqueue──► [SR信道 (PUCCH)] ──(+4TTI)──► handle_sr │   │                     │
 │   │                                                         │   │ schedule_ul()       │
 │   │ ◄─(+4TTI)── [UL Grant信道 (PDCCH)] ◄── enqueue ─────────┼───┼─ grant             │
 │   │ ue.handle_ul_grant()→打包BSR进MAC PDU                    │   │                     │
 │   │ PUSCH ──enqueue──► [MAC PDU信道 (PUSCH)] ──(+4TTI)───────┼───┼─► enb_handle_bsr_pdu│
 │   │                                          解PUSCH PDU取BSR│   │   + receive_tb()    │
 │   │                                           ──(+4TTI)──► [PHICH信道]──enqueue──► handle_harq_feedback │
 │   │ ◄─(+4TTI)── PHICH (ACK/NACK) ◄──────────────────────────┘   │                     │
 └────────────────────────────────────────────────────────────┘   └─────────────────────┘
```

**方案 B（BSR 随 PUSCH 上报）**：UE 在 `handle_ul_grant()` 内用 `mac_pdu_packer::pack_bsr_only`
把 BSR CE 打包进 UL-SCH MAC PDU 字节流（`ue_context::last_pdu_`），随 PUSCH 经 `pdu_msg`
信道上报；eNB 侧 `enb_handle_bsr_pdu()` 用 `mac_pdu_unpacker::unpack` 解出 BSR CE 再喂调度器。
**BSR 不再以结构化 `bsr_ce` 在内存中直传**，而是真实走 MAC PDU 打包/解包，与协议一致。

**SR → 待传量的 srsRAN 式建模（本项目对协议时序的简化）**：
真实协议中 SR 仅是 PUCCH 上的 1-bit 信号，eNB 须等 UE 拿到 PUSCH 授权后发 BSR CE 才知待传量，
即"先发小 Grant 探测 BSR → 下一轮按 BSR 足额调度"的两轮时序。本项目（与 srsRAN 真实基站软件的
"同进程内部捷径"一致）改为：**SR 触发的同一时刻，UE 本地已知各 LCG 待传字节数，经 `sr_msg`
的 `pending_bytes` 字段直接随 SR 信道告知 eNB 调度器**（`handle_sr(rnti, pending_bytes)` 预填
`ul_buffer`），使 `schedule_ul` 在 PUSCH 上的 BSR CE 经空口解包前即可按真实量分配资源，
**跳过"先发小 Grant 探测 BSR"的空口往返**。真实空口的 BSR CE 路径（方案 B 的 `enb_handle_bsr_pdu`
解包）仍生效，并在解包后用更精确的量化值覆盖校准 `ul_buffer`。此简化与协议的差异已在
`handle_sr` / `sr_msg` / `main.cpp` 注释中明确标注。

**方案 C（多线程 + 4-TTI 时延）**：
- **中央 TTI 时钟线程**（主线程 `clock.run()`）统一推进 TTI 计数，所有 worker 经
  `tti_clock::wait_for_tti()` 同步，确定性无真实 wall-clock sleep。
- **每 UE 一个独立线程**，`ue_context` 由该线程独占（无需额外锁）。
- **eNB 调度线程**独占访问 `scheduler`/`enb_bsr`/`enb_harq`（内部本已加锁，作额外保护）。
- **HARQ 接收并行**：eNB 线程内对多 UE 的 PUSCH 接收用 `std::async` 并行 `receive_tb`。
- **四信道时间解耦**（`timed_channel<T>`，见 `include/ul_mac/tti_channel.h`）：SR(PUCCH) /
  UL Grant(PDCCH) / MAC PDU(PUSCH) / PHICH 各带 `CHANNEL_PROPAGATION_TTI = 4` 时延——
  发送方 `enqueue(item, send_tti)` 记 `available_tti = send_tti + 4`，接收方仅当
  `current_tti >= available_tti` 才能 `dequeue`，建模"信息建立后 4 TTI 才对端可见"。

**关键耦合点**：
- `grant.hi_value = rx.crc_ok`：eNB 接收端把 CRC 结果经 PHICH 信道回传 UE（方案 C 下
  经由 `phich_ch.enqueue(pm, tti)` 在 `send_tti + 4` 后才被 UE 线程 `dequeue` 见到）。
- `scheduler.handle_ul_crc(rnti, pid, rx.discarded ? true : rx.crc_ok)`：eNB 线程内
  把接收结果回馈调度器，用于清除/保留重传标志。
- `pdu_msg` 同时携带 `ul_grant`（UE 从 PDCCH 收到的 grant 副本），供 eNB 侧 `receive_tb`
  用正确 pid/ndi/mcs/tbs 收 TB（等效"eNB 用 grant 收 PUSCH TB"的真实语义）。

---

## 2. main() 主流程

```cpp
int main(int, char**) {                  // main.cpp:462
    SetConsoleOutputCP(CP_UTF8);         // 仅 Windows: 切 UTF-8 控制台代码页
    metrics_collector::instance().reset();   // 清全局度量

    scenario1_basic_ul_scheduling();     // 场景1: 基本链路 (单UE, 好信道)
    metrics_collector::instance().reset();

    scenario2_multi_ue_pf();             // 场景2: 多UE PF 调度 (5 UE)
    metrics_collector::instance().reset();

    scenario3_harq_retx();               // 场景3: HARQ 软合并重传 (弱信道)
    metrics_collector::instance().reset();

    scenario4_enhanced_features();      // 场景4: 增强功能 (自适应SR+预测BSR)
    metrics_collector::instance().reset();

    scenario5_epf();                    // 场景5: 华为增强型比例公平调度 (EPF)

    print_separator("最终系统性能指标");
    metrics_collector::instance().print_summary();  // 打印累计度量
    return 0;
}
```

`main()` 本身只负责：①设置控制台编码、②依次跑 5 个场景（每场景前重置度量）、
③最后打印系统级汇总。**所有实际逻辑都在 5 个 `scenario*` 函数里**，它们结构高度一致
（差别只在 UE 数量、信道质量、调度算法、增强特性）。

---

## 3. 通用 TTI 循环（5 个场景的公共骨架，方案 C 多线程版）

方案 C 下每个 scenario 不再是单线程 `for` 循环，而是**多线程 + 中央 TTI 时钟 + 四信道时间解耦**：
- **主线程**启动 `tti_clock`（`clock.run()`），统一推进 TTI 计数；
- **每 UE 一个线程**（`std::thread`，`run_ue(ue, ...)`），内部通过 `clock.wait_for_tti(tti)`
  与中央时钟同步，独占自己的 `ue_context`；
- **一个 eNB 线程**（`run_enb(...)`），独占访问 `scheduler`/`enb_bsr`/`enb_harq`，
  依次 `dequeue` 四信道消息、执行调度与接收；
- 四信道（`sr_channel`/`grant_channel`/`pdu_channel`/`phich_channel`）均为 `timed_channel<T>`，
  **发送方 `enqueue(item, send_tti)` 记 `available_tti = send_tti + 4`**，接收方仅当
  `current_tti >= available_tti` 才能 `dequeue`——即"信息建立后 4 TTI 才对端可见"。

以 `scenario2`（`main.cpp`）为例，UE 线程与 eNB 线程的交互为：

| 步骤 | 角色 | 动作 | 信道 / 时延 |
|------|------|------|-------------|
| 1 | UE 线程 | `ue.data_arrived()` + `ue.run_tti()` 触发 SR、编码 BSR | — |
| 2 | UE → eNB | `sr_channel.enqueue(sr_msg, tti)` | SR(PUCCH) **+4 TTI** |
| 3 | eNB 线程 | `scheduler.handle_sr(rnti)` 置 `sr_pending` | 收到 SR |
| 4 | eNB 线程 | `scheduler.schedule_ul(tti)` 生成 Grant | — |
| 5 | eNB → UE | `grant_channel.enqueue(grant_msg, tti)` | UL Grant(PDCCH) **+4 TTI** |
| 6 | UE 线程 | `ue.handle_ul_grant()` 内 `pack_bsr_only` 把 BSR 打包进 `last_pdu_` | 收到 Grant |
| 7 | UE → eNB | `pdu_channel.enqueue(pdu_msg, tti)`（含 grant 副本 + PDU 字节流） | MAC PDU(PUSCH) **+4 TTI** |
| 8 | eNB 线程 | `enb_handle_bsr_pdu()` 解 PUSCH PDU 取 BSR → `receive_tb()` 软合并+CRC | 收到 PDU |
| 9 | eNB → UE | `phich_channel.enqueue(phich_msg, tti)`（`ack = rx.crc_ok`） | PHICH **+4 TTI** |
| 10 | UE 线程 | `ue.handle_harq_feedback()` 收 ACK/NACK；`scheduler.handle_ul_crc()` | 收到 PHICH |

> **注意**：方案 C 已建模**信道传播时延**——SR/Grant/PDU/PHICH 四者各带 `CHANNEL_PROPAGATION_TTI = 4`
> 的可见时延（见 `include/ul_mac/tti_channel.h`），所以主流程中每一步交互都不再是"同一 TTI 即时闭环"，
> 而是经过 4 TTI 的空口传播才被对端观察到。这与真实 LTE 的时序（DCI 0 → n+4 PUSCH → n+8 PHICH）方向一致，
> 仅数值取 4 作演示常量。`ul_harq_manager` 内部的 `harq_rtt_ttis_` 字段保留但本主流程时延由
> `timed_channel` 统一建模，不再依赖它。

---

## 4. 辅助函数作用

| 函数 | 位置 | 作用 |
|------|------|------|
| `print_separator(title)` | `main.cpp` | 打印带标题的分隔线 |
| `print_ue_metrics(m)` | `main.cpp` | 打印单 UE 统计（SR/BSR/HARQ/吞吐） |
| `print_enb_rx_stats(enb_bsr, enb_harq)` | `main.cpp` | 打印 eNB 接收侧统计（BSR 解码计数 + HARQ 收发/软合并/丢弃） |
| `enb_handle_bsr_pdu(enb_bsr, scheduler, rnti, pdu)` | `main.cpp` | **eNB 接收桥（方案 B）**：用 `mac_pdu_unpacker::unpack` 解 PUSCH MAC PDU 字节流还原 BSR CE，再 `enb_bsr.receive_bsr()` 写 per-UE LCG 视图，逐 LCG `scheduler.handle_bsr()` 喂调度器。BSR 真实走 MAC PDU 打包/解包，与协议一致 |

### `enb_handle_bsr_pdu` 的桥接细节（方案 B）

```cpp
// 解 PUSCH PDU 还原 BSR（UE 侧在 handle_ul_grant 内 pack_bsr_only 打包）
mac_pdu_unpacker unpacker;
auto bsr = unpacker.unpack(pdu);            // PDU 字节流 → BSR CE (结构化对象)
if (bsr.reports.empty()) return;
enb_bsr.receive_bsr(rnti, bsr);             // 索引 → 字节下界, 写 per-UE LCG 视图
for (uint32_t i = 0; i < NOF_LCGS; i++) {
    uint32_t buf = enb_bsr.get_ul_buffer(rnti, i);
    if (buf > 0)
        scheduler.handle_bsr(rnti, i, bytes_to_bsr_index(buf)); // 转回索引写入调度器缓冲变量
}
```

**与旧方案（A）的关键区别**：旧版 `make_ue_bsr_ce` + `enb_handle_bsr_ul` 让 BSR 以结构化
`bsr_ce` 在内存中**直传**（UE 线程直接调 eNB 侧函数），未经过空口字节流。方案 B 改为：
UE 在 `ue_context::handle_ul_grant()` 内调用 `mac_pdu_packer::pack_bsr_only` 把 BSR CE 封装进
UL-SCH MAC PDU 字节流（`ue_context::last_pdu_`），随 PUSCH 经 `pdu_channel` 上报；eNB 侧
`enb_handle_bsr_pdu()` 用 `mac_pdu_unpacker::unpack` 解包还原 CE。**BSR 现在真实走 MAC PDU
打包/解包**，与协议 TS 36.321 §6.1.3 一致（`mac_pdu_packer/unpacker` 能力原本独立存在，现正式接入 BSR 链路）。

---

## 5. 五个场景差异对照

> 方案 C 下 5 个场景**共用同一套多线程 + 中央时钟 + 四信道架构**（每 UE 线程 + eNB 线程 + 四
> `timed_channel`），仅 UE 数量、信道质量、调度算法、增强特性的配置不同。场景 2 为首个改写为
> 多线程原型的场景，其余场景在原型基础上做配置推广。

| 场景 | 函数 | UE 数 | 信道(ul_snr) | 调度算法 | 重点演示（方案 C 行为） |
|------|------|-------|--------------|----------|----------|
| 1 | `scenario1_basic_ul_scheduling` | 1 | 2000 (20dB) | PF | 基本链路：SR→BSR解码→Grant→HARQ 多线程闭环 |
| 2 | `scenario2_multi_ue_pf` | 5 | 2000 (20dB) | PF | **多线程原型**：5 UE 线程 + eNB 线程 + 中央时钟 + 四信道；多 UE 公平共享 PRB |
| 3 | `scenario3_harq_retx` | 1 | **200 (2dB)** | RR | **弱信道新传 NACK→重传 IR 增益后 ACK**；eNB 线程用 `pdu_msg.grant` 收 TB，atomic 计数 retx/fail |
| 4 | `scenario4_enhanced_features` | 1 | 2000 (20dB) | PF | 自适应 SR 周期 + 调度实时性度量（流量模式与 `adjust_sr_period` 移入 UE 线程） |
| 5 | `scenario5_epf` | 5 | 2000/200(弱) | **EPF** | 华为增强型PF：QoS权重(VoIP/视频/BE) + 信道感知 + 饿死保护；`vector<atomic<uint32_t>> sched_count` 按 rnti 递增 |

### 场景3 的 HARQ 软合并验证逻辑

```cpp
enb_harq.set_ul_snr(0x0001, 200);   // 弱信道 2dB
// 新传:   eff = 200 < 300(解码阈值)         → NACK
// 重传:   eff = 200 + 200(IR_GAIN) = 400 >= 300 → ACK
```

这是本项目软合并模型（`eff_snr = ul_snr + (combined_count-1)*IR_GAIN`）的核心卖点演示：
弱信号下新传失败，但重传累积 IR 增益后边缘 MCS 也能解调成功。方案 C 下，eNB 线程从
`pdu_channel` 取出 `pdu_msg` 后直接用 `msg.grant` 调 `receive_tb(rnti, msg.grant)`，
重传/丢弃计数用 `std::atomic` 在 eNB 线程内安全累加。

---

## 6. 涉及的所有核心函数速查表

### UE 发送桩侧（`ue_context` 及其组件）
| 函数 | 文件 | 作用 |
|------|------|------|
| `ue_context::data_arrived(lcid, bytes)` | ue_context.h:90 | RLC 数据到达 → 更新 LCG 缓冲区 + 记录到达 TTI |
| `ue_context::run_tti(tti)` | ue_context.h:107 | 每 TTI：BSR step → SR step → 令牌桶步进 |
| `ue_context::handle_ul_grant(grant)` | ue_context.h:120 | 处理 eNB 授权：清 SR、组 BSR、HARQ 新传/重传、消耗缓冲区、算延迟 |
| `ue_context::handle_harq_feedback(pid, ack)` | ue_context.h:176 | 模拟 PHICH 到达 UE，更新 HARQ 反馈 |
| `bsr_manager::step / generate_bsr / select_bsr_format` | ue_bsr_manager.cpp | BSR 触发判断、编码组包、格式选择 |
| `sr_manager::step / notify_ul_grant_received / adjust_sr_period` | ue_sr_manager.cpp | SR 状态机、授权到达清 SR、自适应周期 |
| `ul_harq_manager::new_grant_ul / handle_harq_feedback / generate_retx` | ue_ul_harq_manager.cpp | HARQ 新传/重传生成、反馈处理 |

### eNB 接收链路侧
| 函数 | 文件 | 作用 |
|------|------|------|
| `ul_scheduler::handle_sr(rnti)` | enb_ul_scheduler.cpp:42 | 置 `sr_pending` 标志（**无独立 SR Manager**） |
| `ul_scheduler::handle_bsr(rnti, lcg, idx)` | enb_ul_scheduler.cpp | 接收 BSR 索引，更新 `ul_buffer[lcg]` |
| `ul_scheduler::schedule_ul(tti)` | enb_ul_scheduler.cpp | 执行 PF/RR/EPF 三种算法，分配 PRB + HARQ PID，输出 `ul_grant` |
| `ul_scheduler::schedule_epf(tti)` | enb_ul_scheduler.cpp | EPF 核心：重传优先 → 饿死保底 → EPF 度量排序新传 |
| `ul_scheduler::compute_epf_metric(ctx, tti)` | enb_ul_scheduler.cpp | EPF 度量 = w_qos·(R_inst/R_avg^α)·(1+β·cqi_norm)，含饿死放大 |
| `ul_scheduler::configure_epf(epf_params)` | enb_ul_scheduler.cpp | 配置公平性因子 α / 信道感知 β / QoS 缩放 γ / 饿死参数 |
| `ul_scheduler::set_ue_qos(rnti, qos_class)` | enb_ul_scheduler.cpp | 注入 UE 业务类型 (VoIP/视频/BE) 及其差异化权重 |
| `ul_scheduler::handle_ul_crc(rnti, pid, ack)` | enb_ul_scheduler.cpp | 接收 CRC 结果，清/留重传标志 |
| `enb_bsr_manager::receive_bsr(rnti, bsr)` | enb_bsr_manager.cpp | 解码 BSR CE，维护 per-UE LCG 视图 |
| `enb_ul_harq_manager::receive_tb(rnti, grant)` | enb_ul_harq_manager.cpp | 软合并 + CRC，产生 PHICH 反馈（核心） |
| `enb_ul_harq_manager::get_phich(rnti, pid)` | enb_ul_harq_manager.cpp:178 | 查询最近 PHICH 值（本项目由 main 直接取 `rx.crc_ok`） |

---

## 7. 协议符合性提示（详见 `ENB_UE_DUALITY.md` 与各源文件注释）

`main.cpp` 使用的模块在以下方面为**演示级简化**，已在对应源文件以
`【协议说明 / 简化实现】` 注释标注：

1. PHICH 用 `phich_channel` 独立信道模拟（含 4-TTI 传播时延），方向正确但未建模真实物理层编码。
2. **信道传播时延已建模**：SR/Grant/PDU/PHICH 四信道各带 `CHANNEL_PROPAGATION_TTI = 4` 的可见时延，
   对端需 `send_tti + 4` 才能读取（见 `include/ul_mac/tti_channel.h`）。数值取 4 为演示常量，
   与真实 LTE 时序（DCI 0 → n+4 PUSCH → n+8 PHICH）方向一致但非精确值。
3. 多线程架构：每 UE 一个 `std::thread` + 一个 eNB 线程 + 中央 `tti_clock` 线程；UE 线程独占
   `ue_context`，eNB 线程独占 `scheduler/enb_bsr/enb_harq`，四信道 `timed_channel<T>` 做时间解耦，
   `std::async` 并行 `receive_tb`，manager 内部 mutex 作额外保护。
4. `MAX_HARQ_PROCESSES = 8` 为 LTE 4G 上行固定值（见 `common_types.h`，已按 LTE 规范取值）。
5. TBS 用线性插值近似，非 3GPP 离散查表。
6. 重传锁定 MCS/PRB（非自适应），未实现 eNB 自适应重传。
7. eNB 侧无独立 SR Manager，SR 退化为 `sr_pending` 标志（经 `sr_channel` 上报）。
8. RA（随机接入）未在 MAC 层实现；SR 失败回调仅作日志占位（见 `ue_sr_manager.cpp` /
   `ue_context.h` 注释）。
```
