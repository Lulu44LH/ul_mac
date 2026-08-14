# main.cpp 调用流程与函数作用说明

> 项目：`ul_mac` (LTE 上行 MAC 教学仿真)
> 文件：`src/main.cpp`
> 定位：eNB 侧上行 MAC **接收链路**演示程序。UE 侧原模块 (`bsr_manager` / `sr_manager` /
> `ul_harq_manager`) 被保留为"发送桩"，仅用于驱动数据到达、SR/BSR 编码、TB 发送；
> eNB 侧由三个核心模块构成接收链路：`enb_bsr_manager` + `ul_scheduler` + `enb_ul_harq_manager`。

---

## 1. 整体架构：发送桩 ↔ 接收链路

```
                        每 TTI 数据流
 ┌──────────────────────────────────────────────────────────────┐
 │  UE 发送桩 (ue_context)              eNB 接收链路              │
 │  ───────────────────               ───────────────            │
 │  data_arrived()                      handle_sr()             │
 │       │                              (sr_pending 置位)         │
 │  run_tti()  ── 触发 SR / 组包 BSR      │                       │
 │       │                              enb_bsr.receive_bsr()    │
 │  get_all_lcg_buffer_sizes() ───────►  (解码 6-bit 索引→bytes)   │
 │                                       │                       │
 │                                       scheduler.handle_bsr()  │
 │                                       scheduler.schedule_ul() │
 │                                            │ 生成 ul_grant     │
 │                                            ▼                  │
 │  enb_harq.receive_tb(grant) ──► 软合并+CRC ──► 产生 PHICH       │
 │       │                                     │                 │
 │  ue.handle_ul_grant() ◄── grant ───────────┘                  │
 │  ue.handle_harq_feedback() ◄── hi_value (PHICH)               │
 │                                            │                  │
 │  scheduler.handle_ul_crc() ◄── crc_ok/discard                 │
 └──────────────────────────────────────────────────────────────┘
```

**关键耦合点**：
- `grant.hi_value = rx.crc_ok`（`main.cpp:182`）：eNB 接收端把 CRC 结果写入 PHICH 值，
  经 `ul_grant` 回传给 UE —— 这是用结构体字段**模拟真实 PHICH 信道**的做法（真实系统里
  PHICH 是独立物理信道，UE 通过 `get_phich()` 读取，本项目用 `main.cpp` 显式赋值等效）。
- `scheduler.handle_ul_crc(rnti, pid, rx.discarded ? true : rx.crc_ok)`（`main.cpp:189`）：
  把 eNB 接收结果回馈给调度器，用于清除/保留重传标志。

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

## 3. 通用 TTI 循环（5 个场景的公共骨架）

每个 scenario 的 `for (tti = 0; tti < N; tti++)` 循环都遵循如下 8 步顺序。以
`scenario1`（`main.cpp:158-192`）为例：

| 步骤 | 代码位置 | 函数/作用 | 所属侧 |
|------|----------|-----------|--------|
| 1 | `:160-161` | `ue.data_arrived(lcid, bytes)` — 模拟 RLC 数据到达，写入 LCG 缓冲区 | UE 桩 |
| 2 | `:162` | `ue.run_tti(tti)` — 驱动 BSR 触发判断 + SR 状态机 + 令牌桶步进 | UE 桩 |
| 3 | `:165-167` | 若 `sr_state::PENDING` → `scheduler.handle_sr(rnti)` 置 `sr_pending` | eNB |
| 4 | `:170-171` | 取 `lcg_sizes` → `enb_handle_bsr_ul()` 编码+解码+喂调度器 | eNB |
| 5 | `:174` | `scheduler.schedule_ul(tti)` — 执行调度算法，输出 `sched_result` 列表 | eNB |
| 6 | `:180` | `enb_harq.receive_tb(rnti, grant)` — 软合并+CRC，返回 `rx_result` | eNB |
| 7 | `:182` | `grant.hi_value = rx.crc_ok` — 把 PHICH 反馈值写回 grant | eNB→UE 桥 |
| 8 | `:185-190` | `ue.handle_ul_grant()` + `ue.handle_harq_feedback()` + `scheduler.handle_ul_crc()` | 两侧闭环 |

> **注意**：步骤 6 在真实系统中发生在"UE 发送后约 8 TTI（HARQ RTT）"，但本项目为演示
> 方便**在同一 TTI 内即时闭环**（无 RTT 延迟建模）。`ul_harq_manager` 内部 `harq_rtt_ttis_`
> 字段保留但主流程未用。

---

## 4. 辅助函数作用

| 函数 | 位置 | 作用 |
|------|------|------|
| `print_separator(title)` | `:54` | 打印带标题的分隔线 |
| `print_ue_metrics(m)` | `:62` | 打印单 UE 统计（SR/BSR/HARQ/吞吐） |
| `print_enb_rx_stats(enb_bsr, enb_harq)` | `:78` | 打印 eNB 接收侧统计（BSR 解码计数 + HARQ 收发/软合并/丢弃） |
| `make_ue_bsr_ce(lcg_sizes)` | `:97` | **UE 发送桩**：模拟 `bsr_manager::generate_bsr` 的 Long BSR 分支，把各 LCG 字节数压成 6-bit 索引的 `bsr_ce` |
| `enb_handle_bsr_ul(enb_bsr, scheduler, rnti, lcg_sizes)` | `:112` | **eNB 接收桥**：①调用 `make_ue_bsr_ce` 编码；②`enb_bsr.receive_bsr()` 解码；③逐 LCG 调 `scheduler.handle_bsr()` 把缓冲视图喂给调度器 |

### `enb_handle_bsr_ul` 的桥接细节（`:112-128`）

```cpp
bsr_ce bsr = make_ue_bsr_ce(lcg_sizes);   // UE 编码
if (bsr.reports.empty()) return;
enb_bsr.receive_bsr(rnti, bsr);          // eNB 解码 → 维护 per-UE LCG 视图
for (uint32_t i = 0; i < NOF_LCGS; i++) {
    uint32_t buf = enb_bsr.get_ul_buffer(rnti, i);
    if (buf > 0)
        scheduler.handle_bsr(rnti, i, bytes_to_bsr_index(buf)); // 转回索引喂调度器
}
```

注意这里**编码→解码→再编码**的往返：UE 把字节压成索引（`make_ue_bsr_ce`），
eNB 把索引解回字节（`receive_bsr` 内部 `bsr_index_to_bytes`），再调 `handle_bsr` 时又
`bytes_to_bsr_index(buf)` 压回索引。这是为了演示"eNB 解码视图 ↔ 调度器索引口径"一致，
真实系统中调度器直接消费解码后的字节数即可（此处多一次往返仅作教学演示）。

---

## 5. 四个场景差异对照

| 场景 | 函数 | UE 数 | 信道(ul_snr) | 调度算法 | 重点演示 |
|------|------|-------|--------------|----------|----------|
| 1 | `scenario1_basic_ul_scheduling` | 1 | 2000 (20dB) | PF | 基本链路：SR→BSR解码→Grant→HARQ |
| 2 | `scenario2_multi_ue_pf` | 5 | 2000 (20dB) | PF | 多 UE 公平共享 PRB |
| 3 | `scenario3_harq_retx` | 1 | **200 (2dB)** | RR | **弱信道新传 NACK→重传 IR 增益后 ACK** |
| 4 | `scenario4_enhanced_features` | 1 | 2000 (20dB) | PF | 自适应 SR 周期 + 调度实时性度量 |
| 5 | `scenario5_epf` | 5 | 2000/200(弱) | **EPF** | 华为增强型PF：QoS权重(VoIP/视频/BE) + 信道感知 + 饿死保护 |

### 场景3 的 HARQ 软合并验证逻辑（`:294-296`）

```cpp
enb_harq.set_ul_snr(0x0001, 200);   // 弱信道 2dB
// 新传:   eff = 200 < 300(解码阈值)         → NACK
// 重传:   eff = 200 + 200(IR_GAIN) = 400 >= 300 → ACK
```

这是本项目软合并模型（`eff_snr = ul_snr + (combined_count-1)*IR_GAIN`）的核心卖点演示：
弱信号下新传失败，但重传累积 IR 增益后边缘 MCS 也能解调成功。

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

1. PHICH 用 `grant.hi_value` 字段模拟，真实系统为独立物理信道。
2. HARQ 反馈即时闭环，未建模 8-TTI RTT 延迟（虽 `harq_rtt_ttis_` 字段保留）。
3. `MAX_HARQ_PROCESSES = 8` 为 LTE 4G 上行固定值（见 `common_types.h`，已按 LTE 规范取值）。
4. TBS 用线性插值近似，非 3GPP 离散查表。
5. 重传锁定 MCS/PRB（非自适应），未实现 eNB 自适应重传。
6. eNB 侧无独立 SR Manager，SR 退化为 `sr_pending` 标志。
7. RA（随机接入）未在 MAC 层实现；SR 失败回调仅作日志占位（见 `ue_sr_manager.cpp` /
   `ue_context.h` 注释）。
```
