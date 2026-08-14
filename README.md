# MAC层上行用户管理演示项目 (UL MAC Manager)

> 基于 srsRAN 开源项目（ocudu 5G NR + srsRAN_4G LTE）的定制化开发  
> 参考协议：3GPP TS 36.321 (LTE MAC) / TS 38.321 (NR MAC)

---

## 目录

1. [项目背景与技术架构](#1-项目背景与技术架构)
2. [代码结构说明](#2-代码结构说明)
3. [核心功能模块设计与实现](#3-核心功能模块设计与实现)
4. [编译构建指南](#4-编译构建指南)
5. [测试验证方法](#5-测试验证方法)
6. [面试重点标注](#6-面试重点标注)
7. [常见面试问题预测](#7-常见面试问题预测)
8. [技术难点解析](#8-技术难点解析)
9. [与标准协议的符合性分析](#9-与标准协议的符合性分析)

---

## 1. 项目背景与技术架构

### 1.1 项目目标

本项目面向通信工程领域 MAC 子层上行用户管理的实习与面试展示，基于 srsRAN 开源 4G/5G 协议栈项目，提取并增强 MAC 层上行管理核心机制，实现一个独立可运行的仿真演示系统。

### 1.2 参考项目

| 项目 | 目录 | 技术 | 说明 |
|------|------|------|------|
| srsRAN_4G | `srsRAN_4G/` | LTE | srsUE 侧 MAC 实现（sr_proc, bsr_proc, ul_harq_entity） |
| ocudu | `ocudu/` | 5G NR | gNB 侧 MAC 实现（mac_impl, scheduler） |

### 1.3 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    演示系统架构                            │
├─────────────────────────────────────────────────────────┤
│                                                          │
│   ┌────────────┐   UL Grant    ┌──────────────────┐    │
│   │  eNB调度器  │ ◄──────────► │   UE上下文管理器   │    │
│   │(ul_scheduler)│              │  (ue_context)     │    │
│   │             │   SR/BSR     │                   │    │
│   │ • PF算法    │              │ ┌───────────────┐ │    │
│   │ • RR算法    │              │ │ sr_manager    │ │    │
│   │ • EPF算法   │              │ │ (SR调度请求)   │ │    │
│   │             │              │ ├───────────────┤ │    │
│   │ • UL Grant  │              │ │ bsr_manager   │ │    │
│   │   生成      │              │ │ (BSR缓冲区)   │ │    │
│   │ • HARQ反馈  │              │ ├───────────────┤ │    │
│   │   管理      │              │ │ul_harq_manager│ │    │
│   └────────────┘              │ │ (HARQ重传)    │ │    │
│                               │ ├───────────────┤ │    │
│                               │ │lcg_buffer_mgr │ │    │
│                               │ │ (LCG缓冲区)   │ │    │
│                               │ └───────────────┘ │    │
│                               └───────────────────┘    │
│                                                          │
│   ┌──────────────────────────────────────────────┐     │
│   │              横切关注点                         │     │
│   │  mac_logger (日志系统) + metrics_collector    │     │
│   └──────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────┘
```

### 1.4 上行调度流程

```
UE侧数据到达 → LCG缓冲区更新 → 触发Regular BSR
     │                              │
     │  无PUCCH资源?                │
     ▼                              ▼
  发送SR ←────── BSR需要资源 ←── 生成BSR CE
  (PUCCH)            │
     │               │
     ▼               ▼
  eNB收到SR → 调度器分配UL Grant → 发送给UE
                    │
                    ▼
           UE收到UL Grant
           │           │
           ▼           ▼
     发送BSR+数据   HARQ管理新传/重传
           │
           ▼
     eNB收到 → CRC校验 → ACK/NACK
                    │
                    ▼
             NACK → 调度重传
             ACK  → 释放HARQ进程
```

---

## 2. 代码结构说明

### 2.1 目录结构

```
ul_mac_manager/
├── CMakeLists.txt                    # 构建配置
├── include/ul_mac/
│   ├── common_types.h               # 公共类型定义（BSR格式、SR状态、HARQ状态、UL Grant等）
│   ├── mac_logger.h                 # 日志系统（线程安全、带时间戳）
│   ├── metrics_collector.h          # 性能指标收集器（单例模式）
│   ├── lcg_buffer.h                 # LCG缓冲区管理器
│   ├── ue_sr_manager.h              # SR调度请求管理器
│   ├── ue_bsr_manager.h             # BSR缓冲区状态报告管理器
│   ├── ue_ul_harq_manager.h         # UL HARQ重传管理器
│   ├── enb_ul_scheduler.h           # eNB侧上行调度器
│   ├── enb_bsr_manager.h            # eNB侧BSR解码器（per-UE LCG视图）
│   ├── enb_ul_harq_manager.h        # eNB侧HARQ接收（IR软合并 + CRC/PHICH判定）
│   ├── mac_pdu.h                    # MAC PDU 组包/解包 (TS 36.321 §6.1.2, P1新增)
│   └── ue_context.h                 # UE上下文（整合所有组件）
├── src/
│   ├── main.cpp                     # 演示入口（4个仿真场景）
│   ├── ue_sr_manager.cpp            # SR管理器实现
│   ├── ue_bsr_manager.cpp           # BSR管理器实现
│   ├── ue_ul_harq_manager.cpp       # HARQ管理器实现
│   ├── enb_ul_scheduler.cpp         # 调度器实现 (含PF/RR/EPF + 实时性统计)
│   ├── enb_bsr_manager.cpp          # eNB侧BSR解码器
│   ├── enb_ul_harq_manager.cpp      # eNB侧HARQ接收 (IR软合并 + PHICH)
│   └── mac_pdu.cpp                  # MAC PDU 组包/解包实现 (P1新增)
```

### 2.2 关键类说明

| 类名 | 文件 | 职责 | 对应 srsRAN 原型 |
|------|------|------|-----------------|
| `sr_manager` | ue_sr_manager.h/cpp | SR状态机、PUCCH资源管理、自适应SR周期 | `srsue::mac::sr_proc` |
| `bsr_manager` | ue_bsr_manager.h/cpp | BSR触发、格式选择、定时器管理、Padding BSR抑制(非标准优化) | `srsue::mac::bsr_proc` |
| `ul_harq_process` | ue_ul_harq_manager.h/cpp | 单HARQ进程状态机（NDI/RV管理） | `srsue::mac::ul_harq_process` |
| `ul_harq_manager` | ue_ul_harq_manager.h/cpp | HARQ实体（8进程管理、统计） | `srsue::mac::ul_harq_entity` |
| `ul_scheduler` | enb_ul_scheduler.h/cpp | eNB调度器（PF/RR/EPF 三种） | `srsenb::mac::sched` |
| `enb_bsr_manager` | enb_bsr_manager.h/cpp | eNB侧BSR解码、per-UE LCG缓冲视图 | `srsenb::mac`（BSR处理） |
| `enb_ul_harq_manager` | enb_ul_harq_manager.h/cpp | eNB侧HARQ接收（IR软合并 + 确定性SNR CRC判定 + PHICH反馈） | `srsenb::mac::ul_harq` |
| `mac_pdu_packer` / `mac_pdu_unpacker` | mac_pdu.h/cpp | UL MAC PDU 组包/解包（subheader + BSR CE + SDU复用 + Padding） | `srsenb::mac::mac_pdu` |
| `ue_context` | ue_context.h | UE上下文整合 | `srsue::mac::mac` |
| `lcg_buffer_manager` | lcg_buffer.h | LCG缓冲区聚合 | `srsue::mac::bsr_proc::lcg_buffer_state` |
| `metrics_collector` | metrics_collector.h | 系统级性能监控 | `srsenb::mac::mac_metrics` |

---

## 3. 核心功能模块设计与实现

### 3.1 SR（调度请求）管理器

**协议参考**：TS 36.321 §5.4.4 / TS 38.321 §5.4.4

**状态机设计**：
```
    ┌─────────┐  数据到达     ┌───────────┐
    │  IDLE   │ ──────────►  │  PENDING  │
    └─────────┘              └─────┬─────┘
         ▲                         │ SR周期到达
         │ UL Grant                ▼
         │ 收到              ┌─────────────┐
         ├─────────────────  │ TRANSMITTING│
         │                   └──────┬──────┘
         │                          │
         │                    ┌─────┴──────┐
         │              成功   │            │ 失败(dsr_transmax)
         │                    ▼            ▼
         │               ┌────────┐  ┌─────────┐
         └───────────────│  IDLE  │  │ FAILED  │→触发RA
                         └────────┘  └─────────┘
```

**增强功能 — 自适应SR周期**：
- 基于流量速率的EMA（指数移动平均）动态调整SR发送周期
- 使用对数连续映射公式 `period = clamp(K / log2(1 + rate), 5, 80)`，平滑过渡无离散跳变
- 迟滞机制：仅当新周期与当前周期差异超过20%时才调整，避免阈值附近频繁切换
- 减少PUCCH资源浪费，同时保证高流量场景的低延迟

**对应srsRAN源码**：
- `srsRAN_4G/srsue/src/stack/mac/proc_sr.cc` 中的 `sr_proc::step()` 方法

### 3.2 BSR（缓冲区状态报告）管理器

**协议参考**：TS 36.321 §5.4.5 / TS 38.321 §5.4.5

**三种触发类型**：
1. **Regular BSR**：新数据到达且优先级高于当前缓冲区 / 新数据到达空LCG
2. **Periodic BSR**：periodicBSR-Timer 超时
3. **Padding BSR**：UL Grant 剩余空间足够填充 BSR CE

**BSR格式选择算法**：
```
UL Grant可用空间 → 计算可报告LCG数
  │
  ├─ 只有1个LCG有数据 → Short BSR (1字节: LCG ID 2bit + BS Index 6bit)
  ├─ 多个LCG且空间够  → Long BSR (3字节: 4个LCG各6-bit BS Index)
  └─ 空间不够报全部   → Truncated BSR (按优先级截断)
```

**增强功能 — Padding BSR 抑制（非 3GPP 标准，仅本项目开销优化）**：
- Padding BSR场景下，若所有LCG缓冲区索引未变化则跳过发送，减少空口信令开销
- 注意：3GPP 标准无"差分 BSR"机制；Regular/Periodic BSR 仍按标准完整上报

**BSR缓冲区大小映射**：
- 3GPP 标准表（TS 36.321 Table 6.1.3.1-1）为 64 级指数映射（Index 0~63 → 0~150000 bytes，比值约 1.12，Index 63 表示 ">150000"）
- 本项目采用同构的**简化量化表**（Index 0~63 → 0~25000 bytes，见 `common_types.h` 的 `bsr_index_to_bytes()`），上限小于标准表；`bytes_to_bsr_index()` 为向下取整的逆映射（避免高估缓冲区）

### 3.3 UL HARQ 重传管理器

**协议参考**：TS 36.321 §5.4.2 / TS 38.321 §5.4.2

**HARQ进程状态机**：
```
INACTIVE ──新传──► ACTIVE(IDLE)
                      │
                      ├── 发送 ──► ACTIVE(TX_PENDING)
                      │               │
                      │          ┌────┴─────┐
                      │        ACK          NACK
                      │          │            │
                      │          ▼            ▼
                      │      INACTIVE   ACTIVE(RETX_PENDING)
                      │                     │
                      │                     ├── 重传 ──► TX_PENDING
                      │                     │
                      │                     └── max_retx达到 ──► INACTIVE(丢弃)
```

**关键决策逻辑（new_grant_ul）**：
1. 若有 PHICH 反馈 → 处理 ACK/NACK
2. 检查是否达到最大重传次数（max_harq_tx）
3. NDI 翻转 → 新传输；NDI 未翻转 → 自适应重传
4. 无 NDI（非自适应重传）→ 按原参数重传

**RV（冗余版本）序列**：
- LTE: `{0, 2, 3, 1}` — 通过 IRV 计数器循环
- RV=0 包含最多系统比特，优先用于首次传输

**增强功能**：
- RTT延迟建模：HARQ反馈需经过RTT（默认8 TTI）后才被处理，更真实地模拟PHICH反馈时序
- 早期终止：连续NACK≥3次且已重传≥2次时提前丢弃TB，避免浪费无线资源
- 令牌桶LCP：实现3GPP TS 36.321 §5.4.3.1两阶段逻辑信道优先级，PBR令牌桶控制各信道速率

**对应srsRAN源码**：
- `srsRAN_4G/srsue/src/stack/mac/ul_harq.cc` 中的 `new_grant_ul()` 方法

### 3.4 上行调度器（eNB侧）

**三种调度算法（PF / RR / EPF）**：

#### 比例公平调度（PF）
```
PF度量值 = 当前请求速率 / (历史平均速率 ^ fairness_coeff)
```
- 重传优先于新传（保证 HARQ RTT）
- PF 度量值越高 → 越优先获得资源
- 历史平均速率使用 EMA 平滑（α = 1/(n+1)）

#### 轮询调度（RR）
- 简单公平轮转，每个 UE 获得均等机会
- 适合对公平性要求严格的场景

#### 增强型比例公平调度（EPF，华为）
- 在 PF 基础上叠加 QoS 权重与信道感知增强项：`metric = w_qos·(R_inst/R_avg^α)·(1+β·cqi_norm)`
- 内置饿死保护（弱信道 UE 自动升优先级并保留最低 PRB 份额）
- 可配公平性因子 `alpha` / 信道感知 `beta` / QoS 缩放 `gamma`

> 原"基于缓冲区大小的优先级调度"已移除：仅按缓冲区排序不感知信道质量（弱信道 UE 会被长期挤占饿死）且无业务区分度。

**UL Grant 生成**：
- MCS 计算：基于 SNR→MCS 区间映射表（29 个阈值点，`calculate_mcs()`），替代早期线性公式
- PRB 计算：`required_prb = ceil(req_bytes / bytes_per_prb)`
- TBS 计算：基于3GPP TS 36.213 Table 7.1.7.2.1-1简化查找表（6个锚点+线性插值）

---

## 4. 编译构建指南

### 4.1 依赖项

```bash
# 基础依赖（Ubuntu/Debian）
sudo apt update
sudo apt install -y build-essential cmake g++

# 最低版本要求
# CMake >= 3.10
# GCC >= 7 (支持 C++17)
```

### 4.2 编译步骤

```bash
cd ul_mac_manager
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

> **Windows + MinGW (g++) 直接使用**：本项目提供纯 `g++` 命令行编译路径（无需 CMake）：
>
> ```bat
> rem 演示主程序
> g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 -static -Iinclude src/main.cpp ^
>     src/ue_sr_manager.cpp src/ue_bsr_manager.cpp src/ue_ul_harq_manager.cpp ^
>     src/enb_ul_scheduler.cpp src/enb_bsr_manager.cpp src/enb_ul_harq_manager.cpp ^
>     src/mac_pdu.cpp -o build/ul_mac_manager.exe -pthread
> rem 单元测试
> g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 -static -Iinclude tests/test_main.cpp ^
>     src/ue_sr_manager.cpp src/ue_bsr_manager.cpp src/ue_ul_harq_manager.cpp ^
>     src/enb_ul_scheduler.cpp src/enb_bsr_manager.cpp src/enb_ul_harq_manager.cpp ^
>     src/mac_pdu.cpp -o build/ul_mac_manager_tests.exe -pthread
> ```
>
> 若在 MinGW 环境下使用 CMake，请指定生成器：`cmake -S . -B build -G "MinGW Makefiles"`。
>
> **注意**：`-static` 用于静态链接 C++ 运行时。若系统 PATH 中存在其他 MinGW 发行版（如 Git for Windows 自带的旧版 `libstdc++-6.dll`），动态链接的 exe 可能加载到版本不匹配的 DLL，导致启动时段错误（详见 PROTOCOL_NOTES.md §10.3 P2-4）。

### 4.3 运行

```bash
./build/ul_mac_manager
```

### 4.4 srsRAN 原项目编译（供参考）

```bash
# srsRAN_4G (LTE)
cd srsRAN_4G
mkdir build && cd build
cmake .. -DENABLE_UHD=OFF -DENABLE_BLADERF=OFF
make -j$(nproc)

# ocudu (5G NR)
cd ../../ocudu
mkdir build && cd build
cmake .. -DENABLE_EXPORT=OFF
make -j$(nproc)
```

---

## 5. 测试验证方法

### 5.1 仿真场景

| 场景 | 参数 | 验证目标 |
|------|------|---------|
| 场景1 | 单UE, 200TTI, PF, ul_snr=20dB（好信道） | SR→BSR→Grant→HARQ完整流程（一次成功为主） |
| 场景2 | 5UE, 1000TTI, PF, ul_snr=20dB（好信道） | 多UE比例公平调度公平性 |
| 场景3 | 单UE, 500TTI, RR, ul_snr=2dB（弱信道） | HARQ重传机制：新传 NACK、IR 软合并（+2dB/次）后 ACK |
| 场景4 | 单UE, 500TTI, 变化流量, ul_snr=20dB | 自适应SR周期 |

> 信道为**确定性 SNR 模型**：`enb_ul_harq_manager::receive_tb()` 按 `eff_snr = ul_snr + (合并次数-1)×2dB` 与 `MCS 解码阈值+1dB` 比较判定 CRC，无随机性、结果可复现（取代了早期的概率 BLER 模型）。

### 5.2 验证指标

- **SR成功率**：应接近100%（SR 信道无差错建模）
- **场景3重传行为**：ul_snr=2dB 时新传必然 NACK（eff=200 < 阈值300），第一次重传后 eff=200+200(IR增益)=400 ≥ 300 → ACK，即每个 TB 恰好重传 1 次，`avg_retx ≈ 1.0`
- **平均重传次数（概率模型下的验算基准）**：若改用概率 BLER=p 建模，每个 TB 的期望传输次数为 1/(1-p)、期望重传次数为 p/(1-p)（几何分布）；p=0.3 时重传次数理论值 ≈ 0.43，早期概率版本实测 0.49，同量级——用理论值验证仿真输出是重要的工程习惯
- **PF公平性**：5个UE吞吐率差异应 < 30%（信道条件相同时）
- **自适应SR**：高流量阶段SR周期应缩短，低流量应延长

### 5.3 日志验证

日志格式：`[HH:MM:SS.mmm] [LEVEL] [MODULE] UE[RNTI] TTI[XXXXX] Message`

关键日志检查：
- `Sending SR on PUCCH` → SR发送
- `Triggering Regular BSR` → BSR触发
- `New TX, RV=0, TBS=xxx` → HARQ新传
- `Adaptive ReTX=xxx` → HARQ自适应重传
- `Max ReTX reached` → 达到最大重传

### 5.4 自动化测试

项目包含34个自动化单元测试，覆盖所有核心组件：

```bash
# 编译并运行测试
cd build
cmake ..
make -j$(nproc)
./ul_mac_manager_tests

# 或使用 CTest
ctest --output-on-failure
```

| 测试类别 | 测试数 | 覆盖内容 |
|---------|--------|---------|
| LCP令牌桶 | 3 | PBR限制、令牌补充、无PBR退化 |
| MCS/TBS | 2 | SNR→MCS区间映射、TBS单调性 |
| HARQ | 5 | 新传/重传/最大重传丢弃/RTT延迟/早期终止 |
| SR | 2 | 状态机转换、自适应周期 |
| BSR | 2 | Regular触发、Padding BSR抑制(非标准) |
| 延迟统计 | 2 | P50/P90/P99计算、样本记录 |
| 死锁修复 | 1 | TB丢弃后pending_retx清除 |
| 线程安全 | 1 | get_all_process_info并发访问 |
| MAC PDU | 5 | Short/Long BSR编解码往返、SDU复用+Padding、多SDU子头链、空/越界容错 |

---

## 6. 面试重点标注

> **面试官重点关注领域** ⭐

### ⭐⭐⭐ 核心知识点

1. **SR过程（§5.4.4）**
   - SR 的触发条件（有数据但无PUCCH资源）
   - dsr-TransMax 限制与 RA 回退机制
   - SR 禁止定时器（sr-ProhibitTimer）的作用

2. **BSR过程（§5.4.5）**
   - 三种触发类型的区别和应用场景
   - BSR格式选择（Short/Long/Truncated）的判断条件
   - periodicBSR-Timer 和 retxBSR-Timer 的作用

3. **UL HARQ（§5.4.2）**
   - NDI翻转机制判断新传/重传
   - 自适应重传 vs 非自适应重传
   - RV序列 `{0, 2, 3, 1}` 的设计原因（CC编码特性）
   - 异步HARQ（LTE上行）vs 同步HARQ（LTE下行）

4. **比例公平调度**
   - PF度量值公式的物理含义
   - 公平性与效率的权衡
   - 历史平均速率的EMA更新

### ⭐⭐ 重要知识点

5. **LCG（逻辑通道组）**
   - 4个LCG，最多8个LC per LCG
   - BSR以LCG为单位上报（不是LC）

6. **UL Grant 资源计算**
   - MCS → TBS 的查表过程（TS 36.213 Table 7.1.7.2.1-1）
   - PRB分配与频率资源管理

### ⭐ 了解即可

7. **NR 5G 与 LTE 的差异**
   - NR 的 Configured Grant（Type1/Type2）vs LTE 的 SPS
   - NR 支持更多HARQ进程（最多16个）
   - NR 的BWP（带宽部分）对调度的影响

---

## 7. 常见面试问题预测

### Q1: 请描述UE发送上行数据的完整MAC层流程

**标准答案**：
1. RLC层将SDU放入MAC层逻辑通道缓冲区
2. MAC层检测到新数据（Regular BSR触发条件）
3. 如果UE没有PUCCH SR资源 → 触发SR过程
4. UE在PUCCH上发送SR（受dsr-TransMax限制）
5. eNB收到SR后，通过PDCCH分配初始UL Grant（可能只够发BSR）
6. UE使用UL Grant发送BSR（Short/Long/Truncated格式）
7. eNB解析BSR后，按缓冲区状态分配足够大的UL Grant
8. UE构造MAC PDU（包含BSR CE + 数据SDU），通过HARQ发送
9. eNB收到后进行CRC校验，发送ACK/NACK
10. NACK时触发重传（RV按{0,2,3,1}序列变化）

### Q2: BSR的三种触发类型有什么区别？

**标准答案**：
- **Regular BSR**：(1) 新数据到达优先级高于当前缓冲区中所有LC的数据；(2) 新数据到达空的LCG。立即触发SR（如果没有资源）。
- **Periodic BSR**：periodicBSR-Timer超时触发，周期性上报缓冲区状态，确保eNB有最新信息。
- **Padding BSR**：UL Grant的剩余空间足以填充BSR CE时触发，利用填充空间携带BSR，无需额外资源。

优先级：Regular > Periodic > Padding（同一TTI只发一个BSR）

### Q3: UL HARQ中如何判断是新传还是重传？

**标准答案**：
通过DCI中的NDI（New Data Indicator）字段判断：
- **NDI翻转**（与上次不同）→ 新传输：重置HARQ缓冲区，用新数据替换
- **NDI未翻转**（与上次相同）→ 重传：使用原HARQ缓冲区数据重传
- **特殊情况**：首次收到Grant（无历史NDI）视为新传；RAR中的Grant始终为新传（Msg3）

自适应重传时，eNB可以改变MCS和PRB（但TBS不变）。非自适应重传（无NDI的PHICH NACK）使用与上次完全相同的参数。

### Q4: 为什么HARQ的RV序列是{0, 2, 3, 1}而不是{0, 1, 2, 3}？

**标准答案**：
这与Turbo码/LDPC码的编码特性有关：
- RV=0 包含最多的系统比特（信息位），解码性能最好
- RV=2 包含最多的校验比特A
- RV=3 包含最多的校验比特B  
- RV=1 是混合类型

序列 `{0, 2, 3, 1}` 的设计使得前两次传输（RV=0 和 RV=2）就包含足够的系统和校验信息，实现最佳的增量冗余（IR）效果。如果使用 `{0, 1, 2, 3}`，第二次传输（RV=1）的自解码能力较弱，浪费一次传输机会。

### Q5: 比例公平(PF)调度算法的原理和优缺点？

**标准答案**：
PF度量值 = R_current(t) / R_avg(t)^α

- R_current：当前UE的可支持速率（由信道质量决定）
- R_avg：历史平均速率（EMA更新）
- α：公平性因子（通常取1）

**优点**：在系统吞吐率和用户公平性之间取得良好平衡。信道好的UE获得高速率，但不会饿死信道差的UE。
**缺点**：计算复杂度高于RR；需要维护每个UE的历史状态；对时变信道的响应有一定延迟。

实际系统中常用 PF 的变种：如加权PF（不同QoS等级不同权重）、最大C/I（纯吞吐率优化）、最大最小公平等。

### Q6: SR失败后会怎样处理？

**标准答案**：
当SR发送次数达到 dsr-TransMax（最大SR传输次数）：
1. MAC层通知RRC层SR失败
2. RRC层释放该UE的PUCCH SR资源和SRS资源
3. 触发随机接入（RA）过程，通过Msg1/Msg3重新请求资源
4. RA过程中的Msg3携带BSR，eNB可以通过Msg4中的UL Grant响应

这是一种回退机制，确保即使SR信道失步也能恢复上行通信。

### Q7: 请解释配置授权（Configured Grant）与动态授权的区别

**标准答案**：
- **动态授权（Dynamic Grant）**：每次传输都需要eNB通过PDCCH发送DCI指示，灵活但信令开销大、延迟高。
- **配置授权 Type1**：RRC信令直接配置周期、MCS、PRB等参数，UE无需PDCCH即可周期性发送。适合固定速率业务（如VoNR）。
- **配置授权 Type2**：RRC配置基本参数，PDCCH激活/去激活。比Type1更灵活。

配置授权是NR的新特性，对应LTE的SPS（半持续调度），但功能更强大。本项目中模拟的是动态授权方式。

---

## 8. 技术难点解析

### 8.1 HARQ RTT与进程管理

**难点**：上行HARQ RTT为8ms（FDD），意味着一个HARQ进程发送后需等待8ms才能收到反馈。在这8ms内如果有新数据到达，必须使用其他空闲HARQ进程。

**解决方案**：
- 维护 8 个 HARQ 进程（`MAX_HARQ_PROCESSES = 8`，LTE 4G 上行固定 8 进程填满 8ms RTT）
- 调度器轮转分配进程ID（`next_pid_ % MAX_HARQ_PROCESSES`）
- 重传优先：调度器先处理pending_retx的进程

### 8.2 BSR量化精度与资源效率

**难点**：BSR使用6bit的Buffer Size Index（0~63）表示缓冲区大小，但实际缓冲区可能远大于此。量化必然引入误差。

**解决方案**：
- 指数（对数）量化表：小缓冲区精度高（本项目简化表中 Index 10 → 52 bytes），大缓冲区精度低（Index 62 → 21956 bytes，Index 63 封顶 25000 bytes）；3GPP 标准表同构但上限为 150000 bytes（Index 63 表示 ">150000"）
- 这是合理的，因为大缓冲区通常意味着大量背景流量，精确值对调度影响不大

### 8.3 自适应SR

**难点**：标准SR周期固定，无法适应突发流量变化。高流量时需要更频繁的SR以降低延迟，低流量时需要更长的SR周期以节省PUCCH资源。

**解决方案**：
```cpp
// 自适应SR周期调整（对数连续映射 + 迟滞）
double ema_rate = alpha * current_rate + (1-alpha) * old_rate;
double log_rate = log2(1.0 + ema_rate);
uint32_t new_period = clamp(K / log_rate, MIN_PERIOD, MAX_PERIOD);
// 迟滞：变化不超过20%则不调整
if (abs(new_period - current_period) / current_period < 0.2) return;
```

### 8.4 线程安全设计

**难点**：仿真系统中UE侧和eNB侧可能在不同线程运行（实际系统中确实如此），共享状态需要同步保护。

**解决方案**：
- `ul_scheduler` 内部所有公开方法使用 `std::lock_guard<std::mutex>` 保护
- `ul_harq_process` 使用 `std::atomic` 原子变量存储关键状态
- `metrics_collector` 使用单例模式 + mutex 保护全局指标

---

## 9. 与标准协议的符合性分析

| 功能模块 | 协议章节 | 符合程度 | 说明 |
|---------|---------|---------|------|
| SR状态机 | TS 36.321 §5.4.4 | 高 | 完整实现SR触发、发送、dsr-TransMax限制、RA回退 |
| BSR触发 | TS 36.321 §5.4.5 | 高 | Regular/Periodic/Padding三种触发全部实现 |
| BSR格式 | TS 36.321 §6.1.3 | 高 | Short/Long/Truncated格式选择逻辑符合协议 |
| HARQ NDI | TS 36.321 §5.4.2.1 | 高 | NDI翻转判断新传/重传，TC-RNTI特殊处理 |
| RV序列 | TS 36.212 §5.2.2 | 高 | {0,2,3,1}序列正确实现 |
| PF调度 | 业界通用 | 中 | 实现基本PF算法，未包含QoS加权等高级特性 |
| PHICH处理 | TS 36.213 §8.0 | 中 | 确定性 SNR 阈值 + IR 软合并模型判定 CRC（eNB 侧 `enb_ul_harq_manager`），未实现完整的PHICH资源映射 |
| MCS/TBS | TS 36.213 §7.1.7 | 低 | 教学近似: SNR→MCS区间映射表 + TBS 6点线性插值 (非完整查表, 偏差可达40%+); UL Grant资源计算已与TBS自洽 |
| MAC PDU | TS 36.321 §6.1.2 | 中 | 已实现UL MAC PDU组包/解包: subheader(R/R/E/LCID)、Short/Long/Truncated BSR CE、SDU复用、Padding |

**项目简化说明**：
- 未实现下行HARQ和下行调度
- 信道模型为确定性 SNR 阈值 + IR 软合并判定（无随机衰落/噪声建模），结果可复现
- MAC PDU 为演示级实现：仅支持 <128B SDU（7-bit L 字段），不支持分段
- LCP 已实现令牌桶两阶段调度（UE 桩侧 `consume_data()`）；PHR 仅预留接口未参与调度
- 参数混用说明：LCG/BSR 格式采用 LTE（4 LCG、6-bit 索引），HARQ 进程数采用 NR 上限（16），属教学性简化

---

## 附录：性能指标示例输出

以下为当前版本实测输出（主程序依次运行场景 1~4，每个场景间重置指标，故下方摘要为**场景 4**（单 UE、500 TTI、变化流量、好信道 20dB）的结果）：

```
==============================================================
              MAC层上行管理系统性能指标摘要
==============================================================
仿真时长:           500 TTI (500 ms)
--------------------------------------------------------------
【调度请求 (SR)】
  总发送次数:        65
  成功次数:          65
  失败次数:          0
  成功率:            100.00%
--------------------------------------------------------------
【缓冲区状态报告 (BSR)】
  总发送次数:        68
  Short BSR:         0
  Long BSR:          68
  Truncated BSR:     0
--------------------------------------------------------------
【上行HARQ】
  新传次数:          130
  重传次数:          0
  失败次数:          0
  平均重传次数:      0.000
  失败率:            0.00%
--------------------------------------------------------------
【上行吞吐】
  总授权数:          65
  总传输字节:        51475 bytes
  系统吞吐率:        823.60 kbps
==============================================================
--------------------------------------------------------------
【延迟统计】
  样本数:            65
  最小延迟:          0 TTI
  P50 (中位数):      1 TTI
  P90:               1 TTI
  P99:               1 TTI
  最大延迟:          1 TTI
  平均延迟:          0.98 TTI
```

> 场景 3（弱信道 2dB）的 HARQ 部分则可观察到"每个 TB 新传 NACK、IR 软合并一次后 ACK"的确定性重传行为（`avg_retx ≈ 1.0`），与 §5.2 的理论分析一致。

---

*本项目基于 srsRAN (https://github.com/srsran/srsRAN) 开源项目定制化开发*  
*遵循 3GPP TS 36.321 / TS 38.321 MAC协议规范*
