# UL MAC Manager 学习路径与知识手册（C++ 初学者版）

> 本文档 = 学习路径 + 知识讲解教材。面向 C++ 初学者，通过本项目同时掌握：
> **① 现代 C++ 工程实践**（C++17、STL、多线程、设计模式）
> **② LTE/NR MAC 层上行管理协议知识**（SR / BSR / HARQ / 调度，对应 3GPP TS 36.321）
>
> 总周期约 4~5 周（每天 2~3 小时），共 6 个阶段。
> **使用方法**：按阶段顺序执行，完成一项勾选一项 `- [x]`，达成"里程碑"后进入下一阶段。
> 每个阶段包含：📖 知识讲解（可直接学习的内容）→ 🔍 代码精读指南 → ✏️ 动手练习 → 🏁 里程碑。

---

## 目录

- [预备知识：术语速查表](#预备知识术语速查表)
- [阶段 0：环境准备与项目跑通](#阶段-0环境准备与项目跑通05~1-天)
- [阶段 1：通用类型与工具层](#阶段-1通用类型与工具层2~3-天)
- [阶段 2：SR 管理器——状态机编程](#阶段-2sr-管理器状态机编程3~4-天)
- [阶段 3：BSR 管理器 + LCG 缓冲区——数据结构实战](#阶段-3bsr-管理器--lcg-缓冲区数据结构实战4~5-天)
- [阶段 4：UL HARQ 管理器——并发与组合设计](#阶段-4ul-harq-管理器并发与组合设计4~5-天)
- [阶段 5：调度器 + 全局整合——算法与架构](#阶段-5调度器--全局整合算法与架构4~5-天)
- [阶段 6：融会贯通与输出](#阶段-6融会贯通与输出3~5-天)
- [附录 A：C++ 知识点 ↔ 项目代码对照总表](#附录-ac-知识点--项目代码对照总表)
- [附录 B：总览时间表与学习建议](#附录-b总览时间表与学习建议)
- [附录 C：通信协议知识补充（面试防问）](#附录-c通信协议知识补充面试防问)
- [附录 D：项目修正记录（面试加分素材）](#附录-d项目修正记录面试加分素材)
- [附录 E：C++ 并发与多线程面试专题](#附录-ec-并发与多线程面试专题)

---

## 预备知识：术语速查表

初学者先通读一遍，学习过程中随时回来查阅。

### 通信术语

| 术语 | 全称 | 通俗解释 |
|---|---|---|
| **UE** | User Equipment | 用户设备，即手机 |
| **eNB/gNB** | evolved NodeB / next-gen NodeB | 4G/5G 基站 |
| **MAC** | Medium Access Control | 介质访问控制层，负责调度、复用、HARQ |
| **TTI** | Transmission Time Interval | 传输时间间隔，LTE 中 = 1ms，是调度的最小时间单位（本项目主循环每迭代一次 = 1 个 TTI） |
| **RNTI** | Radio Network Temporary Identifier | UE 在小区内的临时"身份证号"，16 位整数（如 0x0001） |
| **SR** | Scheduling Request | 调度请求。UE 有数据要发但没资源时，向基站"举手" |
| **BSR** | Buffer Status Report | 缓冲区状态报告。UE 告诉基站"我有多少数据要发" |
| **HARQ** | Hybrid ARQ | 混合自动重传请求。发错了就重传，并合并多次接收的信号 |
| **UL Grant** | Uplink Grant | 上行授权。基站分配给 UE 的上行发送资源（多少字节、什么调制方式） |
| **LC / LCID** | Logical Channel (ID) | 逻辑通道，不同业务走不同通道（信令走 SRB，数据走 DRB） |
| **LCG** | Logical Channel Group | 逻辑通道组，最多 4 个。BSR 按 LCG 粒度上报（不是按 LC） |
| **PUCCH** | Physical Uplink Control Channel | 上行控制信道，SR 在这上面发 |
| **PHICH** | Physical HARQ Indicator Channel | 下行 HARQ 反馈信道，基站用它发 ACK/NACK |
| **NDI** | New Data Indicator | 新数据指示位。**翻转 = 新传，不翻转 = 重传**（HARQ 核心机制） |
| **RV** | Redundancy Version | 冗余版本（0~3），每次重传用不同 RV 实现增量冗余 |
| **PRB** | Physical Resource Block | 物理资源块，频域资源的最小分配单位 |
| **MCS** | Modulation and Coding Scheme | 调制编码方案索引（0~28），信道越好 MCS 越高，速率越快 |
| **TBS** | Transport Block Size | 传输块大小，一次传输能装多少数据 |
| **SNR** | Signal-to-Noise Ratio | 信噪比，衡量信道质量 |
| **BLER** | Block Error Rate | 误块率，传输失败的概率 |
| **PF** | Proportional Fair | 比例公平调度算法 |
| **RR** | Round Robin | 轮询调度算法 |
| **Msg3** | Message 3 | 随机接入过程的第三条消息，HARQ 中有特殊处理 |
| **RA** | Random Access | 随机接入过程，SR 失败后的兜底手段 |

### 本项目核心数据流（先记住这张图）

```
UE侧数据到达 → LCG缓冲区更新 → 触发 Regular BSR → 触发 SR
                                                     │
        eNB 收到 SR ──→ 调度器分配 UL Grant ←─────────┘
                              │
        UE 收到 Grant → 发送 BSR + 数据 → HARQ 管理新传/重传
                              │
        eNB CRC 校验 → ACK(完成) / NACK(调度重传)
```

---

## 阶段 0：环境准备与项目跑通（0.5~1 天）

**目标**：让项目跑起来，理解 C++ 项目从源码到可执行文件的全过程。

### 📖 知识讲解

#### 0.1 C++ 项目的编译流程

C++ 是编译型语言，从源码到运行要经过 4 步：

```
预处理(展开#include/#define) → 编译(.cpp → .o 目标文件) → 链接(多个.o 合成可执行文件) → 运行
```

- **头文件（.h）**：放"声明"（类长什么样、函数叫什么名），被 `#include` 到多个 .cpp 中
- **源文件（.cpp）**：放"定义"（函数体的具体实现），每个 .cpp 独立编译成一个 .o
- **链接器**：把所有 .o 拼起来，解析函数调用关系。"undefined reference" 错误就发生在这一步

#### 0.2 CMake 是什么

手写编译命令太繁琐（本项目 5 个 .cpp 要编译再链接），CMake 用一个配置文件描述"怎么构建"，自动生成 Makefile。本项目 `CMakeLists.txt` 的核心指令：

| 指令 | 作用 |
|---|---|
| `cmake_minimum_required(VERSION 3.10)` | 要求 CMake 最低版本 |
| `project(ul_mac_manager)` | 定义项目名 |
| `set(CMAKE_CXX_STANDARD 17)` | 使用 C++17 标准（本项目用到了 C++17 的结构化绑定等特性） |
| `include_directories(include)` | 告诉编译器去 `include/` 找头文件，所以代码里写 `#include "ul_mac/xxx.h"` |
| `add_executable(...)` | 指定由哪些 .cpp 编译出可执行文件 |

#### 0.3 两种构建模式

| 模式 | 命令 | 特点 |
|---|---|---|
| Release | `cmake ..` （默认或 `-DCMAKE_BUILD_TYPE=Release`） | 开优化，跑得快，但不方便调试 |
| Debug | `cmake .. -DCMAKE_BUILD_TYPE=Debug` | 保留调试信息，可用 gdb 单步跟踪 |

### 🔍 操作步骤

- [ ] 阅读 `README.md` 第 1、2 章，记住核心概念：模拟"手机(UE)向基站(eNB)申请上行资源"
- [ ] 编译运行：
  ```bash
  cd ul_mac_manager && mkdir -p build && cd build
  cmake .. && make -j$(nproc)
  ./ul_mac_manager
  ```
- [ ] 观察 4 个仿真场景的输出，找到这几类关键日志（对照 README §5.3）：
  - `Sending SR on PUCCH` —— SR 发送
  - `Triggering Regular BSR` —— BSR 触发
  - `New TX, RV=0, TBS=xxx` —— HARQ 新传
  - `Adaptive ReTX=xxx` —— HARQ 重传
  - `Max ReTX reached` —— 达到最大重传，丢弃
- [ ] 逐行读懂 `CMakeLists.txt`

### ✏️ 动手练习

- [ ] 删掉 `src/main.cpp` 某行末尾的分号，重新 `make`，观察并读懂编译错误；改回来
- [ ] 在 `main()` 开头加一行 `std::cout << "Hello UL MAC!\n";`，重新编译运行验证

### 🏁 里程碑

✅ 能独立完成"改代码 → 编译 → 运行 → 看日志"的完整闭环，能说出 .h 和 .cpp 的分工。

---

## 阶段 1：通用类型与工具层（2~3 天）

**目标**：掌握项目的"语言基础"——所有模块共用的类型定义、日志和指标系统。

### 阅读顺序

| 顺序 | 文件 | 行数 | 主题 |
|---|---|---|---|
| 1 | `include/ul_mac/common_types.h` | 370 | 类型定义与查表函数 |
| 2 | `include/ul_mac/mac_logger.h` | 155 | 单例日志系统 |
| 3 | `include/ul_mac/metrics_collector.h` | 194 | 性能指标收集器 |

### 📖 知识讲解 1：common_types.h 中的 C++ 语法

#### 1.1 namespace（命名空间）

整个项目的代码都包在 `namespace ul_mac { ... }` 里，防止和其他库的同名符号冲突。外部使用时写 `ul_mac::sr_manager`，或 `using namespace ul_mac;`（main.cpp 第 28 行就是这么做的）。

#### 1.2 constexpr 常量（取代 #define）

```cpp
constexpr uint32_t MAX_HARQ_PROCESSES = 16;  // 编译期常量, 有类型、有作用域
constexpr uint32_t NOF_LCGS = 4;             // 3GPP 规定最多 4 个 LCG
```

比 `#define MAX_HARQ_PROCESSES 16` 好在：有类型检查、遵守作用域规则、可调试。
**协议对应**：LTE HARQ 8 进程 / NR 16 进程，本项目取 NR 上限；LCG 数量 4 是 3GPP 硬性规定。

#### 1.3 enum class（强类型枚举）

```cpp
enum class bsr_format {
    SHORT_BSR,       // 短BSR: 报告单个LCG, 1字节(2-bit LCG ID + 6-bit缓冲区大小)
    TRUNCATED_BSR,   // 截断BSR: 空间不足时只报最高优先级LCG
    LONG_BSR         // 长BSR: 报告所有4个LCG, 3字节(各6-bit)
};
```

和旧式 `enum` 的三大区别（面试常问）：
1. **必须带作用域**：`bsr_format::SHORT_BSR`，不会污染外层命名空间
2. **不能隐式转 int**：`if (fmt == 0)` 编译报错，杜绝一类 bug
3. **可指定底层类型**：`enum class X : uint8_t`

本项目所有状态都用 enum class：`sr_state`、`harq_state`、`bsr_trigger_type`、`harq_feedback`、`sched_algorithm`。

#### 1.4 struct + 构造函数初始化列表

C++ 的 struct 就是默认 public 的 class，可以有构造函数。看 `ul_grant`（第 160~179 行）：

```cpp
struct ul_grant {
    uint16_t rnti;      // 发给哪个UE
    uint32_t pid;       // 用哪个HARQ进程
    uint32_t tbs;       // 能发多少字节
    bool     ndi;       // 新数据指示 ← HARQ判断新传/重传的关键
    // ...
    ul_grant()
        : rnti(0), pid(0), tbs(0), ...   // ← 初始化列表: 冒号后逐个初始化成员
    {}
};
```

**为什么用初始化列表而不在函数体里赋值？** 初始化列表是"直接初始化"，函数体赋值是"先默认构造再赋值"，前者更高效；const 成员和引用成员只能用初始化列表。

**为什么每个 struct 都写默认构造函数？** C++ 内置类型（int、bool）不自动清零，不初始化就是随机值（未定义行为的常见来源）。

#### 1.5 inline 函数与查表法

头文件里定义函数必须加 `inline`（否则被多个 .cpp include 后链接器报"重复定义"）。看 `rv_of_irv`（第 351 行）：

```cpp
inline uint32_t rv_of_irv(uint32_t irv) {
    static const uint32_t rv_table[4] = {0, 2, 3, 1};  // 静态只读表, 只初始化一次
    return rv_table[irv % 4];
}
```

- `static const` 数组：只在第一次调用时构造，之后所有调用共享，避免每次进函数都重建数组
- `irv % 4`：取模实现循环，IRV 计数器 0,1,2,3,4,5... 映射到 RV 0,2,3,1,0,2...
- **协议对应**：RV 序列 {0,2,3,1} 来自 TS 36.321 §5.4.2.1。RV=0 含最多系统比特（信息位），RV=2 含最多校验比特，前两次传输 (0,2) 就能凑齐最佳增量冗余——这就是不用 {0,1,2,3} 的原因

再看 `bsr_index_to_bytes`（第 318 行）：BSR 用 6 bit（0~63）表示缓冲区大小，通过 64 级**对数量化表**映射到字节数。小缓冲区精度高（索引差 1 只差几字节）、大缓冲区精度低（索引差 1 差几千字节）——因为大缓冲区时精确值对调度决策影响不大，这是协议的精妙设计。

### 📖 知识讲解 2：mac_logger.h 中的设计模式

#### 1.6 单例模式（Singleton）—— Magic Static 写法

日志系统全项目只需要一个实例，用单例模式：

```cpp
class mac_logger {
public:
    static mac_logger& instance() {
        static mac_logger inst;   // ← 关键: 静态局部变量
        return inst;
    }
private:
    mac_logger() : ... {}         // ← 构造函数私有, 外部无法 new
};
// 使用: mac_logger::instance().info(...)
```

三个要点：
1. **构造函数放 private** → 外界不能随意创建实例
2. **static 局部变量** → 第一次调用 `instance()` 时才构造（懒加载）
3. **C++11 保证线程安全**：多线程同时首次调用时，编译器自动加锁保证只构造一次（称为 "magic static"）。C++03 时代要手写双重检查锁，现在一行搞定——这是面试高频考点

`metrics_collector` 用了完全相同的写法，读完 logger 再看它就是复习。

#### 1.7 RAII 与 std::lock_guard

`log()` 方法中（第 55 行）：

```cpp
void log(...) {
    if (level < current_level_) return;          // 级别过滤(注意: 在加锁前, 快速返回)
    std::lock_guard<std::mutex> lock(mutex_);    // 构造时加锁
    // ... 拼接和输出日志 ...
}                                                 // ← 函数返回时 lock 析构, 自动解锁
```

**RAII**（Resource Acquisition Is Initialization）是 C++ 最重要的惯用法：把资源（锁/文件/内存）的生命周期绑定到栈对象上，函数无论从哪个 return 退出、甚至抛异常，析构函数都会执行，资源必然释放。`lock_guard` 就是"锁的 RAII 包装"——**永远不会忘记解锁**。

为什么日志要加锁？多个线程同时 `std::cout <<` 会导致输出交错混乱。加锁保证一行日志的完整性。

#### 1.8 std::ostringstream 字符串拼接

```cpp
std::ostringstream oss;
oss << "[" << timestamp << "] [" << level_str << "] " << message;
std::string line = oss.str();   // 取出拼好的字符串
```

像 `cout` 一样用 `<<`，但结果存进字符串。支持 `std::setw(4)`、`std::setfill('0')`、`std::hex` 等格式控制（`to_hex()` 方法把 RNTI 格式化成 4 位十六进制就是这么做的）。

#### 1.9 chrono 时间库（get_timestamp，第 108 行）

```cpp
auto now = std::chrono::system_clock::now();                       // 当前时刻
auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
               now.time_since_epoch()) % 1000;                     // 毫秒部分
```

`system_clock::now()` 拿时间点，`duration_cast` 做单位换算，`put_time` 格式化输出。这是现代 C++ 处理时间的标准方式（取代 C 的 `time()`）。

#### 1.10 宏封装（第 143 行）

```cpp
#define LOG_INFO(comp, rnti, tti, msg) \
    ul_mac::mac_logger::instance().info(comp, rnti, tti, msg)
```

把冗长的单例调用缩短成 `LOG_INFO("SR", rnti_, tti, "...")`。全项目的日志都通过这 4 个宏输出。

### 📖 知识讲解 3：metrics_collector.h 补充知识点

- **`memset(this, 0, sizeof(*this))`**（system_metrics::reset）：把整个结构体清零。只对"全是普通数值成员"的结构体安全（POD 类型），有虚函数/string/vector 的类绝不能这么干——这是一个值得记住的边界
- **`static_cast<double>(a) / b`**：两个整数相除是整数除法（3/2=1）！算比率前必须先转 double。项目里所有 `xx_rate()` 函数都有这个转换
- **const 成员函数**：`double sr_success_rate() const` —— 尾部 const 承诺"不修改成员"，只读接口都应加 const
- **返回副本保证线程安全**：`get_metrics()` 加锁后返回 `system_metrics` 的**拷贝**而不是引用——调用者拿到的是一致的快照，不会读到一半被其他线程改掉

### 🔍 代码精读指南

- [ ] 通读 `common_types.h`，对每个 struct 回答："它模拟协议里的什么消息/配置？"（注释里都标了 3GPP 章节和 srsRAN 原型）
- [ ] 精读 `mac_logger.h` 的 `instance()`、`log()`，理解单例 + 锁 + 流的配合
- [ ] 快速过 `metrics_collector.h`，确认单例写法和 logger 一致

### ✏️ 动手练习

- [ ] 给 logger 新增 `TRACE` 级别（低于 DEBUG）：改 `log_level` 枚举 → `level_to_string()` → 加 `trace()` 便捷方法和 `LOG_TRACE` 宏，编译验证
- [ ] 纸笔推算 `bsr_index_to_bytes(20)` 和 `bytes_to_bsr_index(300)` 的值，再在 main() 里打印验证
- [ ] 推算 `rv_of_irv(0)` 到 `rv_of_irv(5)` 的输出序列（答案应为 0,2,3,1,0,2）

### 🏁 里程碑

✅ 能说出 enum class 与 enum 的 3 个区别；能默写单例的 magic static 写法；理解 lock_guard 的 RAII 原理。

---

## 阶段 2：SR 管理器——状态机编程（3~4 天）

**目标**：通过最小的业务模块掌握状态机编程、回调函数、锁的正确使用。

### 阅读顺序

| 顺序 | 文件 | 行数 |
|---|---|---|
| 1 | `include/ul_mac/ue_sr_manager.h` | 139 |
| 2 | `src/ue_sr_manager.cpp` | 239 |

### 📖 知识讲解 1：SR 协议知识（TS 36.321 §5.4.4）

**SR 解决什么问题？** 上行传输由基站统一调度，UE 不能想发就发。当 UE 有新数据但手上没有上行授权时，它需要一种"最轻量的举手方式"——SR 就是在 PUCCH 上发送的 1 bit 请求信号。

**完整流程**：
1. 新数据到达 → 触发 Regular BSR → BSR 发现没有上行资源 → **触发 SR**（代码里就是 `bsr_manager::set_trigger()` 调用 `sr_proc_->start()`）
2. UE 按 SR 周期在 PUCCH 上重复发送 SR，每发一次 `sr_counter++`
3. 收到 UL Grant → SR 成功，回到空闲
4. 发满 `dsr-TransMax` 次仍无响应 → **SR 失败** → 释放 PUCCH/SRS 资源，回退到随机接入（RA）过程

**状态机**（与代码中 `sr_state` 一一对应）：

```
   ┌────────┐  start() 由BSR触发   ┌─────────┐
   │  IDLE  │ ──────────────────→ │ PENDING │ ←─────┐
   └────────┘                     └────┬────┘       │ 发送后回到PENDING
       ▲                               │ step(): 周期到 & 未超限   │ 等待Grant
       │ notify_ul_grant_received()    ▼                │
       │ (SR成功)                ┌──────────────┐       │
       ├───────────────────────  │ TRANSMITTING │ ──────┘
       │                         └──────────────┘
       │ 失败后重置                     │ sr_counter >= dsr_transmax
       │                               ▼
       │                         ┌────────┐
       └──────────────────────── │ FAILED │ → 触发 fail_callback (模拟RA回退)
                                 └────────┘
```

### 📖 知识讲解 2：C++ 知识点

#### 2.1 头文件/源文件分离

`ue_sr_manager.h` 只有类声明（成员和方法签名），`ue_sr_manager.cpp` 里写实现：

```cpp
// .h 中声明:                          // .cpp 中定义:
void step(uint32_t tti);              void sr_manager::step(uint32_t tti) { ... }
```

`sr_manager::` 是作用域解析符，表示"这是 sr_manager 类的成员函数"。分离的好处：改实现不需要重新编译所有 include 该头文件的文件；接口和实现解耦。

#### 2.2 std::function 回调机制

```cpp
using sr_tx_callback   = std::function<void(uint16_t rnti)>;  // 类型别名
using sr_fail_callback = std::function<void(uint16_t rnti)>;
```

`std::function<void(uint16_t)>` 是"可调用对象的容器"，能装普通函数、lambda、成员函数绑定。SR 模块通过回调通知外部"SR 发出去了/失败了"，而**不需要知道外部是谁**——这就是解耦。

`ue_context.h` 第 47 行展示了怎么传入 lambda：

```cpp
sr_mgr_.init(sr_cfg,
    [this](uint16_t r) { on_sr_sent(r); },     // lambda 捕获 this, 转调成员函数
    [this](uint16_t r) { on_sr_failed(r); });
```

`[this]` 是捕获列表：lambda 内部要用 `ue_context` 的成员，就把 this 指针捕获进来。

#### 2.3 锁内决策、锁外回调（避免死锁的经典模式）

`step()`（ue_sr_manager.cpp 第 112 行）是本项目**最值得学习的并发写法**：

```cpp
void sr_manager::step(uint32_t tti) {
    bool do_send_sr = false;   // 决策标志
    bool do_fail = false;
    {   // ← 人为限定锁的作用域
        std::lock_guard<std::mutex> lock(mutex_);
        // ...在锁内读写状态, 只设置标志位...
        if (...) { do_send_sr = true; }
    }   // ← 花括号结束, 锁在这里释放
    // 锁外执行回调
    if (do_send_sr && tx_callback_) tx_callback_(rnti_);
}
```

**为什么回调必须放锁外？** 回调函数的内容由外部决定，如果它反过来调用 sr_manager 的其他加锁方法（比如 `get_state()`），同一线程对同一个非递归 mutex 加两次锁 → **死锁**。规则：**持有锁时绝不调用外部代码**。

#### 2.4 TTI 回绕处理（can_send_sr，第 87 行）

```cpp
if (tti >= last_sr_tx_tti_) {
    interval = tti - last_sr_tx_tti_;
} else {
    interval = 10240 - last_sr_tx_tti_ + tti;   // LTE 的 TTI 计数 0~10239 循环
}
```

LTE 系统帧号 SFN(0~1023) × 10 子帧 = 10240 个 TTI 后回绕归零。无符号数直接相减会得到巨大的错误值，必须显式处理回绕——嵌入式/通信代码中处处有这种细节。

另一个细节：`last_sr_tx_tti_` 初始化为 `0xFFFFFFFF` 作为"从未发送过"的哨兵值（sentinel value）。

#### 2.5 EMA 指数移动平均（adjust_sr_period，第 200 行）

增强功能"自适应 SR 周期"的核心算法：

```cpp
const double alpha = 0.3;   // 平滑因子: 越大对新样本越敏感
avg_traffic_rate_ = alpha * traffic_rate + (1.0 - alpha) * avg_traffic_rate_;
```

EMA 用一行递推公式实现"带遗忘的平均值"，不需要存历史数据。然后按平均速率分档调整周期：>1000 → 5ms（高流量快响应），<10 → 40ms（低流量省 PUCCH 资源）。

**协议背景**：标准 LTE 的 SR 周期是 RRC 配置的固定值，本项目的"自适应"是增强设计，面试时要能说清"哪些是标准行为、哪些是项目创新"。

### 🔍 代码精读指南

按调用时序精读（不是按文件顺序）：

- [ ] `init()` —— 保存配置和回调；注意 `set_config()` 中 `dsr_transmax==0` 的参数校验
- [ ] `start()` —— BSR 触发入口。注意只有 IDLE/FAILED 状态才接受新触发（防止重复触发打乱计数）
- [ ] `step()` —— 核心状态机，画出它的流程图：PENDING → 检查 enabled → 检查 counter < dsr_transmax → 检查 can_send_sr → 发送/失败
- [ ] `can_send_sr()` —— SR 周期控制 + TTI 回绕
- [ ] `notify_ul_grant_received()` —— 成功路径：收到 Grant 即认为 SR 达到目的
- [ ] `adjust_sr_period()` —— EMA 增强功能

### ✏️ 动手练习

- [ ] 把场景 1 中 `sr_config` 的 `dsr_transmax` 从默认 8 改成 2（提示：在 main.cpp 中调用 `ue.set_sr_config()`），观察 `SR FAILED` 日志是否更容易出现
- [ ] 在 `step()` 每个状态转移处补一条 DEBUG 日志，把日志级别设为 DEBUG（`mac_logger::instance().set_level(log_level::DEBUG)`），画出一次完整 SR 过程的实际时序
- [ ] 思考题：`step()` 第 129~138 行，状态先变 TRANSMITTING 再马上变回 PENDING，为什么？（答案：TRANSMITTING 只表示"这个 TTI 正在发"，发完继续等响应，所以立即回 PENDING 等下一个周期重发或等 Grant）

### 🏁 里程碑

✅ 能在白板上画出 SR 状态机并指出每条转移对应的代码位置；能解释"锁内决策、锁外回调"的原因。

---

## 阶段 3：BSR 管理器 + LCG 缓冲区——数据结构实战（4~5 天）

**目标**：掌握 STL 容器实战选型、结构化绑定、mutable 关键字，吃透 BSR 触发与格式选择逻辑。

### 阅读顺序

| 顺序 | 文件 | 行数 | 备注 |
|---|---|---|---|
| 1 | `include/ul_mac/lcg_buffer.h` | 199 | 数据基础，全部实现在头文件里 |
| 2 | `include/ul_mac/ue_bsr_manager.h` | 194 | 先看接口和注释 |
| 3 | `src/ue_bsr_manager.cpp` | 423 | 分两天：第 1 天触发与定时器，第 2 天格式选择与生成 |

### 📖 知识讲解 1：LCG 缓冲区（lcg_buffer.h）

#### 3.1 核心数据结构

```cpp
std::map<uint32_t, lc_buffer_state> lcgs_[NOF_LCGS];
//       ↑LCID     ↑该通道的缓冲区状态    ↑4个LCG, 每个是一张 LCID→状态 的映射表
```

这是"**数组套 map**"的两级结构：LCG 用下标直接访问（只有 4 个，固定），LCG 内的逻辑通道用 map（数量不定、按 LCID 查找）。容器选型原则：
- 固定小数量 + 下标访问 → 原生数组 / `std::array`
- 动态数量 + 按键查找 → `std::map`（有序）
- 动态数量 + 尾部追加遍历 → `std::vector`

#### 3.2 old_buffer / new_buffer 双缓冲设计

`lc_buffer_state` 有两个字段（对照 common_types.h 第 221 行）：
- `new_buffer`：RLC 层刚更新的最新缓冲量（`update_buffer_state()` 写入）
- `old_buffer`：上一个 TTI 已"确认"的缓冲量（`update_old_buffer()` 把 new 复制到 old）

**为什么要两份？** 为了检测"变化"：
- `check_new_data()`：LCG 的 old 全为 0（之前没数据）但 new > 0（来新数据了）→ 触发 Regular BSR
- `check_highest_priority_channel()`：new > old 且该通道优先级高于当前所有有数据的通道 → 触发 Regular BSR

这两个函数正是 3GPP 规定的 Regular BSR 两大触发条件的代码化。

#### 3.3 C++17 结构化绑定

```cpp
for (const auto& [lcid, state] : lcgs_[lcg_id]) {   // map 的元素是 pair<key, value>
    total += state.old_buffer;                       // 直接拆成两个命名变量
}
```

`auto& [a, b]` 把 pair/tuple 拆开命名，比 `it->first / it->second` 可读性好得多。这就是 CMake 里要求 C++17 的原因之一。

#### 3.4 mutable 关键字与 unlocked 内部方法

```cpp
mutable std::mutex mutex_;   // ← mutable: 允许在 const 成员函数中修改
```

`get_buffer_state_lcg()` 是 const 函数（逻辑上只读），但加锁要修改 mutex 状态——`mutable` 就是给"锁、缓存"这类不影响逻辑状态的成员开的后门。

再看一个重要模式（第 96 行 vs 第 184 行）：

```cpp
bool check_new_data() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ... get_buffer_state_lcg_unlocked(i) ...   // ← 调用"无锁版"私有方法
}
```

为什么不直接调 `get_buffer_state_lcg()`？因为它内部也要加锁，而 `std::mutex` 是**不可重入**的——同一线程加两次锁 = 死锁。解法：公开方法加锁，内部复用逻辑抽成 `_unlocked` 私有版本。**这是多线程代码的标准套路，配合阶段 2 的"锁外回调"一起记**。

### 📖 知识讲解 2：BSR 协议知识（TS 36.321 §5.4.5）

#### 3.5 三种触发类型（优先级 Regular > Periodic > Padding）

| 类型 | 触发条件 | 代码位置 | 是否触发 SR |
|---|---|---|---|
| **Regular** | ① 空 LCG 来了新数据；② 更高优先级通道来了新数据；③ retxBSR-Timer 超时且有数据 | `check_regular_bsr_trigger()` + `handle_timer_expiry()` | **是**（这是 SR 的唯一入口） |
| **Periodic** | periodicBSR-Timer（默认 80ms）超时 | `handle_timer_expiry()` | 否 |
| **Padding** | UL Grant 装完数据还有剩余空间，够塞一个 BSR | `generate_padding_bsr()` | 否 |

关键联动代码（ue_bsr_manager.cpp 第 83 行）——**BSR 是 SR 的触发源**：

```cpp
if (type == bsr_trigger_type::REGULAR && sr_proc_) {
    sr_proc_->start();    // Regular BSR 自动拉起 SR 过程
}
```

#### 3.6 定时器的仿真实现

本项目没有真正的定时器线程，而是**用 TTI 计数模拟**（`step()` 第 143 行）：

```cpp
if (periodic_timer_running_ && periodic_timer_counter_ > 0) {
    periodic_timer_counter_--;          // 每 TTI 减 1
}
// 减到 0 = 超时 → handle_timer_expiry() 里触发 Periodic BSR 并重置计数
```

真实系统（srsRAN）用定时器框架，但原理相同：MAC 层的时间就是以 TTI 为刻度的。两个定时器的作用：
- **periodicBSR-Timer**：保证基站定期刷新对 UE 缓冲区的认知
- **retxBSR-Timer**（默认 320ms）：BSR 发出后启动（`update_bsr_tti_end()`），超时还有数据没发完 → 怀疑 BSR 丢了 → 重新触发 Regular BSR

#### 3.7 BSR 格式选择算法（select_bsr_format，第 171 行）

先记住格式大小（代码顶部常量）：

```cpp
CE_SUBHEADER_LEN = 1;  // MAC CE 子头 1 字节
SHORT_BSR_SIZE = 1;    // Short BSR 本体 1 字节 → 总共 2 字节
LONG_BSR_SIZE = 3;     // Long BSR 本体 3 字节(4个LCG各6-bit=24bit) → 总共 4 字节
```

决策表（把代码逻辑翻译成表格，做练习时对照）：

| PDU 剩余空间 | 有数据的 LCG 数 | 选择 | 理由 |
|---|---|---|---|
| ≥ 4 字节 | ≤ 1 | **Short BSR** | 空间够 Long 但只有 1 个 LCG，用 Short 省 2 字节 |
| ≥ 4 字节 | > 1（或 Padding 触发） | **Long BSR** | 报告所有 LCG |
| 2~3 字节 | 1 | **Short BSR** | 空间只够 Short |
| 2~3 字节 | > 1 | **Truncated BSR** | 报不全，只报最高优先级 LCG（`find_max_priority_lcg_with_data()`） |
| < 2 字节 | — | 不发 | `generate_bsr()` 返回 false |

#### 3.8 BSR 取消规则（need_to_send_bsr_on_ul_grant，第 300 行）

```cpp
if (grant_size >= total_data) {
    triggered_type_ = bsr_trigger_type::NONE;   // 授权装得下所有数据 → 取消BSR
    return false;
}
```

**协议原文**："若 UL Grant 能容纳所有待传数据，取消所有已触发的 BSR"——都能一次发完了，再报缓冲区状态就是浪费。

两个容易忽略的细节（面试加分点）：
1. **Truncated BSR 不取消触发**：`generate_bsr()` 仅在格式非 Truncated 时才清除 `triggered_type_`——因为截断报告不完整，下次授权仍需补发完整 BSR（3GPP 规定）
2. **先存后清**：`need_to_send_bsr_on_ul_grant()` 在调用 `generate_bsr()` 前先把 `triggered_type_` 存入局部变量再用于统计——因为 `generate_bsr()` 内部会把它清成 NONE（早期版本在这里有个统计 bug，见附录 D）

#### 3.9 预测性 BSR（增强功能，predict_buffer_demand，第 387 行）

用 `std::array<uint32_t, 10>` 做**环形缓冲区**记录最近 10 个 TTI 的缓冲量：

```cpp
buffer_history_[history_idx_ % buffer_history_.size()] = current_buffer;
history_idx_++;   // 下标一直递增, 取模后循环覆盖最旧数据
```

预测时做**最小二乘线性回归**：对历史样本拟合 `y = slope*x + intercept`（x为时间索引，y为缓冲区大小），预测下一采样点 `y_pred = slope*count + intercept`。相比加权平均，线性回归能真正捕捉增长/下降趋势——加权平均只能"跟着最新值走"，而回归能"预测拐点"。环形缓冲 + 线性回归是时间序列预测的基础组合，值得掌握。

### 🔍 代码精读指南

- [ ] 第 1 天：`lcg_buffer.h` 全文 + `ue_bsr_manager.cpp` 的 `step()` → `handle_timer_expiry()` → `check_regular_bsr_trigger()` → `set_trigger()`（触发链）
- [ ] 第 2 天：`select_bsr_format()` → `generate_bsr()` → `need_to_send_bsr_on_ul_grant()` → `generate_padding_bsr()`（生成链），注意 `generate_bsr()` 成功后重启周期定时器并清空触发状态
- [ ] 对照 `ue_context::handle_ul_grant()`（ue_context.h 第 106 行）看 BSR 在收到 Grant 时的完整调用位置

### ✏️ 动手练习

- [ ] 纸笔推演：LCG0 有 300 字节、LCG1 有 0 字节、LCG2 有 1000 字节，Grant 剩余空间分别为 10 / 3 / 1 字节时，各生成什么格式的 BSR？（对照上面的决策表，再单步验证）
- [ ] 把 `DEFAULT_PERIODIC_BSR_TIMER` 从 80 改成 20（common_types.h），运行场景 4，对比 BSR 统计中 Periodic 数量的变化
- [ ] 思考题：`check_highest_priority_channel()` 里 `state2.priority <= state.priority` 用的是 `<=` 而不是 `<`，为什么？（提示：priority 数字越小优先级越高；相同优先级的通道已有数据时，新数据不算"更高优先级"）

### 🏁 里程碑

✅ 给定任意"LCG 状态 + Grant 空间"能口算 BSR 触发类型与格式；能解释 `_unlocked` 方法和 `mutable mutex` 的用途。

---

## 阶段 4：UL HARQ 管理器——并发与组合设计（4~5 天）

**目标**：掌握对象组合、unique_ptr、std::atomic；吃透 NDI 新传/重传判断（面试最高频考点）。

### 阅读顺序

| 顺序 | 文件 | 行数 |
|---|---|---|
| 1 | `include/ul_mac/ue_ul_harq_manager.h` | 218 |
| 2 | `src/ue_ul_harq_manager.cpp` | 317 |

### 📖 知识讲解 1：HARQ 协议知识（TS 36.321 §5.4.2）

#### 4.1 为什么需要 16 个并行进程？

一次传输后要等约 8ms（HARQ RTT）才能收到 ACK/NACK。如果只有 1 个进程，这 8ms 里 UE 只能干等。解法：**流水线**——进程 0 在等反馈时，进程 1、2、3…继续发新数据。LTE 用 8 个进程刚好填满 8ms RTT，NR 最多 16 个（本项目 `MAX_HARQ_PROCESSES = 16`）。

#### 4.2 NDI 机制：一个 bit 判断新传/重传

每个进程记住上一次的 NDI 值（`cur_ndi_`）。收到新 Grant 时对比：
- **NDI 翻转**（和上次不同）→ 新传输：清空缓冲区，装新数据，重置计数
- **NDI 未翻转** → 重传：用缓冲区里的旧数据再发一次

为什么用"翻转"而不是"1=新传"？因为 1 bit 无法表达绝对含义，只有"变没变"是可靠的——即使连续多次新传，NDI 0→1→0→1 每次都在翻转。

#### 4.3 自适应 vs 非自适应重传

| | 触发方式 | 参数 | 代码路径 |
|---|---|---|---|
| **自适应重传** | eNB 发新 DCI（含 NDI，未翻转） | eNB 可改 MCS/PRB（TBS 不变） | `grant.ndi_present == true` 分支 → `generate_retx()` 更新参数 |
| **非自适应重传** | 仅 PHICH NACK，无 DCI | 与上次完全相同 | `grant.ndi_present == false` 分支 → 直接 `generate_retx()` |

#### 4.4 new_grant_ul() 决策树（核心中的核心）

对照 `ue_ul_harq_manager.cpp` 第 56~119 行，把代码翻译成决策树：

```
收到 Grant
 ├─ 1. grant.phich_available? → 记录反馈 harq_feedback_ = hi_value
 │      └─ 2. current_tx_nb_ >= max_retx 且反馈是NACK?
 │             → 丢弃传输块 (is_discarded), reset(), 结束   ← "Max ReTX reached"日志
 ├─ 3. grant.ndi_present == true (有DCI):
 │      ├─ tbs == 0 → 无效授权, 结束
 │      ├─ is_new_tx = (非TC-RNTI 且 NDI翻转) || 首次配置授权 || is_rar
 │      │     ├─ true  → generate_new_tx(): 记录NDI/TBS, 计数清零, IRV清零
 │      │     └─ false → generate_retx():  自适应重传, 按grant.rv同步IRV
 └─ 4. ndi_present == false 且已有授权配置:
        └─ 反馈不是ACK → generate_retx(): 非自适应重传(原参数)
```

三个特殊规则（面试常挖的细节）：
- **首次授权**（`!is_grant_configured_`）：没有历史 NDI 可比，直接视为新传
- **RAR 授权**（`grant.is_rar`）：随机接入响应中的授权永远是新传（Msg3）
- **TC-RNTI**（`is_temp_rnti`）：Msg3 重传用独立的 `max_harq_msg3_tx` 上限

#### 4.5 RV/IRV 的运转（generate_tx，第 165 行）

```cpp
current_tx_nb_++;                       // 传输计数+1
action.rv = get_rv();                   // rv_of_irv(current_irv_) → {0,2,3,1}
current_irv_ = (current_irv_ + 1) % 4;  // IRV 步进
```

新传时 IRV 清零 → 第一次 RV=0（系统比特最多，自解码能力最强），重传依次 RV=2、3、1。自适应重传时 eNB 可在 DCI 里指定 RV，此时用 `irv_of_rv()` 反向同步 IRV 计数器（generate_retx 第 147 行）。

### 📖 知识讲解 2：C++ 知识点

#### 4.6 对象组合（实体—进程两级结构）

```
ul_harq_manager (HARQ实体, 1个)
 └── std::unique_ptr<ul_harq_process[]> processes_   (16个进程对象)
       每个进程独立维护: NDI / IRV / 传输计数 / TBS
```

manager 负责"路由"（按 `grant.pid` 找进程）和全局统计，process 负责单进程状态机。**组合（has-a）优于继承（is-a）**——这是面向对象设计的重要原则，阶段 5 的 `ue_context` 会再次见到。

#### 4.7 std::unique_ptr 智能指针

```cpp
std::unique_ptr<ul_harq_process[]> processes_;                    // 声明: 独占所有权
processes_ = std::make_unique<ul_harq_process[]>(MAX_HARQ_PROCESSES);  // 构造: 分配数组
```

`unique_ptr` 在析构时**自动 delete[]**，杜绝内存泄漏（还是 RAII 思想！）。`make_unique` 是创建它的标准方式。为什么不用 `std::array`？因为 `ul_harq_process` 含 mutex 不可拷贝，用堆分配 + 指针管理更灵活。现代 C++ 的铁律：**不裸写 new/delete**。

#### 4.8 std::atomic 原子变量

```cpp
std::atomic<uint32_t> current_tx_nb_;    // 传输计数
std::atomic<bool>     harq_feedback_;    // 反馈标志
std::atomic<float>    average_retx_{0.0};
```

**atomic vs mutex 怎么选？**
- 单个标量的读写/自增 → `atomic`（无锁，硬件指令级保证，快）
- 多个变量要"一起"改（保持一致性）→ `mutex`（临界区）

本项目两者混用是刻意示范：`get_rv()` 只读一个 IRV → 用 atomic 的 `.load()` 无锁读取；`new_grant_ul()` 要同时改 NDI/TBS/计数 → 整体加 mutex。注意 `.load()` / `.store()` 是 atomic 的显式读写接口。

#### 4.9 嵌套结构体作为返回值（tx_action）

```cpp
class ul_harq_process {
public:
    struct tx_action {          // 类内定义的结构体
        bool is_new_tx, is_retx, is_discarded, is_msg3;
        uint32_t rv, tbs, tx_nb;
    };
    tx_action new_grant_ul(...);   // 一次返回全部决策结果
};
// 外部引用: ul_harq_process::tx_action
```

函数需要返回多项信息时，定义一个专用结构体比一堆输出参数（`bool& is_retx, uint32_t& rv...`）清晰得多。

#### 4.10 平均重传次数统计（new_grant_ul 统计部分）

```cpp
// 平均重传次数 = 总重传次数 / 总新传次数
stats_.avg_retx_per_pkt = static_cast<double>(stats_.total_retx) / stats_.total_new_tx;
average_retx_.store(static_cast<float>(stats_.avg_retx_per_pkt));
```

统计口径与 metrics_collector 的 `avg_harq_retx()` 保持一致。**验算方法**：BLER=p 时每个 TB 的期望重传次数 ≈ p/(1-p)（几何分布），p=0.3 时理论值 ≈ 0.43，与场景 3 实测的 0.49 同量级——能用理论值验证仿真输出是面试亮点（早期版本这里用错了样本导致均值恒为 1.0，见附录 D）。

### 🔍 代码精读指南

- [ ] 先读 `ul_harq_process` 类：`reset()` → `new_grant_ul()` → `generate_new_tx()` → `generate_retx()` → `generate_tx()` → `get_state()`
- [ ] 画出 `new_grant_ul()` 的决策树（不看上文自己画，再对照）
- [ ] 再读 `ul_harq_manager`：`new_grant_ul()` 的统计逻辑、`handle_harq_feedback()` 的 BLER 计算、`get_idle_process_id()` 的负载均衡（先找空闲，找不到选传输次数最少的）
- [ ] 观察 `get_state()` 的推导逻辑：无授权→INACTIVE；有反馈→INACTIVE；计数>0→RETX_PENDING；否则→WAITING_FB

### ✏️ 动手练习

- [ ] 场景 3 的 BLER 从 0.30 改成 0.50（main.cpp 第 261 行 `simulate_harq_feedback(0.30, rng)`），先用几何分布期望 1/(1-p) 预测平均传输次数（=2.0），运行对比 `avg_retx` 输出
- [ ] 把场景 3 的 `harq_cfg.max_harq_tx` 从 4 改成 1，观察 "Max ReTX reached" 和 fail 计数的暴涨
- [ ] 思考题：`generate_new_tx()` 里 `current_irv_ = 0` 而 `generate_tx()` 里 `current_irv_ = (current_irv_+1)%4`，那么第一次新传实际用的 RV 是多少？（答案：RV=rv_of_irv(0)=0，先取值后步进）

### 🏁 里程碑

✅ 能脱稿讲清 NDI 翻转判断逻辑、自适应/非自适应重传区别、RV 序列 {0,2,3,1} 的设计原因；能说出 atomic 与 mutex 的选型标准。

---

## 阶段 5：调度器 + 全局整合——算法与架构（4~5 天）

**目标**：理解 PF 调度算法、UE 上下文的组合设计、TTI 主循环，把前四个阶段的模块串成完整系统。

### 阅读顺序

| 顺序 | 文件 | 行数 | 主题 |
|---|---|---|---|
| 1 | `include/ul_mac/enb_ul_scheduler.h` | 149 | eNB 侧调度器接口 |
| 2 | `src/enb_ul_scheduler.cpp` | 309 | 三种调度算法实现 |
| 3 | `include/ul_mac/ue_context.h` | 205 | UE 侧组件整合 |
| 4 | `include/ul_mac/mac_pdu.h` / `src/mac_pdu.cpp` | — | UL-SCH MAC PDU 组包/解包（P1 新增） |
| 5 | `src/main.cpp` | 429 | 4 个仿真场景 + TTI 主循环 |

### 📖 知识讲解 1：eNB 侧调度器（ul_scheduler）

#### 5.1 eNB 为每个 UE 维护什么（ue_sched_context）

基站不知道 UE 内部状态，只依据 UE 上报的信息调度：

| 字段 | 来源 | 用途 |
|---|---|---|
| `sr_pending` | `handle_sr()` | UE 举过手，即使 BSR 还没到也要给资源 |
| `ul_buffer[4]` / `total_ul_buffer` | `handle_bsr()`（BSR 索引→字节数） | 决定分配多少资源 |
| `ul_snr` | 信道测量（本项目固定 200 = 2dB… 实为 x100 编码） | 决定 MCS |
| `ul_avg_rate` / `ul_nof_samples` | 每次调度后 EMA 更新 | PF 算法的"历史平均速率" |
| `pending_retx_pid` | `handle_ul_crc()` CRC 失败时记录 | 重传优先调度；0xFFFF 表示无待重传 |
| `ndi[16]` | 每次发新传授权时翻转 | 每个 HARQ 进程的 NDI 状态（新传翻转/重传保持，与 UE 侧对齐） |

#### 5.1b 授权生成中的 NDI/RV 管理（generate_ul_grant_unlocked）

```cpp
if (!is_retx) {
    it->second.ndi[pid] = !it->second.ndi[pid];  // 新传: 翻转NDI
}
grant.ndi = it->second.ndi[pid];                 // 重传: NDI保持不变
grant.rv = is_retx ? -1 : 0;  // 重传rv=-1 → UE侧按IRV序列{0,2,3,1}自行递进
```

这是基站侧与 UE 侧（阶段 4 的 NDI 判断）的**契约**：基站为每个 UE 的每个 HARQ 进程维护一份 NDI，新传翻转、重传保持；UE 侧检测到 NDI 变化判定新传、未变化判定自适应重传。两端状态必须对齐，否则新传/重传会被彻底判反（早期版本用 `ndi = !is_retx` 就犯了这个错，见附录 D——这是本项目最值得在面试中讲的 bug 故事）。

#### 5.2 资源计算链：SNR → MCS → PRB → TBS

```cpp
// ① SNR → MCS: 区间映射表（29个阈值点，从高到低查找）
//    替代了早期的线性公式 snr_db * 28/30, 现在用 SNR_MCS_THRESHOLD[29] 查表
mcs = calculate_mcs(snr_x100);  // SNR<-6dB→MCS0, SNR≥20dB→MCS28
// ② 每PRB容量: 基于TS 36.213锚点表线性插值
//    6个锚点: MCS0→16bits, MCS5→64, MCS10→120, MCS15→176, MCS20→272, MCS28→456
bytes_per_prb = TBS_efficiency(mcs) / 8;
// ③ 需要多少PRB: 向上取整除法 —— 经典技巧!
n_prb = (req_bytes + bytes_per_prb - 1) / bytes_per_prb;
// ④ 最终TBS
tbs = calculate_tbs(mcs, n_prb);  // 锚点插值 × n_prb / 8
```

`(a + b - 1) / b` 是整数向上取整的标准写法，务必记住。MCS/TBS 已从简化线性公式升级为基于 TS 36.213 的区间映射表 + 锚点插值表，符合协议标准的查找表方法。

#### 5.3 PF 调度算法三阶段（schedule_pf，第 137 行）

```
阶段1: 重传优先 —— 遍历所有UE, pending_retx_pid有效的先分配 (保证HARQ RTT)
阶段2: 计算PF度量 —— 对有数据/有SR的UE:
        // P1 改进: 分子改用"信道可支持速率封顶后的需求"
        mcs = cqi>0 ? calculate_mcs_from_cqi(cqi) : calculate_mcs(ul_snr)
        achievable    = calculate_tbs(mcs, total_prb_)            // 信道可支持瞬时速率
        demand_capped = min(achievable, total_ul_buffer)          // 需求封顶
        pf = demand_capped / pow(avg_rate, fairness_coeff)
        按度量值降序排序
```
> **P1 改进点**：原 `current_rate` 直接取「待传字节数」，无法体现信道能力。改为先用 MCS 算出「信道可支持的瞬时速率」再与缓冲区需求取 min，使高 CQI 用户在该 TTI 更能体现信道优势（详见 PROTOCOL_NOTES.md §7.2.1）。
阶段3: 按序分配 —— PRB用完为止; 每次分配后:
        ① 清除sr_pending (真正拿到授权才清, 避免SR丢失)
        ② 扣减该UE的缓冲区估计 (deduct_ul_buffer_unlocked, 避免重复授权)
        ③ EMA更新该UE的avg_rate:
        alpha = 1/(n+1);  avg = (1-alpha)*avg + alpha*tbs
```

**PF 公式的直觉**：分子是"你现在能跑多快"，分母是"你历史上已经吃了多少"。吃得多的 UE 分母大 → 度量值低 → 让位给吃得少的。这样信道好的 UE 多拿资源（效率），但饿着的 UE 度量值会逐渐升高最终被调度（公平）——**效率与公平的平衡**就体现在这个除法里。

对比另两种算法（同一文件，结构几乎相同，读起来很快）：
- **RR**（schedule_rr）：不排序，按 map 顺序轮流，绝对公平但不看信道
- **Priority**（schedule_priority）：按 `total_ul_buffer` 降序，谁积压多先调度谁，吞吐优先

#### 5.4 排序 + lambda 比较器（第 173 行）

```cpp
std::sort(queue.begin(), queue.end(),
    [](const ue_pf& a, const ue_pf& b) { return a.metric > b.metric; });
//  ↑ lambda作为比较器: 返回true表示a排在b前面, 这里实现降序
```

`std::sort` + lambda 是 STL 最常用的组合。注意比较器的语义是"a 是否应排在 b 前面"。

### 📖 知识讲解 2：UE 上下文整合（ue_context.h）

#### 5.5 组合设计的教科书案例

```cpp
class ue_context {
    lcg_buffer_manager buffer_mgr_;   // 4个组件全部按值持有(不是指针!)
    sr_manager         sr_mgr_;
    bsr_manager        bsr_mgr_;
    ul_harq_manager    harq_mgr_;
};
```

构造函数初始化列表里的**依赖注入**值得细看：

```cpp
ue_context(uint16_t rnti)
    : buffer_mgr_()                      // ① 先构造缓冲区
    , sr_mgr_(rnti)                      // ② SR 独立构造
    , bsr_mgr_(rnti, buffer_mgr_)        // ③ BSR 通过【引用】拿到缓冲区
    , harq_mgr_(rnti)
{
    bsr_mgr_.init(bsr_cfg, &sr_mgr_, ...);   // ④ BSR 通过【指针】关联 SR
}
```

- `bsr_manager` 的成员是 `lcg_buffer_manager& buffer_mgr_`（引用成员，必须在初始化列表绑定，绑定后不可换）
- **成员初始化顺序 = 声明顺序**（不是初始化列表顺序！）——buffer_mgr_ 声明在 bsr_mgr_ 之前，所以 ③ 引用它时它已构造完成。如果声明顺序颠倒就是 bug，编译器只给警告
- 组件间协作关系：BSR **读** buffer（引用），BSR **触发** SR（指针 + `start()`），三者互不知道 ue_context 的存在（通过回调向上通知）

#### 5.6 handle_ul_grant() —— UE 收到授权后的六步曲

```cpp
1. sr_mgr_.notify_ul_grant_received();          // SR成功(若确实发过SR), 状态回IDLE
2. bsr_mgr_.need_to_send_bsr_on_ul_grant(...);  // 判断是否在授权中带BSR
2b. 若未带BSR且授权装完数据有剩余 → generate_padding_bsr()  // Padding BSR
3. harq_mgr_.new_grant_ul(grant, ...);          // HARQ判断新传/重传
4. bsr_mgr_.update_bsr_tti_end(bsr);            // 启动retxBSR-Timer
5. 新传时: total_tx_bytes_ += tbs; buffer_mgr_.consume_data(tbs);  // 统计+排空缓冲区
```

这就是协议规定的 UE 收到 UL Grant 后的处理顺序，把三大模块串在了一起。两个细节：
- **步骤 2b** 让三种 BSR 触发类型在仿真中都能观察到（授权覆盖全部数据时 Regular/Periodic 被取消，剩余空间改发 Padding BSR——这本身就是协议行为）
- **步骤 5 的 `consume_data()`** 实现了 3GPP TS 36.321 §5.4.3.1 的两阶段 LCP（逻辑信道优先级）：第一阶段按优先级排序，每个信道受 PBR 令牌桶限制取数据（`token_count` 按 `pbr * elapsed_ms` 补充，不超过 `pbr * bsd` 桶容量）；第二阶段将剩余空间按优先级分配给仍有数据的信道（不受 PBR 限制）。`pbr=0` 时退化为纯优先级排序。这样缓冲区真正排空——SR/BSR 的触发才能周期性地重现；重传发的是同一个 TB，不重复消耗

### 📖 知识讲解 3：main.cpp 的 TTI 主循环

以场景 1（第 96~137 行）为例，每个 TTI 内的完整"心跳"：

```cpp
for (uint32_t tti = 0; tti < 200; tti++) {
    // ── UE 侧 ──
    if (tti % 10 == 0) ue.data_arrived(2, 500);   // ① 模拟RLC数据到达(周期性)
    ue.run_tti(tti);                              // ② BSR.step() + SR.step()

    // ── UE→eNB 上行信令(仿真中直接函数调用代替空口传输) ──
    if (ue.get_sr_manager().get_state() == sr_state::PENDING)
        scheduler.handle_sr(ue.get_rnti());       // ③ SR 到达基站
    /* 每个有数据的LCG */ scheduler.handle_bsr(rnti, lcg, bsr_idx);  // ④ BSR 到达基站

    // ── eNB 侧 ──
    auto results = scheduler.schedule_ul(tti);    // ⑤ 调度决策, 生成Grant

    // ── eNB→UE 下行 + 反馈闭环 ──
    for (auto& res : results) {
        grant.hi_value = simulate_harq_feedback(0.1, rng); // ⑥ 掷骰子模拟信道(10% BLER)
        ue.handle_ul_grant(grant);                         // ⑦ UE处理授权(六步曲)
        ue.handle_harq_feedback(grant.pid, grant.hi_value);// ⑧ UE收ACK/NACK
        scheduler.handle_ul_crc(rnti, grant.pid, ok);      // ⑨ eNB记录CRC(失败→排重传)
    }
}
```

补充两个 C++ 点：
- **`std::mt19937 rng(42)`**：梅森旋转随机数引擎，**固定种子 42 保证每次运行结果完全相同**（可复现性，调试仿真程序的关键技巧）；`std::uniform_real_distribution` 生成 0~1 均匀分布用于 BLER 判定
- **`std::vector<std::unique_ptr<ue_context>>`**（场景 2）：ue_context 含 mutex 不可拷贝，vector 里只能放它的智能指针；`push_back(std::make_unique<ue_context>(rnti))` 是标准写法

### 🔍 代码精读指南

- [ ] `enb_ul_scheduler.cpp`：`handle_sr/bsr/ul_crc` → `calculate_mcs/tbs` → `generate_ul_grant_unlocked`（含NDI/RV管理）→ `deduct_ul_buffer_unlocked` → `schedule_pf`（重点）→ 快速过 rr 和 priority
- [ ] `schedule_ul()` 里用 `std::chrono::steady_clock` 包裹调度耗时，样本存入 `sched_latency_samples_`（配 `latency_mutex_`），`get_sched_latency_stats()` 返回 P50/P99——这是 P1 新增的实时性评估点（先读 PROTOCOL_NOTES.md §7.2.2）
- [ ] `mac_pdu.cpp`：先读 `mac_pdu.h` 的 subheader 布局（R/R/E/LCID）与 `mac_lcid` 枚举（Short=21/Long=26/Truncated=22/Padding=31）；`pack` 三步（BSR CE 优先→SDU 复用→Padding 填满），`unpack` 子头链解析与越界容错（先读 PROTOCOL_NOTES.md §7.2.3）
- [ ] 注意 `generate_ul_grant_unlocked` 的命名：它不加锁，因为调用它的 `schedule_pf` 已持有锁（又是阶段 3 学过的 `_unlocked` 模式！）
- [ ] `ue_context.h`：构造函数的依赖注入 → `run_tti()` → `handle_ul_grant()` 六步曲 → 三个 `on_xxx` 回调
- [ ] `main.cpp`：精读场景 1 的循环（上面已注解），场景 2/3/4 看差异点即可（多 UE / 高 BLER / 变流量）

### ✏️ 动手练习

- [ ] 场景 2 中通过修改调度器给 5 个 UE 设置不同 SNR（提示：`ue_sched_context::ul_snr` 目前固定 200，可给 `ul_scheduler` 加一个 `set_ue_snr(rnti, snr)` 方法），观察 PF 下五个 UE 的吞吐差异，再切换成 `ROUND_ROBIN` 对比
- [ ] **综合大作业**：新增 Max C/I 调度算法（只按 `current_rate` 排序、不除历史平均）——完整走一遍：`common_types.h` 加枚举值 → `enb_ul_scheduler.h` 声明 `schedule_max_ci()` → `.cpp` 实现（可复制 schedule_pf 改排序键）→ `schedule_ul()` 加 case → main.cpp 换算法运行对比
- [ ] 思考题：场景 1 第 109 行，为什么 eNB 能直接读 `ue.get_sr_manager().get_state()`？真实系统里可以吗？（答案：仿真捷径。真实系统中 SR 通过 PUCCH 物理信号传输，eNB 只能检测到能量，本项目用函数调用模拟了这条空口链路）

### 🏁 里程碑

✅ 能画出一个 TTI 内 ①~⑨ 的完整调用链；能解释 PF 公式如何平衡公平与效率；能说出成员初始化顺序陷阱；能讲清 MAC PDU 的 subheader(R/R/E/LCID) 布局与 BSR CE 编解码、以及调度耗时 P50/P99 的采集方式。

---

## 阶段 6：融会贯通与输出（3~5 天）

**目标**：从"看懂"升级为"能讲、能答、能溯源"，达到面试展示水平。

### 任务清单

- [ ] **画图输出**：不看任何资料，画出 ① SR 状态机 ② HARQ new_grant_ul 决策树 ③ SR→BSR→Grant→HARQ 完整时序图，再与 README §1.4/§3 对照修正
- [ ] **面试自测**：合上代码回答 README §7 的 Q1~Q7（上行完整流程 / BSR 三种触发 / NDI 判断 / RV 序列原因 / PF 原理 / SR 失败处理 / 配置授权 vs 动态授权），答不上的回对应阶段重读
- [ ] **调试器走查**：Debug 构建后用 gdb 在 `sr_manager::step` 和 `ul_harq_process::new_grant_ul` 设断点，单步跟踪一个完整的 SR→Grant→重传周期：
  ```bash
  cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug && make -j$(nproc)
  gdb ./ul_mac_manager
  (gdb) break ul_mac::sr_manager::step
  (gdb) run        # c继续 / n下一行 / s进入函数 / p 变量名 打印
  ```
- [ ] **协议对照**（进阶可选）：下载 3GPP TS 36.321，精读 §5.4.2（HARQ）/ §5.4.4（SR）/ §5.4.5（BSR）三节，对照 README §9 的符合性表，能说出本项目的 4 项简化（无 MAC PDU 构造、无 LCP、无下行、概率信道模型）
- [ ] **溯源 srsRAN**（进阶可选）：按代码注释标注的原型去读真实工程实现：
  - `srsRAN_4G/srsue/src/stack/mac/proc_sr.cc` ↔ 本项目 sr_manager
  - `srsRAN_4G/srsue/src/stack/mac/proc_bsr.cc` ↔ bsr_manager + lcg_buffer
  - `srsRAN_4G/srsue/src/stack/mac/ul_harq.cc` ↔ ul_harq_manager
  - `srsRAN_4G/srsenb/.../schedulers/sched_time_pf.h` ↔ ul_scheduler

### 🏁 里程碑

✅ 能脱稿回答 README 全部 7 道面试题；能指出项目相对协议/srsRAN 的简化点与增强点（自适应 SR、预测性 BSR、负载均衡）。

---

## 附录 A：C++ 知识点 ↔ 项目代码对照总表

复习时用这张表反查："这个知识点在项目哪里有真实用例？"

| C++ 知识点 | 项目中的位置 | 阶段 |
|---|---|---|
| namespace | 全部文件 `namespace ul_mac` | 1 |
| constexpr 常量 | common_types.h 的 MAX_HARQ_PROCESSES 等 | 1 |
| enum class | sr_state / harq_state / bsr_format 等 | 1 |
| struct + 初始化列表 | ul_grant / ue_metrics 等所有 struct | 1 |
| inline + static const 查表 | rv_of_irv() / bsr_index_to_bytes() | 1 |
| 单例模式 (magic static) | mac_logger::instance() / metrics_collector | 1 |
| std::mutex + lock_guard (RAII) | 所有管理器类 | 1~5 |
| ostringstream / iomanip 格式化 | mac_logger 的 log() / to_hex() | 1 |
| chrono 时间库 | mac_logger::get_timestamp() | 1 |
| 头/源分离与作用域解析符 | ue_sr_manager.h/.cpp | 2 |
| std::function + lambda 回调 | sr_tx_callback / ue_context 构造函数 | 2 |
| 锁内决策、锁外回调 | sr_manager::step() | 2 |
| 哨兵值 / 无符号回绕处理 | can_send_sr() 的 0xFFFFFFFF 与 TTI 回绕 | 2 |
| EMA 指数移动平均 | adjust_sr_period() / schedule_pf() | 2, 5 |
| 数组套 map 两级结构 | lcg_buffer_manager::lcgs_ | 3 |
| C++17 结构化绑定 | lcg_buffer.h 的 `for (auto& [lcid, state] : ...)` | 3 |
| mutable mutex + const 成员函数 | lcg_buffer_manager | 3 |
| _unlocked 私有方法防死锁 | get_buffer_state_lcg_unlocked() / generate_ul_grant_unlocked() | 3, 5 |
| 环形缓冲区 (取模下标) | bsr_manager 的 buffer_history_ | 3 |
| 对象组合 (has-a) | harq_manager→process / ue_context→四组件 | 4, 5 |
| std::unique_ptr / make_unique | harq processes_ / main.cpp 的 UE 容器 | 4, 5 |
| std::atomic vs mutex 选型 | ul_harq_process 的成员 | 4 |
| 嵌套结构体返回值 | ul_harq_process::tx_action | 4 |
| 增量式/比率式统计 | ul_harq_manager 的 average_retx_ | 4 |
| std::sort + lambda 比较器 | schedule_pf() / schedule_priority() | 5 |
| 向上取整除法 (a+b-1)/b | generate_ul_grant_unlocked() 的 n_prb 计算 | 5 |
| 引用成员与依赖注入 | bsr_manager::buffer_mgr_ | 5 |
| 成员初始化顺序 = 声明顺序 | ue_context 构造函数 | 5 |
| mt19937 固定种子可复现随机 | main.cpp 各场景 | 5 |
| static_cast 显式转换 | 全项目 (整数除法转 double 等) | 1~5 |

## 附录 B：总览时间表与学习建议

### 时间表

| 周 | 内容 | 里程碑 |
|---|---|---|
| 第 1 周 | 阶段 0 + 1 | 能编译运行；掌握 enum class / 单例 / RAII |
| 第 2 周 | 阶段 2 + 3 前半 | 独立讲清 SR 状态机与回调机制 |
| 第 3 周 | 阶段 3 后半 + 4 | 独立讲清 BSR 触发/格式选择与 NDI 判断 |
| 第 4 周 | 阶段 5 | 理解 PF 调度；能画 TTI 主循环调用链 |
| 第 5 周 | 阶段 6 | 脱稿回答全部面试题 |

### 三条贯穿始终的学习建议

1. **改代码 > 读代码**：每学一个模块，至少改一个参数、加一条日志、跑一次验证。仿真一次只需几秒，试错成本极低。
2. **调试器 > 反复阅读**：gdb 单步跟踪一个 TTI 内发生的所有事情，比读十遍代码更有效（阶段 6 有具体命令）。
3. **双线并进，先协议后代码**：每个阶段先读本文档的"知识讲解"和 README 对应章节建立协议概念，再读代码看实现——顺序不要反。

---

## 附录 C：通信协议知识补充（面试防问）

> 本项目代码只覆盖了协议的一部分，但面试官会顺着项目往外问。本附录把"项目周边一圈"的协议知识补齐，每节都标注了与项目的关联点。

### C.1 LTE 帧结构与时间单位

| 概念 | 值 | 说明 |
|---|---|---|
| 无线帧 (Radio Frame) | 10 ms | 由 10 个子帧组成 |
| 子帧 (Subframe) = **TTI** | 1 ms | 调度的基本时间单位——本项目主循环每次迭代就是 1 个 TTI |
| 时隙 (Slot) | 0.5 ms | 每子帧 2 个时隙，每时隙 7 个 OFDM 符号（常规 CP） |
| SFN (系统帧号) | 0~1023 循环 | 项目中 `tti` 一直递增，真实系统是 `SFN×10 + subframe` 循环回绕——这就是 `can_send_sr()` 处理 TTI 回绕的原因 |
| PRB (物理资源块) | 频域 12 子载波 × 时域 1 时隙 | 频域调度的最小单位；20MHz 带宽 = 100 个 PRB（本项目 `ul_nof_prb=50` 即 10MHz） |

NR 差异：NR 的时隙长度随参数集 μ 变化（15kHz→1ms，30kHz→0.5ms，120kHz→0.125ms），调度单位从固定 1ms TTI 变为灵活的 slot 甚至 mini-slot。

### C.2 MAC 在协议栈中的定位与六大功能

```
  NAS / RRC        (控制面: 信令)
  ─────────
  PDCP    ← 加密、完整性保护、头压缩(ROHC)
  RLC     ← 分段/重组、ARQ重传(AM模式)
  MAC     ← ★本项目所在层
  PHY     ← 编码调制、HARQ软合并的物理实现
```

MAC 层六大功能（TS 36.321 §4.4，面试必背）：
1. **逻辑信道与传输信道的映射**（DTCH/DCCH → UL-SCH）
2. **复用/解复用**：把多个逻辑信道的数据装进一个 MAC PDU（本项目简化省略）
3. **调度信息上报**：SR + BSR ← 本项目核心
4. **HARQ 纠错** ← 本项目核心
5. **逻辑信道优先级处理 (LCP)** ← 本项目 `consume_data()` 实现了两阶段令牌桶 LCP（§5.4.3.1）
6. **随机接入过程**（见 C.9）

关键辨析：**RLC 的 ARQ vs MAC 的 HARQ**——HARQ 快（8ms 级）但只能重传有限次数且靠 1-bit ACK/NACK（可能出错）；RLC AM 的 ARQ 慢但基于状态报告可靠。两层重传是互补关系：HARQ 达到 `max_harq_tx` 丢弃后（项目里的 `is_discarded`），由 RLC ARQ 兜底重传。

### C.3 MAC PDU 与 MAC CE 结构

MAC PDU = MAC 头（若干子头）+ MAC CE（控制单元）+ MAC SDU（RLC 数据）+ Padding。

每个子头含 LCID 字段（5-bit），LCID 既标识逻辑信道也标识 CE 类型。常见上行 MAC CE（LCID 值需记住量级即可）：

| MAC CE | LCID | 大小 | 项目关联 |
|---|---|---|---|
| Short BSR | 11101 | 1 字节 | `SHORT_BSR_SIZE=1` |
| Truncated BSR | 11100 | 1 字节 | 格式同 Short，仅 LCID 不同 |
| Long BSR | 11110 | 3 字节 | `LONG_BSR_SIZE=3` |
| PHR | 11010 | 1 字节 | 见 C.8 |
| C-RNTI | 11011 | 2 字节 | 随机接入 Msg3 中携带 |

CE 装入 PDU 的优先级：**C-RNTI/数据 > BSR > PHR > 普通数据 > Padding BSR**——这解释了为什么 Padding BSR 是"最后有空才发"的机会主义报告。

### C.4 SR 深入（TS 36.321 §5.4.4）

- **物理承载**：SR 走 **PUCCH format 1**，是纯能量检测信号（on/off keying），不携带任何比特——所以 eNB 只知道"该 UE 要资源"，不知道要多少（这是 BSR 存在的意义）
- **前提**：UE 必须已被 RRC 配置专用 SR 资源（`sr-PUCCH-ResourceIndex` + `sr-ConfigIndex` 决定周期和位置，即项目里的 `sr_period`）
- **sr-ProhibitTimer**：发出一次 SR 后的禁止期，防止 SR 风暴——项目里 `can_send_sr()` 的周期判断承担了类似角色
- **dsr-TransMax**：SR 最大发送次数（项目 `max_sr_tx`），取值 {4,8,16,32,64}
- **SR 失败后果**（面试连环问高频）：`sr_counter >= dsr-TransMax` 时，UE **释放 PUCCH/SRS 资源、清空所有配置的下行分配和上行授权，发起随机接入**——项目中 `sr_state::FAILED` + `on_max_sr_reached` 回调即模拟"该发起 RA 了"这个时刻
- **没有 SR 资源的 UE 怎么办**：直接发起随机接入要资源（SR 与 RA 是两条并行的"要资源"通道）

### C.5 BSR 深入（TS 36.321 §5.4.5）

- **LCG 映射**：RRC 通过 `logicalChannelGroup`（0~3）把每个逻辑信道映射到 LCG——项目 `add_lcg()/add_bearer()` 模拟此配置。典型映射：LCG0=SRB（信令），LCG1~3=DRB 按 QCI 分组
- **6-bit 缓冲区索引表**（TS 36.321 Table 6.1.3.1-1）：64 级，**指数分布**（0, ≤10, ≤12, ≤14 … ≤150000, >150000 字节）。为什么指数而不是线性？——小缓冲量需要精确（调度小包），大缓冲量只需量级（反正要多次调度），用 6 bit 覆盖 5 个数量级。项目 `bsr_index_to_bytes()` 即此表的逆映射
- **三种触发的记忆锚点**：Regular=事件驱动（新数据且优先级更高/从空变有），Periodic=定时器驱动，Padding=机会驱动（PDU 有剩余空间）
- **取消规则**（已在 §3.8 详述）：授权容纳全部数据→取消；发出非 Truncated BSR→取消；Truncated 不取消
- **NR 差异**：NR 支持 **8 个 LCG**，Long BSR 变长格式（1 字节 bitmap 指示哪些 LCG 有报告 + 变长 BS 字段），缓冲区索引扩展为 **8-bit（256 级）**

### C.6 HARQ 深入（TS 36.321 §5.4.2）

| 维度 | LTE 上行 | LTE 下行 | NR |
|---|---|---|---|
| 同步性 | **同步**（重传时刻固定 = 初传+8ms，所以 8 进程） | 异步 | 上下行均**异步**（16 进程） |
| 自适应性 | 两者皆可（见下） | 自适应 | 自适应 |
| 反馈信道 | PHICH（ACK/NACK） | PUCCH/PUSCH | NR 废除 PHICH，重传全靠 DCI 调度 |

- **非自适应重传**：UE 收到 PHICH NACK 且无新 DCI → 在固定时刻用**相同资源/MCS**重传，RV 按 IRV 序列自动递进——项目里 `rv=-1` + UE 侧 `current_irv_` 递进即模拟此行为
- **自适应重传**：eNB 发新的 DCI 0（NDI 不翻转）→ 可换资源/MCS——项目里 NDI 未变化 + `generate_retx()` 即此路径
- **为什么 8 进程**：上行 HARQ RTT = 8ms（n 发送 → n+4 收 PHICH → n+8 重传），流水线需要 8 个进程填满时间轴
- **IR vs Chase 合并**：Chase Combining 每次重传相同比特（能量累积）；**增量冗余 IR** 每次发不同 RV（冗余比特不同，合并后等效降低码率）——LTE 用 IR，这就是 RV 存在的意义
- **RV 序列 {0,2,3,1} 的原因**：RV0 含最多系统比特（可自解码），RV2 与 RV0 的冗余比特重叠最少（合并增益最大），RV3、RV1 依次补齐——顺序按"合并增益最大化"设计，而非 0,1,2,3
- **NDI 契约**（本项目最核心的协议点）：NDI 是**每 UE 每进程 1-bit 翻转标志**，不是"新传=1"的绝对值。UE 比较本次与上次：变了=新传（清软缓冲），没变=重传（软合并）。两侧状态必须对齐——项目早期版本 `ndi = !is_retx` 正是把它当绝对值用的典型错误（见附录 D-①）

### C.7 上行调度与 DCI

- **上行授权载体**：**DCI format 0**（LTE 上行），经 PDCCH 下发，CRC 用 C-RNTI 加扰（UE 以此识别"这是给我的"）。DCI 0 主要字段：RB 分配、MCS+RV、**NDI**、TPC（功控）、CQI 请求——项目 `ul_grant` 结构体即其抽象
- **时序**：TTI n 收到 DCI 0 → TTI **n+4** 在 PUSCH 发送（UE 需要处理时间）
- **动态调度 vs 半持续调度**：
  - 动态：每次传输都要 SR→BSR→DCI，信令开销大但灵活
  - **SPS（LTE）/ Configured Grant（NR）**：RRC 预配置周期性授权，UE 到点就发，免 SR/BSR——适合 VoIP（周期小包）；NR CG Type 1 纯 RRC 激活，Type 2 需 DCI 激活。README 面试题 Q7 即此对比
- **三大经典调度算法**（项目全部实现）：

| 算法 | 排序键 | 优点 | 缺点 |
|---|---|---|---|
| Round Robin | 无（轮流） | 绝对公平、简单 | 不看信道，吞吐差 |
| Max C/I | 瞬时速率 | 小区吞吐最大 | 边缘 UE 饿死 |
| **PF** | 瞬时速率 / 历史平均 | 吞吐与公平兼顾 | 实现稍复杂 |

### C.8 PHR（功率余量报告）

PHR = P_max - 当前 PUSCH 所需功率，告诉 eNB "我还有多少功率余量"。**为什么调度器需要它**：给 UE 分配的 PRB 越多，所需发射功率越大；若 UE 已到功率上限，多给 PRB 反而导致每 PRB 能量下降、解调失败。所以调度器 = BSR（要多少数据）+ PHR（能用多大功率）+ CQI/SRS（信道多好）三者联合决策。项目中 `handle_phr()` 是预留接口（参数暂未使用），面试可主动说明这是下一步扩展点。

### C.9 随机接入与 Msg3（TS 36.321 §5.1）

四步 RA（面试常问 SR 失败后的流程）：
1. **Msg1**：UE 在 PRACH 发 preamble（Zadoff-Chu 序列）
2. **Msg2 (RAR)**：eNB 回随机接入响应——含 TA 命令、**首个上行授权**、Temporary C-RNTI
3. **Msg3**：UE 用 RAR 授权发 RRC 请求/C-RNTI CE——**Msg3 的 HARQ 特殊**：这是项目 `is_msg3` 标志的来源，Msg3 重传不看 NDI（此时还没有正常的 DCI 流程），由 RAR/PDCCH order 显式触发
4. **Msg4**：竞争解决

NR 补充：还有 2-step RA（MsgA=preamble+数据，MsgB=响应），降低时延。

### C.10 LTE vs NR 关键差异速查表

| 维度 | LTE | NR | 项目采用 |
|---|---|---|---|
| TTI | 固定 1ms | 灵活 slot（随 μ 变化） | 1ms 抽象 |
| 上行 HARQ | 同步、8 进程、PHICH | 异步、16 进程、无 PHICH | **16 进程**（NR 上限）+ 显式反馈 |
| LCG 数 | 4 | 8 | **4**（LTE） |
| BSR 索引 | 6-bit / 64 级 | 8-bit / 256 级 | 6-bit |
| 上行波形 | SC-FDMA | CP-OFDM 或 DFT-s-OFDM | 不涉及 |
| 免授权传输 | SPS | Configured Grant Type 1/2 | 未实现（面试题 Q7） |

（可主动向面试官说明：本项目参数混用了 LTE 的 LCG/BSR 格式与 NR 的 HARQ 进程数，属教学性简化。）

### C.11 面试连环问 Q&A（按追问深度排列）

1. **Q：UE 有数据要发，完整流程是什么？** A：RLC 数据到达 → 触发 Regular BSR → 无上行资源 → 触发 SR → PUCCH 发 SR → eNB 下发小授权 → UE 发 BSR → eNB 按 BSR 下发足额授权 → UE 发数据（+HARQ 保障）。
2. **Q：为什么有了 SR 还要 BSR？** A：SR 是 1-bit 能量信号只表达"要资源"，BSR 才告知"要多少、哪个优先级"。
3. **Q：为什么有了 BSR 还要 SR？** A：BSR 是 MAC CE，必须有 PUSCH 资源才能发；没资源时只能靠 PUCCH 上的 SR（或 RA）打破僵局——"先有鸡还是先有蛋"问题的协议解法。
4. **Q：UE 怎么区分新传和重传？** A：比较该 HARQ 进程的 NDI 是否翻转：翻转=新传（清软缓冲、IRV 归零），未翻转=自适应重传（软合并、RV 按授权或 IRV 递进）。
5. **Q：NACK 之后一定重传吗？** A：不一定。达到 max_harq_tx 则丢弃交 RLC ARQ 兜底；eNB 也可以直接调度新传（NDI 翻转）放弃该 TB。
6. **Q：BSR 报告的是 RLC 还是 MAC 的缓冲？** A：RLC 待传+重传缓冲 与 PDCP 待传数据之和，按 LCG 聚合——MAC 自己没有缓冲区，项目里 `lcg_buffer_manager` 模拟的正是"RLC 视角的缓冲"。
7. **Q：调度器凭什么决定给谁多少 PRB？** A：BSR（需求量）+ SR（有无请求）+ CQI/SRS（信道质量→MCS）+ PHR（功率约束）+ 调度算法（公平性策略）+ HARQ 重传优先。项目实现了其中 4 项，PHR 是预留接口。

---

## 附录 D：项目修正记录（面试加分素材）

> 2026-07 对全部代码做了一轮协议符合性审查，发现并修复以下问题。**每一条都是现成的面试故事**："我在自查中发现了 X，根因是 Y，我按协议 Z 修复并验证"——比"项目一次写对"可信得多。

| # | 问题 | 为什么错 | 修复 | 验证 |
|---|---|---|---|---|
| ① | 调度器 `grant.ndi = !is_retx` | 把 NDI 当"新传=1"的绝对值，而协议是**每进程翻转标志**；UE 侧比较"是否变化"，会把连续新传误判为重传 | eNB 侧为每 UE 每进程维护 `ndi[16]`：新传翻转、重传保持（TS 36.321 §5.4.2.1） | 日志中新传/重传判定与调度器意图一致 |
| ② | `avg_retx` 恒为 1.0 | 统计时把"本次是否重传(0/1)"当样本求均值，样本集只含重传事件 | 改为比率式：`total_retx / total_new_tx` | BLER=0.3 时实测 0.49 ≈ 几何分布理论值 p/(1-p)=0.43 |
| ③ | BSR 统计 regular_count 恒为 0 | `need_to_send_bsr_on_ul_grant()` 先调 `generate_bsr()`（内部清 `triggered_type_`）再读它做统计 | 调用前先把触发类型存局部变量（"先存后清"） | 场景 4 三种触发计数均非零 |
| ④ | 注释/文档写 "5-bit、Long BSR 4 字节" | 协议是 **6-bit** 缓冲区索引（64 级）、Long BSR **3 字节**（4×6bit=24bit） | 全部注释与 `LONG_BSR_SIZE` 修正 | 对照 TS 36.321 §6.1.3.1 |
| ⑤ | SR 统计在锁外更新 + 收到任意授权都计"SR 成功" | 数据竞争；且未发 SR 时收到授权也被计入成功 | stats 移入锁内；成功计数加 `sr_counter_ > 0` 守卫 | 场景 1 sent=30/success=30 自洽 |
| ⑥ | `sr_pending` 在进入调度队列时就被清除 | 若该 TTI PRB 耗尽未实际授权，SR 请求被无声丢失 | 移到**实际生成授权**的分支里再清 | 多 UE 高负载下 SR 不再丢失 |
| ⑦ | `calculate_required_prb()` 死代码 | 公有函数内加锁后调用同样加锁的函数 → 潜在死锁；且从未被调用 | 直接删除 | 编译零警告 |
| ⑧ | UE 缓冲区永不排空；eNB 授权后不扣减缓冲估计 | 数据只进不出 → SR/BSR 生命周期无法周期重现；eNB 在 BSR 刷新前会重复授权 | UE 侧新增 `consume_data()`（简化 LCP，按优先级扣减）；eNB 侧新增 `deduct_ul_buffer_unlocked()`（对应 srsRAN sched_ue 行为） | 场景 1 出现完整"排空→新数据→再触发"循环 |
| ⑨ | 重传授权 `rv` 由 eNB 直接指定 0 | 非自适应重传应由 UE 按 IRV 序列自行递进 | 重传时 `rv=-1`，UE 侧 `current_irv_` 递进 | 日志 RV 序列 0→2→3→1 |
| ⑩ | `-Wextra` 警告 3 处 | 未用参数、sign-compare | `(void)` 标记 + `static_cast` | -Wall -Wextra -Wpedantic 零警告 |
| ⑪ | **HARQ 丢弃分支自死锁**（Windows 场景 2 卡死） | `new_grant_ul()` 持锁状态下命中"Max ReTX"分支时调用 `reset()`，同一把 `std::mutex` 二次加锁 = UB（实际表现为永久挂起）。Linux 未复现是因为 `uniform_real_distribution` 实现跨平台不同、随机序列恰好未命中该分支——**平台差异暴露潜伏 bug，而非平台问题** | 新增私有 `reset_unlocked()`，持锁分支改调它（项目既有 `_unlocked` 模式） | 单测强制触发 discard 分支不再挂起；完整回归中场景 3 命中同分支正常返回 |
| ⑫ | LCP令牌桶空壳：`lc_config` 声明了 `pbr`/`bsd` 但 `consume_data()` 完全忽略 | 协议要求两阶段LCP：先按PBR令牌桶分配，再按优先级分配剩余；原代码只做纯优先级排序 | `lc_buffer_state` 增加 `token_bucket_size`/`token_count`；`consume_data()` 重写为两阶段算法；`step_token_buckets()` 每TTI补充令牌 | 单测验证PBR限制下高优先级信道不会饿死低优先级 |
| ⑬ | MCS/TBS 用线性公式 `mcs=snr*28/30`、`tbs=n_prb*(mcs+1)*72/8` | 不符合 TS 36.213 标准查找表，高/低SNR区间偏差大 | `calculate_mcs()` 改为29阈值区间映射表；`calculate_tbs()` 改为6锚点TS 36.213效率表+线性插值 | SNR=2dB→MCS=7 与标准一致；TBS随MCS单调递增 |
| ⑭ | HARQ 无 RTT 延迟建模，反馈即时处理 | 实际 PHICH 反馈需 8 TTI 延迟，即时处理导致重传时序不真实 | `ul_harq_process` 新增 `tx_tti_`/`rtt_ttis_`/`feedback_pending_`；RTT 内不处理反馈 | 单测验证 RTT 内 ACK 不生效，RTT 后生效 |
| ⑮ | 无自动测试，只能人工验证 | 代码变更无回归保障 | 新增19个单元测试覆盖LCP/MCS-TBS/HARQ/SR/BSR/延迟/死锁/线程安全；CMakeLists 添加 test target | `ctest` 19/19 全通过 |
| ⑯ | SR自适应4档if-else；BSR预测仅加权平均 | 离散阈值导致周期跳变；加权平均无法预测趋势 | SR改为对数连续映射`K/log2(1+rate)`+20%迟滞；BSR改为最小二乘线性回归 | SR周期变化平滑；BSR预测能捕捉递增/递减趋势 |
| ⑰ | 早期终止/差分BSR 头文件声明但未实现 | 注释提到功能但代码为空 | HARQ：连续NACK≥3+重传≥2→提前丢弃；BSR：Padding BSR全部LCG无变化→跳过 | 单测验证早期终止触发；Padding BSR数量减少 |
| ⑱ | 无延迟统计（P50/P90/P99） | 缺少端到端延迟分布，无法评估QoS | `metrics_collector` 新增 `latency_stats`；`ue_context` 追踪数据到达→发送的TTI差 | 场景4输出 P50=71/P90=200/P99=233 TTI |
| ⑲ | `get_all_process_info()` 16次重复加锁+`cur_tbs_`数据竞争 | manager mutex 重复加锁16次效率低；`get_current_tbs()` 无锁访问非原子字段 | 新增 `fill_process_info()` 在process锁内一次性读取；`get_all_process_info()` 改为1次加锁 | 并发读写测试无崩溃 |

> **后续 P0/P1 修复（2026-08 在 Windows + g++ 下实施，详见 PROTOCOL_NOTES.md §10）**——这是一轮"关键缺陷清零 + 能力增强"的专项，同样是可讲的面试故事（"我拿到一个能跑但不健壮的代码库，做了系统性加固"）。

| # | 问题 | 为什么错 | 修复 | 验证 |
|---|---|---|---|---|
| ⑳ | BSR 索引向上取整（`ceil` 找第一个 ≥ 的索引） | 高估缓冲区，使 eNB 分配超出实际需求的资源 | `bytes_to_bsr_index()` 改为向下取整（找最后一个 ≤ 的索引） | 单测 `test29` 严格校验索引语义 |
| ㉑ | PRB/TBS 计算分散、不统一 | 多处重复计算口径不一致，易埋 bug | 统一 `calculate_tbs` / `required_prb` 入口 | 单测验证 TBS 随 MCS 单调递增 |
| ㉒ | 重传 TBS 被重新计算 | 重传用新算的 TBS/MCS，与首传不一致，破坏 NDI/RV 语义 | 重传锁定 `harq_tb[pid]`（首传记录的 tbs/mcs/prb），新传记录、重传复用 | 单测验证重传 TBS 与首传一致 |
| ㉓ | HARQ 待重传用单值 `pending_retx_pid`（16 进程覆盖） | 多进程并行重传时互相覆盖，丢失重传请求 | 改为 `std::array<bool, MAX_HARQ_PROCESSES> pending_retx` 位图，遍历所有待重传进程 | 单测 `test18` 适配位图 |
| ㉔ | `get_ue_context()` 返回悬垂引用/指针 | 返回栈上或 map 内部引用，调用方持有期失效 → 未定义行为 | 改为返回 `std::optional<ue_sched_context>`（值拷贝） | 编译通过 + 单测验证 |
| ㉕ | 有符号/无符号混用（`ul_snr` 等） | SNR 可为负，无符号导致比较/运算异常与越界 | `ul_snr` 等改 `int32_t`，统一有符号处理 | -Wall -Wextra 零警告 |
| ㉖ | 未初始化成员变量 | 部分字段默认构造值不确定，行为不可复现 | 结构体成员显式初始化 | 编译零警告 |
| ㉗ | CMake 误列不存在的源文件（header-only 被当 cpp 编） | `lcg_buffer_manager.cpp`/`ue_context.cpp` 实际为 header-only，编译报找不到文件 | 修正 CMakeLists，仅含真实源文件；README 列出 6 个 cpp 直接 g++ 编译路径 | g++ 编译 `RUN_EXIT=0` |
| ㉘ | （P1）PF 分子用"待传字节数"，未体现信道能力 | 低需求高信道质量 UE 反而获低权重，违背 PF 初衷 | 分子改为「`calculate_tbs(mcs, total_prb)` 信道可支持速率封顶后的需求」 | 单测 + 仿真吞吐对比 |
| ㉙ | （P1）无调度实时性指标 | 无法评估调度器延迟分布 | `schedule_ul` 用 `steady_clock` 计时，存 `sched_latency_samples_`，新增 `get_sched_latency_stats()` 返回 P50/P99 | 主程序实测 P50≈1μs / P99≈34μs |
| ㉚ | （P1）无 MAC PDU 组包/解包 | 调度结果无法落地为真实字节流，eNB 侧无法解包校验 | 新增 `mac_pdu.h/.cpp`：`pack`/`unpack`，subheader(R/R/E/LCID)、Short/Long/Truncated BSR CE、SDU 复用 + Padding、越界容错 | 新增 test30-34（34/34 全通过） |

**P0/P1 叙事建议**：优先讲 ㉓+㉔（内存安全：从"单值覆盖"到"位图"、从"悬垂引用"到"`optional` 值语义"——最能体现 C++ 内存模型与生命周期理解）、㉒（协议一致性：重传 TBS 锁定的工程必要性）、㉓→㉗ 整体体现"在 Windows + g++ 严格编译(`-Wall -Wextra -Wpedantic`)下做健壮化"的工程能力。P1 的 ㉚ 可作为"不止修 bug，还补模块"的收尾亮点。

**面试叙事建议**：优先讲 ①（NDI 契约，最能体现协议理解深度）、②（用几何分布理论值验证仿真，体现工程严谨）、⑧（建模完整性：闭环仿真必须让状态能循环）、⑪（跨平台死锁排查：从"Windows 才卡死"的表象定位到随机序列差异 + 同锁重入的真因，并用最小复现验证——完整的并发 debug 故事，可与附录 E.6 死锁专题串讲）。

**新增修正（⑫~⑲）的面试叙事建议**：优先讲⑬（MCS/TBS从线性到查表的协议符合性提升）、⑭（HARQ RTT建模体现对物理层时序的理解）、⑮（从0到19个测试的工程规范提升）。⑫（LCP令牌桶）可与⑧串讲，体现"从简化到完整"的迭代过程。⑯（SR/BSR增强深度）适合展示"不只是实现协议，还能优化协议"的工程能力。

---

## 附录 E：C++ 并发与多线程面试专题

> 本项目每个管理器类都是按"可能被多线程调用"设计的（对应真实 srsRAN 中 PHY 线程/RRC 线程/定时器线程并发访问 MAC 的场景），因此并发是本项目面试的必问方向。本附录每题给出【标准答法】+【项目实例】，能把答案落到自己代码上是最强的加分项。

### E.1 多线程基础：线程创建、同步、线程池

**Q：C++ 里怎么创建线程？主线程怎么等它结束？**

```cpp
std::thread t([]{ /* 工作 */ });   // 构造即启动
t.join();      // 阻塞等待结束(必须 join 或 detach, 否则析构时 std::terminate)
t.detach();    // 分离: 后台自生自灭, 风险是访问已销毁的外部变量
```

要点：① `std::thread` 构造即运行，参数按值拷贝（传引用要 `std::ref`）；② C++20 的 `std::jthread` 析构自动 join；③ 同步三件套：`mutex`（互斥）、`condition_variable`（等待/通知）、`atomic`（无锁原子）。

**Q：线程池的原理？** A：预创建 N 个工作线程 + 一个任务队列；工作线程循环从队列取任务执行（队列空则在 condition_variable 上休眠）。好处：避免频繁创建/销毁线程的开销（Linux 上约百微秒级）、限制并发度防止资源耗尽。真实 srsRAN 的 `task_scheduler` 就是典型实现：各层把任务 post 到队列，由 stack 线程串行执行——这也是一种并发控制：**把多线程问题收敛到单线程队列里，比到处加锁更优雅**。

**项目实例**：本项目 main.cpp 是单线程仿真，但所有管理器都按线程安全设计——面试可以主动说："我的锁粒度是每个管理器一把锁，如果未来把 UE 侧和 eNB 侧拆到两个线程（模拟真实协议栈），代码不需修改即可安全运行"。

### E.2 竞态条件与数据竞争

**Q：什么是竞态条件（race condition）？和数据竞争（data race）一样吗？**

- **数据竞争**：两个线程无同步地访问同一内存位置且至少一个是写——C++ 标准定义的 **UB（未定义行为）**
- **竞态条件**：结果依赖线程执行时序——更宽泛的逻辑问题，即使每次访问都加锁也可能存在（"check-then-act"两次加锁之间状态变了）

**项目实例（真实 bug，见附录 D-⑤）**：早期版本 `sr_manager` 在**锁外**执行 `stats_.total_sr_sent++`——`++` 实际是"读-改-写"三步，并发下两个线程可能各自读到 5、各自写回 6，丢失一次计数，且构成数据竞争 UB。修复：把统计更新移入 `lock_guard` 保护区。面试讲这个故事 = 同时证明你懂概念 + 会排查。

### E.3 锁机制详解：mutex / 读写锁 / 自旋锁

| 锁 | C++ 类型 | 机制 | 适用场景 |
|---|---|---|---|
| 互斥锁 | `std::mutex` | 抢不到就**睡眠**（内核调度出去） | 临界区较长、通用首选 |
| 读写锁 | `std::shared_mutex` (C++17) | 多读共享（`shared_lock`）、写独占（`unique_lock`） | **读多写少**（如配置表） |
| 自旋锁 | 标准库无，可用 `atomic_flag` 实现 | 抢不到就**忙等**（烧 CPU） | 临界区极短且线程不会被换出（内核/实时系统） |
| 递归锁 | `std::recursive_mutex` | 同一线程可重复加锁 | 通常是**设计坏味道**，应用 `_unlocked` 模式替代（见 E.6） |

选型直觉：临界区 < 1 微秒且竞争激烈 → 考虑自旋/原子；否则用 mutex（Linux 的 futex 实现在无竞争时只是一次用户态 CAS，开销很小）。

**项目实例**：全项目统一用 `std::mutex`——因为每个临界区都很短（几次字段读写）且读写比例接近，上 shared_mutex 反而增加开销。可主动说：若 `ul_scheduler::ue_db_` 变成"调度频繁读、RRC 偶尔增删 UE"的真实负载，它是全项目最适合改造成 `shared_mutex` 的地方。

### E.4 RAII 与锁管理：lock_guard / unique_lock / scoped_lock

**RAII（资源获取即初始化）**：资源的生命周期绑定到对象生命周期——构造获取、析构释放。好处：**任何退出路径（return/异常/break）都不会忘记释放**。

```cpp
// 项目中随处可见的模式 (如 ue_sr_manager.cpp 每个公有方法开头):
std::lock_guard<std::mutex> lock(mutex_);   // 构造加锁
// ... 中途任意 return, 析构自动解锁, 永不漏
```

三兄弟对比（面试高频）：

| 工具 | 能力 | 何时用 |
|---|---|---|
| `lock_guard` | 构造锁/析构解，零额外功能 | 默认首选（本项目全部用它） |
| `unique_lock` | 可中途 unlock/延迟加锁/可移动 | 配合 `condition_variable::wait` 必须用它 |
| `scoped_lock` (C++17) | 一次锁多把（内部避免死锁） | 需同时持有多把锁时 |

RAII 不止管锁：项目里 `std::unique_ptr`（管内存）、`std::ofstream`（mac_logger 的 file_stream_，析构自动关文件）都是 RAII。一句话总结给面试官：**"C++ 没有 finally，因为有 RAII"**。

### E.5 std::atomic vs mutex：选型标准（项目最佳实例）

选型三问：
1. 保护的是**单个标量**还是**多个相关变量**？单个→atomic，多个→mutex（原子只保证单变量操作不可分，**不保证多变量间一致性**）
2. 是否需要"读-判断-写"的**复合事务**？需要→mutex（或 CAS 循环）
3. 是否在热路径上被高频读？是→atomic（免锁读）

**项目实例（[ue_ul_harq_manager.h](file:///home/lulu44/Desktop/Lulu44/srsRAN/ul_mac_manager/include/ul_mac/ue_ul_harq_manager.h) 的混合设计，面试必讲）**：

```cpp
class ul_harq_process {
    std::atomic<uint32_t> current_tx_nb_;   // 单标量计数, 外部高频查询 → atomic
    std::atomic<bool>     harq_feedback_;   // 1-bit 状态标志 → atomic
    bool                  cur_ndi_;         // 与 cur_tbs_/cur_rv_ 必须成组更新
    uint32_t              cur_tbs_;         //   (一次授权同时改三个) → 用下面的mutex
    mutable std::mutex    mutex_;
};
// 管理器层同理: average_retx_/total_pkts_ 是 atomic (免锁读给 metrics),
//               stats_ 结构体多字段 → mutex
```

为什么 `cur_ndi_/cur_tbs_/cur_rv_` 不能各自用 atomic？——如果三个各自原子，读者可能读到"新 NDI + 旧 TBS"的**擕裂组合**——单个变量都没坏，但组合无效。这就是"原子≠线程安全"的经典例子。

### E.6 死锁：四条件、预防策略、项目中的两个模式

**死锁四必要条件**（背诵）：互斥、持有并等待、不可剥夺、**循环等待**——破坏任一即可预防，工程上最常用的是破坏循环等待（**全局统一加锁顺序**）或避免持有并等待（`scoped_lock` 一次性拿齐）。

本项目沉淀了两个防死锁模式，都是可直接写进简历的实践：

**模式①：锁内决策、锁外回调**（[ue_sr_manager.cpp](file:///home/lulu44/Desktop/Lulu44/srsRAN/ul_mac_manager/src/ue_sr_manager.cpp) `step()`，对应 srsRAN proc_sr.cc）：

```cpp
bool do_send_sr = false, do_fail = false;
{
    std::lock_guard<std::mutex> lock(mutex_);   // 锁内: 只做状态判断与修改
    if (state_ != sr_state::PENDING) return;
    ... do_send_sr = true; ...                   // 把"要做什么"记在局部变量
}                                                // ← 锁在这里释放
if (do_send_sr && tx_callback_) tx_callback_(rnti_);   // 锁外: 执行回调
```

为什么？回调是**外部代码**（`std::function` 里装的是 ue_context 注入的 lambda），它可能反过来调本类的公有方法（再抢同一把锁→自死锁），也可能去抢另一把锁（与其他线程反向加锁→交叉死锁）。**持锁时永远不要调用你不控制的代码**——这是并发编程第一铁律。

**模式②：`_unlocked` 私有方法**（lcg_buffer / ul_scheduler）：公有方法加锁后调用不加锁的私有 `xxx_unlocked()` 干活；私有方法之间互相调用也走 `_unlocked` 版本，避免同线程重复加锁（`std::mutex` 重复加锁是 UB！）。**反面教材就在附录 D-⑦**：被删除的 `calculate_required_prb()` 正是"公有加锁函数内调另一个公有加锁函数"的潜在死锁。

**模式③：状态机死锁（活锁）——非锁引起的无限循环**：

并发死锁不止"锁互相等"这一种。另一种更隐蔽的死锁是**状态机卡死**——没有任何线程阻塞，但程序陷入无限循环，永远无法推进。本项目的**场景3**曾实际出现过这种死锁，这里完整复盘。

#### 问题现象

场景3模拟"高BLER (30%) 下的HARQ重传"：UE每5TTI产生1000字节数据，调度器每TTI发授权，HARQ最多重传4次。正常情况下500TTI应在毫秒级完成，但程序在某次运行中**挂死不动**，`timeout` 被迫终止。

#### 根因分析

死锁触发条件：**当一个TB（传输块）达到最大重传次数被丢弃时**。完整链路如下（4步环）：

```
① TB丢弃 → HARQ进程被 reset_unlocked()
② pending_retx_pid 未清除 → 调度器仍标记该PID为重传待处理
③ 调度器发重传授权(is_retx=true, NDI不翻转) → HARQ进程已reset
④ HARQ误判为新传(!is_grant_configured_ → true) → 消耗缓冲区 → CRC再次失败 → 回到②
```

对应到代码行：

```cpp
// ue_ul_harq_manager.cpp - new_grant_ul() 判定逻辑 (step 3)
bool is_new_tx = (!is_temp_rnti && grant.ndi != cur_ndi_) ||   // NDI不翻转 ≠ 新传
                 (!is_grant_configured_ && !is_temp_rnti) ||   // ← TB丢弃后被reset
                 grant.is_rar;

// enb_ul_scheduler.cpp - handle_ul_crc() (step 2, 丢弃后未清除)
void ul_scheduler::handle_ul_crc(uint16_t rnti, uint32_t pid, bool crc_ok) {
    if (!crc_ok) {
        it->second.pending_retx_pid = pid;   // CRC失败 → 标记重传
    } else {
        it->second.pending_retx_pid = 0xFFFF; // CRC成功 → 清除
    }
    // ← 没有任何分支处理"TB被丢弃"的情况！
}
```

关键盲点：**`handle_ul_crc()` 只区分 CRC OK/FAIL，但 `action.is_discarded`（HARQ层因达到max_retx主动丢弃）是第三种状态，代码完全没考虑。**

#### 排查过程

1. **加日志定位**：在 `new_grant_ul()` 入口打印 `is_new_tx / is_retx / is_grant_configured_`，发现TB丢弃后同一PID反复输出 `New TX` 和 `Adaptive ReTX` 交替出现
2. **画状态转换图**：把 HARQ 进程状态 (INACTIVE/WAITING_FB/RETX_PENDING) 和调度器 `pending_retx_pid` 的组合穷举，发现 "HARQ已RESET + 调度器仍标记重传" 这对组合没有转移表覆盖
3. **最小复现**：将BLER临时调到50%快速触发max_retx丢弃，观察到 `Max ReTX reached` 日志连续刷屏

#### 修复方法

**修复①：HARQ reset 时同步重置 NDI**（[ue_ul_harq_manager.cpp#L33-L41](file:///home/lulu44/Desktop/Lulu44/srsRAN/ul_mac_manager/src/ue_ul_harq_manager.cpp#L33-L41)）

```cpp
void ul_harq_process::reset_unlocked() {
    current_tx_nb_ = 0;
    current_irv_ = 0;
    is_grant_configured_ = false;
    harq_feedback_ = false;
    cur_tbs_ = 0;
    cur_rv_ = -1;
    cur_ndi_ = false;   // ← 新增：原代码遗漏
}
```

**修复②：TB丢弃时主动清除调度器重传标志**（[main.cpp#L267-L274](file:///home/lulu44/Desktop/Lulu44/srsRAN/ul_mac_manager/src/main.cpp#L267-L274)）

```cpp
auto action = ue.handle_ul_grant(grant);
ue.handle_harq_feedback(grant.pid, grant.hi_value);

if (action.is_discarded) {
    // TB已丢弃(达到最大重传), 主动通知调度器"这个PID不需要重传了"
    scheduler.handle_ul_crc(rnti, grant.pid, true);   // crc_ok=true → 清pending_retx_pid
} else {
    scheduler.handle_ul_crc(rnti, grant.pid, grant.hi_value);
}
```

#### 经验总结

> **死锁≠mutex**。状态机驱动的系统（HARQ、RLC、PDCP协议栈本质都是状态机）里，**状态遗漏的边界条件**同样能造成"活锁"——没有线程阻塞，但系统永远在重复同一个错误状态转换。

排查心法：
- 对每个状态转换，穷举"上游状态 × 输入事件"的所有组合，看是否都有正确转移
- 给状态机加**断言**：`assert(!(is_grant_configured_ && action.is_discarded))` 这种"不可能同时成立"的组合，一旦触发就立即暴露
- 单元测试覆盖**边界序列**：连续NACK→max_retx→discard，不只是"单次ACK"的happy path

### E.7 智能指针的线程安全性

面试标准答法分三层：
1. **`shared_ptr` 的引用计数是原子的**：多线程同时拷贝/析构不同的 shared_ptr 副本→安全
2. **但同一个 shared_ptr 对象本身不是**：一个线程 `sp = other` 同时另一个线程读 `sp` → 数据竞争（C++20 提供 `std::atomic<std::shared_ptr>`）
3. **它们指向的对象更不是**：引用计数线程安全 ≠ `*sp` 的成员线程安全，该加锁还得加

`unique_ptr`：无引用计数、零开销，独占语义天然适合"单一所有者"——本身不提供任何线程安全保证，但也不需要（不共享就没有竞争）。

**项目实例**：项目全部用 `unique_ptr`、零 `shared_ptr`——`ul_harq_manager` 独占 16 个进程（`std::unique_ptr<ul_harq_process[]>`），main.cpp 用 `std::vector<std::unique_ptr<ue_context>>` 因为 ue_context 含 mutex 不可拷贝。可以讲："我的设计原则是所有权层次化，能用 unique 就不用 shared——shared_ptr 的原子计数在多核下有缓存行乖乖开销。"

### E.8 条件变量与生产者-消费者

```cpp
std::mutex m; std::condition_variable cv; std::queue<Task> q;
// 生产者
{ std::lock_guard<std::mutex> lk(m); q.push(task); } cv.notify_one();
// 消费者
std::unique_lock<std::mutex> lk(m);
cv.wait(lk, []{ return !q.empty(); });   // ← 谓词形式, 防两大坑
```

两大坑（面试必问）：
- **虚假唤醒 (spurious wakeup)**：`wait` 可能无缘无故返回 → 必须用谓词/while 重查条件，绝不能用 if
- **丢失唤醒 (lost wakeup)**：notify 发生在 wait 之前 → 谓词形式进 wait 前先查一次条件，天然免疫
- 为什么 wait 必须用 `unique_lock`：wait 内部要"解锁→睡→醒来再加锁"，`lock_guard` 不支持中途解锁

**项目关联**：本项目是 TTI 轮询驱动（每 1ms 主动 step）所以不需要 cv；但真实 srsRAN 的层间任务队列（PHY→MAC 投递事件）就是标准生产者-消费者。可主动说："如果把本项目的 UE/eNB 拆成两个线程，中间的 SR/BSR/Grant 传递就应该改成 cv 驱动的消息队列——这是我规划的演进方向。"

### E.9 并发容器：标准库容器的线程安全真相

标准答法：STL 容器只保证——多线程**同时读** const 操作安全；任何写操作必须外部同步。尤其注意：
- `map/set` 的插入不会失效其他元素的迭代器，但 `vector` 扩容会全部失效——并发下更危险
- `operator[]` 对 map 是"没有就插入"的**写操作**，看似读实则写，常见陷阱

工程选择：① 细粒度锁包装（本项目的做法：`lcg_buffer_manager` = `map` 数组 + 一把 mutex，`ul_scheduler::ue_db_` 同理）；② 现成并发容器库（TBB 的 `concurrent_hash_map`、folly 的 `ConcurrentHashMap`）；③ 无锁队列（单生产者单消费者 ring buffer，基站实时路径常用）。面试表达："我选择方案①因为访问模式简单、临界区短；方案②③是吞吐上去后的优化选项。"

### E.10 C++ 内存模型与 memory_order

为什么需要内存模型：编译器和 CPU 都会重排指令；单线程看不出来，多线程会观察到"乱序"。`memory_order` 就是告诉编译器/CPU 哪些重排不允许：

| 序 | 语义 | 典型用途 |
|---|---|---|
| `seq_cst`（默认） | 全局全序，最强最慢 | 不确定就用它（本项目全部默认） |
| `acquire`（读）/ `release`（写） | release 写之前的所有写，对 acquire 读之后都可见 | 标志位发布："数据准备好了再置 flag" |
| `relaxed` | 只保证原子性，不约束顺序 | 纯计数器（无人依赖其顺序） |

经典发布模式：

```cpp
data = 42;                                  // 普通写
ready.store(true, std::memory_order_release);   // 发布
// 另一线程:
if (ready.load(std::memory_order_acquire))      // 获取
    use(data);                              // 保证能看到 42
```

**项目实例与诚实答法**：项目里 `average_retx_.store(...)` 用的是默认 `seq_cst`。面试被问"为什么不用 relaxed"，最佳回答是："这些是低频统计更新，不在热路径，seq_cst 的正确性最容易推理；只有 profile 证明原子操作成为瓶颈时才降级到 relaxed——过早优化内存序是 bug 重灾区"——这比背六种 memory_order 更显工程成熟度。

### E.11 两个项目专属高频追问

**Q：mac_logger 的单例为什么是线程安全的？**（[mac_logger.h](file:///home/lulu44/Desktop/Lulu44/srsRAN/ul_mac_manager/include/ul_mac/mac_logger.h) 第 31 行）

```cpp
static mac_logger& instance() {
    static mac_logger inst;   // Magic Static (C++11 §6.7)
    return inst;
}
```

C++11 起标准保证：**局部 static 变量的初始化是线程安全的**（并发到达时只有一个线程执行构造，其余阻塞等待；编译器用隐藏的 guard 变量 + 双检锁实现）。这称为 Meyers 单例，是 C++11 后写单例的标准答案——面试追问"双检锁单例"时可答："手写 DCLP 在 C++11 前因内存重排是错的（新对象指针可能先于构造完成发布），C++11 后直接用 magic static，编译器帮你做对。"注意区分：单例的**创建**线程安全≠它的**成员函数**线程安全，所以 `log()` 内部还有自己的 `lock_guard`。

**Q：lcg_buffer 里为什么写 `mutable std::mutex mutex_`？**（项目六个类全部如此）

`get_total_buffer()` 这类查询接口逻辑上是只读的，应声明为 `const` 成员函数；但加锁要修改 mutex 的内部状态——不加 `mutable` 就编译不过。`mutable` 的语义是：**该成员不属于对象的逻辑状态**（锁、缓存、计数器都是典型）。进阶表述：C++11 后 `const` 成员函数的实际契约升级为"线程安全可并发调用"（Herb Sutter：const 意味着 bitwise-const **或 internally-synchronized**），`mutable mutex` 正是实现后者的标准手段。

### E.12 并发专题自测清单（合上文档自检）

- [ ] 数据竞争为什么是 UB？用附录 D-⑤ 的 SR 统计 bug 举例
- [ ] lock_guard / unique_lock / scoped_lock 各自什么时候用？
- [ ] 为什么 harq 进程里 atomic 和 mutex 同时存在？举"擕裂组合"的例子
- [ ] 背出死锁四条件，并用 step() 的"锁内决策、锁外回调"和 `_unlocked` 模式各讲一个预防实例
- [ ] shared_ptr 的三层线程安全边界；本项目为什么全用 unique_ptr
- [ ] cv.wait 为什么必须谓词形式 + unique_lock？
- [ ] magic static 为什么线程安全？和手写双检锁的历史恩怨
- [ ] mutable mutex + const 成员函数的语义契约
- [ ] acquire/release 的发布模式；为什么本项目坚持默认 seq_cst

---

*配套文档：项目详细设计、面试题与协议符合性分析见 [README.md](./README.md)*
