// =============================================================================
// common_types.h - MAC层上行管理通用类型定义
//
// 基于3GPP TS 36.321 (LTE MAC) 和 TS 38.321 (NR MAC) 协议规范
// 参考srsRAN项目中的类型定义进行定制化设计
//
// 关键参考:
//   - srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h  (BSR格式定义)
//   - srsRAN_4G/srsue/hdr/stack/mac/proc_sr.h   (SR配置定义)
//   - srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h   (HARQ进程定义)
//   - ocudu/include/ocudu/mac/bsr_config.h       (NR BSR定时器配置)
//   - ocudu/include/ocudu/mac/mac_lc_config.h    (NR逻辑通道配置)
// =============================================================================

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace ul_mac {

// ============================================================================
// 常量定义 - 对应3GPP协议中的固定参数
// ============================================================================

// 【协议说明】
// LTE (TS 36.321 §5.4.2.1) 上行同步 HARQ 固定为 8 个进程; NR (TS 38.321) 上行为 16 个。
// 本项目为 LTE 4G 上行仿真, 严格取 8 (流水线填满 8ms HARQ RTT)。
constexpr uint32_t MAX_HARQ_PROCESSES = 8;

/// 逻辑通道组(LCG)数量 - 3GPP规定最多4个LCG
constexpr uint32_t NOF_LCGS = 4;

/// 最大逻辑通道ID (LCID)
constexpr uint32_t MAX_LCID = 32;

/// 最大SR传输次数 (dsr-TransMax)
constexpr uint32_t MAX_SR_TRANSMISSIONS = 64;

/// 最大HARQ重传次数
constexpr uint32_t MAX_HARQ_RETX = 4;

/// 子帧(subframe/TTI)时长 (ms), LTE为1ms
constexpr uint32_t TTI_DURATION_MS = 1;

/// 缓冲区大小索引表条目数 (LTE BSR缓冲区大小字段为6-bit, 共64个级别)
constexpr uint32_t BSR_BUFFER_SIZE_LEVELS = 64;

/// 默认SR禁止定时器 (ms)
constexpr uint32_t DEFAULT_SR_PROHIBIT_TIMER = 0;

/// 默认周期BSR定时器 (ms)
constexpr uint32_t DEFAULT_PERIODIC_BSR_TIMER = 80;

/// 默认重传BSR定时器 (ms)
constexpr uint32_t DEFAULT_RETX_BSR_TIMER = 320;

// ============================================================================
// BSR (Buffer Status Report) 相关类型
// ============================================================================

/// BSR格式 - 对应3GPP TS 36.321 Section 5.4.5 / TS 38.321 Section 5.4.5
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h 中的 bsr_format_t
enum class bsr_format {
    SHORT_BSR,       ///< 短BSR: 报告单个LCG, 1字节(2-bit LCG ID + 6-bit缓冲区大小)
    TRUNCATED_BSR,   ///< 截断BSR: 当空间不足以发送长BSR时, 只报告最高优先级LCG
    LONG_BSR         ///< 长BSR: 报告所有4个LCG, 3字节(4个LCG各6-bit缓冲区大小, 共24bit)
};

/// BSR触发类型 - 对应3GPP TS 36.321 Section 5.4.4
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h 中的 bsr_trigger_type_t
enum class bsr_trigger_type {
    NONE,     ///< 无BSR触发
    REGULAR,  ///< 常规BSR: 新数据到达或高优先级数据到达时触发, 会触发SR
    PERIODIC, ///< 周期BSR: periodicBSR-Timer超时触发
    PADDING   ///< 填充BSR: UL授权中有剩余空间时触发
};

/// BSR报告结构 - 单个LCG的缓冲区状态
/// 参考: ocudu/lib/mac/mac_ul/ul_bsr.h 中的 lcg_bsr_report
struct bsr_report {
    uint8_t lcg_id;       ///< 逻辑通道组ID (0-3)
    uint8_t buffer_size;  ///< 缓冲区大小索引 (6-bit, 0-63)

    bsr_report() : lcg_id(0), buffer_size(0) {}
    bsr_report(uint8_t id, uint8_t sz) : lcg_id(id), buffer_size(sz) {}
};

/// 完整BSR MAC控制元素
struct bsr_ce {
    bsr_format format;                          ///< BSR格式
    std::vector<bsr_report> reports;            ///< 各LCG的缓冲区状态报告

    bsr_ce() : format(bsr_format::SHORT_BSR) {}
};

// ============================================================================
// SR (Scheduling Request) 相关类型
// ============================================================================

/// SR配置 - 对应3GPP TS 36.331 中的 SchedulingRequestConfig
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/proc_sr.h 中的 sr_cfg_t
struct sr_config {
    bool     enabled;             ///< SR是否使能 (PUCCH是否配置)
    uint32_t dsr_transmax;        ///< 最大SR传输次数 (n4=4, n8=8, n16=16, n32=32, n64=64)
    uint32_t sr_period;           ///< SR周期 (ms), 每隔多少TTI可以发送一次SR
    uint32_t sr_prohibit_timer;   ///< SR禁止定时器 (ms), 防止SR频繁发送

    sr_config()
        : enabled(true)
        , dsr_transmax(8)
        , sr_period(10)
        , sr_prohibit_timer(0)
    {}
};

/// SR状态
enum class sr_state {
    IDLE,           ///< 空闲状态, 无SR待发送
    PENDING,        ///< SR待发送 (BSR触发)
    TRANSMITTING,   ///< SR正在发送
    FAILED          ///< SR失败 (达到最大传输次数)
};

// ============================================================================
// HARQ (Hybrid ARQ) 相关类型
// ============================================================================

/// HARQ进程状态
enum class harq_state {
    INACTIVE,       ///< 未激活 (空闲)
    WAITING_FB,     ///< 等待反馈 (已发送, 等待ACK/NACK)
    RETX_PENDING    ///< 重传待处理 (收到NACK)
};

/// HARQ反馈
enum class harq_feedback {
    NONE,   ///< 无反馈
    ACK,    ///< 确认 (传输成功)
    NACK    ///< 否认 (需要重传)
};

/// 上行HARQ配置
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h 中的 ul_harq_cfg_t
struct ul_harq_config {
    uint32_t max_harq_tx;       ///< 最大HARQ传输次数 (新传+重传)
    uint32_t max_harq_msg3_tx;  ///< Msg3最大传输次数 (随机接入过程)
    uint32_t harq_rtt_ttis;     ///< HARQ RTT时长(TTI), LTE标准约为8, 设为0退化为即时反馈

    ul_harq_config() : max_harq_tx(4), max_harq_msg3_tx(4), harq_rtt_ttis(8) {}
};

/// 上行授权 (UL Grant) - eNB调度器分配给UE的上行资源
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h 中的 mac_grant_ul_t
struct ul_grant {
    uint16_t rnti;          ///< 无线网络临时标识
    uint32_t pid;           ///< HARQ进程ID
    uint32_t tbs;           ///< 传输块大小 (bytes)
    uint32_t n_prb;         ///< 分配的PRB数量
    uint32_t prb_start;     ///< eNB分配的起始PRB索引 (eNB侧填充, UE侧忽略)
    uint8_t  mcs;           ///< 调制编码方案索引
    int8_t   rv;            ///< 冗余版本
                               // 【协议说明 / 简化实现】
                               // 真实协议 (TS 36.321 §5.4.2.1): 上行 RV 序列 {0,2,3,1}。
                               //  - 非自适应重传: 重传 RV 固定为 0 (UE 自行依据"已传次数"决定,
                               //    不依赖 eNB 在 DCI 中重传 RV)。
                               //  - 自适应重传: eNB 可在 DCI0 中显式指定 RV。
                               // 本项目简化: rv=-1 表示"由 UE HARQ 进程按内部 IRV 序列自行推进"
                               // (模拟非自适应); rv>=0 表示 eNB 显式指定 (模拟自适应)。
    bool     ndi;           ///< 新数据指示 (true=新传, false=重传)
                               // 【协议说明】
                               // NDI 是 UE 与 eNB 之间的 HARQ "契约": UE 翻转 NDI 表示新 TB,
                               // eNB 检测 NDI 翻转判定新 TB。重传时 NDI 保持不变。
    bool     ndi_present;   ///< DCI中是否包含NDI (区分自适应/非自适应重传)
                               // 【协议说明】
                               // 真实上行 DCI0 始终携带 NDI (即便重传也携带以区分新旧 TB)。
                               // 本项目用 ndi_present=false 表示"非自适应重传 (无显式 NDI)",
                               // 此时 eNB 退化为依据进程状态 (INACTIVE->新TB) 判定。
    bool     is_rar;        ///< 是否为RAR(Random Access Response)中的授权
    bool     phich_available;  ///< PHICH是否可用 (上行HARQ反馈信道)
    bool     hi_value;      ///< PHICH反馈值 (true=ACK, false=NACK)
    uint32_t tti_tx;        ///< 发送TTI

    ul_grant()
        : rnti(0), pid(0), tbs(0), n_prb(0), prb_start(0), mcs(0), rv(-1)
        , ndi(false), ndi_present(false), is_rar(false)
        , phich_available(false), hi_value(false), tti_tx(0)
    {}
};

/// HARQ进程信息 (用于统计和监控)
struct harq_process_info {
    uint32_t    pid;                ///< 进程ID
    harq_state  state;              ///< 当前状态
    uint32_t    current_tx_nb;      ///< 当前传输次数 (新传+重传)
    uint32_t    current_irv;        ///< 当前IRV计数器 (用于RV序列计算)
    bool        is_grant_configured;///< 是否已配置授权
    uint32_t    last_tbs;           ///< 上次传输的TBS
    harq_feedback last_feedback;    ///< 上次收到的反馈

    harq_process_info()
        : pid(0), state(harq_state::INACTIVE)
        , current_tx_nb(0), current_irv(0)
        , is_grant_configured(false), last_tbs(0)
        , last_feedback(harq_feedback::NONE)
    {}
};

// ============================================================================
// 逻辑通道 (Logical Channel) 相关类型
// ============================================================================

/// 逻辑通道配置
/// 参考: ocudu/include/ocudu/mac/mac_lc_config.h 中的 mac_lc_config
/// 参考: 3GPP TS 38.331 中的 LogicalChannelConfig
struct lc_config {
    uint32_t lcid;       ///< 逻辑通道ID
    uint32_t lcg_id;     ///< 所属逻辑通道组ID (0-3, 0xFF=不属于任何LCG)
    uint32_t priority;   ///< 优先级 (1最高, 16最低, 对应3GPP priority)
    uint32_t pbr;        ///< 优先级比特率 (Prioritized Bit Rate, bytes/ms)
    uint32_t bsd;        ///< 桶大小持续时间 (Bucket Size Duration, ms)

    lc_config()
        : lcid(0), lcg_id(0xFF), priority(8)
        , pbr(0), bsd(100)
    {}
};

/// 逻辑通道缓冲区状态
/// 参考: srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h 中的 lcid_t
struct lc_buffer_state {
    uint32_t priority;     ///< 通道优先级
    uint32_t old_buffer;   ///< 之前已上报的缓冲区大小 (bytes)
    uint32_t new_buffer;   ///< 最新缓冲区大小 (bytes, 从RLC获取)

    // 令牌桶相关字段 - 对应3GPP TS 36.321 Section 5.4.3.1 中的LCP令牌桶机制
    // PBR(Prioritized Bit Rate) + BSD(Bucket Size Duration) 用于限制每个逻辑
    // 信道在第一阶段可获取的资源, 防止低优先级信道饿死高优先级信道
    uint32_t pbr;                  ///< 优先级比特率 (bytes/ms), 0表示无PBR限制
    uint32_t bsd;                  ///< 桶大小持续时间 (ms)
    double   token_bucket_size;    ///< 桶容量 = pbr * bsd (bytes), pbr=0时为无穷大
    double   token_count;          ///< 当前令牌数 (bytes), 随时间补充, 取数据时扣减

    lc_buffer_state()
        : priority(8), old_buffer(0), new_buffer(0)
        , pbr(0), bsd(100)
        , token_bucket_size(0.0), token_count(0.0)
    {}
};

// ============================================================================
// UE (User Equipment) 相关类型
// ============================================================================

/// UE标识
struct ue_identity {
    uint16_t rnti;        ///< C-RNTI (Cell Radio Network Temporary Identifier)
    uint16_t temp_rnti;   ///< TC-RNTI (Temporary C-RNTI, 随机接入过程中使用)

    ue_identity() : rnti(0), temp_rnti(0) {}
};

/// 调度算法类型
enum class sched_algorithm {
    ROUND_ROBIN,    ///< 轮询调度 (RR)
    PROPORTIONAL_FAIR, ///< 比例公平调度 (PF) - 参考 srsRAN_4G sched_time_pf.h
    PRIORITY_BASED, ///< 优先级调度
    EPF             ///< 增强型比例公平调度 (华为 EPF: PF + QoS 权重 + 信道感知)
};

/// UE度量信息
struct ue_metrics {
    uint16_t rnti;
    uint32_t sr_tx_count;        ///< SR发送次数
    uint32_t sr_success_count;   ///< SR成功次数
    uint32_t bsr_tx_count;       ///< BSR发送次数
    uint32_t harq_tx_count;      ///< HARQ新传次数
    uint32_t harq_retx_count;    ///< HARQ重传次数
    uint32_t harq_fail_count;    ///< HARQ失败次数
    uint32_t total_ul_bytes;     ///< 上行总传输字节数
    double   avg_harq_retx;      ///< 平均HARQ重传次数
    double   ul_throughput_kbps; ///< 上行吞吐率 (kbps)
    double   avg_latency;        ///< 平均延迟(TTI)

    ue_metrics()
        : rnti(0), sr_tx_count(0), sr_success_count(0)
        , bsr_tx_count(0), harq_tx_count(0), harq_retx_count(0)
        , harq_fail_count(0), total_ul_bytes(0)
        , avg_harq_retx(0.0), ul_throughput_kbps(0.0)
        , avg_latency(0.0)
    {}
};

// ============================================================================
// 工具函数
// ============================================================================

/// 将BSR格式转换为字符串
inline std::string bsr_format_to_string(bsr_format fmt) {
    switch (fmt) {
        case bsr_format::SHORT_BSR:     return "Short BSR";
        case bsr_format::TRUNCATED_BSR: return "Truncated BSR";
        case bsr_format::LONG_BSR:      return "Long BSR";
        default: return "Unknown";
    }
}

/// 将BSR触发类型转换为字符串
inline std::string bsr_trigger_to_string(bsr_trigger_type type) {
    switch (type) {
        case bsr_trigger_type::NONE:     return "None";
        case bsr_trigger_type::REGULAR:  return "Regular";
        case bsr_trigger_type::PERIODIC: return "Periodic";
        case bsr_trigger_type::PADDING:  return "Padding";
        default: return "Unknown";
    }
}

/// 将HARQ状态转换为字符串
inline std::string harq_state_to_string(harq_state state) {
    switch (state) {
        case harq_state::INACTIVE:      return "Inactive";
        case harq_state::WAITING_FB:    return "Waiting Feedback";
        case harq_state::RETX_PENDING:  return "Retx Pending";
        default: return "Unknown";
    }
}

/// 将SR状态转换为字符串
inline std::string sr_state_to_string(sr_state state) {
    switch (state) {
        case sr_state::IDLE:         return "Idle";
        case sr_state::PENDING:      return "Pending";
        case sr_state::TRANSMITTING: return "Transmitting";
        case sr_state::FAILED:       return "Failed";
        default: return "Unknown";
    }
}

/// 6-bit BSR缓冲区大小索引转字节数
/// 对应3GPP TS 36.321 Table 6.1.3.1-1 (简化版)
/// 参考: ocudu/lib/mac/mac_ul/ul_bsr.h 中的 buff_size_field_to_bytes
inline uint32_t bsr_index_to_bytes(uint8_t index) {
    // 【协议说明 / 简化实现】
    // 真实 3GPP TS 36.321 Table 6.1.3.1-1 的 6-bit 缓冲区大小字段采用非均匀(近似对数)尺度:
    //   索引 0~10 为线性(0,10,12,14,...30), 索引 10 之后为指数增长直到 ~150000 字节。
    // 本项目用一组单调递增的近似表替代, 仅保证"索引越大缓冲区越大"的单调语义,
    // 不保证与标准表逐档一致。用于教学演示 BSR 编解码流程足够, 不可用于互操作。
    static const uint32_t size_table[BSR_BUFFER_SIZE_LEVELS] = {
        0, 10, 14, 18, 22, 26, 30, 34,          // 0-7: 极小缓冲区
        40, 46, 52, 58, 64, 70, 76, 82,         // 8-15: 小缓冲区
        90, 100, 112, 126, 142, 160, 180, 202,  // 16-23: 中小缓冲区
        228, 256, 288, 324, 364, 410, 462, 520, // 24-31: 中等缓冲区
        586, 660, 744, 838, 944, 1064, 1200, 1354, // 32-39: 中大缓冲区
        1528, 1724, 1946, 2196, 2478, 2798, 3158, 3564, // 40-47: 大缓冲区
        4022, 4538, 5120, 5776, 6518, 7356, 8304, 9374, // 48-55: 很大缓冲区
        10582, 11948, 13492, 15238, 17212, 19440, 21956, 25000 // 56-63: 超大缓冲区
    };
    if (index >= BSR_BUFFER_SIZE_LEVELS) return 25000;
    return size_table[index];
}

/// 字节数转6-bit BSR缓冲区大小索引
/// 参考: ocudu/lib/mac/mac_ul/ul_bsr.h (反向映射)
///
/// TS 36.321 §6.1.3.1: BSR 的 6-bit 缓冲区大小字段是一个索引 i, 对应区间
/// [table[i], table[i+1]). UE 应上报满足 "table[i] <= 真实缓冲区" 的最大索引 i,
/// 即"向下取整". eNB 收到索引 i 后按 **下界** 解读: 认为缓冲区 >= table[i].
/// 注意: 索引表达的是下界而非精确值, 标准语义下缓冲区必须 **>=** 表格值(而非 <=).
inline uint8_t bytes_to_bsr_index(uint32_t bytes) {
    if (bytes == 0) return 0;
    // 向下取整: 返回满足 table[i] <= bytes 的最大 i (缓冲区 >= 表格值).
    for (uint32_t i = BSR_BUFFER_SIZE_LEVELS - 1; i > 0; --i) {
        if (bsr_index_to_bytes(static_cast<uint8_t>(i)) <= bytes) {
            return static_cast<uint8_t>(i);
        }
    }
    return 0;
}

/// 构建 Long BSR 负载: 报告所有 buffer>0 的 LCG (每个用 6-bit 索引)
/// 该逻辑被 ue_bsr_manager::generate_bsr (Long BSR 分支) 与 main.cpp 的
/// 演示编码函数共同复用, 提取为避免重复实现。
/// @param lcg_sizes 各 LCG 的字节数 (长度 = NOF_LCGS)
/// @param bsr       输出的 BSR CE (format 置为 LONG_BSR)
inline void build_long_bsr(const uint32_t* lcg_sizes, bsr_ce& bsr) {
    bsr.format = bsr_format::LONG_BSR;
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        if (lcg_sizes[i] > 0) {
            bsr.reports.push_back(
                bsr_report(static_cast<uint8_t>(i),
                           bytes_to_bsr_index(lcg_sizes[i])));
        }
    }
}

/// RV序列计算 - 对应3GPP TS 36.321 Section 5.4.2.1
/// RV序列: {0, 2, 3, 1}, 循环使用
/// 参考: srsRAN_4G/srsue/src/stack/mac/ul_harq.cc 中的 rv_of_irv
/// @param irv IRV计数器 (每次传输后递增并取模4)
/// @return 冗余版本 (0, 2, 3, 1)
inline uint32_t rv_of_irv(uint32_t irv) {
    static const uint32_t rv_table[4] = {0, 2, 3, 1};
    return rv_table[irv % 4];
}

/// IRV反向映射 (RV -> IRV)
/// 参考: srsRAN_4G/srsue/src/stack/mac/ul_harq.cc 中的 irv_of_rv
inline uint32_t irv_of_rv(uint32_t rv) {
    static const uint32_t irv_table[4] = {0, 3, 1, 2};
    return irv_table[rv % 4];
}

/// 格式化TTI为可读字符串
inline std::string format_tti(uint32_t tti) {
    std::ostringstream oss;
    oss << "TTI[" << std::setw(5) << std::setfill('0') << tti << "]";
    return oss.str();
}

// ============================================================================
// HARQ / 调度共用物理层参数表 (避免多文件重复定义)
// 接收端解码阈值 = SNR_MCS_THRESHOLD[mcs] + DECODE_MARGIN_X100
// ============================================================================

/// MCS -> 最小可解码 SNR 阈值表 (x100 dB), 索引 0..28 对应 MCS 0..28
/// 接收端解码阈值 = 选择阈值 + DECODE_MARGIN, 体现 MCS 边缘处新传易失败
inline constexpr int32_t SNR_MCS_THRESHOLD[29] = {
    -1000, -600, -400, -200, -100,    0,  100,  200,  300,  400,
      500,  600,  700,  800,  900, 1000, 1100, 1200, 1300, 1400,
     1500, 1600, 1700, 1800, 1840, 1880, 1920, 1960, 2000
};

/// 每次软合并(重传)的 IR 增益: 2 dB (等效软缓冲合并带来的等效 SNR 提升)
inline constexpr int32_t IR_GAIN_X100 = 200;

/// 解码阈值相对选择阈值的余量: 1 dB
inline constexpr int32_t DECODE_MARGIN_X100 = 100;

// ============================================================================
// QoS / 业务类型 (用于 EPF 增强型比例公平调度)
// ============================================================================

// 【协议说明】 LTE 通过 QCI (QoS Class Identifier, TS 23.203) 区分业务, 不同
//   业务对速率/时延/丢包率要求不同。此处简化为 3 类典型业务:
//   - VOIP  : 语音, 极小包、严格时延、GBR, 高调度权重
//   - VIDEO : 视频流, 较大包、中等时延, 较高权重
//   - BE    : 尽力而为 (网页/下载), 时延不敏感, 基础权重
// 【简化实现】 真实网络 QCI 由核心网下发, MAC 仅执行; 本项目在 UE 创建时静态
//   指定业务类型, 用于演示差异化调度权重, 不做动态 QCI 重映射。
enum class qos_class : uint8_t {
    VOIP = 0,   ///< 语音 (GBR, 高优先级)
    VIDEO = 1,  ///< 视频流
    BE = 2,     ///< 尽力而为
};

/// QoS 业务配置: 调度权重与 GBR 保障
struct qos_profile {
    qos_class cls = qos_class::BE;
    float     weight = 1.0f;  ///< QoS 调度权重 (>=1 表示高于尽力而为)
    bool      is_gbr = false; ///< 是否为保证比特速率业务 (GBR)
    uint32_t  gbr_prb = 0;    ///< GBR 业务期望的最小 PRB 保障 (饿死保护备用)

    qos_profile() = default;
    qos_profile(qos_class c, float w, bool gbr, uint32_t g)
        : cls(c), weight(w), is_gbr(gbr), gbr_prb(g) {}
};

/// 不同业务类型的默认 QoS 配置 (按 36.213/23.203 典型取值简化)
inline const qos_profile& default_qos(qos_class c) {
    static const qos_profile VOIP_Q{qos_class::VOIP,  3.0f, true,  2};
    static const qos_profile VIDEO_Q{qos_class::VIDEO, 2.0f, false, 1};
    static const qos_profile BE_Q{qos_class::BE,    1.0f, false, 0};
    switch (c) {
        case qos_class::VOIP:  return VOIP_Q;
        case qos_class::VIDEO: return VIDEO_Q;
        default:               return BE_Q;
    }
}

// ============================================================================
// EPF (Enhanced Proportional Fair) 参数
// ============================================================================
// 【算法说明】 华为 EPF 在经典比例公平 (PF) 基础上, 引入 QoS 权重与信道感知
//   增强项。本项目实现的调度度量公式:
//
//     metric = w_qos * (R_instant / R_avg^alpha) * (1 + beta * cqi_norm)
//
//   其中:
//     - R_instant : 当前 TTI 按 CQI 可支持的瞬时速率 (PRB * tbs_per_rb[cqi])
//     - R_avg     : 长期平均吞吐 (指数滑动平均, 见 ue_sched_context::avg_rate)
//     - alpha     : 公平性因子. 越大越偏向长期公平 (抑制长期高速用户);
//                   越小越偏向系统吞吐量. 经典 PF 取 1.0.
//     - beta      : 信道感知因子. beta>0 时信道好的用户额外加权 (提升频谱效率).
//     - w_qos     : 业务 QoS 权重 (VoIP/视频 > 尽力而为).
//     - cqi_norm  : 归一化信道质量 CQI/CQI_MAX, 范围 [0,1].
//
// 【饿死保护】 当 R_avg 趋近 0 (长期未被调度) 或距上次调度 TTI 过大时,
//   R_avg^alpha 项趋近 0 -> 瞬时项被放大, 自动获得高优先级; 另有 min_prb_ratio
//   强制为"很久未调度"的 UE 保留最低 PRB 份额, 避免差信道用户长期饿死。
struct epf_params {
    float    alpha = 1.0f;                 ///< 公平性因子 (PF 指数)
    float    beta = 0.5f;                  ///< 信道感知增益因子
    float    gamma = 1.0f;                 ///< QoS 权重全局缩放 (平衡吞吐与业务优先级)
    float    min_prb_ratio = 0.10f;        ///< 饿死保护: 长期未调度 UE 保底 PRB 比例
    uint32_t starve_tti = 200;             ///< 距上次调度超过该 TTI 数视为"濒临饿死"
    static constexpr uint32_t cqi_max = 15;///< CQI 取值范围上限 (36.213)

    epf_params() = default;
};

} // namespace ul_mac
