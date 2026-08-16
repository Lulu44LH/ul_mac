// =============================================================================
// enb_ul_scheduler.h - 上行调度器 (eNB侧)
//
// 模拟eNB/gNB侧的上行调度器, 处理来自多个UE的SR、BSR并分配上行资源
//
// 【调度功能】:
//   1. 接收和处理UE的SR (调度请求)
//   2. 接收和处理UE的BSR (缓冲区状态报告)
//   3. 基于调度算法选择UE并分配上行资源
//   4. 生成上行授权 (UL Grant) 并发送给UE
//   5. 管理HARQ反馈 (ACK/NACK)
//
// 【调度算法】:
//   1. 轮询调度 (RR): 公平轮转, 每个UE获得相同机会
//   2. 比例公平 (PF): 基于历史吞吐率加权, 平衡公平性和效率
//      参考 srsRAN_4G/srsenb/hdr/stack/mac/schedulers/sched_time_pf.h
//   3. 增强型比例公平 (EPF): 华为 EPF, PF + QoS 权重 + 信道感知 + 饿死保护
//      (原"基于缓冲区大小的优先级调度"已移除: 仅按缓冲区排序在真实系统中
//       不可行——不感知信道质量, 弱信道UE会被长期饿死, 且缺乏业务区分度)
//
// 关键参考:
//   - srsRAN_4G/srsenb/hdr/stack/mac/sched.h        调度器主类
//   - srsRAN_4G/srsenb/hdr/stack/mac/sched_ue.h      UE级调度
//   - srsRAN_4G/srsenb/hdr/stack/mac/schedulers/      调度算法
//   - ocudu/include/ocudu/scheduler/mac_scheduler.h   NR调度器接口
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <array>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>

namespace ul_mac {

/// UE调度上下文 (eNB侧维护的每个UE的调度信息)
/// 参考 srsRAN_4G/srsenb/hdr/stack/mac/sched_ue.h 中的 sched_ue 类
struct ue_sched_context {
    // 重传时锁定的TB信息 (保证HARQ重传TBS不变, 符合TS 36.321 §5.4.2.1)
    struct harq_tb_info {
        uint32_t tbs   = 0;   ///< 该进程最近一次新传的TBS
        uint8_t  mcs   = 0;   ///< 对应的MCS
        uint32_t n_prb = 0;   ///< 对应的PRB数
        bool     valid = false;  ///< 是否已记录有效TB
    };

    uint16_t rnti;              ///< UE的C-RNTI
    bool     sr_pending;        ///< SR待处理标志
    uint32_t ul_buffer[NOF_LCGS]; ///< 各LCG的上行缓冲区大小 (来自BSR)
    uint32_t total_ul_buffer;   ///< 总上行缓冲区
    int      phr;               ///< 功率余量 (dB)
    int32_t  ul_snr;            ///< 上行SNR (x100, 可为负; 低SNR区间需负值)
    double   dl_avg_rate;       ///< [已废弃] 下行平均速率 (本项目仅上行, 此字段未被引用)
    double   ul_avg_rate;       ///< 上行平均速率 (PF调度用, EPF 复用为 R_avg)
    uint32_t ul_nof_samples;    ///< 上行调度次数 (PF调度用)
    uint32_t last_scheduled_tti; ///< [已废弃] 上次被调度的TTI (未被任何调度逻辑读取)
    std::array<bool, MAX_HARQ_PROCESSES> pending_retx{};  ///< 各进程是否有待重传 (位图, 避免单值覆盖导致进程泄漏)
    bool     ndi[MAX_HARQ_PROCESSES]; ///< 各HARQ进程当前NDI (新传时翻转, 重传时保持)
    uint8_t  cqi;              ///< UE上报的CQI (0-15, 0=未上报, 回退SNR)
    bool     harq_pid_busy[MAX_HARQ_PROCESSES]; ///< 该UE各HARQ进程忙闲 (eNB跟踪, 避免多UE重复分配)
    harq_tb_info harq_tb[MAX_HARQ_PROCESSES];    ///< 各进程最近新传的TB信息 (重传时回放)

    // ---- EPF (增强型比例公平) 相关字段 ----
    qos_profile qos;            ///< 业务 QoS 配置 (差异化调度权重)
    double   inst_rate;         ///< 最近一次调度的瞬时速率 (PRB*tbs_per_rb[cqi]), 即 R_instant
    uint32_t tti_since_sched;   ///< 距上次调度经历的 TTI 数 (饿死保护计数)

    ue_sched_context()
        : rnti(0), sr_pending(false), total_ul_buffer(0)
        , phr(0), ul_snr(200), dl_avg_rate(0.0), ul_avg_rate(0.0)
        , ul_nof_samples(0), last_scheduled_tti(0)
        , cqi(0)
        , qos(default_qos(qos_class::BE)), inst_rate(0.0), tti_since_sched(0)
    {
        memset(ul_buffer, 0, sizeof(ul_buffer));
        memset(ndi, 0, sizeof(ndi));
        memset(harq_pid_busy, 0, sizeof(harq_pid_busy));
        pending_retx.fill(false);
    }

    /// 设置 UE 业务类型 (同时写入默认 QoS 权重)
    void set_qos(qos_class cls) { qos = default_qos(cls); }
};

/// 上行调度器
///
/// 模拟eNB侧调度器, 处理多UE上行资源分配
class ul_scheduler {
public:
    /// 构造函数
    /// @param algorithm 调度算法
    /// @param total_prb 可用PRB总数
    explicit ul_scheduler(sched_algorithm algorithm = sched_algorithm::PROPORTIONAL_FAIR,
                          uint32_t total_prb = 100);

    /// 添加UE到调度器
    void add_ue(uint16_t rnti);

    /// 移除UE
    void remove_ue(uint16_t rnti);

    /// 处理UE的SR指示
    /// 对应 srsRAN sched.h 中的 ul_sr_info()
    /// @param pending_bytes 各 LCG 的待传字节数 (UE 本地已知, 经 srsRAN 式内部接口提前告知 eNB)。
    ///   在本项目中等价于 srsRAN 的 handle_ul_bsr_indication(): SR 触发的同一时刻,
    ///   UE 本地已算出待传量并直接喂给 eNB 调度器, 从而跳过"先发小 Grant 探测 BSR"的空口往返。
    ///   真实协议里这一步要靠 PUSCH 上的 BSR CE 获得 (见 enb_handle_bsr_pdu), 此处为 srsRAN 简化。
    void handle_sr(uint16_t rnti, const std::array<uint32_t, NOF_LCGS>& pending_bytes = {});

    /// 处理UE的BSR
    /// 对应 srsRAN sched.h 中的 ul_bsr()
    void handle_bsr(uint16_t rnti, uint8_t lcg_id, uint32_t bsr_value);

    /// 处理UE的PHR
    /// 对应 srsRAN sched.h 中的 ul_phr()
    void handle_phr(uint16_t rnti, int phr, uint32_t ul_nof_prb);

    /// 处理UE上报的CQI (0-15)
    /// CQI>0时调度器优先用CQI选MCS; CQI=0表示未上报, 回退SNR
    void handle_cqi(uint16_t rnti, uint8_t cqi);

    /// 处理上行CRC结果 (HARQ反馈)
    /// 对应 srsRAN sched.h 中的 ul_crc_info()
    void handle_ul_crc(uint16_t rnti, uint32_t pid, bool crc_ok);

    /// 执行每TTI的上行调度
    /// 对应 srsRAN sched.h 中的 ul_sched()
    struct ul_sched_result {
        uint16_t rnti;
        ul_grant grant;
        bool     is_retx;
    };
    std::vector<ul_sched_result> schedule_ul(uint32_t tti);

    /// 调度耗时统计 (展示实时性意识: 每 TTI schedule_ul 的执行时间)
    struct sched_latency_stats {
        uint64_t count = 0;       ///< 采样次数
        double   min_us = 0.0;    ///< 最小耗时 (微秒)
        double   max_us = 0.0;    ///< 最大耗时 (微秒)
        double   avg_us = 0.0;    ///< 平均耗时 (微秒)
        double   p50_us = 0.0;    ///< P50 耗时 (微秒)
        double   p99_us = 0.0;    ///< P99 耗时 (微秒)
    };

    /// 获取调度耗时统计 (基于历史采样的 P50/P99)
    sched_latency_stats get_sched_latency_stats() const;

    /// 设置调度算法 (加锁: 与 schedule_ul 并发安全)
    void set_algorithm(sched_algorithm algo) {
        std::lock_guard<std::mutex> lock(mutex_);
        algorithm_ = algo;
    }

    /// 配置 EPF 参数 (公平性因子/信道感知因子/QoS 缩放/饿死保护)
    void configure_epf(const epf_params& p) {
        std::lock_guard<std::mutex> lock(mutex_);
        epf_ = p;
    }

    /// 获取当前 EPF 参数
    epf_params get_epf_params() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return epf_;
    }

    /// 设置 PF 公平性系数 (默认 1.0 = 标准 PF; <1 偏效率, >1 偏公平)
    void set_fairness_coeff(double c) {
        std::lock_guard<std::mutex> lock(mutex_);
        fairness_coeff_ = c;
    }

    /// 设置 UE 业务类型 (差异化 QoS 权重), 需在 add_ue 之后调用
    void set_ue_qos(uint16_t rnti, qos_class cls);

    /// 获取指定 UE 的 EPF 调度度量 (调试/日志用)
    /// @return 该 UE 当前 TTI 的 EPF 度量值, 无数据返回 0
    double get_epf_metric(uint16_t rnti, uint32_t tti) const;

    /// 获取当前调度UE数量
    size_t get_nof_ues() const;

    /// 获取UE调度上下文 (返回值拷贝, 调用方无需持有锁, 避免悬垂指针)
    std::optional<ue_sched_context> get_ue_context(uint16_t rnti) const;

private:
    /// 比例公平(PF)调度算法
    /// 参考 srsRAN_4G/srsenb/hdr/stack/mac/schedulers/sched_time_pf.h
    /// 优先调度: retx > 高优先级 > PF度量值最大
    std::vector<ul_sched_result> schedule_pf(uint32_t tti);

    /// 轮询(RR)调度算法
    std::vector<ul_sched_result> schedule_rr(uint32_t tti);

    /// 增强型比例公平(EPF)调度算法 (华为 EPF: PF + QoS 权重 + 信道感知)
    /// 度量: metric = w_qos * (R_instant / R_avg^alpha) * (1 + beta*cqi_norm)
    /// 含饿死保护 (长期未调度 UE 自动升优先级, 并保留最低 PRB 份额)
    std::vector<ul_sched_result> schedule_epf(uint32_t tti);

    /// 计算 EPF 调度度量 (供 schedule_epf 与日志复用)
    double compute_epf_metric(const ue_sched_context& ctx, uint32_t tti) const;

    /// 为UE生成上行授权 (无锁版本, 供内部调度方法使用)
    ul_grant generate_ul_grant_unlocked(uint16_t rnti, uint32_t tti,
                                         uint32_t pid, bool is_retx);

    /// 发出新传授权后扣减UE的缓冲区估计 (无锁版本)
    /// 对应 srsRAN sched_ue: 避免BSR更新前对同一份数据重复授权
    void deduct_ul_buffer_unlocked(ue_sched_context& ctx, uint32_t bytes);

    /// 根据SNR(x100, 可为负)计算MCS
    uint8_t calculate_mcs(int32_t snr_x100) const;

    /// 根据MCS和PRB计算TBS
    uint32_t calculate_tbs(uint8_t mcs, uint32_t n_prb) const;

    /// 根据CQI(0-15)计算MCS (CQI=0返回0, 由调用方回退SNR)
    uint8_t calculate_mcs_from_cqi(uint8_t cqi) const;

    /// 为UE分配一个空闲HARQ进程ID (无锁版本)
    /// @return 进程ID (0..MAX_HARQ_PROCESSES-1), -1表示全部忙
    int32_t alloc_free_pid_unlocked(ue_sched_context& ctx);

    /// 从PRB池首次适配分配n个连续PRB (无锁版本, 每TTI重置)
    /// @return 起始PRB索引, -1表示无足够连续PRB
    int32_t prb_alloc_unlocked(uint32_t n);

    /// 重置PRB池 (每TTI调度开始时调用)
    void prb_reset_unlocked();

    /// 【重构】重传优先调度 (无锁版本, 供 schedule_pf/rr/epf 复用)
    /// 遍历所有 UE 的 pending_retx 位图, 为待重传进程生成 grant 并写入 results。
    /// 含无效 TB 守卫 (无原始 TB 记录的进程直接丢弃释放)。
    /// @param results 输出: 重传调度结果追加到此 vector
    /// @param tti 当前 TTI (日志用)
    /// @param algo_name 算法名称 (日志前缀, 如 "PF"/"RR"/"EPF")
    void schedule_retx_first_unlocked(std::vector<ul_sched_result>& results,
                                       uint32_t tti, const char* algo_name);

    /// 【重构】新传授权后提交 (无锁版本, 供 schedule_pf/rr/epf 复用)
    /// 统一处理: HARQ 进程占用标记、SR 清除、缓冲区扣减、PF 平均速率 EMA 更新。
    /// @param ctx UE 调度上下文 (已持锁)
    /// @param pid 已分配的 HARQ 进程 ID
    /// @param res 调度结果 (grant 已填充)
    void commit_new_grant_unlocked(ue_sched_context& ctx, int32_t pid,
                                    ul_sched_result& res);

    sched_algorithm algorithm_;
    uint32_t total_prb_;

    mutable std::mutex mutex_;
    std::map<uint16_t, ue_sched_context> ue_db_;

    std::vector<uint8_t> prb_busy_; ///< PRB占用表 (1=已分配), 每TTI重置

    double fairness_coeff_; ///< PF公平性系数
    epf_params epf_;        ///< EPF 参数 (公平性因子/信道感知/QoS缩放/饿死保护)
    uint32_t rr_offset_ = 0; ///< RR轮询起点偏移 (每TTI递增, 避免总偏向小RNTI)

    // 调度耗时采样 (微秒), 用于 P50/P99 统计
    mutable std::mutex latency_mutex_;
    std::vector<uint64_t> sched_latency_samples_;
};

} // namespace ul_mac
