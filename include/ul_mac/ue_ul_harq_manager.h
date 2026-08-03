// =============================================================================
// ue_ul_harq_manager.h - 上行HARQ重传管理器 (增强版)
//
// 实现基于3GPP TS 36.321 Section 5.4.2 / TS 38.321 Section 5.4.2 的上行HARQ过程
//
// 【标准HARQ机制】:
//   1. HARQ使用多个并行进程 (LTE: 8个, NR: 最多16个)
//   2. 每个进程维护: NDI(新数据指示), RV(冗余版本), 传输计数
//   3. RV序列: {0, 2, 3, 1} (循环使用, 通过IRV计数器计算)
//   4. 两种重传模式:
//      - 自适应重传: eNB发送新DCI (含NDI), UE根据DCI重传
//      - 非自适应重传: 仅通过PHICH NACK触发, 使用之前的授权参数
//   5. NDI翻转 = 新传输; NDI不变 = 重传
//   6. 达到最大重传次数后丢弃传输块
//
// 【增强功能】:
//   1. 早期终止: 基于BLER统计提前终止重传
//   2. HARQ进程负载均衡: 避免单个进程过度使用
//   3. 重传统计和BLER追踪
//   4. Msg3特殊处理 (随机接入过程的Msg3传输)
//
// 关键参考:
//   - srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h    HARQ实体和进程定义
//   - srsRAN_4G/srsue/src/stack/mac/ul_harq.cc   HARQ完整实现
//   - ocudu/include/ocudu/scheduler/scheduler_configurator.h 中的 max_pusch_harq_retxs
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <memory>
#include <mutex>
#include <array>
#include <atomic>

namespace ul_mac {

/// 单个HARQ进程
///
/// 管理一个HARQ进程的完整状态, 包括新传/重传决策
/// 对应 srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h 中的 ul_harq_process 内部类
class ul_harq_process {
public:
    ul_harq_process() : pid_(0), current_tx_nb_(0), current_irv_(0)
                       , harq_feedback_(false), is_grant_configured_(false)
                       , cur_ndi_(false), cur_tbs_(0), cur_rv_(-1)
                       , tx_tti_(0xFFFFFFFF), rtt_ttis_(8), feedback_pending_(false)
                       , consecutive_nack_(0), early_termination_enabled_(true) {}

    explicit ul_harq_process(uint32_t pid);

    /// 重置进程状态
    void reset();

    /// 设置进程ID (用于初始化)
    void init_pid(uint32_t pid) { pid_ = pid; }

    /// 重置NDI (用于切换/重配)
    void reset_ndi();

    /// 处理新的上行授权
    /// 对应 srsRAN ul_harq.cc 中的 new_grant_ul() 方法
    /// 根据NDI、PHICH和授权信息决定是新传还是重传
    /// @param grant 上行授权
    /// @param harq_cfg HARQ配置
    /// @param is_temp_rnti 是否使用TC-RNTI (Msg3传输)
    /// @return 传输动作信息
    struct tx_action {
        bool     is_new_tx;       ///< 是否为新传输
        bool     is_retx;         ///< 是否为重传
        uint32_t rv;              ///< 冗余版本
        uint32_t tbs;             ///< 传输块大小
        uint32_t tx_nb;           ///< 当前传输次数
        bool     is_discarded;    ///< 是否被丢弃 (达到最大重传)
        bool     is_msg3;         ///< 是否为Msg3传输

        tx_action() : is_new_tx(false), is_retx(false), rv(0), tbs(0)
                    , tx_nb(0), is_discarded(false), is_msg3(false) {}
    };

    tx_action new_grant_ul(const ul_grant& grant, const ul_harq_config& harq_cfg,
                           bool is_temp_rnti);

    /// 获取当前RV (通过IRV计算)
    /// 对应 srsRAN ul_harq.cc 中的 get_rv()
    uint32_t get_rv() const;

    /// 是否已配置授权
    bool has_grant() const { return is_grant_configured_; }

    /// 获取当前NDI
    bool get_ndi() const { return cur_ndi_; }

    /// 获取重传次数
    uint32_t get_nof_retx() const { return current_tx_nb_; }

    /// 获取当前TBS
    uint32_t get_current_tbs() const { return cur_tbs_; }

    /// 获取进程状态
    harq_state get_state() const;

    /// 获取进程ID
    uint32_t get_pid() const { return pid_; }

    /// 检查HARQ反馈是否已到达 (RTT是否已过)
    /// 实际系统中, PHICH反馈从发送到UE收到需要经过若干TTI (LTE约8ms)
    /// @param current_tti 当前TTI
    /// @return true=反馈已到达, 可以处理; false=反馈仍在途中
    bool is_feedback_ready(uint32_t current_tti) const;

    /// 设置RTT时长 (TTI数)
    /// @param rtt RTT时长, LTE标准约为8; 设为0退化为即时反馈
    void set_rtt_ttis(uint32_t rtt) { rtt_ttis_ = rtt; }

    /// 是否有反馈在途中等待 (RTT未到)
    bool is_feedback_pending() const { return feedback_pending_; }

    // ========================================================================
    // 增强功能接口 - 早期终止
    // ========================================================================

    /// 【增强】更新HARQ反馈统计 (用于早期终止判断)
    /// ACK重置连续NACK计数, NACK递增连续NACK计数
    /// @param ack true=ACK, false=NACK
    void update_feedback_stats(bool ack);

    /// 【增强】设置早期终止开关
    /// @param enable true=启用早期终止, false=禁用
    void set_early_termination_enabled(bool enable) {
        std::lock_guard<std::mutex> lock(mutex_);
        early_termination_enabled_ = enable;
    }

    /// 【增强】查询早期终止是否启用
    bool is_early_termination_enabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return early_termination_enabled_;
    }

    // manager 需调用私有的 fill_process_info() 在 process 自身锁内读取状态,
    // 以避免 manager 锁嵌套 process 锁
    friend class ul_harq_manager;

private:
    /// 填充进程信息（线程安全，在自身锁内一次性读取所有状态）
    /// 用于避免 manager 锁嵌套 process 锁, 以及 cur_tbs_ 等非原子成员的数据竞争
    void fill_process_info(harq_process_info& info) const;

    /// 重置进程状态 (不加锁版本, 供已持锁的 new_grant_ul() 内部调用)
    /// 同一把 std::mutex 重复加锁是未定义行为(实际表现为死锁)
    void reset_unlocked();

    /// 生成新传输
    /// 对应 srsRAN ul_harq.cc generate_new_tx()
    tx_action generate_new_tx(const ul_grant& grant, bool is_temp_rnti);

    /// 生成重传
    /// 对应 srsRAN ul_harq.cc generate_retx()
    tx_action generate_retx(const ul_grant& grant);

    /// 生成传输 (公共传输逻辑)
    /// 对应 srsRAN ul_harq.cc generate_tx()
    tx_action generate_tx();

    uint32_t pid_;
    std::atomic<uint32_t> current_tx_nb_;    ///< 当前传输次数 (新传+重传)
    std::atomic<uint32_t> current_irv_;      ///< IRV计数器 (用于RV序列)
    std::atomic<bool>     harq_feedback_;    ///< HARQ反馈 (ACK/NACK)
    std::atomic<bool>     is_grant_configured_;
    bool                  cur_ndi_;          ///< 当前NDI值
    uint32_t              cur_tbs_;          ///< 当前TBS
    int8_t                cur_rv_;           ///< 当前RV (-1表示由IRV计算)
    uint32_t              tx_tti_;           ///< 本次传输的TTI (用于RTT计算, 0xFFFFFFFF=未传输)
    uint32_t              rtt_ttis_;         ///< RTT时长(TTI数), LTE通常为8, 0=即时反馈
    bool                  feedback_pending_; ///< 是否有反馈在途中等待 (RTT未到)

    // 【增强】早期终止相关
    // 基于BLER统计提前终止重传: 当连续收到多次NACK且已重传若干次时,
    // 认为该传输块在当前信道条件下难以成功, 提前丢弃以节省无线资源
    static constexpr uint32_t EARLY_TERM_WINDOW = 10; ///< 统计窗口 (保留用于扩展的BLER窗口统计)
    uint32_t              consecutive_nack_;   ///< 连续NACK计数 (ACK时清零)
    bool                  early_termination_enabled_; ///< 早期终止开关, 默认启用

    mutable std::mutex    mutex_;
};

/// 上行HARQ实体
///
/// 管理所有HARQ进程, 处理上行授权和HARQ反馈
/// 对应 srsRAN_4G/srsue/hdr/stack/mac/ul_harq.h 中的 ul_harq_entity
class ul_harq_manager {
public:
    /// 构造函数
    /// @param rnti UE的C-RNTI
    explicit ul_harq_manager(uint16_t rnti);

    /// 初始化HARQ管理器
    /// @param config HARQ配置
    void init(const ul_harq_config& config);

    /// 处理新的上行授权
    /// 对应 srsRAN ul_harq.cc ul_harq_entity::new_grant_ul()
    /// @param grant 上行授权
    /// @param is_temp_rnti 是否使用TC-RNTI (Msg3)
    /// @return 传输动作
    ul_harq_process::tx_action new_grant_ul(const ul_grant& grant, bool is_temp_rnti);

    /// 处理HARQ反馈 (PHICH/eNB反馈)
    /// @param pid HARQ进程ID
    /// @param feedback 反馈类型 (ACK/NACK)
    void handle_harq_feedback(uint32_t pid, harq_feedback feedback);

    /// 重置所有HARQ进程
    void reset();

    /// 重置所有NDI
    void reset_ndi();

    /// 设置HARQ配置
    void set_config(const ul_harq_config& config);

    /// 获取指定HARQ进程信息
    harq_process_info get_process_info(uint32_t pid) const;

    /// 获取所有HARQ进程信息
    std::array<harq_process_info, MAX_HARQ_PROCESSES> get_all_process_info() const;

    /// 获取平均重传次数
    double get_average_retx() const { return average_retx_.load(); }

    /// 获取总传输包数
    uint64_t get_total_pkts() const { return total_pkts_.load(); }

    // ========================================================================
    // 增强功能接口
    // ========================================================================

    /// 【增强】获取HARQ统计信息
    struct harq_stats {
        uint32_t total_new_tx;       ///< 新传总次数
        uint32_t total_retx;         ///< 重传总次数
        uint32_t total_fail;         ///< 失败总次数 (达到最大重传)
        uint32_t total_ack;          ///< ACK总次数
        uint32_t total_nack;         ///< NACK总次数
        double   avg_retx_per_pkt;   ///< 每包平均重传次数
        double   bler;               ///< 误块率 (NACK / (ACK+NACK))

        harq_stats() : total_new_tx(0), total_retx(0), total_fail(0)
                     , total_ack(0), total_nack(0)
                     , avg_retx_per_pkt(0.0), bler(0.0) {}
    };

    harq_stats get_stats() const { return stats_; }

    /// 【增强】获取空闲HARQ进程ID
    /// 用于负载均衡, 优先选择空闲最久的进程
    uint32_t get_idle_process_id() const;

    /// 【增强】设置所有HARQ进程的早期终止开关
    /// @param enable true=启用早期终止, false=禁用
    void set_early_termination_enabled(bool enable);

private:
    /// 获取指定HARQ进程信息 (不加锁版本)
    /// 调用方必须已持有 manager 的 mutex_
    harq_process_info get_process_info_unlocked(uint32_t pid) const;

    uint16_t rnti_;
    bool initialized_;
    ul_harq_config harq_cfg_;

    // HARQ进程数组 (对应 srsRAN ul_harq.h 中的 std::vector<ul_harq_process> proc)
    std::unique_ptr<ul_harq_process[]> processes_;

    // 统计信息 (对应 srsRAN ul_harq.h 中的 average_retx, nof_pkts)
    std::atomic<float>    average_retx_{0.0};
    std::atomic<uint64_t> total_pkts_{0};

    mutable std::mutex mutex_;
    harq_stats stats_;
};

} // namespace ul_mac
