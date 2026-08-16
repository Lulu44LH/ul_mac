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
//   2. 重传统计和BLER追踪
//   3. Msg3特殊处理 (随机接入过程的Msg3传输)
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
                       , feedback_received_(false)
                       , cur_ndi_(false), cur_tbs_(0), cur_rv_(-1)
                       , consecutive_nack_(0), early_termination_enabled_(false) {}

    explicit ul_harq_process(uint32_t pid);

    /// 单次授权产生的传输动作 (新传/重传/丢弃的统一返回体)
    struct tx_action {
        bool     is_new_tx;       ///< 是否为新传输
        bool     is_retx;         ///< 是否为重传
        uint32_t rv;              ///< 冗余版本
        uint32_t tbs;             ///< 传输块大小
        uint32_t tx_nb;           ///< 当前传输次数
        bool     is_discarded;    ///< 是否被丢弃 (达到最大重传 / 早期终止)
        bool     is_msg3;         ///< 是否为Msg3传输

        tx_action() : is_new_tx(false), is_retx(false), rv(0), tbs(0)
                    , tx_nb(0), is_discarded(false), is_msg3(false) {}
    };

    /// 应用经独立 PHICH 信道到达的 HARQ 反馈 (区别于随 DCI grant 携带的反馈)
    /// 【协议语义】TS 36.321 §5.4.2: ACK -> 进程释放 (等待新 TB);
    /// NACK -> 进程置为待重传 (非自适应重传由 eNB 的重传授权驱动)。
    /// 守卫: 仅当当前 TB 在途且尚未收到反馈时应用, 防止迟到的旧反馈
    /// 误释放/误伤害已发出的新 TB (PHICH 与 PDCCH 到达顺序不保证)。
    /// 内部不再建模 RTT (延时由 timed_channel 统一移交), 但在 NACK 时完成
    /// "重传上限 / 早期终止" 的丢弃判定。
    /// @param ack true=ACK, false=NACK
    /// @return 本次反馈产生的传输动作 (NACK 且达到丢弃条件时 is_discarded=true,
    ///         tbs 为被丢弃 TB 大小, 供 UE 上下文回滚缓冲区)
    tx_action apply_phich_feedback(bool ack);

    /// 重置进程状态
    void reset();

    /// 设置进程ID (用于初始化)
    void init_pid(uint32_t pid) { pid_ = pid; }

    /// 处理新的上行授权
    /// 对应 srsRAN ul_harq.cc 中的 new_grant_ul() 方法
    /// 根据NDI、PHICH和授权信息决定是新传还是重传
    /// @param grant 上行授权
    /// @param harq_cfg HARQ配置 (透传, 供反馈到达时做丢弃判定)
    /// @param is_temp_rnti 是否使用TC-RNTI (Msg3传输)
    /// @return 传输动作信息
    tx_action new_grant_ul(const ul_grant& grant, const ul_harq_config& harq_cfg,
                           bool is_temp_rnti);

    /// 获取当前RV (通过IRV计算)
    /// 对应 srsRAN ul_harq.cc 中的 get_rv()
    uint32_t get_rv() const;

    /// 获取进程状态
    harq_state get_state() const;

    /// 获取进程ID
    uint32_t get_pid() const { return pid_; }

    // ========================================================================
    // 增强功能接口 - 早期终止
    // 【默认关闭】该机制非 3GPP 标准 (标准上限由 RRC maxHARQ-Tx 决定, 无损投递
    // 由 RLC ARQ 兜底; 本项目 RLC 缺位, 提前丢弃即数据丢失, 故默认禁用),
    // 如需演示资源优化效果请显式调用 set_early_termination_enabled(true)。
    // ========================================================================

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
    std::atomic<bool>     harq_feedback_;    ///< 最近一次反馈结果 (true=ACK)
    std::atomic<bool>     is_grant_configured_;
    bool                  feedback_received_;///< 当前 TB 是否已收到反馈 (区分"未收到"与"NACK")
    bool                  cur_ndi_;          ///< 当前NDI值
    uint32_t              cur_tbs_;          ///< 当前TBS
    int8_t                cur_rv_;           ///< 当前RV (-1表示由IRV计算)

    // HARQ 配置快照: 收到 grant 时由 manager 透传, 反馈到达时已就绪 (丢弃判定用)
    uint32_t              max_harq_tx_;       ///< 最大HARQ传输次数 (新传+重传)
    uint32_t              max_harq_msg3_tx_;  ///< Msg3最大传输次数
    bool                  is_temp_rnti_;      ///< 是否使用TC-RNTI (Msg3)

    // 【增强】早期终止相关
    // 基于BLER统计提前终止重传: 当连续收到多次NACK且已重传若干次时,
    // 认为该传输块在当前信道条件下难以成功, 提前丢弃以节省无线资源
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
    /// 管理器内部持有 HARQ 配置快照, 透传给具体进程 (供反馈到达时做丢弃判定)
    /// @param grant 上行授权
    /// @param is_temp_rnti 是否使用TC-RNTI (Msg3)
    /// @return 传输动作
    ul_harq_process::tx_action new_grant_ul(const ul_grant& grant, bool is_temp_rnti);

    /// 处理HARQ反馈 (PHICH/eNB反馈)
    /// @param pid HARQ进程ID
    /// @param feedback 反馈类型 (ACK/NACK)
    /// @return 本次反馈产生的传输动作 (含丢弃判定, 供 UE 回滚缓冲)
    ul_harq_process::tx_action handle_harq_feedback(uint32_t pid, harq_feedback feedback);

    /// 重置所有HARQ进程
    void reset();

    /// 设置HARQ配置
    void set_config(const ul_harq_config& config);

    /// 获取指定HARQ进程信息
    harq_process_info get_process_info(uint32_t pid) const;

    /// 获取所有HARQ进程信息
    std::array<harq_process_info, MAX_HARQ_PROCESSES> get_all_process_info() const;

    /// 获取平均重传次数
    double get_average_retx() const { return average_retx_.load(); }

    // ========================================================================
    // 增强功能接口
    // ========================================================================

    /// 查询是否仍有"在途的上行授权" (即存在仍占用 grant 的 HARQ 进程)
    /// 【线程安全】stats_ 在 mutex_ 下被写线程更新, 读取须持锁拷贝
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

    harq_stats get_stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    /// 查询是否仍有"在途的上行授权" (即存在仍占用 grant 的 HARQ 进程)
    /// 【协议语义】UL grant 的有效性绑定在 HARQ 进程上: 只要某进程处于
    /// WAITING_FB (已发, 等PHICH) 或 RETX_PENDING (收到NACK, 待重传),
    /// 该 grant 仍"在途", UE 视为仍持有可用授权, 不应触发 SR (§5.4.4)。
    /// 仅当所有进程均 INACTIVE (grant 已释放/ACK/丢弃) 时返回 false。
    /// 对应 srsRAN ul_harq_entity 中基于进程状态的 grant 有效性判断。
    bool has_pending_transmission() const;

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

    // 统计信息 (对应 srsRAN ul_harq.h 中的 average_retx)
    std::atomic<float>    average_retx_{0.0};

    mutable std::mutex mutex_;
    harq_stats stats_;
};

} // namespace ul_mac
