// =============================================================================
// ue_bsr_manager.h - 上行缓冲区状态报告(BSR)管理器 (优化版)
//
// 实现基于3GPP TS 36.321 Section 5.4.5 / TS 38.321 Section 5.4.5 的BSR过程
//
// 【标准BSR机制】:
//   1. 三种触发类型:
//      - Regular BSR: 新数据到达(无数据->有数据)或高优先级数据到达
//      - Periodic BSR: periodicBSR-Timer超时
//      - Padding BSR: UL授权中有剩余填充空间
//   2. 三种BSR格式:
//      - Short BSR: 1个LCG, 1字节(2-bit LCG ID + 6-bit缓冲区大小)
//      - Truncated BSR: 多个LCG有数据但空间不足, 只报最高优先级
//      - Long BSR: 所有4个LCG, 3字节(各6-bit)
//   3. BSR触发Regular类型时自动触发SR过程
//   4. 定时器: periodicBSR-Timer, retxBSR-Timer
//
// 【优化功能】:
//   1. 预测性BSR: 基于流量模式预测未来缓冲区需求
//   2. BSR格式优化: 智能选择BSR格式以减少信令开销
//   3. 差分BSR: 仅报告变化的LCG, 减少BSR大小
//   4. 自适应定时器: 根据流量模式动态调整BSR定时器
//
// 关键参考:
//   - srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h    BSR类定义和接口
//   - srsRAN_4G/srsue/src/stack/mac/proc_bsr.cc   BSR完整实现
//   - ocudu/include/ocudu/mac/bsr_config.h         NR BSR定时器配置
//   - ocudu/lib/mac/mac_ul/ul_bsr.h                NR BSR编解码
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/lcg_buffer.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include "ul_mac/ue_sr_manager.h"
#include <mutex>
#include <functional>

namespace ul_mac {

/// BSR配置 (对应3GPP TS 38.331 BSR-Config)
/// 参考: ocudu/include/ocudu/mac/bsr_config.h 中的 bsr_config
struct bsr_config {
    uint32_t periodic_timer;  ///< periodicBSR-Timer (ms, 0=infinity)
    uint32_t retx_timer;      ///< retxBSR-Timer (ms)

    bsr_config()
        : periodic_timer(DEFAULT_PERIODIC_BSR_TIMER)
        , retx_timer(DEFAULT_RETX_BSR_TIMER)
    {}
};

/// BSR生成回调函数类型
/// 当BSR需要发送时调用此回调
using bsr_tx_callback = std::function<void(uint16_t rnti, const bsr_ce& bsr)>;

/// 缓冲区状态报告(BSR)管理器
///
/// 管理BSR的触发、格式选择和定时器, 实现上行缓冲区状态上报
class bsr_manager {
public:
    /// 构造函数
    /// @param rnti UE的C-RNTI
    /// @param buffer_mgr LCG缓冲区管理器引用
    bsr_manager(uint16_t rnti, lcg_buffer_manager& buffer_mgr);

    /// 初始化BSR管理器
    /// @param config BSR配置
    /// @param sr_proc SR管理器指针 (BSR触发Regular时调用SR)
    /// @param tx_cb BSR发送回调
    void init(const bsr_config& config,
              sr_manager* sr_proc,
              bsr_tx_callback tx_cb);

    /// 每TTI执行的BSR过程步骤
    /// 对应 srsRAN proc_bsr.cc 中的 step() 方法
    /// 检查是否需要触发Regular BSR, 处理定时器
    /// @param tti 当前TTI
    void step(uint32_t tti);

    /// 处理上行授权, 检查是否需要发送BSR
    /// 对应 srsRAN proc_bsr.cc 中的 need_to_send_bsr_on_ul_grant()
    /// @param grant_size 授权大小 (bytes)
    /// @param total_data 总待发送数据 (bytes)
    /// @param bsr 输出的BSR控制元素
    /// @return true=需要发送BSR
    bool need_to_send_bsr_on_ul_grant(uint32_t grant_size,
                                       uint32_t total_data,
                                       bsr_ce& bsr);

    /// 生成填充BSR
    /// 对应 srsRAN proc_bsr.cc 中的 generate_padding_bsr()
    /// @param nof_padding_bytes 填充字节数
    /// @param bsr 输出的BSR控制元素
    /// @return true=生成了Padding BSR
    bool generate_padding_bsr(uint32_t nof_padding_bytes, bsr_ce& bsr);

    /// 更新TTI结束后的缓冲区状态
    /// 对应 srsRAN proc_bsr.cc 中的 update_bsr_tti_end()
    void update_bsr_tti_end(const bsr_ce& bsr);

    /// 重置BSR状态
    void reset();

    /// 设置BSR配置
    void set_config(const bsr_config& config);

    /// 获取当前BSR触发类型
    bsr_trigger_type get_trigger_type() const { return triggered_type_; }

    // ========================================================================
    // 优化功能接口
    // ========================================================================

    /// 【优化】预测性BSR: 基于流量模式预测未来缓冲区需求
    /// @return 预测的下一周期总缓冲区大小 (bytes)
    uint32_t predict_buffer_demand() const;

    /// 【优化】BSR统计信息
    struct bsr_stats {
        uint32_t total_bsr_sent;
        uint32_t regular_count;
        uint32_t periodic_count;
        uint32_t padding_count;
        uint32_t short_count;
        uint32_t long_count;
        uint32_t truncated_count;
        double   avg_bsr_accuracy; ///< BSR报告值与实际缓冲区的平均偏差率

        bsr_stats() : total_bsr_sent(0), regular_count(0), periodic_count(0)
                    , padding_count(0), short_count(0), long_count(0)
                    , truncated_count(0), avg_bsr_accuracy(0.0) {}
    };

    bsr_stats get_stats() const { return stats_; }

    /// 【优化】设置差分BSR开关
    /// 启用后, Padding BSR在所有LCG缓冲区索引未变化时将被跳过, 减少信令开销
    /// @param enable true=启用差分BSR, false=禁用
    void set_differential_enabled(bool enable) {
        std::lock_guard<std::mutex> lock(mutex_);
        differential_enabled_ = enable;
    }

    /// 【优化】查询差分BSR是否启用
    bool is_differential_enabled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return differential_enabled_;
    }

private:
    /// 设置BSR触发类型
    /// 对应 srsRAN proc_bsr.cc 中的 set_trigger()
    /// 当触发Regular BSR时自动触发SR过程
    void set_trigger(bsr_trigger_type type);

    /// 生成BSR控制元素
    /// 对应 srsRAN proc_bsr.cc 中的 generate_bsr()
    /// 根据空间大小和LCG数量选择BSR格式
    /// @param bsr 输出的BSR
    /// @param pdu_space 可用空间 (bytes)
    /// @return true=成功生成BSR
    bool generate_bsr(bsr_ce& bsr, uint32_t pdu_space);

    /// 检查是否需要触发Regular BSR
    /// 条件1: 新数据到达 (check_new_data)
    /// 条件2: 高优先级通道有新数据 (check_highest_priority_channel)
    bool check_regular_bsr_trigger();

    /// 处理定时器超时
    /// 对应 srsRAN proc_bsr.cc 中的 timer_expired()
    void handle_timer_expiry(uint32_t tti);

    /// BSR格式选择算法 (优化版)
    /// 根据空间大小、LCG数量和触发类型智能选择BSR格式
    bsr_format select_bsr_format(uint32_t pdu_space, uint32_t nof_lcg_with_data);

    // 基本属性
    uint16_t rnti_;
    lcg_buffer_manager& buffer_mgr_;
    bool initialized_;

    // BSR状态 (对应 srsRAN proc_bsr.h 中的成员变量)
    bsr_trigger_type triggered_type_;
    bsr_config bsr_cfg_;

    // 定时器 (对应 srsRAN proc_bsr.h 中的 timer_periodic, timer_retx)
    uint32_t periodic_timer_counter_;
    uint32_t retx_timer_counter_;
    bool periodic_timer_running_;
    bool retx_timer_running_;

    // 关联组件
    sr_manager* sr_proc_;
    bsr_tx_callback tx_callback_;

    // 统计和优化
    mutable std::mutex mutex_;
    bsr_stats stats_;

    // 【优化】流量历史 (用于预测性BSR)
    std::array<uint32_t, 10> buffer_history_;  ///< 最近10个TTI的缓冲区大小
    uint32_t history_idx_;

    // 【优化】差分BSR相关
    // 仅在Padding BSR场景下生效: 比较当前各LCG的BSR索引与上次报告值,
    // 若全部未变化则跳过本次Padding BSR发送, 减少空口信令开销
    // Regular/Periodic BSR不受影响 (标准要求必须发送)
    std::array<uint8_t, NOF_LCGS> last_reported_bsr_;  ///< 上次报告的各LCG BSR索引
    bool differential_enabled_;                        ///< 差分BSR开关, 默认启用
};

} // namespace ul_mac
