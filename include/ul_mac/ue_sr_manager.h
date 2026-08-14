// =============================================================================
// ue_sr_manager.h - 上行调度请求(SR)管理器 (增强版)
//
// 实现基于3GPP TS 36.321 Section 5.4.4 / TS 38.321 Section 5.4.4 的SR过程
//
// 【标准SR机制】:
//   1. 当Regular BSR被触发且UE无上行授权时, BSR触发SR过程
//   2. UE在PUCCH上发送SR信号 (如果PUCCH已配置)
//   3. 若PUCCH未配置, 启动随机接入(RA)过程
//   4. SR有最大传输次数限制(dsr-TransMax), 超限则释放PUCCH/SRS并启动RA
//
// 【增强功能 (仿真增强, 注意协议对偶)】:
//   1. 自适应SR周期: 根据UE流量模式动态调整SR发送频率
//      【协议澄清】3GPP标准里SR周期(SR periodicity)由eNB通过RRC
//      (SchedulingRequestConfig / sr-ConfigIndex)半静态配置, UE无权单方面修改。
//      本项目的"UE自适应调整"是仿真/教学增强, 用于演示流量-时延权衡;
//      真实部署应由eNB根据测量触发RRC重配置来调整SR周期。
//   2. SR优先级管理: 不同LCID可以配置不同的SR资源 (非标准, 仿真增强)
//   3. SR禁止定时器优化: 避免SR频繁发送导致PUCCH拥塞
//   4. SR失败快速恢复: 快速回退到RA过程
//
// 关键参考:
//   - srsRAN_4G/srsue/hdr/stack/mac/proc_sr.h    SR类定义
//   - srsRAN_4G/srsue/src/stack/mac/proc_sr.cc   SR实现逻辑
//   - ocudu/include/ocudu/mac/mac_lc_config.h     NR逻辑通道SR配置(sr_id, lc_sr_mask)
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <mutex>
#include <functional>

namespace ul_mac {

/// SR传输回调函数类型
/// 当SR需要发送时调用此回调通知PHY层
using sr_tx_callback = std::function<void(uint16_t rnti)>;

/// SR失败回调函数类型
/// 当SR达到最大传输次数失败时调用, 触发RA过程或释放PUCCH
using sr_fail_callback = std::function<void(uint16_t rnti)>;

/// 调度请求(SR)管理器
///
/// 实现3GPP标准的SR过程并增加自适应优化功能
/// 状态机: IDLE -> PENDING -> TRANSMITTING -> (IDLE | FAILED)
class sr_manager {
public:
    /// 构造函数
    /// @param rnti UE的C-RNTI
    sr_manager(uint16_t rnti);

    /// 初始化SR配置
    /// @param config SR配置参数
    /// @param tx_cb SR发送回调
    /// @param fail_cb SR失败回调
    void init(const sr_config& config,
              sr_tx_callback tx_cb,
              sr_fail_callback fail_cb);

    /// 每TTI执行的SR过程步骤
    /// 对应 srsRAN proc_sr.cc 中的 step() 方法
    /// @param tti 当前TTI
    void step(uint32_t tti);

    /// 触发SR (由BSR过程调用)
    /// 对应 srsRAN proc_sr.cc 中的 start() 方法
    /// 当Regular BSR触发且无上行授权时调用
    void start();

    /// 重置SR状态
    /// 对应 srsRAN proc_sr.cc 中的 reset() 方法
    void reset();

    /// 设置SR配置
    /// 对应 srsRAN proc_sr.cc 中的 set_config() 方法
    void set_config(const sr_config& config);

    /// 通知SR成功收到上行授权 (收到UL Grant即表示SR成功)
    void notify_ul_grant_received();

    /// 获取当前SR状态
    sr_state get_state() const { return state_; }

    /// 获取SR发送计数
    uint32_t get_sr_counter() const { return sr_counter_; }

    // ========================================================================
    // 增强功能接口
    // ========================================================================

    /// 【增强】自适应SR周期调整
    /// 根据UE流量模式动态调整SR周期, 减少SR信令开销
    /// @param traffic_rate 当前流量速率 (bytes/TTI)
    void adjust_sr_period(double traffic_rate);

    /// 【增强】获取SR统计信息
    struct sr_stats {
        uint32_t total_sr_sent;       ///< 总SR发送次数
        uint32_t total_sr_success;    ///< 总SR成功次数
        uint32_t total_sr_fail;       ///< 总SR失败次数
        uint32_t current_sr_period;   ///< 当前SR周期 (ms)

        sr_stats() : total_sr_sent(0), total_sr_success(0)
                   , total_sr_fail(0), current_sr_period(0) {}
    };

    sr_stats get_stats() const { return stats_; }

private:
    /// 检查是否可以发送SR (SR间隔检查)
    /// 对应 srsRAN proc_sr.cc 中的 need_tx() 方法
    /// 确保两次SR发送之间的间隔满足SR周期要求
    bool can_send_sr(uint32_t tti) const;

    // 基本属性
    uint16_t rnti_;
    bool initialized_;

    // SR状态 (对应 srsRAN proc_sr.h 中的成员变量)
    sr_state state_;
    int sr_counter_;              ///< SR发送计数器
    sr_config sr_cfg_;            ///< SR配置
    uint32_t last_sr_tx_tti_;     ///< 上次发送SR的TTI
    uint32_t sr_prohibit_counter_; ///< SR禁止定时器计数

    // 回调函数
    sr_tx_callback tx_callback_;
    sr_fail_callback fail_callback_;

    // 统计信息
    mutable std::mutex mutex_;
    sr_stats stats_;

    // 【增强】自适应SR相关
    double avg_traffic_rate_;     ///< 平均流量速率
    uint32_t adaptive_sr_period_; ///< 自适应SR周期
};

} // namespace ul_mac
