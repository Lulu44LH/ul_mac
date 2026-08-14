// =============================================================================
// ue_sr_manager.cpp - 上行调度请求(SR)管理器实现
//
// 实现基于3GPP TS 36.321 Section 5.4.4 的SR过程
// 参考 srsRAN_4G/srsue/src/stack/mac/proc_sr.cc
// =============================================================================

#include "ul_mac/ue_sr_manager.h"
#include <cmath>
#include <algorithm>

namespace ul_mac {

sr_manager::sr_manager(uint16_t rnti)
    : rnti_(rnti)
    , initialized_(false)
    , state_(sr_state::IDLE)
    , sr_counter_(0)
    , last_sr_tx_tti_(0xFFFFFFFF)  // 初始化为极大值, 表示尚未发送过SR
    , sr_prohibit_counter_(0)  // 【说明】sr-ProhibitTimer 标准存在, 但本项目未实现其递减/判定逻辑 (字段空置)
    , tx_callback_(nullptr)
    , fail_callback_(nullptr)
    , avg_traffic_rate_(0.0)
    , adaptive_sr_period_(0)
{
    stats_.current_sr_period = 0;
}

void sr_manager::init(const sr_config& config,
                       sr_tx_callback tx_cb,
                       sr_fail_callback fail_cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    sr_cfg_ = config;
    tx_callback_ = tx_cb;
    fail_callback_ = fail_cb;
    initialized_ = true;
    sr_counter_ = 0;
    state_ = sr_state::IDLE;
    adaptive_sr_period_ = config.sr_period;
    stats_.current_sr_period = config.sr_period;

    LOG_INFO("SR", rnti_, 0,
        "SR initialized: enabled=" + std::string(sr_cfg_.enabled ? "true" : "false") +
        ", dsr_transmax=" + std::to_string(sr_cfg_.dsr_transmax) +
        ", sr_period=" + std::to_string(sr_cfg_.sr_period) + "ms");
}

void sr_manager::set_config(const sr_config& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 参数校验 (对应 srsRAN proc_sr.cc set_config 中的检查)
    if (config.enabled && config.dsr_transmax == 0) {
        LOG_ERROR("SR", rnti_, 0, "Invalid dsr_transmax=0, disabling SR");
        sr_cfg_.enabled = false;
        return;
    }

    sr_cfg_ = config;
    adaptive_sr_period_ = config.sr_period;
    stats_.current_sr_period = config.sr_period;

    if (config.enabled) {
        LOG_INFO("SR", rnti_, 0,
            "SR configured: dsr_transmax=" + std::to_string(config.dsr_transmax));
    }
}

void sr_manager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = sr_state::IDLE;
    sr_counter_ = 0;
    sr_prohibit_counter_ = 0;
    LOG_DEBUG("SR", rnti_, 0, "SR state reset");
}

void sr_manager::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // 仅在当前无SR待发送时才触发新的SR
    // 对应 srsRAN proc_sr.cc start() 中的 is_pending_sr 检查
    if (state_ == sr_state::IDLE || state_ == sr_state::FAILED) {
        sr_counter_ = 0;
        state_ = sr_state::PENDING;
        LOG_DEBUG("SR", rnti_, 0, "SR triggered by BSR (Regular BSR)");
    }
}

bool sr_manager::can_send_sr(uint32_t tti) const {
    // 对应 srsRAN proc_sr.cc 中的 need_tx() 方法
    // 检查距上次SR发送是否已满足SR周期要求

    if (last_sr_tx_tti_ == 0xFFFFFFFF) {
        // 从未发送过SR, 可以发送
        return true;
    }

    // 计算距上次SR发送的TTI间隔
    uint32_t interval;
    if (tti >= last_sr_tx_tti_) {
        interval = tti - last_sr_tx_tti_;
    } else {
        // TTI回绕处理
        interval = 10240 - last_sr_tx_tti_ + tti;
    }

    // 使用自适应SR周期 (如果有) 或配置的SR周期
    uint32_t effective_period = adaptive_sr_period_ > 0
        ? adaptive_sr_period_ : sr_cfg_.sr_period;

    return interval >= effective_period;
}

void sr_manager::step(uint32_t tti) {
    // 将状态和决策变量在锁内计算, 回调在锁外执行
    // 对应 srsRAN proc_sr.cc step() 的设计模式
    bool do_send_sr = false;
    bool do_fail = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) return;
        if (state_ != sr_state::PENDING) return;

        // 【协议说明 / 简化实现】标准 sr-ProhibitTimer 用于限制 SR 发送间隔、避免 PUCCH
        // 拥塞; 本项目未实现该定时器判定 (sr_prohibit_counter_ 恒为 0),
        // 实际发送间隔仅由 SR 周期 (can_send_sr) 控制。
        if (sr_cfg_.enabled) {
            // PUCCH已配置, 在PUCCH上发送SR
            if (sr_counter_ < static_cast<int>(sr_cfg_.dsr_transmax)) {
                // 未达到最大传输次数, 检查是否可以发送
                if (sr_counter_ == 0 || can_send_sr(tti)) {
                    sr_counter_++;
                    state_ = sr_state::TRANSMITTING;
                    last_sr_tx_tti_ = tti;
                    do_send_sr = true;

                    // 统计在锁内更新, 避免与其他线程的stats_访问产生数据竞争
                    stats_.total_sr_sent++;
                    metrics_collector::instance().record_sr_tx();

                    LOG_INFO("SR", rnti_, tti,
                        "Sending SR on PUCCH, sr_counter=" + std::to_string(sr_counter_) +
                        "/" + std::to_string(sr_cfg_.dsr_transmax));

                    // 发送后回到PENDING状态等待响应
                    state_ = sr_state::PENDING;
                }
            } else {
                // 达到最大SR传输次数, SR失败
                // 对应 srsRAN proc_sr.cc: "Releasing PUCCH/SRS resources"
                if (can_send_sr(tti)) {
                    LOG_ERROR("SR", rnti_, tti,
                        "SR FAILED: max transmissions reached (" +
                        std::to_string(sr_counter_) + "/" +
                        std::to_string(sr_cfg_.dsr_transmax) + ")");
                    state_ = sr_state::FAILED;
                    do_fail = true;

                    // 记录统计
                    stats_.total_sr_fail++;
                    metrics_collector::instance().record_sr_fail();
                }
            }
        } else {
            // 【协议说明 / 简化实现】
            // 真实协议 (TS 36.321 §5.4.4): PUCCH 未配置 SR 时, 数据到达应直接触发随机接入(RA),
            // 且 RA 过程横跨 MAC/RLC/PHY 与 eNB 侧, 不属于 SR 模块职责。
            // 本项目未实现 RA 模块, 此处仅通过 fail_callback_ 通知上层 "需要 RA"。
            // 措辞上应为 "RA fallback required (deferred to upper layer)", 而非模块内触发。
            // 对应 srsRAN proc_sr.cc: "PUCCH not configured. Starting RA procedure"
            LOG_WARN("SR", rnti_, tti,
                "PUCCH not configured, SR-FAIL -> RA fallback required (deferred to upper layer)");
            state_ = sr_state::FAILED;
            do_fail = true;
            stats_.total_sr_fail++;
            metrics_collector::instance().record_sr_fail();
        }
    }

    // 锁外执行回调 (避免死锁)
    if (do_send_sr) {
        if (tx_callback_) {
            tx_callback_(rnti_);
        }
    }

    if (do_fail) {
        if (fail_callback_) {
            fail_callback_(rnti_);
        }
        // SR失败后重置, 准备可能的重试
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = sr_state::IDLE;
        sr_counter_ = 0;
    }
}

void sr_manager::notify_ul_grant_received() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == sr_state::PENDING || state_ == sr_state::TRANSMITTING) {
        // 仅当确实发送过SR才计为SR成功
        // (BSR途径可能在SR尚未发出时就已获得授权, 此时只需取消pending状态)
        if (sr_counter_ > 0) {
            LOG_INFO("SR", rnti_, 0,
                "SR SUCCESS: UL grant received, sr_counter=" + std::to_string(sr_counter_));
            stats_.total_sr_success++;
            metrics_collector::instance().record_sr_success();
        }
        state_ = sr_state::IDLE;
        sr_counter_ = 0;
    }
}

void sr_manager::adjust_sr_period(double traffic_rate) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 【增强/仿真优化】自适应SR周期调整算法 (深化版)
    //
    // 【协议澄清】3GPP标准里SR周期由eNB通过RRC (SchedulingRequestConfig /
    // sr-ConfigIndex) 半静态配置, UE无权单方面修改。本函数为仿真/教学增强,
    // 由UE根据流量模式自决调整SR周期, 用于演示"流量大->SR更频繁->接入时延更低"
    // 的权衡; 真实部署应改由eNB侧根据测量触发RRC重配置。
    //
    // 原理: 根据UE的流量模式动态调整SR发送周期
    //   - 高流量UE: 缩短SR周期, 快速获取上行授权
    //   - 低流量UE: 增大SR周期, 减少PUCCH信令开销
    //
    // 深化点:
    //   1. 使用指数移动平均(EMA)平滑流量速率, 抑制瞬时抖动
    //   2. 连续映射: 用对数尺度将流量速率平滑映射到SR周期,
    //      避免离散档位在阈值附近反复跳变
    //   3. 迟滞机制: 仅当新周期与当前周期差异超过20%时才调整,
    //      进一步避免在阈值附近频繁切换造成的信令开销

    // 1. EMA平滑流量速率
    const double alpha = 0.3; // EMA平滑因子
    avg_traffic_rate_ = alpha * traffic_rate + (1.0 - alpha) * avg_traffic_rate_;

    // 2. 连续映射: 使用对数尺度将流量速率映射到SR周期
    //    公式: period = clamp(K / log2(1 + rate), MIN_PERIOD, MAX_PERIOD)
    //    低流量 -> log2(1+rate)小 -> 周期长; 高流量 -> log2(1+rate)大 -> 周期短
    //    对数尺度使周期随流量变化是平滑连续的, 无离散跳变
    const uint32_t MIN_SR_PERIOD = 5;   // 最短5ms (高流量场景)
    const uint32_t MAX_SR_PERIOD = 80;  // 最长80ms (低流量场景, 减少PUCCH开销)
    const double K = 50.0;              // 缩放常数, 调整流量到周期的映射比例

    double log_rate = std::log2(1.0 + std::max(0.0, avg_traffic_rate_));
    // 防止除零: log_rate理论上>=0, 但avg_traffic_rate_=0时log_rate=0, 需保护
    uint32_t new_period = static_cast<uint32_t>(K / std::max(0.1, log_rate));

    // 3. 迟滞机制: 避免在阈值附近频繁切换
    //    仅当新周期与当前周期差异超过20%时才调整
    if (adaptive_sr_period_ > 0) {
        double change_ratio = std::abs(static_cast<double>(new_period) - adaptive_sr_period_)
                            / adaptive_sr_period_;
        if (change_ratio < 0.2) {
            return; // 变化不够大, 保持当前周期
        }
    }

    // 4. 限幅到 [MIN_SR_PERIOD, MAX_SR_PERIOD]
    new_period = std::max(MIN_SR_PERIOD, std::min(MAX_SR_PERIOD, new_period));

    if (new_period != adaptive_sr_period_) {
        adaptive_sr_period_ = new_period;
        stats_.current_sr_period = new_period;
        LOG_DEBUG("SR", rnti_, 0,
            "Adaptive SR period adjusted to " + std::to_string(new_period) +
            "ms (avg_rate=" + std::to_string(static_cast<int>(avg_traffic_rate_)) + " bytes/TTI)");
    }
}

} // namespace ul_mac
