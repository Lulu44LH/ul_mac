// =============================================================================
// ue_ul_harq_manager.cpp - 上行HARQ重传管理器实现
// 参考 srsRAN_4G/srsue/src/stack/mac/ul_harq.cc
// =============================================================================

#include "ul_mac/ue_ul_harq_manager.h"
#include <algorithm>
#include <cstring>

namespace ul_mac {

// ============================================================================
// ul_harq_process
// ============================================================================

ul_harq_process::ul_harq_process(uint32_t pid)
    : pid_(pid)
    , current_tx_nb_(0)
    , current_irv_(0)
    , harq_feedback_(false)
    , is_grant_configured_(false)
    , cur_ndi_(false)
    , cur_tbs_(0)
    , cur_rv_(-1)
    , tx_tti_(0xFFFFFFFF)
    , rtt_ttis_(8)
    , feedback_pending_(false)
    , consecutive_nack_(0)
    , early_termination_enabled_(true)
{
}

void ul_harq_process::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    reset_unlocked();
}

void ul_harq_process::reset_unlocked() {
    current_tx_nb_ = 0;
    current_irv_ = 0;
    is_grant_configured_ = false;
    harq_feedback_ = false;
    cur_tbs_ = 0;
    cur_rv_ = -1;
    cur_ndi_ = false;
    // 重置RTT相关状态: 标记为未传输, 清除在途反馈
    tx_tti_ = 0xFFFFFFFF;
    feedback_pending_ = false;
    // 重置早期终止统计: 清零连续NACK计数 (开关状态保持不变)
    consecutive_nack_ = 0;
}

void ul_harq_process::reset_ndi() {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_ndi_ = false;
}

uint32_t ul_harq_process::get_rv() const {
    // RV序列: {0, 2, 3, 1}, 通过IRV计数器循环
    return rv_of_irv(current_irv_.load());
}

harq_state ul_harq_process::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_grant_configured_) return harq_state::INACTIVE;
    if (harq_feedback_) return harq_state::INACTIVE;
    if (current_tx_nb_ > 0) return harq_state::RETX_PENDING;
    return harq_state::WAITING_FB;
}

void ul_harq_process::fill_process_info(harq_process_info& info) const {
    // 在 process 自身锁内一次性读取所有状态, 避免:
    //   1. manager 锁嵌套 process 锁 (调用方持 manager 锁时直接访问 process 成员)
    //   2. cur_tbs_ 等非原子成员的数据竞争
    std::lock_guard<std::mutex> lock(mutex_);
    info.pid = pid_;
    // 在锁内一次性读取所有状态
    if (!is_grant_configured_) {
        info.state = harq_state::INACTIVE;
    } else if (harq_feedback_) {
        info.state = harq_state::INACTIVE;
    } else if (current_tx_nb_ > 0) {
        info.state = harq_state::RETX_PENDING;
    } else {
        info.state = harq_state::WAITING_FB;
    }
    info.current_tx_nb = current_tx_nb_;
    info.is_grant_configured = is_grant_configured_;
    info.last_tbs = cur_tbs_;
    info.current_irv = current_irv_;
    info.last_feedback = harq_feedback_ ? harq_feedback::ACK : harq_feedback::NACK;
}

bool ul_harq_process::is_feedback_ready(uint32_t current_tti) const {
    // RTT建模: 从发送到收到PHICH反馈需要经过若干TTI (LTE标准约8ms)
    // tx_tti_ == 0xFFFFFFFF 表示尚未传输, 反馈不可能到达
    if (tx_tti_ == 0xFFFFFFFF) return false;
    // TTI单调递增, uint32_t减法天然处理回绕
    uint32_t elapsed = current_tti - tx_tti_;
    // rtt_ttis_ = 0 时退化为即时反馈 (向后兼容)
    return elapsed >= rtt_ttis_;
}

void ul_harq_process::update_feedback_stats(bool ack) {
    // 【增强】更新HARQ反馈统计, 用于早期终止判断
    // ACK: 重置连续NACK计数 (传输成功, 信道条件改善)
    // NACK: 递增连续NACK计数 (传输失败, 可能信道条件恶化)
    std::lock_guard<std::mutex> lock(mutex_);
    if (ack) {
        consecutive_nack_ = 0;
    } else {
        consecutive_nack_++;
    }
}

ul_harq_process::tx_action ul_harq_process::new_grant_ul(
    const ul_grant& grant, const ul_harq_config& harq_cfg, bool is_temp_rnti) {
    //
    // 3GPP TS 36.321 Section 5.4.2.1 决策逻辑:
    //   1. 处理HARQ反馈 (PHICH)
    //   2. 检查最大重传次数
    //   3. 判断新传输 vs 重传 (通过NDI)
    //   4. 生成传输动作

    std::lock_guard<std::mutex> lock(mutex_);

    // 从配置同步RTT时长 (允许运行时通过 set_rtt_ttis() 覆盖, 此处保证配置生效)
    rtt_ttis_ = harq_cfg.harq_rtt_ttis;

    // 步骤1: 处理HARQ反馈
    if (grant.phich_available) {
        // RTT延迟建模: PHICH反馈需要经过若干TTI才能到达UE
        // 在RTT未到时, 反馈仍在途中, 不立即处理, 仅标记反馈在途
        if (!is_feedback_ready(grant.tti_tx)) {
            // RTT未到, 反馈仍在途中, 暂不处理
            feedback_pending_ = true;
            LOG_DEBUG("HARQ", 0, grant.tti_tx,
                "PID=" + std::to_string(pid_) +
                ": PHICH available but RTT not elapsed (tx_tti=" +
                std::to_string(tx_tti_) + ", rtt=" + std::to_string(rtt_ttis_) +
                "), feedback deferred");
        } else {
            // RTT已到, 反馈已到达, 正常处理
            feedback_pending_ = false;
            if (grant.ndi_present && grant.ndi == cur_ndi_ && grant.tbs != 0) {
                harq_feedback_ = false;
            } else {
                harq_feedback_ = grant.hi_value;
            }

            // 【增强/非标准】早期终止: 基于连续NACK统计提前终止重传
            // 【协议澄清】3GPP 标准 HARQ 重传上限由 RRC maxHARQ-Tx 决定, 并无
            // "连续NACK提前丢弃"机制 (无损投递保证由 RLC 重传兜底)。本逻辑是
            // 本项目额外资源优化: 当连续收到3次NACK且已重传至少2次时, 认为该
            // TB 在当前信道条件下难以成功解码, 提前丢弃以节省无线资源。
            // 开关 early_termination_enabled_ 默认启用, 可通过 set_early_termination_enabled() 关闭
            if (early_termination_enabled_ && current_tx_nb_ >= 2
                && consecutive_nack_ >= 3) {
                LOG_WARN("HARQ", 0, grant.tti_tx,
                    "PID=" + std::to_string(pid_) +
                    ": Early termination triggered (consecutive_nack=" +
                    std::to_string(consecutive_nack_) + ")");
                tx_action action;
                action.is_discarded = true;
                action.tx_nb = current_tx_nb_;
                // 注意: 此处已持有 mutex_, 必须用不加锁版本
                reset_unlocked();
                return action;
            }

            // 步骤2: 检查最大重传次数
            uint32_t max_retx = is_temp_rnti ? harq_cfg.max_harq_msg3_tx : harq_cfg.max_harq_tx;
            if (current_tx_nb_ >= max_retx && !grant.hi_value) {
                LOG_ERROR("HARQ", 0, grant.tti_tx,
                    "PID=" + std::to_string(pid_) +
                    ": Max ReTX reached (" + std::to_string(max_retx) + "), discarding TB");
                tx_action action;
                action.is_discarded = true;
                action.tx_nb = current_tx_nb_;
                // 注意: 此处已持有 mutex_, 必须用不加锁版本
                // (早期版本调用 reset() 导致同锁重入自死锁, 见 LEARNING_PATH.md 附录 D)
                reset_unlocked();
                return action;
            }
        }
    }

    // 步骤3: 判断新传输 vs 重传
    tx_action action;

    if (grant.ndi_present) {
        if (grant.tbs == 0) {
            action.tbs = 0;
            return action;
        }

        bool is_new_tx = (!is_temp_rnti && grant.ndi != cur_ndi_) ||
                         (!is_grant_configured_ && !is_temp_rnti) ||
                         grant.is_rar;

        if (is_new_tx) {
            action = generate_new_tx(grant, is_temp_rnti);
        } else if (is_grant_configured_) {
            action = generate_retx(grant);
        } else {
            LOG_WARN("HARQ", 0, grant.tti_tx,
                "PID=" + std::to_string(pid_) + ": Retx but no previous grant");
            return action;
        }
    } else if (is_grant_configured_) {
        if (!harq_feedback_) {
            action = generate_retx(grant);
            action.is_retx = true;
        }
    }

    return action;
}

ul_harq_process::tx_action ul_harq_process::generate_new_tx(
    const ul_grant& grant, bool is_temp_rnti) {
    cur_ndi_ = grant.ndi;
    cur_tbs_ = grant.tbs;
    cur_rv_ = grant.rv;
    harq_feedback_ = false;
    is_grant_configured_ = true;
    current_tx_nb_ = 0;
    current_irv_ = 0;
    // 记录本次传输的TTI, 用于RTT延迟计算 (从此时起等待PHICH反馈)
    tx_tti_ = grant.tti_tx;
    feedback_pending_ = false;

    tx_action action = generate_tx();
    action.is_new_tx = true;
    action.is_msg3 = grant.is_rar || is_temp_rnti;

    LOG_INFO("HARQ", 0, grant.tti_tx,
        "PID=" + std::to_string(pid_) +
        ": New TX" + (action.is_msg3 ? " (Msg3)" : "") +
        ", RV=" + std::to_string(action.rv) +
        ", TBS=" + std::to_string(action.tbs) +
        ", NDI=" + (cur_ndi_ ? "1" : "0"));
    return action;
}

ul_harq_process::tx_action ul_harq_process::generate_retx(const ul_grant& grant) {
    if (grant.ndi_present) {
        if (grant.rv >= 0) {
            current_irv_ = irv_of_rv(grant.rv);
        }
        cur_tbs_ = grant.tbs;
        cur_rv_ = grant.rv;
        harq_feedback_ = false;

        LOG_INFO("HARQ", 0, grant.tti_tx,
            "PID=" + std::to_string(pid_) +
            ": Adaptive ReTX=" + std::to_string(current_tx_nb_.load()) +
            ", RV=" + std::to_string(get_rv()) +
            ", TBS=" + std::to_string(cur_tbs_));
    }

    // 记录本次重传的TTI, 用于RTT延迟计算 (重传后重新等待PHICH反馈)
    tx_tti_ = grant.tti_tx;
    feedback_pending_ = false;

    tx_action action = generate_tx();
    action.is_retx = true;
    return action;
}

ul_harq_process::tx_action ul_harq_process::generate_tx() {
    current_tx_nb_++;

    tx_action action;
    action.tx_nb = current_tx_nb_;

    if (cur_rv_ >= 0) {
        action.rv = static_cast<uint32_t>(cur_rv_);
        cur_rv_ = -1;
    } else {
        action.rv = get_rv();
    }
    action.tbs = cur_tbs_;
    current_irv_ = (current_irv_ + 1) % 4;
    return action;
}

// ============================================================================
// ul_harq_manager
// ============================================================================

ul_harq_manager::ul_harq_manager(uint16_t rnti)
    : rnti_(rnti), initialized_(false)
{
    processes_ = std::make_unique<ul_harq_process[]>(MAX_HARQ_PROCESSES);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        processes_[i].init_pid(i);
    }
}

void ul_harq_manager::init(const ul_harq_config& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    harq_cfg_ = config;
    initialized_ = true;
    LOG_INFO("HARQ", rnti_, 0,
        "HARQ initialized: max_harq_tx=" + std::to_string(config.max_harq_tx) +
        ", max_msg3_tx=" + std::to_string(config.max_harq_msg3_tx) +
        ", nof_processes=" + std::to_string(MAX_HARQ_PROCESSES));
}

void ul_harq_manager::set_config(const ul_harq_config& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    harq_cfg_ = config;
}

ul_harq_process::tx_action ul_harq_manager::new_grant_ul(
    const ul_grant& grant, bool is_temp_rnti) {
    if (grant.pid >= MAX_HARQ_PROCESSES) {
        LOG_ERROR("HARQ", rnti_, grant.tti_tx,
            "Invalid PID: " + std::to_string(grant.pid));
        return ul_harq_process::tx_action();
    }

    ul_harq_process::tx_action action =
        processes_[grant.pid].new_grant_ul(grant, harq_cfg_, is_temp_rnti);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (action.is_new_tx) {
            stats_.total_new_tx++;
            total_pkts_++;
            metrics_collector::instance().record_harq_new_tx();
        } else if (action.is_retx) {
            stats_.total_retx++;
            metrics_collector::instance().record_harq_retx();
        } else if (action.is_discarded) {
            stats_.total_fail++;
            metrics_collector::instance().record_harq_fail();
        }
        // 平均重传次数 = 总重传次数 / 总新传次数 (与metrics_collector口径一致)
        if (stats_.total_new_tx > 0) {
            stats_.avg_retx_per_pkt =
                static_cast<double>(stats_.total_retx) / stats_.total_new_tx;
            average_retx_.store(static_cast<float>(stats_.avg_retx_per_pkt));
        }
    }

    if (action.tbs > 0) {
        metrics_collector::instance().record_ul_bytes(action.tbs);
    }
    return action;
}

void ul_harq_manager::handle_harq_feedback(uint32_t pid, harq_feedback feedback) {
    if (pid >= MAX_HARQ_PROCESSES) return;
    std::lock_guard<std::mutex> lock(mutex_);

    if (feedback == harq_feedback::ACK) {
        stats_.total_ack++;
        LOG_DEBUG("HARQ", rnti_, 0,
            "PID=" + std::to_string(pid) + ": ACK received");
    } else if (feedback == harq_feedback::NACK) {
        stats_.total_nack++;
        LOG_DEBUG("HARQ", rnti_, 0,
            "PID=" + std::to_string(pid) + ": NACK received, retx pending");
    }

    // 【增强】更新进程级反馈统计, 用于早期终止判断
    // 仅ACK/NACK参与统计, NONE不影响连续NACK计数
    // (manager->process 锁序与 fill_process_info/reset 一致, 无死锁风险)
    if (feedback == harq_feedback::ACK || feedback == harq_feedback::NACK) {
        processes_[pid].update_feedback_stats(feedback == harq_feedback::ACK);
    }

    uint32_t total_fb = stats_.total_ack + stats_.total_nack;
    if (total_fb > 0) {
        stats_.bler = static_cast<double>(stats_.total_nack) / total_fb;
    }
}

void ul_harq_manager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        processes_[i].reset();
    }
    LOG_INFO("HARQ", rnti_, 0, "All HARQ processes reset");
}

void ul_harq_manager::reset_ndi() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        processes_[i].reset_ndi();
    }
}

harq_process_info ul_harq_manager::get_process_info_unlocked(uint32_t pid) const {
    // 调用方必须已持有 manager 的 mutex_
    // 通过 fill_process_info() 在 process 自身锁内读取数据,
    // 不再嵌套调用 get_state()/get_current_tbs() 等会再次加锁 process 的方法
    harq_process_info info;
    if (pid >= MAX_HARQ_PROCESSES) return info;
    processes_[pid].fill_process_info(info);
    return info;
}

harq_process_info ul_harq_manager::get_process_info(uint32_t pid) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return get_process_info_unlocked(pid);
}

std::array<harq_process_info, MAX_HARQ_PROCESSES>
ul_harq_manager::get_all_process_info() const {
    std::array<harq_process_info, MAX_HARQ_PROCESSES> infos;
    // 仅加锁一次 manager 的 mutex_, 避免重复加锁/解锁 16 次
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        infos[i] = get_process_info_unlocked(i);
    }
    return infos;
}

uint32_t ul_harq_manager::get_idle_process_id() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        if (!processes_[i].has_grant()) return i;
    }
    uint32_t min_tx = UINT32_MAX;
    uint32_t min_pid = 0;
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        uint32_t tx_nb = processes_[i].get_nof_retx();
        if (tx_nb < min_tx) { min_tx = tx_nb; min_pid = i; }
    }
    return min_pid;
}

void ul_harq_manager::set_early_termination_enabled(bool enable) {
    // 【增强】批量设置所有HARQ进程的早期终止开关
    // 持有 manager 锁后逐个调用 process 的 setter (manager->process 锁序一致)
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        processes_[i].set_early_termination_enabled(enable);
    }
    LOG_INFO("HARQ", rnti_, 0,
        "Early termination " + std::string(enable ? "enabled" : "disabled") +
        " for all HARQ processes");
}

} // namespace ul_mac
