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
    , feedback_received_(false)
    , cur_ndi_(false)
    , cur_tbs_(0)
    , cur_rv_(-1)
    , max_harq_tx_(4)
    , max_harq_msg3_tx_(4)
    , is_temp_rnti_(false)
    , consecutive_nack_(0)
    // 早期终止默认关闭: 非标准机制且无 RLC 兜底, 提前丢弃即数据丢失 (见头文件说明)
    , early_termination_enabled_(false)
{
}

ul_harq_process::tx_action ul_harq_process::apply_phich_feedback(bool ack) {
    std::lock_guard<std::mutex> lock(mutex_);
    tx_action result;  // is_discarded 默认 false

    // ACK: 仅当进程在途且尚未 ACK 释放时处理, 否则忽略 (空闲/已释放)
    if (ack) {
        if (!is_grant_configured_ || harq_feedback_) return result;
        consecutive_nack_ = 0;
        harq_feedback_ = true;
        feedback_received_ = true;
        LOG_DEBUG("HARQ", 0, 0,
            "PID=" + std::to_string(pid_) + ": PHICH ACK -> process released");
        return result;
    }

    // NACK 分支:
    //  - 连续 NACK 统计始终累加 (早期终止判定), 不受下方守卫影响;
    //  - 守卫仅阻止对已释放/空闲进程的重复状态写, 以及重复触发丢弃。
    ++consecutive_nack_;
    if (!is_grant_configured_ || harq_feedback_) {
        // 进程空闲或已 ACK 释放: 迟到的 NACK 不改变状态, 仅计数。
        return result;
    }

    // 首次标记反馈已收到 (重复 NACK 不重复写 harq_feedback_)
    if (!feedback_received_) {
        harq_feedback_ = false;
        feedback_received_ = true;
    }

    // 丢弃判定 (RTT 延时由 timed_channel 统一建模, 此处不再处理):
    //   1) 达到最大重传次数 (max_harq_tx_/max_harq_msg3_tx_);
    //   2) 早期终止 (默认关闭): 连续3次NACK且已重传≥2次。
    uint32_t max_retx = is_temp_rnti_ ? max_harq_msg3_tx_ : max_harq_tx_;
    bool max_reached = (current_tx_nb_ >= max_retx);
    bool early_term  = early_termination_enabled_ &&
                       (current_tx_nb_ >= 2) && (consecutive_nack_ >= 3);
    if (max_reached || early_term) {
        // 达到最大重传 / 早期终止: 丢弃当前 TB, 释放进程。
        result.is_discarded = true;
        result.tbs          = cur_tbs_;
        result.is_msg3      = is_temp_rnti_;
        reset_unlocked();
        LOG_WARN("HARQ", 0, 0,
            "PID=" + std::to_string(pid_) + ": TB discarded (tbs=" +
            std::to_string(cur_tbs_) + "B), " +
            std::string(max_reached ? "max_retx reached" : "early termination"));
        return result;
    }

    LOG_DEBUG("HARQ", 0, 0,
        "PID=" + std::to_string(pid_) + ": PHICH NACK -> retx pending");
    return result;
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
    feedback_received_ = false;
    cur_tbs_ = 0;
    cur_rv_ = -1;
    cur_ndi_ = false;
    // 重置早期终止统计: 清零连续NACK计数 (开关状态保持不变)
    consecutive_nack_ = 0;
}

uint32_t ul_harq_process::get_rv() const {
    // RV序列: {0, 2, 3, 1}, 通过IRV计数器循环
    return rv_of_irv(current_irv_.load());
}

harq_state ul_harq_process::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_grant_configured_) return harq_state::INACTIVE;
    if (harq_feedback_) return harq_state::INACTIVE;       // ACK -> 进程释放
    if (feedback_received_) return harq_state::RETX_PENDING; // NACK 已收到 -> 等重传授权
    // TB 已发送但尚未收到任何 PHICH 反馈 -> 等待中
    return harq_state::WAITING_FB;
}

void ul_harq_process::fill_process_info(harq_process_info& info) const {
    // 在 process 自身锁内一次性读取所有状态, 避免:
    //   1. manager 锁嵌套 process 锁 (调用方持 manager 锁时直接访问 process 成员)
    //   2. cur_tbs_ 等非原子成员的数据竞争
    std::lock_guard<std::mutex> lock(mutex_);
    info.pid = pid_;
    // 状态推导 (与 get_state 同源, 保持一致):
    //   INACTIVE: 无 grant 或已 ACK 释放
    //   WAITING_FB: TB 已发送, 尚未收到任何 PHICH 反馈
    //   RETX_PENDING: 明确收到 PHICH NACK, 等待 eNB 重传授权
    if (!is_grant_configured_) {
        info.state = harq_state::INACTIVE;
    } else if (harq_feedback_) {
        info.state = harq_state::INACTIVE;
    } else if (feedback_received_) {
        info.state = harq_state::RETX_PENDING;
    } else {
        info.state = harq_state::WAITING_FB;
    }
    info.current_tx_nb = current_tx_nb_;
    info.is_grant_configured = is_grant_configured_;
    info.last_tbs = cur_tbs_;
    info.current_irv = current_irv_;
    // 【last_feedback 语义】
    //   feedback_received_=true + harq_feedback_=true -> ACK (已收到)
    //   feedback_received_=true + harq_feedback_=false -> NACK (已收到)
    //   feedback_received_=false + harq_feedback_=false -> NACK (尚未收到)
    //   注意: harq_feedback_ 默认 false 且不区分"未收到"与"NACK",
    //     但 feedback_received_ 帮助区分两种情况; 外部对 last_feedback 的
    //     观察: WAITING_FB -> NACK, RETX_PENDING -> NACK, INACTIVE -> ACK
    info.last_feedback = harq_feedback_ ? harq_feedback::ACK : harq_feedback::NACK;
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

    // 快照 HARQ 配置 (供后续 PHICH 反馈到达时做丢弃判定; 反馈经 timed_channel
    // 统一 +4 TTI 延时, 本进程不再内部建模 RTT)。
    max_harq_tx_   = harq_cfg.max_harq_tx;
    max_harq_msg3_tx_ = harq_cfg.max_harq_msg3_tx;
    is_temp_rnti_  = is_temp_rnti;

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
        // 非自适应重传: 仅在明确收到 NACK 后才重传
        // (feedback_received_=false 表示尚未收到任何反馈, 进程仍 WAITING_FB,
        //  不应触发重传; 真实协议中非自适应重传由 PHICH NACK 直接触发)
        if (feedback_received_ && !harq_feedback_) {
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
    feedback_received_ = false;  // 新 TB: 尚未收到任何 PHICH 反馈
    is_grant_configured_ = true;
    current_tx_nb_ = 0;
    current_irv_ = 0;

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
        feedback_received_ = false;  // 重传后等待新的 PHICH 反馈

        LOG_INFO("HARQ", 0, grant.tti_tx,
            "PID=" + std::to_string(pid_) +
            ": Adaptive ReTX=" + std::to_string(current_tx_nb_.load()) +
            ", RV=" + std::to_string(get_rv()) +
            ", TBS=" + std::to_string(cur_tbs_));
    }

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

    // 【线程安全】harq_cfg_ 由 init/set_config 在 mutex_ 下写入,
    // 此处锁内取快照再交给进程处理, 避免无锁读竞态
    ul_harq_config cfg_snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg_snapshot = harq_cfg_;
    }

    ul_harq_process::tx_action action =
        processes_[grant.pid].new_grant_ul(grant, cfg_snapshot, is_temp_rnti);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (action.is_new_tx) {
            stats_.total_new_tx++;
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

ul_harq_process::tx_action ul_harq_manager::handle_harq_feedback(
    uint32_t pid, harq_feedback feedback) {
    if (pid >= MAX_HARQ_PROCESSES) return ul_harq_process::tx_action();
    std::lock_guard<std::mutex> lock(mutex_);

    // 【PHICH 反馈落地】通过 process 的 apply_phich_feedback() 应用反馈:
    //   ACK -> harq_feedback_=true, 进程在 get_state()/fill_process_info 呈现 INACTIVE,
    //        下一次 has_pending_transmission() 查询时视为 grant 已释放 -> 不再抑制 SR;
    //   NACK -> harq_feedback_=false, feedback_received_=true, 呈现 RETX_PENDING,
    //        仍视为 grant 在途 -> SR 继续抑制。NACK 达到重传上限/早期终止时
    //        由 process 判定丢弃, 返回的 tx_action.is_discarded=true 供 UE 回滚缓冲。
    //   这解决了"ACK 后进程永不释放 -> has_pending_transmission() 永真 -> SR 被永久
    //   抑制"的问题 (场景2 1000TTI 仅发5次 SR 的根因)。
    ul_harq_process::tx_action result;
    if (feedback == harq_feedback::ACK || feedback == harq_feedback::NACK) {
        result = processes_[pid].apply_phich_feedback(feedback == harq_feedback::ACK);
        if (feedback == harq_feedback::ACK)      stats_.total_ack++;
        else                                     stats_.total_nack++;
        if (result.is_discarded)                 stats_.total_fail++;
    }

    uint32_t total_fb = stats_.total_ack + stats_.total_nack;
    if (total_fb > 0) {
        stats_.bler = static_cast<double>(stats_.total_nack) / total_fb;
    }
    return result;
}

void ul_harq_manager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        processes_[i].reset();
    }
    LOG_INFO("HARQ", rnti_, 0, "All HARQ processes reset");
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

bool ul_harq_manager::has_pending_transmission() const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; i++) {
        harq_process_info info;
        processes_[i].fill_process_info(info);
        // WAITING_FB: 已发送, 等待PHICH反馈; RETX_PENDING: 收到NACK待重传
        // 两者均表示 grant 仍在途, UL 授权仍有效 (不应触发 SR)
        if (info.state == harq_state::WAITING_FB ||
            info.state == harq_state::RETX_PENDING) {
            return true;
        }
    }
    return false;
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
