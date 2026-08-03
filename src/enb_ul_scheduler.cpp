// =============================================================================
// enb_ul_scheduler.cpp - 上行调度器实现 (eNB侧)
// 参考 srsRAN_4G/srsenb 中的调度器实现
//
// 演示级增强:
//   1. PRB池: 首次适配连续分配, ul_grant.prb_start 记录起始位置, 每TTI重置
//   2. CQI输入: handle_cqi() 接收UE上报CQI(0-15), 优先于SNR选MCS
//   3. per-UE HARQ PID: 每UE独立跟踪进程忙闲, 修正全局轮转PID在多UE下
//      可能给同一UE重复分配未释放进程的缺陷
// =============================================================================

#include "ul_mac/enb_ul_scheduler.h"
#include <algorithm>
#include <cmath>

namespace ul_mac {

ul_scheduler::ul_scheduler(sched_algorithm algorithm, uint32_t total_prb)
    : algorithm_(algorithm), total_prb_(total_prb)
    , fairness_coeff_(1.0)
{
    prb_busy_.assign(total_prb_, 0);
    LOG_INFO("SCHED", 0, 0, "UL Scheduler initialized, total_prb=" + std::to_string(total_prb));
}

void ul_scheduler::add_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ue_db_.count(rnti) > 0) return;
    ue_sched_context ctx;
    ctx.rnti = rnti;
    ue_db_[rnti] = ctx;
    LOG_INFO("SCHED", rnti, 0, "UE added to scheduler");
}

void ul_scheduler::remove_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    ue_db_.erase(rnti);
}

void ul_scheduler::handle_sr(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.sr_pending = true;
    LOG_DEBUG("SCHED", rnti, 0, "SR received from UE");
}

void ul_scheduler::handle_bsr(uint16_t rnti, uint8_t lcg_id, uint32_t bsr_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end() || lcg_id >= NOF_LCGS) return;
    uint32_t bytes = bsr_index_to_bytes(bsr_value);
    it->second.ul_buffer[lcg_id] = bytes;
    it->second.total_ul_buffer = 0;
    for (uint32_t i = 0; i < NOF_LCGS; i++)
        it->second.total_ul_buffer += it->second.ul_buffer[i];
    LOG_DEBUG("SCHED", rnti, 0,
        "BSR: LCG=" + std::to_string(lcg_id) +
        ", idx=" + std::to_string(bsr_value) +
        ", bytes=" + std::to_string(bytes) +
        ", total=" + std::to_string(it->second.total_ul_buffer));
}

void ul_scheduler::handle_phr(uint16_t rnti, int phr, uint32_t ul_nof_prb) {
    (void)ul_nof_prb; // 预留参数: 真实系统中用于将PHR归一化到单PRB发送功率
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.phr = phr;
}

void ul_scheduler::handle_cqi(uint16_t rnti, uint8_t cqi) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end() || cqi > 15) return;
    it->second.cqi = cqi;
    LOG_DEBUG("SCHED", rnti, 0, "CQI=" + std::to_string(cqi));
}

void ul_scheduler::handle_ul_crc(uint16_t rnti, uint32_t pid, bool crc_ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end() || pid >= MAX_HARQ_PROCESSES) return;
    if (!crc_ok) {
        // NACK: 标记待重传, 进程保持占用 (等待重传)
        it->second.pending_retx_pid = pid;
        it->second.harq_pid_busy[pid] = true;
        LOG_DEBUG("SCHED", rnti, 0, "UL CRC FAIL PID=" + std::to_string(pid));
    } else {
        // ACK: 清除待重传标记, 释放HARQ进程
        if (it->second.pending_retx_pid == pid)
            it->second.pending_retx_pid = 0xFFFF;
        it->second.harq_pid_busy[pid] = false;
        LOG_DEBUG("SCHED", rnti, 0, "UL CRC OK PID=" + std::to_string(pid));
    }
}

// ---------------------------------------------------------------------------
// MCS/TBS 查找表 (基于3GPP TS 36.213, 简化版)
// 替代原先的线性公式, 采用区间映射 + 锚点线性插值
// ---------------------------------------------------------------------------

// SNR -> MCS 映射阈值表 (单位: x100 dB, 与snr_x100一致)
// 每项为采用该MCS索引所需的最低SNR; 采用区间映射而非线性公式
// 锚点 (典型LTE CQI/MCS映射关系):
//   SNR < -6dB  -> MCS 0  (最低)
//   -6 ~ -4dB   -> MCS 1
//   -4 ~ -2dB   -> MCS 2
//   SNR >= 20dB -> MCS 28 (最高, 256QAM)
static const int32_t SNR_MCS_THRESHOLD[29] = {
    -1000, // MCS 0:  SNR < -6 dB (兜底)
    -600,  // MCS 1:  SNR >= -6 dB
    -400,  // MCS 2:  SNR >= -4 dB
    -200,  // MCS 3:  SNR >= -2 dB
    -100,  // MCS 4:  SNR >= -1 dB
       0,  // MCS 5:  SNR >=  0 dB
     100,  // MCS 6:  SNR >=  1 dB
     200,  // MCS 7:  SNR >=  2 dB
     300,  // MCS 8:  SNR >=  3 dB
     400,  // MCS 9:  SNR >=  4 dB
     500,  // MCS 10: SNR >=  5 dB
     600,  // MCS 11: SNR >=  6 dB
     700,  // MCS 12: SNR >=  7 dB
     800,  // MCS 13: SNR >=  8 dB
     900,  // MCS 14: SNR >=  9 dB
    1000,  // MCS 15: SNR >= 10 dB
    1100,  // MCS 16: SNR >= 11 dB
    1200,  // MCS 17: SNR >= 12 dB
    1300,  // MCS 18: SNR >= 13 dB
    1400,  // MCS 19: SNR >= 14 dB
    1500,  // MCS 20: SNR >= 15 dB
    1600,  // MCS 21: SNR >= 16 dB
    1700,  // MCS 22: SNR >= 17 dB
    1800,  // MCS 23: SNR >= 18 dB
    1840,  // MCS 24: SNR >= 18.4 dB
    1880,  // MCS 25: SNR >= 18.8 dB
    1920,  // MCS 26: SNR >= 19.2 dB
    1960,  // MCS 27: SNR >= 19.6 dB
    2000,  // MCS 28: SNR >= 20 dB (256QAM最高阶)
};

// 每PRB效率锚点表 (MCS -> bits/PRB), 基于3GPP TS 36.213 Table 7.1.7.2.1-1简化
// 关键MCS点的每PRB效率; 中间MCS点由calculate_tbs()线性插值得到
static const struct { uint8_t mcs; uint16_t bits_per_prb; } TBS_EFFICIENCY_ANCHOR[] = {
    {  0,  16 }, // QPSK, 最低阶
    {  5,  64 },
    { 10, 120 },
    { 15, 176 },
    { 20, 272 },
    { 28, 456 }, // 256QAM, 最高阶
};
static const size_t TBS_ANCHOR_COUNT =
    sizeof(TBS_EFFICIENCY_ANCHOR) / sizeof(TBS_EFFICIENCY_ANCHOR[0]);

uint8_t ul_scheduler::calculate_mcs(uint32_t snr_x100) {
    // snr_x100为uint32_t, 转为int32_t以便与负阈值正确比较
    // (避免有符号/无符号比较时负阈值被转换为巨大无符号值导致判断错误)
    int32_t snr = static_cast<int32_t>(snr_x100);
    // 从高到低查找首个满足阈值的MCS索引, 实现区间映射
    for (int i = 28; i >= 0; --i) {
        if (snr >= SNR_MCS_THRESHOLD[i]) {
            return static_cast<uint8_t>(i);
        }
    }
    return 0;
}

uint8_t ul_scheduler::calculate_mcs_from_cqi(uint8_t cqi) {
    // LTE CQI(0-15) -> MCS 简化映射 (近似 TS 36.213 Table 7.2.3-1)
    // CQI 0: 超出范围 (调用方应回退SNR); CQI k -> MCS (k-1)*2, 上限28
    // CQI 1->0, 2->2, 3->4 ... 15->28
    if (cqi == 0) return 0;
    uint32_t mcs = (static_cast<uint32_t>(cqi) - 1) * 2;
    return static_cast<uint8_t>(std::min<uint32_t>(mcs, 28));
}

uint32_t ul_scheduler::calculate_tbs(uint8_t mcs, uint32_t n_prb) {
    // 基于每PRB效率锚点表线性插值得到bits/PRB, 再乘以PRB数得到TBS(比特)
    // 低于最低锚点取最低锚点效率, 高于最高锚点取最高锚点效率
    if (mcs <= TBS_EFFICIENCY_ANCHOR[0].mcs) {
        uint32_t bits = static_cast<uint32_t>(TBS_EFFICIENCY_ANCHOR[0].bits_per_prb) * n_prb;
        return bits / 8;
    }
    if (mcs >= TBS_EFFICIENCY_ANCHOR[TBS_ANCHOR_COUNT - 1].mcs) {
        uint32_t bits = static_cast<uint32_t>(TBS_EFFICIENCY_ANCHOR[TBS_ANCHOR_COUNT - 1].bits_per_prb) * n_prb;
        return bits / 8;
    }
    // 定位mcs所在的锚点区间 [lo, lo+1]
    size_t lo = 0;
    while (lo + 1 < TBS_ANCHOR_COUNT && TBS_EFFICIENCY_ANCHOR[lo + 1].mcs <= mcs) {
        lo++;
    }
    uint16_t m0 = TBS_EFFICIENCY_ANCHOR[lo].mcs;
    uint16_t m1 = TBS_EFFICIENCY_ANCHOR[lo + 1].mcs;
    uint32_t e0 = TBS_EFFICIENCY_ANCHOR[lo].bits_per_prb;
    uint32_t e1 = TBS_EFFICIENCY_ANCHOR[lo + 1].bits_per_prb;
    // 线性插值: e = e0 + (e1 - e0) * (mcs - m0) / (m1 - m0)
    uint32_t bits_per_prb = e0 + (e1 - e0) * (mcs - m0) / (m1 - m0);
    uint32_t tbs_bits = bits_per_prb * n_prb;
    return tbs_bits / 8;
}

// ---------------------------------------------------------------------------
// PRB 池管理 (首次适配连续分配, 每TTI重置)
// ---------------------------------------------------------------------------

int32_t ul_scheduler::prb_alloc_unlocked(uint32_t n) {
    if (n == 0 || n > total_prb_) return -1;
    uint32_t run = 0;
    for (uint32_t i = 0; i < total_prb_; ++i) {
        if (prb_busy_[i] == 0) {
            ++run;
            if (run >= n) {
                uint32_t start = i + 1 - n;
                for (uint32_t k = start; k < start + n; ++k) prb_busy_[k] = 1;
                return static_cast<int32_t>(start);
            }
        } else {
            run = 0;
        }
    }
    return -1; // 无足够连续PRB (碎片化)
}

void ul_scheduler::prb_reset_unlocked() {
    std::fill(prb_busy_.begin(), prb_busy_.end(), 0);
}

// ---------------------------------------------------------------------------
// per-UE HARQ PID 分配
// ---------------------------------------------------------------------------

int32_t ul_scheduler::alloc_free_pid_unlocked(ue_sched_context& ctx) {
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; ++i) {
        if (!ctx.harq_pid_busy[i]) return static_cast<int32_t>(i);
    }
    return -1; // 全部进程都在忙
}

ul_grant ul_scheduler::generate_ul_grant_unlocked(
    uint16_t rnti, uint32_t tti, uint32_t pid, bool is_retx) {
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return ul_grant();
    auto& ctx = it->second;

    uint32_t req_bytes = ctx.total_ul_buffer;
    if (req_bytes == 0) req_bytes = 100;
    // MCS: 优先用UE上报的CQI, 未上报(cqi=0)则回退SNR
    uint8_t mcs = (ctx.cqi > 0) ? calculate_mcs_from_cqi(ctx.cqi)
                                : calculate_mcs(ctx.ul_snr);
    uint32_t bytes_per_prb = (static_cast<uint32_t>(mcs) + 1) * 72 / 8;
    uint32_t n_prb = (bytes_per_prb > 0)
        ? (req_bytes + bytes_per_prb - 1) / bytes_per_prb : total_prb_;
    n_prb = std::min(n_prb, total_prb_);
    // 从PRB池首次适配分配连续区间
    int32_t start = prb_alloc_unlocked(n_prb);
    if (start < 0) return ul_grant(); // 本TTI无连续PRB可用
    uint32_t tbs = calculate_tbs(mcs, n_prb);

    ul_grant grant;
    grant.rnti = rnti;
    grant.pid = pid;
    grant.tbs = tbs;
    grant.n_prb = n_prb;
    grant.prb_start = static_cast<uint32_t>(start);
    grant.mcs = mcs;
    // NDI管理 (3GPP TS 36.321 §5.4.2.1):
    //   新传: 翻转该HARQ进程的NDI, UE侧检测到NDI变化即判定为新传
    //   重传: NDI保持不变, UE侧判定为自适应重传
    if (!is_retx) {
        ctx.ndi[pid] = !ctx.ndi[pid];
    }
    grant.ndi = ctx.ndi[pid];
    grant.ndi_present = true;
    // 新传固定RV=0; 重传时rv=-1, 由UE侧HARQ进程按IRV序列{0,2,3,1}自行递进
    grant.rv = is_retx ? -1 : 0;
    grant.tti_tx = tti;
    return grant;
}

void ul_scheduler::deduct_ul_buffer_unlocked(ue_sched_context& ctx, uint32_t bytes) {
    // 授权发出后立即扣减缓冲区估计, 下次BSR到达时会重新校准
    for (uint32_t i = 0; i < NOF_LCGS && bytes > 0; i++) {
        uint32_t taken = std::min(bytes, ctx.ul_buffer[i]);
        ctx.ul_buffer[i] -= taken;
        bytes -= taken;
    }
    ctx.total_ul_buffer = 0;
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        ctx.total_ul_buffer += ctx.ul_buffer[i];
    }
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_ul(uint32_t tti) {
    switch (algorithm_) {
        case sched_algorithm::PROPORTIONAL_FAIR: return schedule_pf(tti);
        case sched_algorithm::ROUND_ROBIN:       return schedule_rr(tti);
        case sched_algorithm::PRIORITY_BASED:    return schedule_priority(tti);
        default: return schedule_pf(tti);
    }
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_pf(uint32_t tti) {
    // 比例公平调度: 重传优先 > PF度量值排序新传
    std::vector<ul_sched_result> results;
    std::lock_guard<std::mutex> lock(mutex_);
    prb_reset_unlocked();

    // 阶段1: 重传优先 (用 pending_retx_pid, 进程仍在忙)
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.pending_retx_pid != 0xFFFF) {
            ul_sched_result res;
            res.rnti = rnti;
            res.grant = generate_ul_grant_unlocked(rnti, tti, ctx.pending_retx_pid, true);
            res.is_retx = true;
            if (res.grant.tbs > 0) {
                // 已调度重传: 清除待重传标记, 进程保持忙直到收到ACK/NACK
                ctx.pending_retx_pid = 0xFFFF;
                results.push_back(res);
                metrics_collector::instance().record_ul_grant();
                LOG_INFO("SCHED", rnti, tti,
                    "PF RETX: PID=" + std::to_string(res.grant.pid) +
                    ", TBS=" + std::to_string(res.grant.tbs) + "B");
            }
        }
    }

    // 阶段2: 计算PF度量值
    struct ue_pf { uint16_t rnti; double metric; };
    std::vector<ue_pf> queue;
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue;
        double current_rate = static_cast<double>(ctx.total_ul_buffer);
        double avg = (ctx.ul_avg_rate > 0) ? ctx.ul_avg_rate : 1.0;
        double pf = current_rate / std::pow(avg, fairness_coeff_);
        queue.push_back({rnti, pf});
    }
    std::sort(queue.begin(), queue.end(),
        [](const ue_pf& a, const ue_pf& b) { return a.metric > b.metric; });

    // 阶段3: 按PF顺序分配
    for (const auto& e : queue) {
        auto& ctx = ue_db_[e.rnti];
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue; // 该UE所有HARQ进程都在忙, 跳过
        ul_sched_result res;
        res.rnti = e.rnti;
        res.grant = generate_ul_grant_unlocked(e.rnti, tti, static_cast<uint32_t>(pid), false);
        res.is_retx = false;
        if (res.grant.tbs > 0) {
            // 新传占用该HARQ进程, 等CRC反馈释放
            ctx.harq_pid_busy[pid] = true;
            ctx.sr_pending = false; // 实际获得授权后才清除SR标志
            deduct_ul_buffer_unlocked(ctx, res.grant.tbs);
            double alpha = 1.0 / (ctx.ul_nof_samples + 1);
            ctx.ul_avg_rate = (1.0 - alpha) * ctx.ul_avg_rate + alpha * res.grant.tbs;
            ctx.ul_nof_samples++;
            results.push_back(res);
            metrics_collector::instance().record_ul_grant();
            LOG_INFO("SCHED", e.rnti, tti,
                "PF NEW: PID=" + std::to_string(res.grant.pid) +
                ", MCS=" + std::to_string(res.grant.mcs) +
                ", TBS=" + std::to_string(res.grant.tbs) + "B" +
                ", PRB[" + std::to_string(res.grant.prb_start) + "+" +
                std::to_string(res.grant.n_prb) + "]");
        }
    }
    return results;
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_rr(uint32_t tti) {
    std::vector<ul_sched_result> results;
    std::lock_guard<std::mutex> lock(mutex_);
    prb_reset_unlocked();

    // 重传优先
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.pending_retx_pid != 0xFFFF) {
            ul_sched_result res;
            res.rnti = rnti;
            res.grant = generate_ul_grant_unlocked(rnti, tti, ctx.pending_retx_pid, true);
            res.is_retx = true;
            if (res.grant.tbs > 0) {
                ctx.pending_retx_pid = 0xFFFF;
                results.push_back(res);
                metrics_collector::instance().record_ul_grant();
            }
        }
    }
    // 轮询新传
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue;
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue;
        ul_sched_result res;
        res.rnti = rnti;
        res.grant = generate_ul_grant_unlocked(rnti, tti, static_cast<uint32_t>(pid), false);
        res.is_retx = false;
        if (res.grant.tbs > 0) {
            ctx.harq_pid_busy[pid] = true;
            ctx.sr_pending = false;
            deduct_ul_buffer_unlocked(ctx, res.grant.tbs);
            results.push_back(res);
            metrics_collector::instance().record_ul_grant();
        }
    }
    return results;
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_priority(uint32_t tti) {
    std::vector<ul_sched_result> results;
    std::lock_guard<std::mutex> lock(mutex_);
    prb_reset_unlocked();

    // 重传优先
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.pending_retx_pid != 0xFFFF) {
            ul_sched_result res;
            res.rnti = rnti;
            res.grant = generate_ul_grant_unlocked(rnti, tti, ctx.pending_retx_pid, true);
            res.is_retx = true;
            if (res.grant.tbs > 0) {
                ctx.pending_retx_pid = 0xFFFF;
                results.push_back(res);
                metrics_collector::instance().record_ul_grant();
            }
        }
    }
    // 按缓冲区大小排序新传
    std::vector<std::pair<uint16_t, uint32_t>> sorted;
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.total_ul_buffer > 0 || ctx.sr_pending) {
            sorted.push_back({rnti, ctx.total_ul_buffer});
        }
    }
    std::sort(sorted.begin(), sorted.end(),
        [](const auto& a, const auto& b) { return a.second > b.second; });
    for (const auto& [rnti, buf] : sorted) {
        (void)buf;
        auto& ctx = ue_db_[rnti];
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue;
        ul_sched_result res;
        res.rnti = rnti;
        res.grant = generate_ul_grant_unlocked(rnti, tti, static_cast<uint32_t>(pid), false);
        res.is_retx = false;
        if (res.grant.tbs > 0) {
            ctx.harq_pid_busy[pid] = true;
            ctx.sr_pending = false;
            deduct_ul_buffer_unlocked(ctx, res.grant.tbs);
            results.push_back(res);
            metrics_collector::instance().record_ul_grant();
        }
    }
    return results;
}

size_t ul_scheduler::get_nof_ues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ue_db_.size();
}

const ue_sched_context* ul_scheduler::get_ue_context(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return nullptr;
    return &it->second;
}

} // namespace ul_mac
