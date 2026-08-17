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
#include <chrono>
#include <cmath>
#include <numeric>
#include <set>

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

void ul_scheduler::handle_sr(uint16_t rnti,
                              const std::array<uint32_t, NOF_LCGS>& pending_bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.sr_pending = true;
    // 【srsRAN 式简化】SR 触发时, UE 本地已算出待传字节数, 经内部接口直接告知 eNB 调度器
    // (等价 srsRAN handle_ul_bsr_indication), 使 schedule_ul 在 BSR CE 经 PUSCH 解包前就能
    // 按真实待传量分配资源, 无需"先发小 Grant 探测 BSR"的空口往返。
    // 注意: 这与真实协议时序不同——真实系统中 eNB 须等 UE 拿到 PUSCH 授权后发 BSR CE 才知道
    // 待传量 (见 enb_handle_bsr_pdu 的空口解包路径)。此处用 srsRAN 的"同进程内部捷径"建模。
    // SR 带来的 pending_bytes 是 UE 本地最新待传量(可能已累积多轮数据), 取"与 eNB 当前视图的
    // 较大值"作为预填: 既覆盖首次(视图为0)的情形, 又能在 UE 缓冲区持续增长时把最新需求带给
    // eNB, 避免"只在为0时填"导致的调度量被旧值锁死、后续数据饿死的问题。
    bool updated = false;
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        if (pending_bytes[i] > it->second.ul_buffer[i]) {
            it->second.ul_buffer[i] = pending_bytes[i];
            updated = true;
        }
    }
    if (updated) {
        it->second.total_ul_buffer = 0;
        for (uint32_t i = 0; i < NOF_LCGS; i++)
            it->second.total_ul_buffer += it->second.ul_buffer[i];
    }
    LOG_DEBUG("SCHED", rnti, 0, "SR received from UE (srsRAN-style pre-BSR fill)");
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
    auto& ctx = it->second;
    if (!crc_ok) {
        // NACK: 标记该进程待重传(位图, 支持多进程并发NACK), 进程保持占用
        ctx.pending_retx[pid] = true;
        ctx.harq_pid_busy[pid] = true;
        LOG_DEBUG("SCHED", rnti, 0, "UL CRC FAIL PID=" + std::to_string(pid));
    } else {
        // ACK: 清除待重传标记, 释放HARQ进程
        ctx.pending_retx[pid] = false;
        ctx.harq_pid_busy[pid] = false;
        ctx.harq_tb[pid].valid = false;  // 原始TB已成功, 重传信息失效
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
// SNR_MCS_THRESHOLD[29] (MCS->最小可解码SNR阈值) 已统一定义于 common_types.h,
// 此处直接复用, 与 enb_ul_harq_manager 共用同一张表, 避免重复定义导致的不一致。

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

uint8_t ul_scheduler::calculate_mcs(int32_t snr_x100) const {
    // 参数直接为 int32_t (SNR x100, 可为负). 负值合法(低SNR区间).
    int32_t snr = snr_x100;
    // 从高到低查找首个满足阈值的MCS索引, 实现区间映射
    for (int i = 28; i >= 0; --i) {
        if (snr >= SNR_MCS_THRESHOLD[i]) {
            return static_cast<uint8_t>(i);
        }
    }
    return 0;
}

uint8_t ul_scheduler::calculate_mcs_from_cqi(uint8_t cqi) const {
    // LTE CQI(0-15) -> MCS 简化映射 (近似 TS 36.213 Table 7.2.3-1)
    // CQI 0: 超出范围 (调用方应回退SNR); CQI k -> MCS (k-1)*2, 上限28
    // CQI 1->0, 2->2, 3->4 ... 15->28
    if (cqi == 0) return 0;
    uint32_t mcs = (static_cast<uint32_t>(cqi) - 1) * 2;
    return static_cast<uint8_t>(std::min<uint32_t>(mcs, 28));
}

uint32_t ul_scheduler::calculate_tbs(uint8_t mcs, uint32_t n_prb) const {
    // 【协议说明 / 简化实现】
    // 真实 3GPP TS 36.213 Table 7.1.7.2.1-1 的 TBS 由 (PRB数, MCS索引) 二维查表得到,
    // 且 TBS 只能取表中离散值 (一组 2 的幂附近的值), 并非任意整数。
    // 本项目用 "每 PRB 效率锚点 + 线性插值" 近似 TBS: 先算 bits/PRB 再乘 n_prb 除 8。
    // 优点: 连续可调, 适合演示; 缺点: 结果非标准离散 TBS, 不可用于互操作。
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
// 公共调度阶段 (重构: 原先在 schedule_pf/rr/epf 中三处重复)
// ---------------------------------------------------------------------------

void ul_scheduler::schedule_retx_first_unlocked(std::vector<ul_sched_result>& results,
                                                 uint32_t tti, const char* algo_name) {
    // 重传优先 (HARQ RTT 内的 TB 必须先发完, 对应 3GPP UL HARQ 语义)
    for (auto& [rnti, ctx] : ue_db_) {
        for (uint32_t pid = 0; pid < MAX_HARQ_PROCESSES; ++pid) {
            if (!ctx.pending_retx[pid]) continue;
            // 【健壮性守卫】原始新传TB记录缺失(异常): 无法锁定TBS/MCS重传。
            // 若不清理, pending_retx 会每TTI空转重试永不释放 (死循环缺陷)。
            // 此处丢弃该TB并释放进程, 交由上层(RLC ARQ)语义兜底。
            if (!ctx.harq_tb[pid].valid) {
                ctx.pending_retx[pid] = false;
                ctx.harq_pid_busy[pid] = false;
                metrics_collector::instance().record_harq_fail();
                LOG_WARN("SCHED", rnti, tti,
                    std::string(algo_name) + " RETX DROP: PID=" + std::to_string(pid) +
                    " has no valid TB record, releasing process");
                continue;
            }
            ul_sched_result res;
            res.rnti = rnti;
            res.grant = generate_ul_grant_unlocked(rnti, tti, pid, true);
            res.is_retx = true;
            if (res.grant.tbs > 0) {
                // 已调度重传: 清除待重传标记, 进程保持忙直到收到ACK/NACK
                ctx.pending_retx[pid] = false;
                results.push_back(res);
                metrics_collector::instance().record_ul_grant();
                LOG_INFO("SCHED", rnti, tti,
                    std::string(algo_name) + " RETX: PID=" + std::to_string(res.grant.pid) +
                    ", TBS=" + std::to_string(res.grant.tbs) + "B");
                // 【每 UE 每 TTI 至多一条授权】LTE 中一个 UE 每子帧至多收到一条
                // UL grant (一条 DCI0); 多个 NACK 进程应跨 TTI 轮流重传,
                // 故本 UE 拿到一条重传授权后即跳出进程循环。
                // 【饿死计时重置】重传同样占用该 UE 的 PRB 资源, 它并非"未被调度";
                // EPF 的 tti_since_sched 在持续重传期间累积会导致弱信道 UE 被误判
                // 濒临饿死, 保底授权与重传同 TTI 叠加、双重挤占 PRB (PF/RR 不读
                // 此字段, 重置对它们无影响)。
                ctx.tti_since_sched = 0;
                break;
            }
            // tbs==0: 本TTI无连续PRB可用 (暂时性), 保留 pending_retx 下TTI再试
        }
    }
}

void ul_scheduler::commit_new_grant_unlocked(ue_sched_context& ctx, int32_t pid,
                                              ul_sched_result& res) {
    // 新传占用该HARQ进程, 等CRC反馈释放
    ctx.harq_pid_busy[static_cast<uint32_t>(pid)] = true;
    // 实际获得授权后才清除SR标志
    ctx.sr_pending = false;
    // 授权发出后立即扣减缓冲区估计, 下次BSR到达时会重新校准
    deduct_ul_buffer_unlocked(ctx, res.grant.tbs);
    // PF平均速率EMA更新 (EPF 的 R_avg 也复用此字段)
    double alpha = 1.0 / (ctx.ul_nof_samples + 1);
    ctx.ul_avg_rate = (1.0 - alpha) * ctx.ul_avg_rate + alpha * res.grant.tbs;
    ctx.ul_nof_samples++;
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

    ul_grant grant;
    grant.rnti = rnti;
    grant.pid = pid;

    uint8_t mcs = (ctx.cqi > 0) ? calculate_mcs_from_cqi(ctx.cqi)
                                : calculate_mcs(ctx.ul_snr);
    uint32_t n_prb = 0;
    uint32_t tbs = 0;

    if (is_retx) {
        // 重传: TBS/MCS/PRB必须锁定为原始新传值 (TS 36.321 §5.4.2.1)
        // 保证同一TB的IR软合并前提; NDI不翻转, RV由UE侧递进.
        // 【协议说明 / 简化实现】
        // 真实协议的"自适应重传"允许 eNB 在 DCI0 中为新传/重传重新选择 MCS/RV/PRB
        // (如信道恶化时降 MCS, 或重传用更高 RV)。本项目为简化, 重传一律锁定为原始
        // 新传的 MCS/PRB (视为"非自适应重传"), 仅 NDI 保持、RV 由 UE 内部序列推进。
        // 这种锁定的非自适应重传是 3GPP 允许且最常见的方式, 故不失协议正确性。
        const auto& tb = ctx.harq_tb[pid];
        if (!tb.valid) {
            // 没有记录原始TB信息(异常), 退回空grant, 避免产生不一致的重传
            return ul_grant();
        }
        n_prb = tb.n_prb;
        mcs = tb.mcs;
        // 从PRB池首次适配分配连续区间
        int32_t start = prb_alloc_unlocked(n_prb);
        if (start < 0) return ul_grant(); // 本TTI无连续PRB可用
        grant.prb_start = static_cast<uint32_t>(start);
        tbs = tb.tbs;
        grant.ndi = ctx.ndi[pid];
        grant.rv = -1;
    } else {
        uint32_t req_bytes = ctx.total_ul_buffer;
        if (req_bytes == 0) req_bytes = 100; // 仅够承载 BSR CE + subheader
        // 用与 calculate_tbs 一致的每PRB效率反推所需PRB数, 保证 PRB/TBS 自洽
        uint32_t bytes_per_prb = calculate_tbs(mcs, 1);
        if (bytes_per_prb == 0) bytes_per_prb = 1;
        n_prb = (req_bytes + bytes_per_prb - 1) / bytes_per_prb;
        n_prb = std::min(n_prb, total_prb_);
        // 从PRB池首次适配分配连续区间
        int32_t start = prb_alloc_unlocked(n_prb);
        if (start < 0) return ul_grant(); // 本TTI无连续PRB可用
        grant.prb_start = static_cast<uint32_t>(start);
        tbs = calculate_tbs(mcs, n_prb);
        // 记录本次新传的TB信息, 供后续重传回放 (锁定TBS)
        ctx.harq_tb[pid].tbs = tbs;
        ctx.harq_tb[pid].mcs = mcs;
        ctx.harq_tb[pid].n_prb = n_prb;
        ctx.harq_tb[pid].valid = true;
        // NDI管理 (3GPP TS 36.321 §5.4.2.1): 新传翻转NDI
        ctx.ndi[pid] = !ctx.ndi[pid];
        grant.ndi = ctx.ndi[pid];
        grant.rv = 0;
    }

    grant.tbs = tbs;
    grant.n_prb = n_prb;
    grant.mcs = mcs;
    grant.ndi_present = true;
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
    // 实时性度量: 测量本次调度决策的执行耗时 (微秒)
    auto t0 = std::chrono::steady_clock::now();
    // algorithm_ 可能被 set_algorithm 并发写入, 锁内取值
    sched_algorithm algo;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        algo = algorithm_;
    }
    std::vector<ul_sched_result> results;
    switch (algo) {
        case sched_algorithm::PROPORTIONAL_FAIR: results = schedule_pf(tti); break;
        case sched_algorithm::ROUND_ROBIN:       results = schedule_rr(tti); break;
        case sched_algorithm::EPF:               results = schedule_epf(tti); break;
        default:                                 results = schedule_pf(tti); break;
    }
    auto t1 = std::chrono::steady_clock::now();
    uint64_t us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        sched_latency_samples_.push_back(us);
        // 【健壮性】限制采样内存: 超长仿真下避免无限增长 (丢弃最旧一半,
        // 摊还 O(1); 统计量仍反映近期调度耗时分布)
        constexpr size_t MAX_LATENCY_SAMPLES = 200000;
        if (sched_latency_samples_.size() > MAX_LATENCY_SAMPLES) {
            sched_latency_samples_.erase(
                sched_latency_samples_.begin(),
                sched_latency_samples_.begin() + MAX_LATENCY_SAMPLES / 2);
        }
    }
    return results;
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_pf(uint32_t tti) {
    // 比例公平调度: 重传优先 > PF度量值排序新传
    std::vector<ul_sched_result> results;
    std::lock_guard<std::mutex> lock(mutex_);
    prb_reset_unlocked();

    // 阶段1: 重传优先 (公共函数, 原三处重复逻辑已抽取)
    schedule_retx_first_unlocked(results, tti, "PF");

    // 本 TTI 已获 (重传) 授权的 UE: 新传阶段跳过 (LTE 每 UE 每子帧至多一条 grant)
    std::set<uint16_t> granted;
    for (const auto& r : results) granted.insert(r.rnti);

    // 阶段2: 计算PF度量值
    struct ue_pf { uint16_t rnti; double metric; };
    std::vector<ue_pf> queue;
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue;
        // 标准PF: 分子 R_current 为信道可支持瞬时速率 (由CQI/SNR决定的MCS推导)
        uint8_t mcs = (ctx.cqi > 0) ? calculate_mcs_from_cqi(ctx.cqi)
                                    : calculate_mcs(ctx.ul_snr);
        double achievable = static_cast<double>(calculate_tbs(mcs, total_prb_));
        // 与缓冲区需求取小, 避免无数据UE被空转调度; 同时反映"有多少需要发"
        double demand_capped = std::min(achievable,
                                        static_cast<double>(ctx.total_ul_buffer));
        double avg = (ctx.ul_avg_rate > 0) ? ctx.ul_avg_rate : 1.0;
        double pf = demand_capped / std::pow(avg, fairness_coeff_);
        queue.push_back({rnti, pf});
    }
    std::sort(queue.begin(), queue.end(),
        [](const ue_pf& a, const ue_pf& b) { return a.metric > b.metric; });

    // 阶段3: 按PF顺序分配
    for (const auto& e : queue) {
        if (granted.count(e.rnti)) continue;   // 本 TTI 已有重传授权
        auto& ctx = ue_db_[e.rnti];
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue; // 该UE所有HARQ进程都在忙, 跳过
        ul_sched_result res;
        res.rnti = e.rnti;
        res.grant = generate_ul_grant_unlocked(e.rnti, tti, static_cast<uint32_t>(pid), false);
        res.is_retx = false;
        if (res.grant.tbs > 0) {
            commit_new_grant_unlocked(ctx, pid, res);
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

    // 阶段1: 重传优先 (公共函数, 原三处重复逻辑已抽取)
    schedule_retx_first_unlocked(results, tti, "RR");

    // 本 TTI 已获 (重传) 授权的 UE: 新传阶段跳过 (LTE 每 UE 每子帧至多一条 grant)
    std::set<uint16_t> granted;
    for (const auto& r : results) granted.insert(r.rnti);

    // 阶段2: 轮询新传 — 真正的 RR 需每 TTI 轮转起点。ue_db_ 为 std::map (按 RNTI
    // 升序), 若每 TTI 都从头部遍历, 低 RNTI 恒先获得 PRB, 高 RNTI 在 PRB
    // 紧张时会被持续饿死 —— 名不副实的"轮询"。此处以 rr_offset_ 作轮转
    // 起点 (每 TTI +1), 保证各 UE 的遍历优先级随时间均等轮换。
    std::vector<uint16_t> order;
    order.reserve(ue_db_.size());
    for (const auto& [rnti, ctx] : ue_db_) { order.push_back(rnti); }
    if (!order.empty()) {
        size_t n = order.size();
        size_t start = static_cast<size_t>(rr_offset_ % n);
        for (size_t k = 0; k < n; ++k) {
            uint16_t rnti = order[(start + k) % n];
            if (granted.count(rnti)) continue;   // 本 TTI 已有重传授权
            auto& ctx = ue_db_[rnti];
            if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue;
            int32_t pid = alloc_free_pid_unlocked(ctx);
            if (pid < 0) continue;
            ul_sched_result res;
            res.rnti = rnti;
            res.grant = generate_ul_grant_unlocked(rnti, tti, static_cast<uint32_t>(pid), false);
            res.is_retx = false;
            if (res.grant.tbs > 0) {
                commit_new_grant_unlocked(ctx, pid, res);
                results.push_back(res);
                metrics_collector::instance().record_ul_grant();
            }
        }
        rr_offset_++;
    }
    return results;
}

size_t ul_scheduler::get_nof_ues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ue_db_.size();
}

ul_scheduler::sched_latency_stats ul_scheduler::get_sched_latency_stats() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    sched_latency_stats s;
    if (sched_latency_samples_.empty()) return s;
    // 复制后排序计算分位数 (避免修改采样序列)
    std::vector<uint64_t> sorted = sched_latency_samples_;
    std::sort(sorted.begin(), sorted.end());
    s.count  = sorted.size();
    s.min_us = static_cast<double>(sorted.front());
    s.max_us = static_cast<double>(sorted.back());
    s.avg_us = static_cast<double>(std::accumulate(sorted.begin(), sorted.end(), 0ULL))
               / static_cast<double>(sorted.size());
    // P50 / P99: 线性插值分位数 (nearest-rank 风格)
    auto q = [&](double p) -> double {
        size_t idx = static_cast<size_t>(std::ceil(p * sorted.size())) - 1;
        if (idx >= sorted.size()) idx = sorted.size() - 1;
        return static_cast<double>(sorted[idx]);
    };
    s.p50_us = q(0.50);
    s.p99_us = q(0.99);
    return s;
}

std::optional<ue_sched_context> ul_scheduler::get_ue_context(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return std::nullopt;
    return it->second;  // 值拷贝, 锁内完成, 调用方无需持锁, 无悬垂指针风险
}

// ---------------------------------------------------------------------------
// EPF (Enhanced Proportional Fair) 调度
//  度量: metric = w_qos * (R_instant / R_avg^alpha) * (1 + beta * cqi_norm)
//  含饿死保护: 长期未调度 UE 度量放大, 且保留最低 PRB 份额
// ---------------------------------------------------------------------------

double ul_scheduler::compute_epf_metric(const ue_sched_context& ctx, uint32_t /*tti*/) const {
    // R_instant: 当前 TTI 按 CQI/SNR 可支持的瞬时速率 (以 total_prb 计),
    // 并与实际待发量取小 (需求封顶, 与 schedule_pf 的 demand_capped 一致)。
    // 【修复】原实现未封顶, 小缓冲 UE (如 VoIP 100B) 与大缓冲 UE (视频 800B+)
    // 以相同满 PRB 速率参与度量竞争, 调度份额只由 QoS 权重与信道项决定,
    // 与业务量完全脱钩 —— 度量高的 UE 被反复授权但每次只传极少数据,
    // 造成大量小 TBS 空转授权 (观测: 平均 170B/授权)。封顶后度量与
    // "实际可发送的量"成正比, 份额回归业务量 (仍保留 QoS 权重倾斜)。
    uint8_t mcs = (ctx.cqi > 0) ? calculate_mcs_from_cqi(ctx.cqi)
                                : calculate_mcs(ctx.ul_snr);
    double r_inst = static_cast<double>(calculate_tbs(mcs, total_prb_));
    r_inst = std::min(r_inst, static_cast<double>(ctx.total_ul_buffer));

    // R_avg: 长期平均吞吐 (指数滑动平均), 为 0 时用极小值避免除零 (新用户优先)
    double r_avg = ctx.ul_avg_rate;
    if (r_avg < 1.0) r_avg = 1.0;

    // cqi_norm: 归一化信道质量 [0,1]; CQI=0 时由 SNR 推导的 MCS 近似归一
    double cqi_norm;
    if (ctx.cqi > 0) {
        cqi_norm = static_cast<double>(ctx.cqi) / static_cast<double>(epf_.cqi_max);
    } else {
        // SNR 区间约 [-10dB, 20dB] 映射至 [0,1]
        cqi_norm = std::max(0.0, std::min(1.0,
            (static_cast<double>(ctx.ul_snr) + 1000.0) / 3000.0));
    }

    // QoS 权重 (全局缩放 gamma * 业务权重)
    double w_qos = epf_.gamma * ctx.qos.weight;

    // 核心度量 (PF 项取 alpha 次幂; 信道感知项 (1 + beta*cqi_norm))
    double pf_term = r_inst / std::pow(r_avg, epf_.alpha);
    double metric = w_qos * pf_term * (1.0 + epf_.beta * cqi_norm);

    // 饿死保护: 距上次调度过久 -> 度量放大, 保证最低调度机会
    // (注意: 持续重传的 UE 已在 schedule_retx_first 中重置 tti_since_sched,
    //  不会被误判濒临饿死; 此处针对的是真正长期未获新传机会的 UE)
    if (ctx.tti_since_sched > epf_.starve_tti) {
        metric *= 10.0;
    }
    return metric;
}

void ul_scheduler::set_ue_qos(uint16_t rnti, qos_class cls) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.set_qos(cls);
    LOG_INFO("SCHED", rnti, 0,
        "UE QoS set: " + std::to_string(static_cast<int>(cls)) +
        ", weight=" + std::to_string(it->second.qos.weight));
}

double ul_scheduler::get_epf_metric(uint16_t rnti, uint32_t tti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return 0.0;
    return compute_epf_metric(it->second, tti);
}

std::vector<ul_scheduler::ul_sched_result> ul_scheduler::schedule_epf(uint32_t tti) {
    // EPF: 重传优先 > 饿死保护保底分配 > EPF度量值排序新传
    std::vector<ul_sched_result> results;
    std::lock_guard<std::mutex> lock(mutex_);
    prb_reset_unlocked();

    // 每 TTI 推进所有 UE 的"距上次调度"计数 (饿死保护计时器)
    for (auto& [rnti, ctx] : ue_db_) {
        ctx.tti_since_sched += 1;
    }

    // 阶段1: 重传优先 (公共函数, 原三处重复逻辑已抽取)
    schedule_retx_first_unlocked(results, tti, "EPF");

    // 【每 UE 每 TTI 至多一条授权】LTE 中一个 UE 每子帧至多收到一条 UL grant
    // (一条 DCI0)。本集合记录本 TTI 已获授权 (重传/保底/新传) 的 UE,
    // 后续阶段一律跳过 —— 修复原实现中"阶段3 保底 + 阶段4 新传"可对同一 UE
    // 同 TTI 重复授权、双倍消耗 PRB 的缺陷。
    std::set<uint16_t> granted;
    for (const auto& r : results) granted.insert(r.rnti);

    // 阶段2: 计算 EPF 度量并排序
    struct ue_epf { uint16_t rnti; double metric; };
    std::vector<ue_epf> queue;
    for (auto& [rnti, ctx] : ue_db_) {
        if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue;
        // 度量内部已按"实际待发量"封顶 R_instant (与 PF 一致), 避免空转调度
        double m = compute_epf_metric(ctx, tti);
        queue.push_back({rnti, m});
    }
    std::sort(queue.begin(), queue.end(),
        [](const ue_epf& a, const ue_epf& b) { return a.metric > b.metric; });

    // 阶段3: 饿死保护保底分配 (EPF 增强, 非 3GPP 标准机制)
    //  【协议澄清】3GPP 标准上行调度无"min_prb_ratio 保底份额"概念; 比例公平本身
    //  已通过 R_avg 提供长期公平。本保底分配是华为 EPF 风格的资源保障增强, 用于
    //  保证弱信道/小缓冲 UE 不被好信道 UE 长期挤占。
    //  为"濒临饿死"的 UE 在 PRB 池中先预留 min_prb_ratio 份额, 防止被大缓冲/好信道 UE 挤占
    uint32_t min_prb_total = static_cast<uint32_t>(
        std::ceil(epf_.min_prb_ratio * static_cast<double>(total_prb_)));
    for (const auto& e : queue) {
        if (granted.count(e.rnti)) continue;   // 本 TTI 已有 (重传) 授权
        auto& ctx = ue_db_[e.rnti];
        bool starving = ctx.tti_since_sched > epf_.starve_tti;
        if (!starving) continue;
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue;
        // 保底份额按实际需求封顶: 保底语义是"份额不被好信道 UE 挤占",
        // 若 UE 实际待发量小于保底份额, 只按需求给 PRB —— 修复原实现固定
        // 给满 min_prb_total (10 PRB ~ 数百字节) 而不顾小缓冲浪费 PRB 的缺陷。
        uint8_t mcs = (ctx.cqi > 0) ? calculate_mcs_from_cqi(ctx.cqi)
                                    : calculate_mcs(ctx.ul_snr);
        uint32_t req_bytes = (ctx.total_ul_buffer > 0) ? ctx.total_ul_buffer : 100;
        uint32_t bytes_per_prb = calculate_tbs(mcs, 1);
        if (bytes_per_prb == 0) bytes_per_prb = 1;
        uint32_t n = (req_bytes + bytes_per_prb - 1) / bytes_per_prb; // 需求所需 PRB
        n = std::min(n, min_prb_total);   // 不超过保底份额
        n = std::max(n, 1u);
        n = std::min(n, total_prb_);
        // 从PRB池首次适配分配连续区间
        int32_t start = prb_alloc_unlocked(n);
        if (start < 0) continue; // 无连续 PRB, 跳过保底
        ul_grant grant;
        grant.rnti = e.rnti;
        grant.pid = static_cast<uint32_t>(pid);
        grant.n_prb = n;
        grant.prb_start = static_cast<uint32_t>(start);
        grant.mcs = mcs;
        grant.tbs = calculate_tbs(mcs, n);
        // 保底分配视为"新传", 按标准翻转 NDI (与 generate_ul_grant_unlocked 新传分支一致)。
        // 此处手动翻转而非复用 generate_ul_grant_unlocked, 是因为保底按需求封顶 PRB,
        // 不走"缓冲区需求反推满额 PRB"的路径, 故独立构造授权。
        grant.ndi = !ctx.ndi[pid];
        ctx.ndi[pid] = grant.ndi;
        grant.ndi_present = true;
        grant.rv = 0;
        grant.tti_tx = tti;
        ctx.harq_tb[pid].tbs = grant.tbs;
        ctx.harq_tb[pid].mcs = mcs;
        ctx.harq_tb[pid].n_prb = n;
        ctx.harq_tb[pid].valid = true;
        ctx.inst_rate = static_cast<double>(grant.tbs);
        ctx.tti_since_sched = 0;
        ul_sched_result res;
        res.rnti = e.rnti;
        res.grant = grant;
        res.is_retx = false;
        // 保底授权提交: 占用进程/清SR/扣缓冲/更新平均速率 (公共函数)
        commit_new_grant_unlocked(ctx, pid, res);
        granted.insert(e.rnti);
        results.push_back(res);
        metrics_collector::instance().record_ul_grant();
        LOG_INFO("SCHED", e.rnti, tti,
            "EPF STARVE-GUARD: PRB=" + std::to_string(n) +
            ", MCS=" + std::to_string(mcs) +
            ", TBS=" + std::to_string(grant.tbs) + "B");
    }

    // 阶段4: EPF 度量排序的新传分配 (剩余 PRB)
    for (const auto& e : queue) {
        if (granted.count(e.rnti)) continue;   // 本 TTI 已有授权 (重传/保底)
        auto& ctx = ue_db_[e.rnti];
        if (ctx.total_ul_buffer == 0 && !ctx.sr_pending) continue; // 已被保底清空
        int32_t pid = alloc_free_pid_unlocked(ctx);
        if (pid < 0) continue; // 该UE所有HARQ进程忙, 跳过
        ul_sched_result res;
        res.rnti = e.rnti;
        res.grant = generate_ul_grant_unlocked(e.rnti, tti, static_cast<uint32_t>(pid), false);
        res.is_retx = false;
        if (res.grant.tbs > 0) {
            commit_new_grant_unlocked(ctx, pid, res);
            ctx.inst_rate = static_cast<double>(res.grant.tbs);
            ctx.tti_since_sched = 0;
            granted.insert(e.rnti);
            results.push_back(res);
            metrics_collector::instance().record_ul_grant();
            LOG_INFO("SCHED", e.rnti, tti,
                "EPF NEW: PID=" + std::to_string(res.grant.pid) +
                ", MCS=" + std::to_string(res.grant.mcs) +
                ", TBS=" + std::to_string(res.grant.tbs) + "B" +
                ", PRB[" + std::to_string(res.grant.prb_start) + "+" +
                std::to_string(res.grant.n_prb) + "]" +
                ", metric=" + std::to_string(e.metric));
        }
    }
    return results;
}

} // namespace ul_mac
