// =============================================================================
// enb_ul_harq_manager.cpp - eNB侧上行接收HARQ管理器实现
//
// 接收端HARQ: PUSCH接收 -> 软合并(IR) -> CRC -> PHICH(ACK/NACK) -> 重传/释放
// =============================================================================

#include "ul_mac/enb_ul_harq_manager.h"

namespace ul_mac {

// 软合并模型常量 (SNR_MCS_THRESHOLD / IR_GAIN_X100 / DECODE_MARGIN_X100)
// 已统一定义于 common_types.h, 此处直接复用, 避免多文件重复定义。

enb_ul_harq_manager::enb_ul_harq_manager(uint32_t max_harq_tx)
    : max_harq_tx_(max_harq_tx)
{
    LOG_INFO("ENB_HARQ", 0, 0,
        "eNB UL HARQ (rx) initialized, max_harq_tx=" + std::to_string(max_harq_tx));
}

void enb_ul_harq_manager::set_config(uint32_t max_harq_tx) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_harq_tx_ = max_harq_tx;
}

void enb_ul_harq_manager::add_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ue_db_.count(rnti) > 0) return;
    ue_db_[rnti]; // 默认构造 ue_rx_entity
    LOG_INFO("ENB_HARQ", rnti, 0, "UE added to eNB HARQ");
}

void enb_ul_harq_manager::remove_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    ue_db_.erase(rnti);
}

void enb_ul_harq_manager::set_ul_snr(uint16_t rnti, int32_t snr_x100) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.ul_snr_x100 = snr_x100;
}

int32_t enb_ul_harq_manager::decode_threshold_x100(uint8_t mcs) const {
    if (mcs > 28) mcs = 28;
    return SNR_MCS_THRESHOLD[mcs] + DECODE_MARGIN_X100;
}

int32_t enb_ul_harq_manager::effective_snr_x100(
    int32_t ul_snr, uint32_t combined_count) const {
    // combined_count=1 (新传) 时增益为0; 每次额外合并 +IR_GAIN
    return ul_snr + static_cast<int32_t>(combined_count - 1) * IR_GAIN_X100;
}

void enb_ul_harq_manager::release_process_unlocked(enb_rx_process& p) {
    p.state = harq_state::INACTIVE;
    p.ndi_known = false;
    p.tx_count = 0;
    p.combined_count = 0;
    p.cur_mcs = 0;
    p.cur_tbs = 0;
    // 保留 last_crc_ok 供 get_phich() 查询最近一次反馈
}

enb_ul_harq_manager::rx_result enb_ul_harq_manager::receive_tb(
    uint16_t rnti, const ul_grant& grant) {
    std::lock_guard<std::mutex> lock(mutex_);
    rx_result res;

    auto uit = ue_db_.find(rnti);
    if (uit == ue_db_.end() || grant.pid >= MAX_HARQ_PROCESSES) {
        LOG_WARN("ENB_HARQ", rnti, grant.tti_tx,
            "receive_tb: unknown UE or invalid PID=" + std::to_string(grant.pid));
        return res;
    }
    auto& ue = uit->second;
    auto& p = ue.procs[grant.pid];

    // 步骤1: 判定新TB vs 重传 (NDI翻转 = 新TB, 对应3GPP TS 36.321 §5.4.2.1)
    bool is_new_tb = false;
    if (grant.ndi_present) {
        if (!p.ndi_known || grant.ndi != p.cur_ndi) {
            is_new_tb = true;
        }
    } else {
        // 无NDI (非自适应重传): 进程非INACTIVE视为重传
        is_new_tb = (p.state == harq_state::INACTIVE);
    }

    if (is_new_tb) {
        p.ndi_known = true;
        p.cur_ndi = grant.ndi;
        p.cur_mcs = grant.mcs;
        p.cur_tbs = grant.tbs;
        p.tx_count = 1;
        p.combined_count = 1;
        ue.stats.total_new_tb++;
        total_stats_.total_new_tb++;
    } else {
        p.tx_count++;
        p.combined_count++;
        res.soft_combined = true;
        ue.stats.total_soft_combine++;
        total_stats_.total_soft_combine++;
    }

    // 步骤2: 软合并 + CRC解码
    // 【协议说明 / 简化实现】
    // 真实 eNB 接收端: 把当前 TB 的软比特与历史软缓冲 (enb_rx_process 的 soft buffer)
    // 做 Max-Log-MAP 合并后再 turbo 解码得到 CRC 结果。
    // 本项目不做真实编码/解码, 用 "有效 SNR 模型" 等效表达软合并增益:
    //   eff_snr = ul_snr + (combined_count-1)*IR_GAIN, 再与 MCS 解码阈值比较得到 crc_ok。
    // 这样可在无真实编译码器的情况下演示 IR 增益 (重传累积后边缘 MCS 也能解调)。
    int32_t eff = effective_snr_x100(ue.ul_snr_x100, p.combined_count);
    int32_t threshold = decode_threshold_x100(p.cur_mcs);
    bool crc_ok = (eff >= threshold);

    // 填充结果 (在释放进程前捕获, 避免release后tx_count归零)
    res.crc_ok = crc_ok;
    res.is_new_tb = is_new_tb;
    res.tx_count = p.tx_count;
    res.mcs = p.cur_mcs;
    res.tbs = p.cur_tbs;
    res.eff_snr_x100 = eff;
    p.last_crc_ok = crc_ok;
    p.last_tti = grant.tti_tx;

    ue.stats.total_rx++;
    total_stats_.total_rx++;

    // 步骤3: 状态转移 + PHICH反馈
    if (crc_ok) {
        ue.stats.total_ack++;
        total_stats_.total_ack++;
        release_process_unlocked(p); // ACK: 释放进程
        metrics_collector::instance().record_harq_new_tx(); // 复用计数: 成功传输
        LOG_INFO("ENB_HARQ", rnti, grant.tti_tx,
            "PID=" + std::to_string(grant.pid) +
            ": ACK (tx=" + std::to_string(res.tx_count) +
            (res.soft_combined ? ", combined" : "") +
            ", eff=" + std::to_string(eff) +
            ">=" + std::to_string(threshold) + ")");
    } else {
        ue.stats.total_nack++;
        total_stats_.total_nack++;
        if (p.tx_count >= max_harq_tx_) {
            // 达到最大重传, 丢弃TB
            res.discarded = true;
            ue.stats.total_discard++;
            total_stats_.total_discard++;
            metrics_collector::instance().record_harq_fail();
            LOG_ERROR("ENB_HARQ", rnti, grant.tti_tx,
                "PID=" + std::to_string(grant.pid) +
                ": NACK -> Discard (max_tx=" + std::to_string(max_harq_tx_) +
                ", eff=" + std::to_string(eff) +
                "<" + std::to_string(threshold) + ")");
            release_process_unlocked(p);
        } else {
            // NACK: 等待重传
            p.state = harq_state::RETX_PENDING;
            metrics_collector::instance().record_harq_retx();
            LOG_INFO("ENB_HARQ", rnti, grant.tti_tx,
                "PID=" + std::to_string(grant.pid) +
                ": NACK -> RetxPending (tx=" + std::to_string(res.tx_count) +
                ", eff=" + std::to_string(eff) +
                "<" + std::to_string(threshold) + ")");
        }
    }
    return res;
}

bool enb_ul_harq_manager::get_phich(uint16_t rnti, uint32_t pid) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto uit = ue_db_.find(rnti);
    if (uit == ue_db_.end() || pid >= MAX_HARQ_PROCESSES) return false;
    return uit->second.procs[pid].last_crc_ok;
}

bool enb_ul_harq_manager::has_retx_pending(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto uit = ue_db_.find(rnti);
    if (uit == ue_db_.end()) return false;
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; ++i) {
        if (uit->second.procs[i].state == harq_state::RETX_PENDING) return true;
    }
    return false;
}

int32_t enb_ul_harq_manager::get_retx_pid(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto uit = ue_db_.find(rnti);
    if (uit == ue_db_.end()) return -1;
    for (uint32_t i = 0; i < MAX_HARQ_PROCESSES; ++i) {
        if (uit->second.procs[i].state == harq_state::RETX_PENDING) {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

harq_state enb_ul_harq_manager::get_process_state(uint16_t rnti, uint32_t pid) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto uit = ue_db_.find(rnti);
    if (uit == ue_db_.end() || pid >= MAX_HARQ_PROCESSES) return harq_state::INACTIVE;
    return uit->second.procs[pid].state;
}

enb_ul_harq_manager::enb_harq_stats enb_ul_harq_manager::get_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_stats_;
}

enb_ul_harq_manager::enb_harq_stats enb_ul_harq_manager::get_ue_stats(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return enb_harq_stats();
    return it->second.stats;
}

size_t enb_ul_harq_manager::get_nof_ues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ue_db_.size();
}

} // namespace ul_mac
