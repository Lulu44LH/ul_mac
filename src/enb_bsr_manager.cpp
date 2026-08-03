// =============================================================================
// enb_bsr_manager.cpp - eNB侧上行BSR解码器实现
//
// 接收端BSR: 解码 BSR MAC CE → 维护 per-UE LCG 缓冲区视图 → 供调度器查询
// =============================================================================

#include "ul_mac/enb_bsr_manager.h"

namespace ul_mac {

enb_bsr_manager::enb_bsr_manager() {
    LOG_INFO("ENB_BSR", 0, 0, "eNB BSR decoder initialized");
}

void enb_bsr_manager::add_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ue_db_.count(rnti) > 0) return;
    ue_db_[rnti]; // 默认构造 ue_bsr_entity (lcg_buffer 全0)
    LOG_INFO("ENB_BSR", rnti, 0, "UE added to eNB BSR decoder");
}

void enb_bsr_manager::remove_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    ue_db_.erase(rnti);
}

bool enb_bsr_manager::receive_bsr(uint16_t rnti, const bsr_ce& bsr) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) {
        LOG_ERROR("ENB_BSR", rnti, 0,
            "receive_bsr: UE not registered, ignore");
        return false;
    }
    if (bsr.reports.empty()) {
        LOG_WARN("ENB_BSR", rnti, 0, "receive_bsr: empty BSR reports, ignore");
        return false;
    }

    ue_bsr_entity& ent = it->second;
    auto& view = ent.lcg_buffer;

    // 解码语义 (镜像 UE 编码器 bsr_manager::generate_bsr):
    //   Short/Truncated BSR: 只 1 个 LCG 报告 → 仅更新该 LCG, 不影响其它
    //   Long BSR: UE 端只编码 buffer>0 的 LCG → 先清零所有, 再填充报告的
    if (bsr.format == bsr_format::LONG_BSR) {
        view.fill(0);
    }

    for (const auto& r : bsr.reports) {
        if (r.lcg_id >= NOF_LCGS) {
            LOG_WARN("ENB_BSR", rnti, 0,
                "receive_bsr: invalid lcg_id=" + std::to_string(r.lcg_id) +
                ", skip");
            continue;
        }
        // TS 36.321 §6.1.3.1: 索引 i 对应区间下界, eNB 按"缓冲区 >= table[i]"解读.
        // 此处取 table[i] 作为下界估计(保守, 不会高估导致过度授权).
        view[r.lcg_id] = bsr_index_to_bytes(r.buffer_size);
    }

    // 更新统计
    ent.stats.total_bsr_rx++;
    total_stats_.total_bsr_rx++;
    switch (bsr.format) {
        case bsr_format::SHORT_BSR:
            ent.stats.short_count++;
            total_stats_.short_count++;
            break;
        case bsr_format::TRUNCATED_BSR:
            ent.stats.truncated_count++;
            total_stats_.truncated_count++;
            break;
        case bsr_format::LONG_BSR:
            ent.stats.long_count++;
            total_stats_.long_count++;
            break;
    }
    metrics_collector::instance().record_bsr_tx(bsr.format);

    LOG_DEBUG("ENB_BSR", rnti, 0,
        "decoded " + bsr_format_to_string(bsr.format) +
        " (reports=" + std::to_string(bsr.reports.size()) +
        ", total_buf=" + std::to_string(
            view[0] + view[1] + view[2] + view[3]) + "B)");

    return true;
}

uint32_t enb_bsr_manager::get_ul_buffer(uint16_t rnti, uint8_t lcg_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end() || lcg_id >= NOF_LCGS) return 0;
    return it->second.lcg_buffer[lcg_id];
}

uint32_t enb_bsr_manager::get_total_buffer(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return 0;
    const auto& v = it->second.lcg_buffer;
    return v[0] + v[1] + v[2] + v[3];
}

std::array<uint32_t, NOF_LCGS> enb_bsr_manager::get_ue_lcg_view(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<uint32_t, NOF_LCGS> result;
    result.fill(0);
    auto it = ue_db_.find(rnti);
    if (it != ue_db_.end()) {
        result = it->second.lcg_buffer;
    }
    return result;
}

void enb_bsr_manager::reset_ue(uint16_t rnti) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return;
    it->second.lcg_buffer.fill(0);
    LOG_DEBUG("ENB_BSR", rnti, 0, "UE LCG buffer view reset");
}

enb_bsr_manager::enb_bsr_stats enb_bsr_manager::get_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_stats_;
}

enb_bsr_manager::enb_bsr_stats enb_bsr_manager::get_ue_stats(uint16_t rnti) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ue_db_.find(rnti);
    if (it == ue_db_.end()) return enb_bsr_stats();
    return it->second.stats;
}

size_t enb_bsr_manager::get_nof_ues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ue_db_.size();
}

} // namespace ul_mac
