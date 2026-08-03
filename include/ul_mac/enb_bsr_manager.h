// =============================================================================
// enb_bsr_manager.h - eNB侧上行BSR解码器
//
// 与 UE 侧 bsr_manager (发送端/编码器) 镜像, 实现 eNB 接收端 BSR 解码:
//   1. 接收UE上报的 BSR MAC CE (Short / Truncated / Long)
//   2. 解码 6-bit 缓冲区大小索引 → 字节数 (复用 bsr_index_to_bytes)
//   3. 维护 per-UE LCG 缓冲区视图 (lcg_id → bytes), 供调度器查询
//
// 【解码语义】(对应 UE 编码器 bsr_manager::generate_bsr):
//   - Short BSR / Truncated BSR: 仅 1 个 LCG 报告
//       → 只更新该 LCG, 不影响其它 LCG (其它 LCG 状态未知, 不清零)
//   - Long BSR: 报告所有有数据的 LCG (UE 端编码时跳过 buffer=0 的 LCG)
//       → 先清零所有 LCG, 再填充报告的 LCG (未报告视为 0)
//
// 【设计模式】与 enb_ul_harq_manager 一致:
//   多UE / add_ue / remove_ue / std::map<uint16_t, ue_xxx> / mutable mutex
//   per-UE stats + total stats / LOG_INFO/LOG_DEBUG/LOG_ERROR
//
// 关键参考:
//   - srsRAN_4G/srsenb 中的 BSR 接收解码 (与 UE 侧 proc_bsr.cc 对称)
//   - 3GPP TS 36.321 Section 5.4.5 (BSR 过程, 收发两侧对称)
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <array>
#include <map>
#include <mutex>

namespace ul_mac {

/// eNB侧BSR解码器 (多UE)
///
/// 镜像 UE 侧 bsr_manager (编码器), 实现接收端解码 + per-UE LCG 缓冲区视图
class enb_bsr_manager {
public:
    /// eNB BSR统计 (per-UE 与全局共用此结构)
    struct enb_bsr_stats {
        uint32_t total_bsr_rx;        ///< 接收BSR总数
        uint32_t short_count;         ///< Short BSR 次数
        uint32_t truncated_count;     ///< Truncated BSR 次数
        uint32_t long_count;          ///< Long BSR 次数

        enb_bsr_stats()
            : total_bsr_rx(0), short_count(0)
            , truncated_count(0), long_count(0) {}
    };

    /// 构造函数
    enb_bsr_manager();

    /// 添加UE
    void add_ue(uint16_t rnti);

    /// 移除UE
    void remove_ue(uint16_t rnti);

    /// 核心: 接收并解码 BSR MAC CE, 更新该UE的LCG缓冲区视图
    /// @param rnti UE的C-RNTI
    /// @param bsr  UE上报的BSR控制元素 (由UE侧bsr_manager编码产生)
    /// @return true=解码成功并更新视图; false=UE未注册或空BSR
    bool receive_bsr(uint16_t rnti, const bsr_ce& bsr);

    /// 查询某UE某LCG的缓冲区大小 (bytes)
    /// @return 字节数; UE未注册时返回0
    uint32_t get_ul_buffer(uint16_t rnti, uint8_t lcg_id) const;

    /// 查询某UE总缓冲区大小 (所有LCG之和, bytes)
    /// 供调度器按总缓冲区排队
    uint32_t get_total_buffer(uint16_t rnti) const;

    /// 获取某UE所有LCG缓冲区视图 (4个LCG)
    /// @return array<bytes, 4>; UE未注册时返回全0
    std::array<uint32_t, NOF_LCGS> get_ue_lcg_view(uint16_t rnti) const;

    /// 重置某UE的LCG缓冲区视图 (不清除统计)
    void reset_ue(uint16_t rnti);

    /// 获取全局统计
    enb_bsr_stats get_stats() const;

    /// 获取指定UE统计
    enb_bsr_stats get_ue_stats(uint16_t rnti) const;

    /// 获取已注册UE数量
    size_t get_nof_ues() const;

private:
    /// 单个UE的BSR接收实体
    struct ue_bsr_entity {
        std::array<uint32_t, NOF_LCGS> lcg_buffer; ///< 各LCG缓冲区视图 (bytes)
        enb_bsr_stats stats;

        ue_bsr_entity() {
            lcg_buffer.fill(0);
        }
    };

    mutable std::mutex mutex_;
    std::map<uint16_t, ue_bsr_entity> ue_db_;
    enb_bsr_stats total_stats_;
};

} // namespace ul_mac
