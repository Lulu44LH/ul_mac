// =============================================================================
// enb_ul_harq_manager.h - eNB侧上行接收HARQ管理器
//
// 与 UE 侧 ul_harq_manager (发送端) 镜像, 实现 eNB 接收端 HARQ:
//   1. 接收PUSCH传输块 (TB), 做CRC校验
//   2. 软合并 (IR: 增量冗余) - 每次重传累积增益, 提升解码成功率
//   3. 产生PHICH反馈 (ACK/NACK) 给UE
//   4. 跟踪待重传进程, 达到最大重传次数则丢弃
//
// 【状态机复用】复用 harq_state 枚举, 接收端语义:
//   INACTIVE      - 进程空闲
//   WAITING_FB    - 已接收TB并完成本轮CRC, 等待下一轮 (瞬时态, receive_tb后即转移)
//   RETX_PENDING  - CRC失败, 等待调度器下发重传授权
//
// 【软合并模型】(演示级, 无真实编码比特):
//   eff_snr = ul_snr + (combined_count - 1) * IR_GAIN
//   crc_ok  = eff_snr >= decode_threshold(mcs)
//   decode_threshold(mcs) = 选择阈值 + DECODE_MARGIN (MCS边缘处新传易失败,
//   重传累积IR增益后解码成功 - 真实体现HARQ软合并价值)
//
// 关键参考:
//   - srsRAN_4G/srsenb/hdr/stack/mac/sched.h 中的 ul_crc_info() (eNB收CRC)
//   - srsRAN_4G/srsenb 中的 PHICH生成与HARQ软合并buffer
//   - 3GPP TS 36.321 Section 5.4.2 (HARQ过程, 收发两侧对称)
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <array>
#include <map>
#include <mutex>

namespace ul_mac {

/// eNB侧上行接收HARQ管理器 (多UE)
class enb_ul_harq_manager {
public:
    /// 接收PUSCH TB的结果
    struct rx_result {
        bool     crc_ok;         ///< CRC校验结果 (true=ACK, false=NACK), 即PHICH反馈值
        bool     is_new_tb;      ///< 是否为新TB (NDI翻转判定)
        uint32_t tx_count;       ///< 本TB累计传输次数 (含本次, 新传=1)
        bool     soft_combined;  ///< 本次是否发生软合并 (重传时为true)
        bool     discarded;      ///< 是否达到最大重传被丢弃
        uint8_t  mcs;            ///< 本次MCS
        uint32_t tbs;            ///< 本次TBS
        int32_t  eff_snr_x100;   ///< 软合并后有效SNR (x100 dB)

        rx_result()
            : crc_ok(false), is_new_tb(false), tx_count(0)
            , soft_combined(false), discarded(false)
            , mcs(0), tbs(0), eff_snr_x100(0) {}
    };

    /// eNB HARQ统计
    struct enb_harq_stats {
        uint32_t total_rx;           ///< 接收TB总数 (新传+重传)
        uint32_t total_new_tb;       ///< 新TB数 (NDI翻转次数)
        uint32_t total_ack;          ///< ACK次数
        uint32_t total_nack;         ///< NACK次数
        uint32_t total_discard;      ///< 丢弃次数 (达到最大重传)
        uint32_t total_soft_combine; ///< 软合并次数 (重传接收次数)

        enb_harq_stats()
            : total_rx(0), total_new_tb(0), total_ack(0)
            , total_nack(0), total_discard(0), total_soft_combine(0) {}
    };

    /// 构造函数
    /// @param max_harq_tx 单个TB最大传输次数 (新传+重传), 默认MAX_HARQ_RETX
    explicit enb_ul_harq_manager(uint32_t max_harq_tx = MAX_HARQ_RETX);

    /// 设置HARQ配置 (最大传输次数)
    void set_config(uint32_t max_harq_tx);

    /// 添加UE
    void add_ue(uint16_t rnti);

    /// 移除UE
    void remove_ue(uint16_t rnti);

    /// 设置UE上行信道质量 (x100 dB, 与ul_snr口径一致)
    /// 用于软合并解码模型; 默认200 (2dB)
    void set_ul_snr(uint16_t rnti, int32_t snr_x100);

    /// 核心: 接收PUSCH TB, 软合并 + CRC, 产生PHICH反馈
    /// @param grant 调度器下发的上行授权 (用 pid/ndi/ndi_present/mcs/tbs/tti_tx)
    /// @return 接收结果 (crc_ok 即为PHICH反馈值)
    rx_result receive_tb(uint16_t rnti, const ul_grant& grant);

    /// 查询某UE某进程最近一次PHICH反馈值
    bool get_phich(uint16_t rnti, uint32_t pid) const;

    /// 查询某UE是否有待重传进程
    bool has_retx_pending(uint16_t rnti) const;

    /// 获取某UE待重传进程ID (扫描第一个RETX_PENDING)
    /// @return 进程ID, -1表示无
    int32_t get_retx_pid(uint16_t rnti) const;

    /// 获取某UE某进程状态
    harq_state get_process_state(uint16_t rnti, uint32_t pid) const;

    /// 获取全局统计
    enb_harq_stats get_stats() const;

    /// 获取指定UE统计
    enb_harq_stats get_ue_stats(uint16_t rnti) const;

    /// 获取已注册UE数量
    size_t get_nof_ues() const;

private:
    /// eNB接收端单个HARQ进程
    struct enb_rx_process {
        harq_state state;
        bool     ndi_known;     ///< 是否已知当前NDI (首次接收前为false)
        bool     cur_ndi;       ///< 当前TB的NDI
        uint32_t tx_count;      ///< 本TB累计传输次数
        uint32_t combined_count;///< 软合并次数 (= tx_count, 显式保留便于扩展)
        uint8_t  cur_mcs;       ///< 当前TB的MCS
        uint32_t cur_tbs;       ///< 当前TB的TBS
        bool     last_crc_ok;   ///< 最近一次PHICH反馈
        uint32_t last_tti;      ///< 最近接收TTI

        enb_rx_process()
            : state(harq_state::INACTIVE), ndi_known(false), cur_ndi(false)
            , tx_count(0), combined_count(0), cur_mcs(0), cur_tbs(0)
            , last_crc_ok(false), last_tti(0) {}
    };

    /// 单个UE的接收HARQ实体
    struct ue_rx_entity {
        int32_t ul_snr_x100;
        std::array<enb_rx_process, MAX_HARQ_PROCESSES> procs;
        enb_harq_stats stats;
        ue_rx_entity() : ul_snr_x100(200) {} // 默认2dB
    };

    /// 计算MCS对应的解码阈值 (x100 dB)
    /// = 选择阈值 + DECODE_MARGIN (MCS边缘处新传易失败)
    int32_t decode_threshold_x100(uint8_t mcs) const;

    /// 计算软合并后有效SNR
    int32_t effective_snr_x100(int32_t ul_snr, uint32_t combined_count) const;

    /// 释放进程 (回归INACTIVE), 不加锁版本
    void release_process_unlocked(enb_rx_process& p);

    uint32_t max_harq_tx_;
    mutable std::mutex mutex_;
    std::map<uint16_t, ue_rx_entity> ue_db_;
    enb_harq_stats total_stats_;
};

} // namespace ul_mac
