// =============================================================================
// ue_context.h - UE上下文管理器
//
// 整合SR、BSR、HARQ等MAC层上行管理组件, 模拟UE侧完整MAC上行行为
//
// 参考:
//   - srsRAN_4G/srsue/hdr/stack/mac/mac.h 中的 mac 类
//     mac类集成了sr_proc, bsr_proc, ul_harq_entity等组件
//   - ocudu/lib/mac/mac_ul/mac_ul_ue_manager.h 中的 mac_ul_ue_context
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include "ul_mac/lcg_buffer.h"
#include "ul_mac/ue_sr_manager.h"
#include "ul_mac/ue_bsr_manager.h"
#include "ul_mac/ue_ul_harq_manager.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <memory>

namespace ul_mac {

/// UE上下文 - 整合MAC层上行管理所有组件
///
/// 对应 srsRAN_4G/srsue/hdr/stack/mac/mac.h 中的 mac 类
/// 该类将SR、BSR、HARQ、LCG缓冲区管理器组合在一起,
/// 提供统一的UE侧上行MAC操作接口
class ue_context {
public:
    /// 构造函数
    /// @param rnti UE的C-RNTI
    explicit ue_context(uint16_t rnti)
        : rnti_(rnti)
        , is_temp_rnti_(false)
        , buffer_mgr_()
        , sr_mgr_(rnti)
        , bsr_mgr_(rnti, buffer_mgr_)
        , harq_mgr_(rnti)
        , current_tti_(0)
        , has_ul_grant_(false)
        , total_tx_bytes_(0)
    {
        // 初始化SR: 设置回调
        sr_config sr_cfg;
        sr_mgr_.init(sr_cfg,
            [this](uint16_t r) { on_sr_sent(r); },
            [this](uint16_t r) { on_sr_failed(r); });

        // 初始化BSR: 关联SR管理器
        bsr_config bsr_cfg;
        bsr_mgr_.init(bsr_cfg, &sr_mgr_,
            [this](uint16_t r, const bsr_ce& b) { on_bsr_sent(r, b); });

        // 初始化HARQ
        ul_harq_config harq_cfg;
        harq_mgr_.init(harq_cfg);

        LOG_INFO("UE", rnti_, 0, "UE context created");
    }

    /// 配置逻辑通道
    /// 对应 srsRAN mac.h 中的 setup_lcid()
    /// @param pbr 优先级比特率 (bytes/ms), 0表示无PBR限制 (默认0, 保持向后兼容)
    /// @param bsd 桶大小持续时间 (ms), 默认100
    void setup_lcid(uint32_t lcid, uint32_t lcg_id, uint32_t priority,
                    uint32_t pbr = 0, uint32_t bsd = 100) {
        buffer_mgr_.setup_lcid(lcid, lcg_id, priority, pbr, bsd);
        LOG_INFO("UE", rnti_, 0,
            "LCID configured: lcid=" + std::to_string(lcid) +
            ", lcg=" + std::to_string(lcg_id) +
            ", priority=" + std::to_string(priority) +
            ", pbr=" + std::to_string(pbr) +
            ", bsd=" + std::to_string(bsd));
    }

    /// 设置SR配置
    void set_sr_config(const sr_config& cfg) { sr_mgr_.set_config(cfg); }

    /// 设置BSR配置
    void set_bsr_config(const bsr_config& cfg) { bsr_mgr_.set_config(cfg); }

    /// 设置HARQ配置
    void set_harq_config(const ul_harq_config& cfg) { harq_mgr_.set_config(cfg); }

    /// 数据到达 (模拟RLC层通知MAC有新数据)
    /// @param lcid 逻辑通道ID
    /// @param bytes 缓冲区字节数
    void data_arrived(uint32_t lcid, uint32_t bytes) {
        buffer_mgr_.update_buffer_state(lcid, bytes);
        if (bytes > 0) {
            // 记录数据到达TTI, 用于后续在新传发送时计算端到端延迟
            pending_data_tracker tracker;
            tracker.lcid = lcid;
            tracker.arrival_tti = current_tti_;
            tracker.bytes = bytes;
            pending_data_.push_back(tracker);
            LOG_DEBUG("UE", rnti_, current_tti_,
                "Data arrived: lcid=" + std::to_string(lcid) +
                ", bytes=" + std::to_string(bytes));
        }
    }

    /// 每TTI执行的UE侧MAC步骤
    /// 对应 srsRAN mac.h 中的 run_tti()
    void run_tti(uint32_t tti) {
        current_tti_ = tti;
        // 1. BSR步骤: 检查触发条件, 处理定时器
        bsr_mgr_.step(tti);
        // 2. SR步骤: 检查是否需要发送SR
        sr_mgr_.step(tti);
        // 3. 令牌桶步进: 按PBR为各逻辑信道补充令牌 (每TTI补充1ms)
        //    对应3GPP TS 36.321 Section 5.4.3.1 令牌桶维护
        buffer_mgr_.step_token_buckets(TTI_DURATION_MS);
    }

    /// 处理来自eNB的上行授权
    /// 对应 srsRAN mac.h 中的 new_grant_ul()
    ul_harq_process::tx_action handle_ul_grant(const ul_grant& grant) {
        // 1. 通知SR管理器收到上行授权 (SR成功)
        sr_mgr_.notify_ul_grant_received();
        has_ul_grant_ = true;

        // 2. 检查是否需要在授权中发送BSR
        bsr_ce bsr;
        uint32_t total_data = buffer_mgr_.get_total_buffer_state();
        bool bsr_sent = bsr_mgr_.need_to_send_bsr_on_ul_grant(grant.tbs, total_data, bsr);

        // 2b. 若未发BSR且授权装完数据后仍有剩余空间, 尝试发送Padding BSR
        // 对应3GPP TS 36.321: 填充空间足够容纳BSR CE时触发Padding BSR
        if (!bsr_sent && total_data > 0 && grant.tbs > total_data) {
            bsr_mgr_.generate_padding_bsr(grant.tbs - total_data, bsr);
        }

        // 3. 处理HARQ (新传或重传)
        ul_harq_process::tx_action action =
            harq_mgr_.new_grant_ul(grant, is_temp_rnti_);

        // 4. 更新缓冲区状态 (TTI结束后)
        bsr_mgr_.update_bsr_tti_end(bsr);

        // 5. 新传时: 统计传输字节, 并从缓冲区取走已发送的数据 (简化版LCP)
        //    【协议说明 / 简化实现】
        //    真实协议 (TS 36.321 §5.4.3) 的 LCP 需按优先级逐 LC 分配、遵守令牌桶(PBR/BSD)。
        //    本项目用 lcg_buffer_manager::consume_data() 做两阶段近似 (先高优先级 LC 后低优先级),
        //    且用 new_buffer 实时值而非"已上报值"扣减, 因此不存在 BSR 上报值与实际发送不一致问题。
        //    重传发送的是同一个TB, 不重复计入也不重复消耗缓冲区
        if (action.tbs > 0 && action.is_new_tx) {
            total_tx_bytes_ += action.tbs;
            uint32_t sent_bytes = action.tbs;
            // 从pending_data_中按FIFO匹配已发送字节, 计算端到端延迟并记录
            // 延迟 = 当前发送TTI - 数据到达TTI
            while (sent_bytes > 0 && !pending_data_.empty()) {
                auto& front = pending_data_.front();
                uint32_t latency = grant.tti_tx - front.arrival_tti;
                metrics_collector::instance().record_latency(latency);
                if (front.bytes <= sent_bytes) {
                    sent_bytes -= front.bytes;
                    pending_data_.erase(pending_data_.begin());
                } else {
                    front.bytes -= sent_bytes;
                    sent_bytes = 0;
                }
            }
            buffer_mgr_.consume_data(action.tbs);
        }

        return action;
    }

    /// 处理HARQ反馈 (模拟PHICH反馈)
    ///
    /// 【RTT延迟说明】
    /// 实际系统中, UE发送PUSCH后需要等待约8 TTI (LTE HARQ RTT)才能在PHICH上
    /// 收到eNB的ACK/NACK反馈。本方法模拟"反馈到达UE"的时刻, 因此调用方应在
    /// 发送后经过RTT个TTI再调用本方法 (main.cpp中已按此方式在后续TTI调用)。
    /// ul_harq_process 内部通过 tx_tti_ + rtt_ttis_ 建模该延迟, 在RTT未到时
    /// 即便 grant.phich_available=true 也会延迟处理反馈。
    void handle_harq_feedback(uint32_t pid, bool ack) {
        harq_mgr_.handle_harq_feedback(pid,
            ack ? harq_feedback::ACK : harq_feedback::NACK);
        if (!ack) {
            LOG_DEBUG("UE", rnti_, current_tti_,
                "HARQ NACK for PID=" + std::to_string(pid) + ", will retx");
        }
    }

    uint16_t get_rnti() const { return rnti_; }
    lcg_buffer_manager& get_buffer_manager() { return buffer_mgr_; }
    sr_manager& get_sr_manager() { return sr_mgr_; }
    bsr_manager& get_bsr_manager() { return bsr_mgr_; }
    ul_harq_manager& get_harq_manager() { return harq_mgr_; }
    uint32_t get_total_tx_bytes() const { return total_tx_bytes_; }

    /// 获取UE度量信息
    ue_metrics get_metrics() const {
        ue_metrics m;
        m.rnti = rnti_;
        auto sr_stats = sr_mgr_.get_stats();
        m.sr_tx_count = sr_stats.total_sr_sent;
        m.sr_success_count = sr_stats.total_sr_success;
        auto bsr_stats = bsr_mgr_.get_stats();
        m.bsr_tx_count = bsr_stats.total_bsr_sent;
        auto harq_stats = harq_mgr_.get_stats();
        m.harq_tx_count = harq_stats.total_new_tx;
        m.harq_retx_count = harq_stats.total_retx;
        m.harq_fail_count = harq_stats.total_fail;
        m.total_ul_bytes = total_tx_bytes_;
        m.avg_harq_retx = harq_mgr_.get_average_retx();
        if (current_tti_ > 0) {
            m.ul_throughput_kbps = (double)total_tx_bytes_ * 8.0 / current_tti_;
        }
        return m;
    }

    void set_temp_rnti(bool is_temp) { is_temp_rnti_ = is_temp; }

private:
    void on_sr_sent(uint16_t rnti) {
        LOG_INFO("UE", rnti, current_tti_, "SR sent on PUCCH");
    }

    void on_sr_failed(uint16_t rnti) {
        // 【协议说明 / 简化实现】
        // 真实协议: SR 达到 dsr-TransMax 后 UE 应释放 SR/PUCCH 配置并触发随机接入(RA)。
        // 本项目未在 MAC 层实现 RA 过程 (RA 跨层且需 eNB 配合), 此处仅作日志占位,
        // 表达 "SR 失败 → 应由上层启动 RA" 的语义。详情见 ue_sr_manager.cpp 对应注释。
        LOG_ERROR("UE", rnti, current_tti_,
            "SR failed - RA fallback should be triggered by upper layer (not implemented here)");
    }

    void on_bsr_sent(uint16_t rnti, const bsr_ce& bsr) {
        std::string msg = "BSR sent: format=" + bsr_format_to_string(bsr.format);
        for (const auto& r : bsr.reports) {
            msg += ", LCG" + std::to_string(r.lcg_id) + "=" + std::to_string(r.buffer_size);
        }
        LOG_INFO("UE", rnti, current_tti_, msg);
    }

    // UE标识
    uint16_t rnti_;
    bool is_temp_rnti_;

    // MAC上行管理组件
    lcg_buffer_manager buffer_mgr_;   ///< LCG缓冲区管理
    sr_manager         sr_mgr_;       ///< SR调度请求管理
    bsr_manager        bsr_mgr_;      ///< BSR缓冲区状态报告管理
    ul_harq_manager    harq_mgr_;     ///< UL HARQ管理

    // 状态
    uint32_t current_tti_;
    bool has_ul_grant_;
    uint32_t total_tx_bytes_;

    /// 延迟追踪: 记录每批数据的到达TTI (lcid -> arrival_tti)
    struct pending_data_tracker {
        uint32_t lcid;
        uint32_t arrival_tti;
        uint32_t bytes;
    };
    std::vector<pending_data_tracker> pending_data_;
};

} // namespace ul_mac
