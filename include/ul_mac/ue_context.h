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
#include "ul_mac/mac_pdu.h"
#include <deque>
#include <memory>

namespace ul_mac {

/// UE上下文 - 整合MAC层上行管理所有组件
///
/// 对应 srsRAN_4G/srsue/hdr/stack/mac/mac.h 中的 mac 类
/// 该类将SR、BSR、HARQ、LCG缓冲区管理器组合在一起,
/// 提供统一的UE侧上行MAC操作接口
class ue_context {
public:
    /// 接收阶段 (receive_ul_grant) 打包好、待经 PUSCH 发送的 PDU
    struct pending_pdu {
        ul_grant grant;                 // 对应的 UL grant (含 tti_tx 等调度信息)
        std::vector<uint8_t> pdu;       // 已打包好的 UL-SCH MAC PDU (BSR CE 字节流)
    };

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
    ///
    /// 协议链路 (TS 36.321 §5.4.4 / §5.4.5):
    ///   数据到达 → BSR 触发 Regular → (若 UE 无 UL grant) SR 置 PENDING → SR 在 PUCCH 发送
    /// 本函数把这条链路显式拆为三步, 避免隐式依赖:
    void run_tti(uint32_t tti) {
        current_tti_ = tti;

        // 0. 同步 UL grant 状态给 BSR: has_ul_grant_ 由 run_tti 之前的 check_ul_grant()
        //    (仅"检查"本 TTI PDCCH 信道上是否有到达的 UL Grant, 置标志位) 决定, 不再用
        //    HARQ 进程状态近似。处理/打包该 grant 在 run_tti 之后的 process_ul_grant()
        //    中进行。此处仅把当前标志位透传给 BSR, 供其触发 SR 时判定"无 UL grant"前提
        //    (§5.4.4: 有可用 grant 时不触发 SR)。顺序上必须先于 BSR/SR 步进。
        bsr_mgr_.set_ul_grant_available(has_ul_grant_);

        // 1. BSR 步骤: 检查触发条件 (Regular/Periodic), 处理定时器。
        //    【显式桥接】若 BSR 判定触发 Regular 且当前无可用 UL grant, bsr_manager::set_trigger()
        //    内部会调用 sr_proc_->start() 将 SR 置为 PENDING —— 这一步就是协议要求的
        //    "Regular BSR 触发 SR"。SR 的触发判断在此处完成, 而非在下方 SR step 中。
        bsr_mgr_.step(tti);

        // 2. SR 步骤: 此时 state_ 可能已被 BSR 桥接置为 PENDING, 本步才在 PUCCH 上发送;
        //    若未被触发 (仍 IDLE), 则 step 直接返回 (对应协议"无 pending SR 则不发")。
        sr_mgr_.step(tti);

        // 3. 令牌桶步进: 按PBR为各逻辑信道补充令牌 (每TTI补充1ms)
        //    对应3GPP TS 36.321 Section 5.4.3.1 令牌桶维护
        buffer_mgr_.step_token_buckets(TTI_DURATION_MS);
    }

    /// 检查本 TTI 是否收到 PDCCH 上的 UL Grant (run_tti 之前调用)
    /// 仅做"置标志位", 不做任何处理 (不解 HARQ、不打包、不扣缓冲)。
    /// 这样 run_tti 内部做 BSR/SR 步进时, 可直接用 has_ul_grant_ 判定
    /// "有可用 grant 则不触发 SR" (§5.4.4), 而真正的 grant 处理留给
    /// run_tti 之后的 process_ul_grant()。
    /// @return 是否本 TTI 持有可用 UL grant (即刚置上的标志)
    bool check_ul_grant(const ul_grant& grant) {
        has_ul_grant_ = (grant.tbs > 0);   // 仅当 grant 携带有效 TB (tbs>0) 才置上本 TTI 的授权标志
        return has_ul_grant_;
    }

    /// 处理来自eNB的上行授权 (run_tti 之后调用)
    /// 对应 srsRAN mac.h 中的 new_grant_ul()
    /// 【协议时序】标志位 has_ul_grant_ 已在 run_tti 之前的 check_ul_grant() 置好,
    /// 故 run_tti 内的 BSR/SR 步进已据此判定过是否需要触发 SR。本方法才真正"处理"
    /// 该 grant: 判定重传/新传、打包 BSR CE 进 MAC PDU、扣减缓冲区, 并把 PDU 暂存
    /// 到待发队列, 由调用方经 PUSCH 信道发出。
    ul_harq_process::tx_action process_ul_grant(const ul_grant& grant) {
        // 1. 通知SR管理器收到上行授权 (SR成功): 清除 pending SR
        sr_mgr_.notify_ul_grant_received();

        // 2. 检查是否需要在授权中发送BSR
        bsr_ce bsr;
        // 【取消判定上界】用 max(old,new) 而非仅 old 快照: 新数据刚到达(未快照)时
        // old 偏小, 仅按 old 判定会误认为"授权装得下全部数据"而取消 BSR 触发 (§5.4.5)
        uint32_t total_data = buffer_mgr_.get_total_buffer_state_upper_bound();
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

        // 5. 【方案B】将 BSR 打包为 UL-SCH MAC PDU 字节流 (随 PUSCH 上报)
        //    真实协议 (TS 36.321 §5.4.5): BSR 作为 MAC CE 经 MAC PDU 打包
        //    (加子头 + 复用 + Padding) 成字节流, 在 PUSCH 上传输, 由 eNB 解包取出。
        //    本项目演示级: 仅打包 BSR CE (不承载真实 RLC SDU 字节, 数据量以 TBS 体现),
        //    用 mac_pdu_packer::pack_bsr_only 生成 PDU。重传沿用上一次 PDU
        //    (同一 TB 不应重新触发 BSR / 重新打包)。
        // 【陈旧 PDU 防护】新传 TB 若不携带 BSR, 必须清空 last_pdu_ ——
        //    否则上一 TB 的旧 BSR CE 会被当作本次 PUSCH 内容发出去, eNB 解出
        //    过期的缓冲区视图 (可能覆盖已增长的真实缓冲, 造成调度饥饿)。
        //    打包失败 (grant 装不下子头+CE) 时同样清空, 避免发送全零伪 PDU。
        if (action.is_new_tx) {
            last_pdu_.clear();
            if (!bsr.reports.empty()) {
                last_pdu_.assign(action.tbs, 0);
                auto pr = mac_pdu_packer::pack_bsr_only(last_pdu_.data(),
                                                        last_pdu_.size(), bsr);
                if (pr.bsr_bytes > 0) {
                    LOG_DEBUG("UE", rnti_, current_tti_,
                        "Packed BSR into MAC PDU (" +
                        std::to_string(last_pdu_.size()) + " bytes)");
                } else {
                    last_pdu_.clear();  // 空间不足装不下 BSR CE, 不上发
                }
            }
        }
        // 重传: 不再重新打包, last_pdu_ 保留上一次内容供 PUSCH 重传

        // 6. 新传时: 统计传输字节, 并从缓冲区取走已发送的数据 (简化版LCP)
        //    【协议说明 / 简化实现】
        //    真实协议 (TS 36.321 §5.4.3) 的 LCP 需按优先级逐 LC 分配、遵守令牌桶(PBR/BSD)。
        //    本项目用 lcg_buffer_manager::consume_data() 做两阶段近似 (先高优先级 LC 后低优先级),
        //    且用 new_buffer 实时值而非"已上报值"扣减, 因此不存在 BSR 上报值与实际发送不一致问题。
        //    重传发送的是同一个TB, 不重复计入也不重复消耗缓冲区
        if (action.tbs > 0 && action.is_new_tx) {
            total_tx_bytes_ += action.tbs;
            uint32_t sent_bytes = action.tbs;
            // 从pending_data_中按FIFO匹配已发送字节, 计算端到端延迟并记录
            // 延迟 = 当前发送TTI - 数据到达TTI (下溢保护: 异常时序按0计)
            while (sent_bytes > 0 && !pending_data_.empty()) {
                auto& front = pending_data_.front();
                uint32_t latency = (grant.tti_tx >= front.arrival_tti)
                                 ? (grant.tti_tx - front.arrival_tti) : 0;
                metrics_collector::instance().record_latency(latency);
                if (front.bytes <= sent_bytes) {
                    sent_bytes -= front.bytes;
                    pending_data_.pop_front();   // deque: O(1) 头部弹出
                } else {
                    front.bytes -= sent_bytes;
                    sent_bytes = 0;
                }
            }
            buffer_mgr_.consume_data(action.tbs);
        }

        // 6b. 【丢弃回滚已移至 handle_harq_feedback】TB 被丢弃 (达到最大重传/早期终止)
        //     的判定现在在 PHICH 反馈落地时发生 (见 handle_harq_feedback), 而非在
        //     process_ul_grant 处理 grant 的当下 —— 因为 HARQ RTT 延时 (统一由
        //     timed_channel 建模) 意味着反馈总在 grant 处理之后才到达。

        // 2. 暂存待发 PDU: 本方法只"打包"不"发送", 把 grant 与打包好的 PDU 入队,
        //    交由调用方在 run_tti 之后通过 drain_pending_tx() 经 PUSCH 信道发出。
        pending_tx_.push_back({grant, last_pdu_});

        return action;
    }

    /// 用尽本 TTI 的 UL grant: 发送 PDU 后调用, 清零 has_ul_grant_, 防止下一 TTI
    /// 误判仍持有 grant 而错误抑制 SR (见 receive_ul_grant / run_tti 注释)。
    void clear_ul_grant() { has_ul_grant_ = false; }

    /// 接收阶段是否还有经 PUSCH 待发送的 PDU (由 receive_ul_grant 入队)
    bool has_pending_tx() const { return !pending_tx_.empty(); }

    /// 取走一个待发 PDU (FIFO)。调用方在 run_tti 之后按原时序经 PUSCH 信道发送。
    pending_pdu take_pending_tx() {
        pending_pdu p = pending_tx_.front();
        pending_tx_.erase(pending_tx_.begin());
        return p;
    }

    /// 处理HARQ反馈 (模拟PHICH反馈)
    ///
    /// 【RTT延迟说明】
    /// 实际系统中, UE发送PUSCH后需要等待约8 TTI (LTE HARQ RTT)才能在PHICH上
    /// 收到eNB的ACK/NACK反馈。本方法模拟"反馈到达UE"的时刻, 因此调用方应在
    /// HARQ 反馈落地 (模拟 PHICH)。反馈经 timed_channel 统一 +4 TTI 延时到达,
    /// 故总在对应 PUSCH 发送之后。NACK 且达到重传上限/早期终止时, HARQ 进程判定
    /// 丢弃 TB, 此处把已乐观扣减的缓冲回滚 (RLC 重递交近似)。
    void handle_harq_feedback(uint32_t pid, bool ack) {
        auto fb = harq_mgr_.handle_harq_feedback(pid,
            ack ? harq_feedback::ACK : harq_feedback::NACK);
        if (ack) {
            LOG_DEBUG("UE", rnti_, current_tti_,
                "HARQ ACK for PID=" + std::to_string(pid));
        } else {
            LOG_DEBUG("UE", rnti_, current_tti_,
                "HARQ NACK for PID=" + std::to_string(pid) + ", will retx");
        }

        // TB 被丢弃: 把数据放回缓冲区 (RLC 重递交近似)。新传时已乐观扣减缓冲
        // (见 process_ul_grant 步骤6), 不回滚则丢弃的字节静默丢失。放回最高优先级
        // LC (简化, 见 lcg_buffer_manager::restore_data) 并重新入延迟跟踪队列,
        // 等价于"RLC 重递交一批新数据", 会重新触发 Regular BSR 走完整流程。
        if (fb.is_discarded && fb.tbs > 0) {
            buffer_mgr_.restore_data(fb.tbs);
            pending_data_tracker tracker;
            tracker.lcid = 0;  // 简化: 未记录原 LC 归属, lcid 仅存档不参与逻辑
            tracker.arrival_tti = current_tti_;
            tracker.bytes = fb.tbs;
            pending_data_.push_back(tracker);
            LOG_WARN("UE", rnti_, current_tti_,
                "TB discarded (tbs=" + std::to_string(fb.tbs) +
                "B), data restored to buffer (RLC re-delivery approximation)");
        }
    }

    uint16_t get_rnti() const { return rnti_; }
    lcg_buffer_manager& get_buffer_manager() { return buffer_mgr_; }

    /// 最近一次打包好的 UL-SCH MAC PDU (已由 receive_ul_grant 暂存进 pending_tx_)
    const std::vector<uint8_t>& get_last_pdu() const { return last_pdu_; }
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

    // 【方案B】最近一次打包的 UL-SCH MAC PDU (BSR CE 字节流), 随 PUSCH 上报
    std::vector<uint8_t> last_pdu_;

    // 接收阶段 (receive_ul_grant) 打包好、等待 run_tti 之后经 PUSCH 发送的 PDU 队列
    std::vector<pending_pdu> pending_tx_;

    /// 延迟追踪: 记录每批数据的到达TTI (lcid -> arrival_tti)
    /// 用 deque: 头部弹出 (pop_front) 为 O(1), 避免 vector 头部 erase 的 O(n^2)
    struct pending_data_tracker {
        uint32_t lcid;
        uint32_t arrival_tti;
        uint32_t bytes;
    };
    std::deque<pending_data_tracker> pending_data_;
};

} // namespace ul_mac
