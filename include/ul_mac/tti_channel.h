// =============================================================================
// tti_channel.h - 中央 TTI 时钟 + 时间解耦信道 (方案C: 多线程 + 4-TTI 时延)
//
// 建模 LTE 上下行空口的"信息处理时延": UE 与 eNB 之间通过四条独立空口信道
// 交互, 每条信道上的信息在"建立"后需经过固定 TTI 数(本项目=4)才能被对端
// 看见/处理, 对应真实系统中 PUCCH/PDCCH/PUSCH/PHICH 的传播与处理时延。
//
//   信道              方向        承载           时延建模
//   ---------------------------------------------------------------------
//   SR     (PUCCH)   UE  -> eNB   rnti           enqueue(tti) 后 +4 TTI 可见
//   UL Grant(PDCCH)  eNB -> UE    ul_grant       enqueue(tti) 后 +4 TTI 可见
//   MAC PDU(PUSCH)   UE  -> eNB   {rnti, pdu}    enqueue(tti) 后 +4 TTI 可见
//   PHICH            eNB -> UE    {rnti,pid,ack} enqueue(tti) 后 +4 TTI 可见
//
// 设计说明 (演示级, 确定性):
//   - 不引入真实 wall-clock sleep, 由中央 tti_clock 线程统一推进 TTI 计数;
//     所有 worker 线程通过 wait_for_tti() 在指定 TTI 上被唤醒工作。时钟每
//     TTI 以屏障 (mark_tti_done) 等待全部 worker 完成后才推进, 保证仿真
//     结果与单线程版本一致 (确定性), 同时体现并发与时间解耦。
//   - 信道用 std::mutex + std::condition_variable 保护, 支持多线程并发 enqueue/dequeue。
//   - UL Grant/PHICH 为点对点信道: 用 per_rnti_channel_map 为每个 UE 建立
//     专属信道, 避免共享 FIFO 被错误 UE 取出 (RNTI 加扰语义)。
//
// 关键参考:
//   - 3GPP TS 36.321 / 36.213 空口时序 (PUCCH/PDCCH/PUSCH/PHICH 处理时延)
//   - 真实系统 HARQ RTT ≈ 8 TTI; 本项目取 4 TTI 作为"信息可见时延"演示值
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>

namespace ul_mac {

/// 信息建立后到对端可见所需的 TTI 数 (方案C 演示值)
constexpr uint32_t CHANNEL_PROPAGATION_TTI = 4;

// ---------------------------------------------------------------------------
// 时间解耦信道: 发送方在 send_tti 入队, 接收方仅能在
// current_tti >= send_tti + CHANNEL_PROPAGATION_TTI 时取出
// ---------------------------------------------------------------------------
template <typename T>
class timed_channel {
public:
    /// 入队一条信息, 记录发送 TTI
    void enqueue(const T& item, uint32_t send_tti) {
        std::lock_guard<std::mutex> lock(mutex_);
        q_.push({item, send_tti});
    }

    /// 尝试出队: 仅当存在一条 "已经过传播时延" 的信息时返回它
    /// @param current_tti 当前时钟 TTI
    /// @return 若该 TTI 有可见信息则返回之 (并出队); 否则 std::nullopt
    std::optional<T> dequeue(uint32_t current_tti) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (q_.empty()) return std::nullopt;
        auto& front = q_.front();
        if (current_tti >= front.send_tti + CHANNEL_PROPAGATION_TTI) {
            T item = front.item;
            q_.pop();
            return item;
        }
        return std::nullopt;  // 信息尚未到达可见时刻
    }

    /// 是否存在"已经过传播时延、可在 current_tti 取出"的信息
    bool ready(uint32_t current_tti) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (q_.empty()) return false;
        return current_tti >= q_.front().send_tti + CHANNEL_PROPAGATION_TTI;
    }

    /// 队列中等待的信息条数 (含未到可见时刻的)
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return q_.size();
    }

private:
    struct entry {
        T item;
        uint32_t send_tti;
    };
    mutable std::mutex mutex_;
    std::queue<entry> q_;
};

// ---------------------------------------------------------------------------
// Per-RNTI 单播信道组: 每个 UE 独占一条 timed_channel
// ---------------------------------------------------------------------------
// UL Grant (PDCCH) 与 PHICH 都是 RNTI 加扰的点对点信道 —— 真实空口中每个 UE
// 只解自己 RNTI 对应的 PDCCH/PHICH。若多个 UE 线程共享一个 FIFO 并各自
// dequeue, 会"错领"别人的 grant/反馈 (数据一致性缺陷)。本容器为每个 rnti
// 预注册一条独立信道: 发送方 (eNB) 按 rnti 投递, 接收方 (UE) 只取自己的信道。
// 注意: 必须在启动线程前 (单线程初始化阶段) 调用 add_rnti 完成全部注册,
// 运行期并发访问只触碰各自信道, std::map 节点地址稳定, 无需额外加锁。
template <typename T>
class per_rnti_channel_map {
public:
    /// 预注册 rnti 的专属信道 (单线程初始化阶段调用)
    void add_rnti(uint16_t rnti) { channels_[rnti]; }

    /// 取 rnti 的专属信道 (运行期: 各线程只访问自己 rnti 的信道)
    timed_channel<T>& at(uint16_t rnti) { return channels_.at(rnti); }

private:
    std::map<uint16_t, timed_channel<T>> channels_;
};

// ---------------------------------------------------------------------------
// 信道载荷类型
// ---------------------------------------------------------------------------

/// PUSCH 信道载荷: UE 上报的 MAC PDU (BSR CE 字节流) + 对应的 UL grant
/// 【说明】真实 eNB 接收 PUSCH TB 时用此前下发的 ul_grant (pid/ndi/mcs/tbs) 解析 TB;
/// 本项目 PUSCH 仅承载 BSR CE, 故把 UE 收到的 ul_grant 一并随信道传递,
/// 供 eNB 侧 receive_tb 使用 —— 等效于"eNB 用 grant 收 TB"的真实语义。
struct pdu_msg {
    uint16_t  rnti;
    ul_grant  grant;  ///< UE 收到的 UL grant (eNB 下发, 用于收 TB)
    std::vector<uint8_t> pdu;
};

/// PHICH 信道载荷: eNB 反馈给 UE 的 HARQ 结果
struct phich_msg {
    uint16_t rnti;
    uint32_t pid;
    bool     ack;       ///< true=ACK, false=NACK
};

/// SR 信道载荷 (PUCCH 1-bit 调度请求, 本项目扩展携带 UE 本地待传字节数)
/// 说明: 真实协议中 SR 仅是 1-bit 能量信号, 不携带任何缓冲量; 待传量须等 UE 拿到
/// PUSCH 授权后发 BSR CE 才被 eNB 知晓 (见 pdu_msg 的空口解包路径)。
/// 本项目为贴合 srsRAN 的"同进程内部捷径"建模: SR 触发的同一时刻, UE 本地已算出
/// 各 LCG 待传字节数, 经此字段直接告知 eNB 调度器 (等价 srsRAN handle_ul_bsr_indication),
/// 使 schedule_ul 在 BSR CE 经 PUSCH 解包前即可按真实量分配, 跳过"先发小 Grant 探测 BSR"
/// 的空口往返。pending_bytes 为空数组时退化为纯 SR 提示 (BSR 滞后一轮, 由空口解包补校准)。
struct sr_msg {
    uint16_t rnti;
    std::array<uint32_t, NOF_LCGS> pending_bytes{};  ///< UE 本地已知各 LCG 待传字节
};

// ---------------------------------------------------------------------------
// 中央 TTI 时钟: 单一线程推进 TTI 计数, worker 通过 wait_for_tti 同步
//
// 【屏障同步】时钟每推进一个 TTI, 必须等待所有 worker 线程调用 mark_tti_done()
// 汇报"本 TTI 工作已完成"后才推进下一个 TTI。这样保证:
//   1) 任一 TTI 内所有 worker 的处理完整完成, 不会被时钟"甩在后面";
//   2) worker 不会因时钟提前到达终点 (done_=true) 而中途退出、丢失 TTI;
//   3) 仿真的时间语义与单线程版本一致 (确定性), 同时体现并发与时间解耦。
// ---------------------------------------------------------------------------
class tti_clock {
public:
    /// @param max_tti 仿真的最大 TTI (含)
    /// @param num_workers 参与屏障同步的 worker 线程数 (不含时钟线程自身)
    explicit tti_clock(uint32_t max_tti, uint32_t num_workers = 1)
        : max_tti_(max_tti), num_workers_(num_workers) {}

    /// 时钟线程主循环: 从 0 推进到 max_tti_, 每个 TTI 等待全部 worker 完成
    void run() {
        for (uint32_t t = 0; t <= max_tti_; ++t) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_ = t;       // 明确"宣布第 t 个 TTI 开始"
                done_count_ = 0;    // 重置本 TTI 的完成计数 (在宣布之后, worker 才能工作)
            }
            cv_.notify_all();       // 唤醒等待 current_==t 的 worker

            // 屏障: 等待所有 worker 完成本 TTI 工作 (各自调用 mark_tti_done)
            // 由于 worker 必须等 current_==t 才开始工作, 其 mark 必然发生在本 wait 期间
            // 或之后, 不会早于 run 进入 wait (杜绝丢失唤醒)。
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return done_count_ >= num_workers_; });
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_ = max_tti_ + 1;  // 推进到"结束"态, 让仍在等待的 worker 看到
            done_ = true;
        }
        cv_.notify_all();
    }

    /// 阻塞当前线程, 直到时钟明确推进到 target_tti (即 run 已宣布该 TTI 开始)。
    /// 注意: 必须严格等待 current_==target_tti, 而非 current_>=target_tti —— 否则
    /// worker 可能在 run 宣布本轮之前就因初始 current_>=0 提前通过并 mark, 导致
    /// run 的屏障 wait 丢失该次唤醒 (经典 condvar 丢失唤醒死锁)。
    void wait_for_tti(uint32_t target_tti) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this, target_tti] {
            return done_ || current_ == target_tti;
        });
    }

    /// worker 完成当前 TTI 的工作后调用, 向时钟汇报一次;
    /// 全部 worker 汇报后时钟才推进到下一 TTI (屏障)
    void mark_tti_done() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++done_count_;
        }
        cv_.notify_all();
    }

    uint32_t current() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_;
    }

    bool done() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return done_;
    }

private:
    uint32_t max_tti_;
    uint32_t num_workers_;
    uint32_t current_ = UINT32_MAX;  ///< 初始为"未来值", 逼 worker 等 run 明确宣布才开始
    uint32_t done_count_ = 0;        ///< 当前 TTI 已完成的 worker 数 (屏障计数)
    bool     done_ = false;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
};

} // namespace ul_mac
