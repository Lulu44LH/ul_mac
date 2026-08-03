// =============================================================================
// lcg_buffer.h - 逻辑通道组(LCG)缓冲区管理
//
// 管理UE侧各逻辑通道(LCID)和逻辑通道组(LCG)的缓冲区状态
// 这是BSR生成的基础数据结构, 也是上行调度的输入
//
// 关键参考:
//   - srsRAN_4G/srsue/src/stack/mac/proc_bsr.cc 中 lcgs[] 数组的管理逻辑
//   - srsRAN_4G/srsue/hdr/stack/mac/proc_bsr.h 中的 lcid_t 结构
//   - ocudu/include/ocudu/mac/mac_lc_config.h 中的逻辑通道配置
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include <algorithm>
#include <map>
#include <mutex>
#include <vector>

namespace ul_mac {

/// 逻辑通道组(LCG)缓冲区管理器
///
/// 每个LCG包含多个逻辑通道(LCID), BSR按LCG粒度上报缓冲区状态
/// 管理:
///   - 每个LCID的优先级、新旧缓冲区大小
///   - LCG级别的缓冲区聚合
///   - 新数据检测 (触发Regular BSR的条件)
///   - 最高优先级通道检测 (触发Regular BSR的条件)
class lcg_buffer_manager {
public:
    lcg_buffer_manager() = default;

    /// 设置逻辑通道配置
    /// 初始化令牌桶: token_bucket_size = pbr * bsd, token_count = 0
    /// @param lcid 逻辑通道ID
    /// @param lcg_id 逻辑通道组ID
    /// @param priority 优先级 (1最高)
    /// @param pbr 优先级比特率 (bytes/ms), 0表示无PBR限制 (退化为纯优先级排序)
    /// @param bsd 桶大小持续时间 (ms)
    void setup_lcid(uint32_t lcid, uint32_t lcg_id, uint32_t priority,
                    uint32_t pbr = 0, uint32_t bsd = 100) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lcg_id >= NOF_LCGS) return;
        lc_buffer_state state;
        state.priority = priority;
        state.pbr = pbr;
        state.bsd = bsd;
        // 桶容量 = PBR * BSD; pbr=0时桶容量置0表示不受PBR限制(无穷大)
        state.token_bucket_size = static_cast<double>(pbr) * static_cast<double>(bsd);
        state.token_count = 0.0;
        lcgs_[lcg_id][lcid] = state;
    }

    /// 令牌桶步进 - 每TTI调用, 按PBR为各逻辑信道补充令牌
    /// 对应3GPP TS 36.321 Section 5.4.3.1: 每个TTI令牌按PBR速率增加, 但不超过桶容量
    /// @param elapsed_ms 自上次步进以来经过的时间 (ms), 通常为1
    void step_token_buckets(uint32_t elapsed_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        step_token_buckets_unlocked(elapsed_ms);
    }

    /// 更新逻辑通道的缓冲区大小 (模拟RLC层缓冲区状态查询)
    /// 对应 srsRAN proc_bsr.cc 中的 update_new_data() 方法
    /// @param lcid 逻辑通道ID
    /// @param bytes 当前缓冲区字节数
    void update_buffer_state(uint32_t lcid, uint32_t bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            auto it = lcgs_[i].find(lcid);
            if (it != lcgs_[i].end()) {
                it->second.new_buffer = bytes;
                return;
            }
        }
    }

    /// 获取指定LCG的缓冲区大小 (使用old_buffer, 对应已上报的状态)
    /// 对应 srsRAN proc_bsr.cc 中的 get_buffer_state_lcg()
    uint32_t get_buffer_state_lcg(uint32_t lcg_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lcg_id >= NOF_LCGS) return 0;
        uint32_t total = 0;
        for (const auto& [lcid, state] : lcgs_[lcg_id]) {
            total += state.old_buffer;
        }
        return total;
    }

    /// 获取所有LCG的总缓冲区大小
    /// 对应 srsRAN proc_bsr.cc 中的 get_buffer_state()
    uint32_t get_total_buffer_state() const {
        uint32_t total = 0;
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            total += get_buffer_state_lcg(i);
        }
        return total;
    }

    /// 更新旧缓冲区状态 (将new_buffer复制到old_buffer)
    /// 对应 srsRAN proc_bsr.cc 中的 update_old_buffer()
    void update_old_buffer() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (auto& [lcid, state] : lcgs_[i]) {
                state.old_buffer = state.new_buffer;
            }
        }
    }

    /// 检查是否有新数据到达 (LCG之前无数据, 现在有新数据)
    /// 对应 srsRAN proc_bsr.cc 中的 check_new_data()
    /// 这是触发Regular BSR的条件之一
    bool check_new_data() const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            if (get_buffer_state_lcg_unlocked(i) == 0) {
                for (const auto& [lcid, state] : lcgs_[i]) {
                    if (state.new_buffer > 0) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// 检查是否有高优先级通道的新数据到达
    /// 对应 srsRAN proc_bsr.cc 中的 check_highest_channel()
    /// 这是触发Regular BSR的条件之一
    bool check_highest_priority_channel() const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (const auto& [lcid, state] : lcgs_[i]) {
                if (state.new_buffer > state.old_buffer) {
                    bool is_max_priority = true;
                    for (uint32_t j = 0; j < NOF_LCGS; j++) {
                        for (const auto& [lcid2, state2] : lcgs_[j]) {
                            if (state2.priority <= state.priority && state2.old_buffer > 0) {
                                is_max_priority = false;
                            }
                        }
                    }
                    if (is_max_priority) return true;
                }
            }
        }
        return false;
    }

    /// 检查任意通道是否有数据待发送
    /// 对应 srsRAN proc_bsr.cc 中的 check_any_channel()
    bool check_any_channel_has_data() const {
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            if (get_buffer_state_lcg(i) > 0) {
                return true;
            }
        }
        return false;
    }

    /// 找到有数据的最高优先级LCG
    /// 用于生成Truncated BSR时选择要报告的LCG
    /// 对应 srsRAN proc_bsr.cc 中的 find_max_priority_lcg_with_data()
    uint32_t find_max_priority_lcg_with_data() const {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t best_lcg = 0;
        uint32_t best_priority = UINT32_MAX;

        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (const auto& [lcid, state] : lcgs_[i]) {
                if (state.old_buffer > 0 && state.priority < best_priority) {
                    best_priority = state.priority;
                    best_lcg = i;
                }
            }
        }
        return best_lcg;
    }

    /// 消耗缓冲区数据 (模拟MAC PDU组装从RLC取走数据)
    /// 实现3GPP TS 36.321 Section 5.4.3.1 的逻辑信道优先级(LCP)算法:
    ///   第一阶段: 按优先级从高到低, 每个信道按PBR取数据 (受令牌桶限制)
    ///             每个信道取 min(token_count, new_buffer, remaining_bytes)
    ///             扣减令牌: token_count -= taken
    ///   第二阶段: 剩余空间按优先级从高到低分配给仍有数据的信道 (不受PBR限制)
    /// 向后兼容: 当pbr=0时, 令牌桶不限制(等价于无穷大), 退化为纯优先级排序
    /// @param bytes 本次传输可携带的字节数 (TBS)
    void consume_data(uint32_t bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t remaining = bytes;

        // 收集所有有数据的逻辑信道, 按优先级排序 (数值小=优先级高)
        std::vector<lc_buffer_state*> channels;
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (auto& [lcid, state] : lcgs_[i]) {
                if (state.new_buffer > 0) {
                    channels.push_back(&state);
                }
            }
        }
        std::sort(channels.begin(), channels.end(),
            [](const lc_buffer_state* a, const lc_buffer_state* b) {
                return a->priority < b->priority;
            });

        // 第一阶段: 按优先级顺序, 每个信道按PBR(令牌桶)取数据
        // 受令牌桶限制: 取 min(token_count, new_buffer, remaining)
        for (auto* ch : channels) {
            if (remaining == 0) break;
            // pbr=0表示不受PBR限制, 跳过第一阶段(留到第二阶段按优先级分配)
            if (ch->pbr == 0) continue;

            double take_d = ch->token_count;
            if (take_d > static_cast<double>(ch->new_buffer)) take_d = static_cast<double>(ch->new_buffer);
            if (take_d > static_cast<double>(remaining))      take_d = static_cast<double>(remaining);
            if (take_d <= 0.0) continue;

            uint32_t taken = static_cast<uint32_t>(take_d);
            ch->new_buffer   -= taken;
            ch->token_count  -= static_cast<double>(taken);
            remaining        -= taken;
        }

        // 第二阶段: 剩余空间按优先级从高到低分配给仍有数据的信道 (不受PBR限制)
        // 按优先级排序的顺序保持不变, 依次取尽
        for (auto* ch : channels) {
            if (remaining == 0) break;
            if (ch->new_buffer == 0) continue;
            uint32_t taken = std::min(remaining, ch->new_buffer);
            ch->new_buffer -= taken;
            remaining      -= taken;
        }
    }

    /// 重置所有缓冲区状态
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (auto& [lcid, state] : lcgs_[i]) {
                state.old_buffer = 0;
                state.new_buffer = 0;
            }
        }
    }

    /// 获取各LCG的缓冲区大小 (用于BSR生成)
    std::array<uint32_t, NOF_LCGS> get_all_lcg_buffer_sizes() const {
        std::array<uint32_t, NOF_LCGS> sizes = {};
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            sizes[i] = get_buffer_state_lcg(i);
        }
        return sizes;
    }

private:
    uint32_t get_buffer_state_lcg_unlocked(uint32_t lcg_id) const {
        if (lcg_id >= NOF_LCGS) return 0;
        uint32_t total = 0;
        for (const auto& [lcid, state] : lcgs_[lcg_id]) {
            total += state.old_buffer;
        }
        return total;
    }

    /// 令牌桶步进的内部实现 (不加锁, 由公有方法加锁后调用)
    /// 每个逻辑信道: token_count += pbr * elapsed_ms, 不超过 token_bucket_size
    /// pbr=0的信道无PBR限制, 不需要补充令牌
    void step_token_buckets_unlocked(uint32_t elapsed_ms) {
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            for (auto& [lcid, state] : lcgs_[i]) {
                if (state.pbr == 0) continue;  // 无PBR限制, 跳过
                state.token_count += static_cast<double>(state.pbr) *
                                     static_cast<double>(elapsed_ms);
                if (state.token_count > state.token_bucket_size) {
                    state.token_count = state.token_bucket_size;
                }
            }
        }
    }

    // 每个LCG包含一个LCID到缓冲区状态的映射
    // 对应 srsRAN proc_bsr.h 中的 std::map<uint32_t, lcid_t> lcgs[NOF_LCG]
    std::map<uint32_t, lc_buffer_state> lcgs_[NOF_LCGS];
    mutable std::mutex mutex_;
};

} // namespace ul_mac
