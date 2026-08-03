// =============================================================================
// metrics_collector.h - 性能指标收集器
//
// 收集和统计MAC层上行管理的关键性能指标:
//   - SR发送/成功/失败率
//   - BSR发送次数和格式分布
//   - HARQ新传/重传/失败次数和平均重传率
//   - 上行吞吐率和延迟统计
// 参考 srsRAN_4G/srsenb/hdr/stack/mac/common/mac_metrics.h 中的指标设计
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include <mutex>
#include <numeric>
#include <algorithm>
#include <cstring>

namespace ul_mac {

/// 系统级性能指标
struct system_metrics {
    uint32_t total_sr_tx;           ///< 系统总SR发送次数
    uint32_t total_sr_success;      ///< 系统总SR成功次数
    uint32_t total_sr_fail;         ///< 系统总SR失败次数
    uint32_t total_bsr_tx;          ///< 系统总BSR发送次数
    uint32_t total_bsr_short;       ///< Short BSR次数
    uint32_t total_bsr_long;        ///< Long BSR次数
    uint32_t total_bsr_truncated;   ///< Truncated BSR次数
    uint32_t total_harq_new_tx;     ///< HARQ新传总次数
    uint32_t total_harq_retx;       ///< HARQ重传总次数
    uint32_t total_harq_fail;       ///< HARQ失败总次数
    uint32_t total_ul_bytes;        ///< 上行总字节数
    uint32_t total_ul_grants;       ///< 上行总授权数
    uint32_t simulation_tti;        ///< 仿真TTI数

    system_metrics() { reset(); }

    void reset() {
        memset(this, 0, sizeof(*this));
    }

    /// 计算SR成功率
    double sr_success_rate() const {
        if (total_sr_tx == 0) return 0.0;
        return static_cast<double>(total_sr_success) / total_sr_tx * 100.0;
    }

    /// 计算平均HARQ重传次数
    double avg_harq_retx() const {
        if (total_harq_new_tx == 0) return 0.0;
        return static_cast<double>(total_harq_retx) / total_harq_new_tx;
    }

    /// 计算HARQ失败率
    double harq_fail_rate() const {
        uint32_t total = total_harq_new_tx + total_harq_retx;
        if (total == 0) return 0.0;
        return static_cast<double>(total_harq_fail) / total * 100.0;
    }

    /// 计算系统吞吐率 (kbps)
    double system_throughput_kbps() const {
        if (simulation_tti == 0) return 0.0;
        // 每TTI = 1ms, 吞吐率 = bytes * 8 / (tti * 0.001) / 1000 = bytes * 8 / tti
        return static_cast<double>(total_ul_bytes) * 8.0 / simulation_tti;
    }

    /// 打印指标摘要
    void print_summary() const {
        std::cout << "\n";
        std::cout << "==============================================================\n";
        std::cout << "              MAC层上行管理系统性能指标摘要\n";
        std::cout << "==============================================================\n";
        std::cout << "仿真时长:           " << simulation_tti << " TTI ("
                  << simulation_tti << " ms)\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "【调度请求 (SR)】\n";
        std::cout << "  总发送次数:        " << total_sr_tx << "\n";
        std::cout << "  成功次数:          " << total_sr_success << "\n";
        std::cout << "  失败次数:          " << total_sr_fail << "\n";
        std::cout << "  成功率:            " << std::fixed << std::setprecision(2)
                  << sr_success_rate() << "%\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "【缓冲区状态报告 (BSR)】\n";
        std::cout << "  总发送次数:        " << total_bsr_tx << "\n";
        std::cout << "  Short BSR:         " << total_bsr_short << "\n";
        std::cout << "  Long BSR:          " << total_bsr_long << "\n";
        std::cout << "  Truncated BSR:     " << total_bsr_truncated << "\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "【上行HARQ】\n";
        std::cout << "  新传次数:          " << total_harq_new_tx << "\n";
        std::cout << "  重传次数:          " << total_harq_retx << "\n";
        std::cout << "  失败次数:          " << total_harq_fail << "\n";
        std::cout << "  平均重传次数:      " << std::fixed << std::setprecision(3)
                  << avg_harq_retx() << "\n";
        std::cout << "  失败率:            " << std::fixed << std::setprecision(2)
                  << harq_fail_rate() << "%\n";
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "【上行吞吐】\n";
        std::cout << "  总授权数:          " << total_ul_grants << "\n";
        std::cout << "  总传输字节:        " << total_ul_bytes << " bytes\n";
        std::cout << "  系统吞吐率:        " << std::fixed << std::setprecision(2)
                  << system_throughput_kbps() << " kbps\n";
        std::cout << "==============================================================\n";
    }
};

/// 延迟统计信息
/// 记录端到端延迟分布, 用于计算P50/P90/P99等百分位指标
struct latency_stats {
    std::vector<uint32_t> latencies;  ///< 所有延迟样本(TTI)
    uint32_t p50;  ///< 中位数延迟
    uint32_t p90;  ///< 90分位延迟
    uint32_t p99;  ///< 99分位延迟
    uint32_t min;  ///< 最小延迟
    uint32_t max;  ///< 最大延迟
    double avg;    ///< 平均延迟

    latency_stats() : p50(0), p90(0), p99(0), min(0), max(0), avg(0.0) {}

    void reset() {
        latencies.clear();
        p50 = p90 = p99 = min = max = 0;
        avg = 0.0;
    }

    /// 计算百分位数
    void compute_percentiles() {
        if (latencies.empty()) return;
        std::sort(latencies.begin(), latencies.end());
        min = latencies.front();
        max = latencies.back();
        size_t n = latencies.size();
        avg = static_cast<double>(std::accumulate(latencies.begin(), latencies.end(), 0ULL)) / n;
        p50 = latencies[n * 50 / 100];
        p90 = latencies[n * 90 / 100];
        p99 = latencies[n * 99 / 100];
    }
};

/// 全局性能指标收集器
class metrics_collector {
public:
    static metrics_collector& instance() {
        static metrics_collector inst;
        return inst;
    }

    void record_sr_tx() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_sr_tx++;
    }

    void record_sr_success() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_sr_success++;
    }

    void record_sr_fail() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_sr_fail++;
    }

    void record_bsr_tx(bsr_format fmt) {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_bsr_tx++;
        switch (fmt) {
            case bsr_format::SHORT_BSR:     metrics_.total_bsr_short++;     break;
            case bsr_format::LONG_BSR:      metrics_.total_bsr_long++;      break;
            case bsr_format::TRUNCATED_BSR: metrics_.total_bsr_truncated++; break;
        }
    }

    void record_harq_new_tx() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_harq_new_tx++;
    }

    void record_harq_retx() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_harq_retx++;
    }

    void record_harq_fail() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_harq_fail++;
    }

    void record_ul_bytes(uint32_t bytes) {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_ul_bytes += bytes;
    }

    void record_ul_grant() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.total_ul_grants++;
    }

    void set_simulation_tti(uint32_t tti) {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.simulation_tti = tti;
    }

    /// 记录延迟样本 (数据从到达 to 发送成功的TTI数)
    void record_latency(uint32_t latency_ttis) {
        std::lock_guard<std::mutex> lock(mutex_);
        latency_stats_.latencies.push_back(latency_ttis);
    }

    /// 获取延迟统计
    latency_stats get_latency_stats() {
        std::lock_guard<std::mutex> lock(mutex_);
        latency_stats_.compute_percentiles();
        return latency_stats_;
    }

    void print_summary() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.print_summary();

        // 延迟统计
        latency_stats_.compute_percentiles();
        std::cout << "--------------------------------------------------------------\n";
        std::cout << "【延迟统计】\n";
        std::cout << "  样本数:            " << latency_stats_.latencies.size() << "\n";
        std::cout << "  最小延迟:          " << latency_stats_.min << " TTI\n";
        std::cout << "  P50 (中位数):      " << latency_stats_.p50 << " TTI\n";
        std::cout << "  P90:               " << latency_stats_.p90 << " TTI\n";
        std::cout << "  P99:               " << latency_stats_.p99 << " TTI\n";
        std::cout << "  最大延迟:          " << latency_stats_.max << " TTI\n";
        std::cout << "  平均延迟:          " << std::fixed << std::setprecision(2)
                  << latency_stats_.avg << " TTI\n";
        std::cout << "==============================================================\n";
    }

    system_metrics get_metrics() {
        std::lock_guard<std::mutex> lock(mutex_);
        return metrics_;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        metrics_.reset();
        latency_stats_.reset();
    }

private:
    metrics_collector() {}
    std::mutex mutex_;
    system_metrics metrics_;
    latency_stats latency_stats_;
};

} // namespace ul_mac
