// =============================================================================
// main.cpp - eNB侧上行MAC接收链路演示程序
//
// 【项目定位】基站(eNB/gNB)侧 L2 上行用户管理 - 接收链路集成
//   UE 侧原模块(bsr_manager/sr_manager/ul_harq_manager)保留作发送桩,
//   驱动数据到达 / SR / BSR 编码 / TB 发送; eNB 侧用三个核心模块构成接收链路:
//     1. enb_bsr_manager   - 解码 UE 上报的 BSR CE, 维护 per-UE LCG 缓冲区视图
//     2. ul_scheduler      - eNB 上行调度器 (SR/BSR 驱动, PRB 分配, HARQ PID 管理)
//     3. enb_ul_harq_manager - 接收 PUSCH TB, IR 软合并 + CRC, 产生 PHICH 反馈
//
// 【每 TTI 接收链路数据流】:
//   UE 桩: data_arrived → run_tti (触发 SR / 构造 BSR CE)
//   eNB:   handle_sr → enb_bsr.receive_bsr(解码) → scheduler.handle_bsr(查询 enb_bsr)
//          → scheduler.schedule_ul → enb_harq.receive_tb(软合并+CRC) → PHICH → scheduler.handle_ul_crc
//
// 演示场景:
//   - 场景1: 基本接收链路 (单UE, 验证 SR→BSR解码→Grant→HARQ软合并)
//   - 场景2: 多UE比例公平调度 (5 UE, 好信道, 一次成功率为主)
//   - 场景3: HARQ软合并重传 (弱信号新传 NACK, 重传 IR 增益后 ACK)
//   - 场景4: 增强功能 (UE 桩自适应 SR + 预测性 BSR, eNB 接收侧统计)
// =============================================================================

#include "ul_mac/ue_context.h"
#include "ul_mac/enb_ul_scheduler.h"
#include "ul_mac/enb_bsr_manager.h"
#include "ul_mac/enb_ul_harq_manager.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <vector>
#include <random>
#include <cstdlib>
#include <cstring>

// Windows 控制台默认使用 GBK(CP936) 代码页，会导致 UTF-8 字符串乱码。
// 在程序启动时将控制台切换为 UTF-8，确保中文输出在任意 PowerShell/cmd 下均正确显示。
// 注意: 不直接 #include <windows.h>，因为其中定义的 ERROR 宏会与 log_level::ERROR 冲突。
#ifdef _WIN32
extern "C" {
__declspec(dllimport) int __stdcall SetConsoleOutputCP(unsigned int cp);
__declspec(dllimport) int __stdcall SetConsoleCP(unsigned int cp);
}
#ifndef CP_UTF8
#define CP_UTF8 65001u
#endif
#endif

using namespace ul_mac;

// ============================================================================
// 辅助函数
// ============================================================================

/// 打印分隔线
void print_separator(const std::string& title) {
    std::cout << "\n";
    std::cout << "==============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "==============================================================\n\n";
}

/// 打印UE度量信息 (UE 发送桩侧统计)
void print_ue_metrics(const ue_metrics& m) {
    std::cout << "  UE[0x" << std::hex << m.rnti << std::dec << "]:\n";
    std::cout << "    SR:  tx=" << m.sr_tx_count
              << ", success=" << m.sr_success_count << "\n";
    std::cout << "    BSR: tx=" << m.bsr_tx_count << "\n";
    std::cout << "    HARQ: new_tx=" << m.harq_tx_count
              << ", retx=" << m.harq_retx_count
              << ", fail=" << m.harq_fail_count
              << ", avg_retx=" << std::fixed << std::setprecision(3)
              << m.avg_harq_retx << "\n";
    std::cout << "    UL:  bytes=" << m.total_ul_bytes
              << ", throughput=" << std::fixed << std::setprecision(2)
              << m.ul_throughput_kbps << " kbps\n";
}

/// 打印 eNB 侧接收链路统计 (BSR 解码 + HARQ 接收)
void print_enb_rx_stats(const enb_bsr_manager& enb_bsr,
                        const enb_ul_harq_manager& enb_harq) {
    auto bs = enb_bsr.get_stats();
    auto hs = enb_harq.get_stats();
    std::cout << "  [eNB BSR解码] total_rx=" << bs.total_bsr_rx
              << ", Short=" << bs.short_count
              << ", Truncated=" << bs.truncated_count
              << ", Long=" << bs.long_count << "\n";
    std::cout << "  [eNB HARQ接收] rx=" << hs.total_rx
              << ", new_tb=" << hs.total_new_tb
              << ", ACK=" << hs.total_ack
              << ", NACK=" << hs.total_nack
              << ", soft_combine=" << hs.total_soft_combine
              << ", discard=" << hs.total_discard << "\n";
}

/// 模拟 UE 侧 bsr_manager 编码 BSR CE (UE 发送桩)
/// 对应 bsr_manager::generate_bsr 中的 Long BSR 分支:
///   报告所有 buffer>0 的 LCG, 每个用 6-bit 索引
bsr_ce make_ue_bsr_ce(const uint32_t* lcg_sizes) {
    bsr_ce bsr;
    bsr.format = bsr_format::LONG_BSR;
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        if (lcg_sizes[i] > 0) {
            bsr.reports.push_back(
                bsr_report(static_cast<uint8_t>(i),
                           bytes_to_bsr_index(lcg_sizes[i])));
        }
    }
    return bsr;
}

/// eNB 侧处理某 UE 的 BSR 上报: 解码 → 喂给调度器
/// 数据流: UE 编码 bsr_ce → enb_bsr.receive_bsr 解码 → 调度器从 enb_bsr 查询 buffer
void enb_handle_bsr_ul(enb_bsr_manager& enb_bsr,
                       ul_scheduler& scheduler,
                       uint16_t rnti,
                       const uint32_t* lcg_sizes) {
    bsr_ce bsr = make_ue_bsr_ce(lcg_sizes);
    if (bsr.reports.empty()) return;
    // eNB 解码 BSR CE (维护 per-UE LCG 缓冲区视图)
    enb_bsr.receive_bsr(rnti, bsr);
    // 调度器从 eNB BSR 解码器查询各 LCG 缓冲区, 转成索引喂给调度器
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        uint32_t buf = enb_bsr.get_ul_buffer(rnti, static_cast<uint8_t>(i));
        if (buf > 0) {
            scheduler.handle_bsr(rnti, static_cast<uint8_t>(i),
                                 bytes_to_bsr_index(buf));
        }
    }
}

// ============================================================================
// 场景1: 基本接收链路演示 (好信道, 一次成功率为主)
// ============================================================================

void scenario1_basic_ul_scheduling() {
    print_separator("场景1: 基本接收链路 (SR -> BSR解码 -> Grant -> HARQ软合并)");

    mac_logger::instance().set_level(log_level::INFO);

    // UE 发送桩
    ue_context ue(0x0001);
    ue.setup_lcid(1, 0, 1);   // SRB1, LCG=0, priority=1
    ue.setup_lcid(2, 1, 8);   // DRB,  LCG=1, priority=8
    ue.setup_lcid(3, 2, 10);  // DRB,  LCG=2, priority=10

    // eNB 侧三个核心模块
    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);
    scheduler.add_ue(0x0001);
    enb_bsr.add_ue(0x0001);
    enb_harq.add_ue(0x0001);
    enb_harq.set_ul_snr(0x0001, 2000); // 好信道 (20dB), MCS=7 解码阈值300, 一次成功

    std::cout << ">>> 仿真开始: 200 TTI\n";
    std::cout << ">>> UE 0x0001: 3个逻辑通道 (SRB1 + 2 DRB)\n";
    std::cout << ">>> eNB: PF调度 + BSR解码 + HARQ软合并 (ul_snr=20dB)\n\n";

    for (uint32_t tti = 0; tti < 200; tti++) {
        // === UE 桩: 数据到达 + run_tti (触发 SR/BSR) ===
        if (tti % 10 == 0) ue.data_arrived(2, 500);
        if (tti % 20 == 5) ue.data_arrived(3, 300);
        ue.run_tti(tti);

        // === eNB: 处理 SR ===
        if (ue.get_sr_manager().get_state() == sr_state::PENDING) {
            scheduler.handle_sr(ue.get_rnti());
        }

        // === eNB: BSR 解码 (UE 编码 → enb_bsr 解码 → 调度器查询) ===
        auto lcg_sizes = ue.get_buffer_manager().get_all_lcg_buffer_sizes();
        enb_handle_bsr_ul(enb_bsr, scheduler, ue.get_rnti(), lcg_sizes.data());

        // === eNB: 上行调度 ===
        auto sched_results = scheduler.schedule_ul(tti);

        // === eNB: 接收 TB (软合并+CRC) → PHICH → 调度器 CRC ===
        for (const auto& res : sched_results) {
            if (res.rnti != ue.get_rnti()) continue;
            ul_grant grant = res.grant;
            auto rx = enb_harq.receive_tb(ue.get_rnti(), grant);
            grant.phich_available = true;
            grant.hi_value = rx.crc_ok; // PHICH 反馈值由 eNB 接收端决定

            // UE 桩: 处理授权 + 处理 PHICH
            ue.handle_ul_grant(grant);
            ue.handle_harq_feedback(grant.pid, grant.hi_value);

            // eNB 调度器: 处理 CRC (丢弃视为 ACK 清重传标志)
            scheduler.handle_ul_crc(ue.get_rnti(), grant.pid,
                                    rx.discarded ? true : rx.crc_ok);
        }
    }

    metrics_collector::instance().set_simulation_tti(200);
    print_ue_metrics(ue.get_metrics());
    print_enb_rx_stats(enb_bsr, enb_harq);
}

// ============================================================================
// 场景2: 多UE比例公平调度 (好信道, 一次成功率为主)
// ============================================================================

void scenario2_multi_ue_pf() {
    print_separator("场景2: 多UE比例公平调度 (5个UE, 好信道)");

    mac_logger::instance().set_level(log_level::WARNING);

    std::vector<std::unique_ptr<ue_context>> ues;
    uint16_t rntis[] = {0x0001, 0x0002, 0x0003, 0x0004, 0x0005};

    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);

    for (int i = 0; i < 5; i++) {
        ues.push_back(std::make_unique<ue_context>(rntis[i]));
        ues.back()->setup_lcid(2, 0, 8); // DRB, LCG=0, priority=8
        scheduler.add_ue(rntis[i]);
        enb_bsr.add_ue(rntis[i]);
        enb_harq.add_ue(rntis[i]);
        enb_harq.set_ul_snr(rntis[i], 2000); // 好信道
    }

    std::cout << ">>> 仿真开始: 1000 TTI, 5个UE, PF调度\n";
    std::cout << ">>> 每个UE每5个TTI产生500字节数据, ul_snr=20dB\n\n";

    for (uint32_t tti = 0; tti < 1000; tti++) {
        // UE 桩
        for (int i = 0; i < 5; i++) {
            if (tti % 5 == static_cast<uint32_t>(i % 5)) {
                ues[i]->data_arrived(2, 500);
            }
            ues[i]->run_tti(tti);
        }

        // eNB: SR + BSR 解码
        for (int i = 0; i < 5; i++) {
            if (ues[i]->get_sr_manager().get_state() == sr_state::PENDING) {
                scheduler.handle_sr(rntis[i]);
            }
            auto sizes = ues[i]->get_buffer_manager().get_all_lcg_buffer_sizes();
            enb_handle_bsr_ul(enb_bsr, scheduler, rntis[i], sizes.data());
        }

        // eNB: 调度 + 接收
        auto results = scheduler.schedule_ul(tti);
        for (const auto& res : results) {
            for (int i = 0; i < 5; i++) {
                if (res.rnti != rntis[i]) continue;
                ul_grant grant = res.grant;
                auto rx = enb_harq.receive_tb(rntis[i], grant);
                grant.phich_available = true;
                grant.hi_value = rx.crc_ok;
                ues[i]->handle_ul_grant(grant);
                ues[i]->handle_harq_feedback(grant.pid, grant.hi_value);
                scheduler.handle_ul_crc(rntis[i], grant.pid,
                                        rx.discarded ? true : rx.crc_ok);
                break;
            }
        }
    }

    metrics_collector::instance().set_simulation_tti(1000);
    for (int i = 0; i < 5; i++) {
        print_ue_metrics(ues[i]->get_metrics());
    }
    print_enb_rx_stats(enb_bsr, enb_harq);
}

// ============================================================================
// 场景3: HARQ 软合并重传 (弱信号新传 NACK, 重传 IR 增益后 ACK)
// ============================================================================

void scenario3_harq_retx() {
    print_separator("场景3: HARQ软合并重传 (弱信道, 验证IR增益)");

    mac_logger::instance().set_level(log_level::INFO);

    ue_context ue(0x0001);
    ue.setup_lcid(2, 0, 8);

    // UE 桩 HARQ 配置 (与 eNB 侧 max 一致, 保证两套状态机同步)
    ul_harq_config harq_cfg;
    harq_cfg.max_harq_tx = 4;
    ue.set_harq_config(harq_cfg);

    // eNB 侧
    ul_scheduler scheduler(sched_algorithm::ROUND_ROBIN, 50);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4); // max_harq_tx=4, 与 UE 桩一致
    scheduler.add_ue(0x0001);
    enb_bsr.add_ue(0x0001);
    enb_harq.add_ue(0x0001);
    // 弱信道: scheduler ul_snr=200 → MCS=7 (选择阈值200), 解码阈值=200+100=300
    //   新传: eff=200 < 300 → NACK
    //   重传: eff=200+200(IR)=400 >= 300 → ACK (体现 HARQ 软合并价值)
    enb_harq.set_ul_snr(0x0001, 200);

    std::cout << ">>> 仿真开始: 500 TTI\n";
    std::cout << ">>> ul_snr=2dB (MCS=7 边缘), 新传NACK, 重传IR增益后ACK\n";
    std::cout << ">>> 最大HARQ重传: 4次\n\n";

    uint32_t retx_count = 0;
    uint32_t fail_count = 0;

    for (uint32_t tti = 0; tti < 500; tti++) {
        if (tti % 5 == 0) ue.data_arrived(2, 1000);
        ue.run_tti(tti);

        if (ue.get_sr_manager().get_state() == sr_state::PENDING) {
            scheduler.handle_sr(0x0001);
        }
        auto sizes = ue.get_buffer_manager().get_all_lcg_buffer_sizes();
        enb_handle_bsr_ul(enb_bsr, scheduler, 0x0001, sizes.data());

        auto results = scheduler.schedule_ul(tti);
        for (const auto& res : results) {
            ul_grant grant = res.grant;
            auto rx = enb_harq.receive_tb(0x0001, grant);
            grant.phich_available = true;
            grant.hi_value = rx.crc_ok;

            auto action = ue.handle_ul_grant(grant); // UE 桩
            if (action.is_retx) retx_count++;
            if (rx.discarded) fail_count++;

            ue.handle_harq_feedback(grant.pid, grant.hi_value);
            scheduler.handle_ul_crc(0x0001, grant.pid,
                                    rx.discarded ? true : rx.crc_ok);
        }
    }

    metrics_collector::instance().set_simulation_tti(500);

    auto m = ue.get_metrics();
    print_ue_metrics(m);
    print_enb_rx_stats(enb_bsr, enb_harq);
    std::cout << "\n  HARQ重传统计 (UE 桩视角):\n";
    std::cout << "    总重传次数: " << retx_count << "\n";
    std::cout << "    丢弃次数(达到最大重传): " << fail_count << "\n";
    std::cout << "    平均每包重传: " << std::fixed << std::setprecision(3)
              << m.avg_harq_retx << "\n";

    auto harq_stats = ue.get_harq_manager().get_stats();
    std::cout << "    UE桩 BLER: " << std::fixed << std::setprecision(2)
              << harq_stats.bler * 100 << "%\n";
    std::cout << "    UE桩 ACK: " << harq_stats.total_ack
              << ", NACK: " << harq_stats.total_nack << "\n";
}

// ============================================================================
// 场景4: 增强功能演示 (UE 桩自适应SR + 预测性BSR, eNB 接收侧统计)
// ============================================================================

void scenario4_enhanced_features() {
    print_separator("场景4: 增强功能 (UE桩自适应SR + 预测性BSR, eNB接收侧统计)");

    mac_logger::instance().set_level(log_level::INFO);

    ue_context ue(0x0001);
    ue.setup_lcid(2, 0, 8);
    ue.setup_lcid(3, 1, 10);

    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);
    scheduler.add_ue(0x0001);
    enb_bsr.add_ue(0x0001);
    enb_harq.add_ue(0x0001);
    enb_harq.set_ul_snr(0x0001, 2000); // 好信道

    std::cout << ">>> 仿真开始: 500 TTI\n";
    std::cout << ">>> 演示 UE 桩自适应 SR 周期 + 预测性 BSR, eNB 侧统计接收\n\n";

    for (uint32_t tti = 0; tti < 500; tti++) {
        // 模拟变化的流量模式
        double traffic_rate;
        if (tti < 100) {
            if (tti % 20 == 0) ue.data_arrived(2, 100);
            traffic_rate = 5.0;
        } else if (tti < 300) {
            if (tti % 5 == 0) ue.data_arrived(2, 1000);
            traffic_rate = 200.0;
        } else {
            if (tti % 10 == 0) ue.data_arrived(2, 500);
            traffic_rate = 50.0;
        }

        // UE 桩: 自适应 SR 周期调整
        ue.get_sr_manager().adjust_sr_period(traffic_rate);
        ue.run_tti(tti);

        // UE 桩: 预测性 BSR (UE 侧预测, eNB 侧不关心, 仅日志)
        if (tti % 50 == 0 && tti > 0) {
            uint32_t predicted = ue.get_bsr_manager().predict_buffer_demand();
            LOG_INFO("BSR", 0x0001, tti,
                "UE predicted buffer demand: " + std::to_string(predicted) + " bytes");
        }

        if (ue.get_sr_manager().get_state() == sr_state::PENDING) {
            scheduler.handle_sr(0x0001);
        }
        auto sizes = ue.get_buffer_manager().get_all_lcg_buffer_sizes();
        enb_handle_bsr_ul(enb_bsr, scheduler, 0x0001, sizes.data());

        auto results = scheduler.schedule_ul(tti);
        for (const auto& res : results) {
            ul_grant grant = res.grant;
            auto rx = enb_harq.receive_tb(0x0001, grant);
            grant.phich_available = true;
            grant.hi_value = rx.crc_ok;
            ue.handle_ul_grant(grant);
            ue.handle_harq_feedback(grant.pid, grant.hi_value);
            scheduler.handle_ul_crc(0x0001, grant.pid,
                                    rx.discarded ? true : rx.crc_ok);
        }

        if (tti % 100 == 99) {
            auto sr_stats = ue.get_sr_manager().get_stats();
            std::cout << "  TTI=" << tti
                      << ", SR period=" << sr_stats.current_sr_period << "ms"
                      << ", SR sent=" << sr_stats.total_sr_sent
                      << ", SR success=" << sr_stats.total_sr_success << "\n";
        }
    }

    metrics_collector::instance().set_simulation_tti(500);
    std::cout << "\n";
    print_ue_metrics(ue.get_metrics());
    print_enb_rx_stats(enb_bsr, enb_harq);

    auto sr_stats = ue.get_sr_manager().get_stats();
    std::cout << "\n  UE 桩自适应SR统计:\n";
    std::cout << "    最终SR周期: " << sr_stats.current_sr_period << "ms\n";
    std::cout << "    总SR发送: " << sr_stats.total_sr_sent << "\n";
    std::cout << "    SR成功: " << sr_stats.total_sr_success << "\n";
    std::cout << "    SR失败: " << sr_stats.total_sr_fail << "\n";

    auto bsr_stats = ue.get_bsr_manager().get_stats();
    std::cout << "\n  UE 桩 BSR 发送统计:\n";
    std::cout << "    总BSR发送: " << bsr_stats.total_bsr_sent << "\n";
    std::cout << "    Regular: " << bsr_stats.regular_count << "\n";
    std::cout << "    Periodic: " << bsr_stats.periodic_count << "\n";
    std::cout << "    Padding: " << bsr_stats.padding_count << "\n";
    std::cout << "    Short: " << bsr_stats.short_count << "\n";
    std::cout << "    Long: " << bsr_stats.long_count << "\n";
    std::cout << "    Truncated: " << bsr_stats.truncated_count << "\n";
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "==============================================================\n";
    std::cout << "   eNB侧上行MAC接收链路演示项目 (UL MAC Manager - eNB side)\n";
    std::cout << "   基于3GPP TS 36.321 (LTE) / TS 38.321 (NR) MAC协议\n";
    std::cout << "   eNB核心模块: enb_bsr_manager + ul_scheduler + enb_ul_harq_manager\n";
    std::cout << "   UE侧模块保留作发送桩 (方案A原地改造)\n";
    std::cout << "==============================================================\n";

    metrics_collector::instance().reset();

    scenario1_basic_ul_scheduling();
    metrics_collector::instance().reset();

    scenario2_multi_ue_pf();
    metrics_collector::instance().reset();

    scenario3_harq_retx();
    metrics_collector::instance().reset();

    scenario4_enhanced_features();

    print_separator("最终系统性能指标");
    metrics_collector::instance().print_summary();

    std::cout << "\n演示完成。eNB侧上行MAC接收链路核心机制:\n";
    std::cout << "  - BSR解码 (enb_bsr_manager): UE编码CE → eNB解码 → per-UE LCG视图\n";
    std::cout << "  - 上行调度 (ul_scheduler): SR/BSR驱动 + PRB分配 + HARQ PID管理\n";
    std::cout << "  - HARQ接收 (enb_ul_harq_manager): IR软合并 + CRC + PHICH反馈\n";
    std::cout << "  - 调度算法: PF/RR/优先级调度\n";
    std::cout << "  - UE桩增强: 自适应SR、预测性BSR (保留作发送端行为)\n";

    return 0;
}
