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
// 【每 TTI 接收链路数据流 (方案B: BSR 随 PUSCH 上报)】:
//   UE 桩: data_arrived → run_tti (触发 SR / 构造 BSR CE)
//   eNB:   handle_sr (PUCCH 1-bit 提示) → scheduler.schedule_ul (给 grant)
//          → enb_harq.receive_tb(软合并+CRC) → UE 打包 BSR 进 PUSCH MAC PDU
//          → enb_bsr 解 PUSCH PDU 取 BSR → scheduler.handle_bsr(查询 enb_bsr)
//          → PHICH → scheduler.handle_ul_crc
//   (SR 走 PUCCH、BSR 随 PUSCH MAC CE, 两条独立空口路径)
//
// 演示场景:
//   - 场景1: 基本接收链路 (单UE, 验证 SR→BSR解码→Grant→HARQ软合并)
//   - 场景2: 多UE比例公平调度 (5 UE, 好信道, 一次成功率为主)
//   - 场景3: HARQ软合并重传 (弱信号新传 NACK, 重传 IR 增益后 ACK)
//   - 场景4: 增强功能 (UE 桩自适应 SR, eNB 接收侧统计)
// =============================================================================

#include "ul_mac/ue_context.h"
#include "ul_mac/enb_ul_scheduler.h"
#include "ul_mac/enb_bsr_manager.h"
#include "ul_mac/enb_ul_harq_manager.h"
#include "ul_mac/mac_pdu.h"
#include "ul_mac/tti_channel.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"
#include <vector>
#include <random>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <future>
#include <atomic>

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

// ============================================================================
// 公共 worker 模板与注册辅助 (重构: 原先各场景内联的 10 份 TTI 循环已统一)
// ============================================================================

/// 通用 TTI worker 循环模板 (原 5 个场景内联重复的循环骨架)
/// 统一处理: wait_for_tti -> done 检查 -> body(tti) -> mark_tti_done。
/// 【异常安全】body 抛出的异常在此捕获并记录, 但仍保证 mark_tti_done 被调用:
/// 中央时钟是屏障同步, 任一 worker 不汇报完成会导致其余 worker 永久阻塞
/// (整个仿真死锁), 因此"吞异常继续推进"是屏障模型下唯一安全的降级策略。
template <typename Fn>
void run_tti_worker(tti_clock& clock, uint32_t max_tti, Fn&& body) {
    for (uint32_t tti = 0; tti <= max_tti; ++tti) {
        clock.wait_for_tti(tti);
        if (clock.done()) break;
        try {
            body(tti);
        } catch (const std::exception& e) {
            std::cerr << "[worker] TTI=" << tti
                      << " exception: " << e.what() << " (continuing)\n";
        } catch (...) {
            std::cerr << "[worker] TTI=" << tti
                      << " unknown exception (continuing)\n";
        }
        clock.mark_tti_done();
    }
}

/// 【事务化注册】将 UE 一次性注册到 eNB 三个核心模块
/// 原先 add_ue 分三处散落调用, 漏掉任一处会导致该模块对未知 RNTI 静默丢包
/// (如 receive_tb 对未注册 UE 直接 return)。集中为一个函数消除遗漏可能。
void register_ue_at_enb(ul_scheduler& scheduler,
                        enb_bsr_manager& enb_bsr,
                        enb_ul_harq_manager& enb_harq,
                        uint16_t rnti) {
    scheduler.add_ue(rnti);
    enb_bsr.add_ue(rnti);
    enb_harq.add_ue(rnti);
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

/// eNB 侧处理某 UE 的 BSR 上报: 解 PUSCH PDU → 取 BSR CE → 解码 → 喂给调度器
/// 数据流 (方案B, 贴合真实协议空口):
///   UE 打包: UE 缓冲(字节) → build_long_bsr 编码为 bsr_ce(6-bit索引)
///           → mac_pdu_packer 打包为 UL-SCH MAC PDU 字节流 (随 PUSCH 上报)
///   eNB 解包: mac_pdu_unpacker 解 PUSCH PDU → 取出 BSR CE
///           → enb_bsr.receive_bsr 解码回字节下界(写 eNB LCG 视图)
///           → 再 bytes_to_bsr_index 转回索引 → scheduler.handle_bsr 写入 eNB 调度器 UE 队列的缓冲变量
///
/// 【方案B 说明】与方案A(结构化 bsr_ce 内存直传, 未经 PDU 打包/解包)不同, 此处
/// BSR 真正以 MAC CE 形式随 PUSCH 的 MAC PDU 字节流传空口, 由 eNB 侧解包取出 ——
/// 即真实协议链路: SR(PUCCH 1-bit) → eNB 给 grant → UE 在 PUSCH 携带 BSR → eNB 解包取 BSR。
void enb_handle_bsr_pdu(enb_bsr_manager& enb_bsr,
                         ul_scheduler& scheduler,
                         uint16_t rnti,
                         const std::vector<uint8_t>& pdu) {
    if (pdu.empty()) return;
    // eNB 解包 PUSCH MAC PDU, 取出其中的 BSR CE (无 SDU 复用场景下仅含 BSR CE + Padding)
    auto r = mac_pdu_unpacker::unpack(pdu.data(), pdu.size());
    if (r.bsr.reports.empty()) return;
    // eNB 解码 BSR CE: 索引 → 字节下界, 写入 per-UE LCG 缓冲区视图
    enb_bsr.receive_bsr(rnti, r.bsr);
    // 调度器从 eNB BSR 解码器查询各 LCG 缓冲区, 转成索引写入 eNB 调度器 UE 队列的缓冲变量
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
    print_separator("场景1: 基本接收链路 (SR -> BSR解码 -> Grant -> HARQ软合并) [方案C: 多线程 + 4-TTI时延]");

    mac_logger::instance().set_level(log_level::INFO);

    // ---- 单 UE 桩 + eNB 三核心模块 ----
    uint16_t rnti = 0x0001;
    ue_context ue(rnti);
    ue.setup_lcid(1, 0, 1);   // SRB1, LCG=0, priority=1
    ue.setup_lcid(2, 1, 8);   // DRB,  LCG=1, priority=8
    ue.setup_lcid(3, 2, 10);  // DRB,  LCG=2, priority=10

    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);
    register_ue_at_enb(scheduler, enb_bsr, enb_harq, rnti); // 事务化: 三模块一次注册
    enb_harq.set_ul_snr(rnti, 2000); // 好信道 (20dB), MCS=7 解码阈值300, 一次成功

    // ---- 方案C: 中央TTI时钟(屏障同步, 2个worker: UE线程+eNB线程) + 四空口信道 ----
    // 复用场景2/3 已实现的 tti_channel + 中央时钟机制, 让场景1 也具备真实 +4 TTI 空口时延,
    // 且 SR/PDCCH/PUSCH/PHICH 四条信道物理分离, 时序清晰对应协议步骤:
    //   [UE] 数据到达→触发SR ──SR(PUCCH,+4)──▶ [eNB] 收SR→预填待传量
    //   [eNB] 调度→发Grant(PDCCH,+4) ──▶ [UE] 收Grant→发PUSCH(含BSR CE)
    //   [UE] 发PUSCH(+4) ──▶ [eNB] 收PUSCH→解BSR→CRC→发PHICH(+4) ──▶ [UE] 收PHICH
    constexpr uint32_t MAX_TTI = 200;
    tti_clock clock(MAX_TTI, 2);
    timed_channel<sr_msg>            sr_ch;     // UE->eNB (PUCCH)
    per_rnti_channel_map<ul_grant>   grant_ch;  // eNB->UE (PDCCH, 点对点)
    timed_channel<pdu_msg>           pdu_ch;    // UE->eNB (PUSCH)
    per_rnti_channel_map<phich_msg>  phich_ch;  // eNB->UE (PHICH, 点对点)
    grant_ch.add_rnti(rnti);
    phich_ch.add_rnti(rnti);

    std::cout << ">>> 仿真开始: " << MAX_TTI << " TTI, 单UE, PF调度\n";
    std::cout << ">>> UE 0x0001: 3个逻辑通道 (SRB1 + 2 DRB)\n";
    std::cout << ">>> eNB: PF调度 + BSR解码 + HARQ软合并 (ul_snr=20dB)\n";
    std::cout << ">>> 中央TTI时钟线程 + 四条空口信道(各+"
              << CHANNEL_PROPAGATION_TTI << " TTI可见时延)\n\n";

    // ---- UE 线程: 数据到达 + 触发SR + 收Grant发PUSCH + 收PHICH ----
    std::thread ue_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            // 1) 数据到达 + run_tti (触发 SR / 构造 BSR)
            if (tti % 10 == 0) ue.data_arrived(2, 500);
            if (tti % 20 == 5) ue.data_arrived(3, 300);
            ue.run_tti(tti);

            // 2) SR (PUCCH): 仅真正发送的那个 TTI 入队, +4 TTI 后 eNB 可见
            if (ue.get_sr_manager().take_sr_transmitted()) {
                // srsRAN 式: SR 触发同时把 UE 本地待传字节数随 SR 信道告知 eNB,
                // 使调度器在 BSR CE 经 PUSCH 解包前即可按真实量分配 (跳过"小Grant探测")
                sr_ch.enqueue({rnti, ue.get_buffer_manager().get_all_lcg_buffer_sizes()}, tti);
            }

            // 3) 收 UL Grant (PDCCH): 只取本 RNTI 专属信道, 且已度过传播时延
            while (auto g = grant_ch.at(rnti).dequeue(tti)) {
                ul_grant grant = g.value();
                ue.handle_ul_grant(grant);          // 收Grant + 发PUSCH(打包BSR CE)
                pdu_msg m;
                m.rnti = rnti;
                m.grant = grant;
                m.pdu = ue.get_last_pdu();          // PUSCH 空口承载的 MAC PDU
                pdu_ch.enqueue(m, tti);             // 上报 eNB (+4 TTI 可见)
            }

            // 4) 收 PHICH (HARQ 反馈): 本 RNTI 专属信道
            while (auto ph = phich_ch.at(rnti).dequeue(tti)) {
                ue.handle_harq_feedback(ph.value().pid, ph.value().ack);
            }
        });
    });

    // ---- eNB 线程: 收SR + 调度 + 收PUSCH解BSR + 发PHICH ----
    std::thread enb_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            // 1) 处理可见的 SR (PUCCH, +4 TTI 后才到)
            while (auto s = sr_ch.dequeue(tti)) {
                scheduler.handle_sr(s.value().rnti, s.value().pending_bytes);
            }

            // 2) 上行调度 → 经 PDCCH 下发 Grant (send_tti=tti, +4 TTI 后 UE 可见)
            auto results = scheduler.schedule_ul(tti);
            for (const auto& res : results) {
                grant_ch.at(res.rnti).enqueue(res.grant, tti);
            }

            // 3) 处理可见的 PUSCH PDU → 解 BSR + HARQ 软合并 + 发 PHICH
            //    【方案C】解包与HARQ接收串行 (单UE, 无需并行)
            while (auto m = pdu_ch.dequeue(tti)) {
                pdu_msg msg = m.value();
                // eNB 解 PUSCH PDU 取 BSR CE → 喂调度器 (方案B 空口路径)
                enb_handle_bsr_pdu(enb_bsr, scheduler, msg.rnti, msg.pdu);
                // eNB 用此前下发的 ul_grant 收 TB (软合并+CRC)
                auto rx = enb_harq.receive_tb(msg.rnti, msg.grant);
                // PHICH 反馈按目的 RNTI 投递到专属信道 (独立物理信道, 非 Grant 一部分)
                phich_msg pm;
                pm.rnti = msg.rnti;
                pm.pid = msg.grant.pid;
                pm.ack = rx.crc_ok;
                phich_ch.at(msg.rnti).enqueue(pm, tti);
                scheduler.handle_ul_crc(msg.rnti, pm.pid, rx.discarded ? true : rx.crc_ok);
            }
        });
    });

    // 启动中央时钟 (主线程驱动) + 等待 worker 结束
    clock.run();
    ue_thread.join();
    enb_thread.join();

    metrics_collector::instance().set_simulation_tti(MAX_TTI);
    print_ue_metrics(ue.get_metrics());
    print_enb_rx_stats(enb_bsr, enb_harq);
}

// ============================================================================
// 场景2: 多UE比例公平调度 (好信道, 一次成功率为主)
// ============================================================================

void scenario2_multi_ue_pf() {
    print_separator("场景2: 多UE比例公平调度 (5个UE, 好信道) [方案C: 多线程 + 4-TTI时延]");

    mac_logger::instance().set_level(log_level::WARNING);

    constexpr int NOF = 5;
    constexpr uint32_t MAX_TTI = 1000;
    uint16_t rntis[NOF] = {0x0001, 0x0002, 0x0003, 0x0004, 0x0005};

    // eNB 侧核心模块 (多线程并发访问, 内部已加锁)
    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);

    // 每个 UE 一个独立线程 (UE 桩独占, 无需额外锁)
    std::vector<std::unique_ptr<ue_context>> ues;
    for (int i = 0; i < NOF; i++) {
        ues.push_back(std::make_unique<ue_context>(rntis[i]));
        ues.back()->setup_lcid(2, 0, 8); // DRB, LCG=0, priority=8
        register_ue_at_enb(scheduler, enb_bsr, enb_harq, rntis[i]); // 事务化注册
        enb_harq.set_ul_snr(rntis[i], 2000); // 好信道
    }

    // ---- 方案C: 中央TTI时钟(屏障同步) + 四条空口信道 ----
    // 屏障 worker 数 = 5 个 UE 线程 + 1 个 eNB 线程: 时钟每 TTI 等全部
    // worker 汇报完成后才推进, 杜绝"时钟跑在前面、worker 丢 TTI"的竞态。
    tti_clock clock(MAX_TTI, NOF + 1);
    timed_channel<sr_msg>            sr_ch;     // UE->eNB (PUCCH, 载荷含sr_msg)
    per_rnti_channel_map<ul_grant>   grant_ch;  // eNB->UE (PDCCH, RNTI点对点)
    timed_channel<pdu_msg>           pdu_ch;    // UE->eNB (PUSCH, 载荷含rnti)
    per_rnti_channel_map<phich_msg>  phich_ch;  // eNB->UE (PHICH, RNTI点对点)
    // 【修复】grant/PHICH 原为共享 FIFO: 多 UE 线程各自 dequeue 会"错领"
    // 别人的 grant/反馈 (真实空口中 PDCCH/PHICH 按 RNTI 加扰, UE 只解自己的)。
    // 现改为每 UE 专属信道, 发送方按 rnti 投递、接收方只取自己的。
    for (int i = 0; i < NOF; i++) {
        grant_ch.add_rnti(rntis[i]);
        phich_ch.add_rnti(rntis[i]);
    }

    std::cout << ">>> 仿真开始: " << MAX_TTI << " TTI, 5个UE(每UE1线程), PF调度\n";
    std::cout << ">>> 中央TTI时钟线程 + 四条空口信道(各+"
              << CHANNEL_PROPAGATION_TTI << " TTI可见时延)\n";
    std::cout << ">>> 每个UE每5个TTI产生500字节数据, ul_snr=20dB\n\n";

    // ---- UE 工作线程: 数据到达 + run_tti + 发SR + 取grant打包PUSCH + 收PHICH ----
    std::vector<std::thread> ue_threads;
    for (int i = 0; i < NOF; i++) {
        ue_threads.emplace_back([&, i]() {
            ue_context& ue = *ues[i];
            uint16_t rnti = rntis[i];
            run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
                // 1) 数据到达 + run_tti (触发 SR / 构造 BSR)
                if (tti % 5 == static_cast<uint32_t>(i % 5)) {
                    ue.data_arrived(2, 500);
                }
                ue.run_tti(tti);

                // 2) SR: 仅在 SR 真正发送的那个 TTI 入队 PUCCH 信道
                //    (+4 TTI 后 eNB 可见)。不可按 state==PENDING 判断,
                //    否则 pending 期间每 TTI 重复上报 (信令泛洪)。
                if (ue.get_sr_manager().take_sr_transmitted()) {
                    // srsRAN 式: SR 触发同时把 UE 本地待传字节数随 SR 信道告知 eNB,
                    // 使调度器在 BSR CE 经 PUSCH 解包前即可按真实量分配 (跳过"小 Grant 探测")
                    sr_ch.enqueue({ue.get_rnti(),
                        ue.get_buffer_manager().get_all_lcg_buffer_sizes()}, tti);
                }

                // 3) 收 UL Grant (PDCCH): 只取本 RNTI 专属信道 ——
                //    PDCCH 按 RNTI 加扰, UE 只能解出自己的 grant;
                //    仅当该 grant 已度过传播时延才可见
                while (auto g = grant_ch.at(rnti).dequeue(tti)) {
                    ul_grant grant = g.value();
                    ue.handle_ul_grant(grant);
                    // 打包 PUSCH MAC PDU 并经 PUSCH 信道上报 (send_tti=tti)
                    pdu_msg m;
                    m.rnti = rnti;
                    m.grant = grant;
                    m.pdu = ue.get_last_pdu();
                    pdu_ch.enqueue(m, tti);
                }

                // 4) 收 PHICH (HARQ 反馈): 同样只取本 RNTI 专属信道
                while (auto ph = phich_ch.at(rnti).dequeue(tti)) {
                    ue.handle_harq_feedback(ph.value().pid, ph.value().ack);
                }
            });
        });
    }

    // ---- eNB 调度线程: 收SR + 调度 + 收PUSCH解BSR + 收HARQ + 发PHICH ----
    std::thread enb_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            // 1) 处理可见的 SR (PUCCH, +4 TTI 后才到)
            while (auto s = sr_ch.dequeue(tti)) {
                scheduler.handle_sr(s.value().rnti, s.value().pending_bytes);
            }

            // 2) 上行调度: 生成 grants, 经 PDCCH 下发 (send_tti=tti)
            //    按目的 RNTI 投递到该 UE 的专属信道 (点对点语义)
            auto results = scheduler.schedule_ul(tti);
            for (const auto& res : results) {
                grant_ch.at(res.rnti).enqueue(res.grant, tti);
            }

            // 3) 处理可见的 PUSCH PDU (UE 上报的 BSR), 并行解调 HARQ
            //    【方案C】多 UE 的 HARQ 接收并行执行 (std::async)
            std::vector<std::future<void>> tasks;
            while (auto m = pdu_ch.dequeue(tti)) {
                pdu_msg msg = m.value();
                tasks.emplace_back(std::async(std::launch::async, [&, msg]() {
                    // eNB 解 PUSCH PDU 取 BSR CE -> 解码 -> 喂调度器
                    enb_handle_bsr_pdu(enb_bsr, scheduler, msg.rnti, msg.pdu);
                    // eNB 用此前下发的 ul_grant 收 TB (软合并+CRC)
                    auto rx = enb_harq.receive_tb(msg.rnti, msg.grant);
                    // PHICH 反馈按目的 RNTI 投递到该 UE 的专属信道
                    phich_msg pm;
                    pm.rnti = msg.rnti;
                    pm.pid = msg.grant.pid;
                    pm.ack = rx.crc_ok;
                    phich_ch.at(msg.rnti).enqueue(pm, tti);
                    scheduler.handle_ul_crc(msg.rnti, pm.pid,
                                            rx.discarded ? true : rx.crc_ok);
                }));
            }
            for (auto& t : tasks) t.get();
        });
    });

    // 启动中央时钟 (主线程驱动)
    clock.run();

    // 等待所有 worker 结束
    for (auto& t : ue_threads) t.join();
    enb_thread.join();

    metrics_collector::instance().set_simulation_tti(MAX_TTI);
    for (int i = 0; i < NOF; i++) {
        print_ue_metrics(ues[i]->get_metrics());
    }
    print_enb_rx_stats(enb_bsr, enb_harq);
}

// ============================================================================
// 场景3: HARQ 软合并重传 (弱信号新传 NACK, 重传 IR 增益后 ACK)
// ============================================================================

void scenario3_harq_retx() {
    print_separator("场景3: HARQ软合并重传 (弱信道, 验证IR增益) [方案C: 多线程 + 4-TTI时延]");

    mac_logger::instance().set_level(log_level::WARNING);

    constexpr uint32_t MAX_TTI = 500;
    uint16_t rnti = 0x0001;

    // UE 桩 (独立线程)
    ue_context ue(rnti);
    ue.setup_lcid(2, 0, 8);

    // UE 桩 HARQ 配置 (与 eNB 侧 max 一致, 保证两套状态机同步)
    ul_harq_config harq_cfg;
    harq_cfg.max_harq_tx = 4;
    ue.set_harq_config(harq_cfg);

    // eNB 侧核心模块 (eNB 线程独占访问, 内部已加锁)
    ul_scheduler scheduler(sched_algorithm::ROUND_ROBIN, 50);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4); // max_harq_tx=4, 与 UE 桩一致
    register_ue_at_enb(scheduler, enb_bsr, enb_harq, rnti);
    // 弱信道: scheduler ul_snr=200 → MCS=7 (选择阈值200), 解码阈值=200+100=300
    //   新传: eff=200 < 300 → NACK
    //   重传: eff=200+200(IR)=400 >= 300 → ACK (体现 HARQ 软合并价值)
    enb_harq.set_ul_snr(rnti, 200);

    std::cout << ">>> 仿真开始: " << MAX_TTI << " TTI, 单UE, RR调度, 弱信道\n";
    std::cout << ">>> ul_snr=2dB (MCS=7 边缘), 新传NACK, 重传IR增益后ACK\n";
    std::cout << ">>> 最大HARQ重传: 4次; 四信道各+" << CHANNEL_PROPAGATION_TTI << " TTI可见时延\n\n";

    // ---- 方案C: 中央TTI时钟(屏障同步, 2个worker: UE线程+eNB线程) + 四信道 ----
    tti_clock clock(MAX_TTI, 2);
    timed_channel<sr_msg>            sr_ch;
    per_rnti_channel_map<ul_grant>   grant_ch;   // PDCCH: RNTI 点对点
    timed_channel<pdu_msg>           pdu_ch;
    per_rnti_channel_map<phich_msg>  phich_ch;   // PHICH: RNTI 点对点
    grant_ch.add_rnti(rnti);
    phich_ch.add_rnti(rnti);

    // 统计 (跨线程汇总, 用原子量保护)
    std::atomic<uint32_t> retx_count{0};
    std::atomic<uint32_t> fail_count{0};

    // UE 线程
    std::thread ue_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            if (tti % 5 == 0) ue.data_arrived(2, 1000);
            ue.run_tti(tti);

            // SR 仅在实际发送时入队 (防 PENDING 期间每 TTI 重复上报)
            if (ue.get_sr_manager().take_sr_transmitted()) {
                sr_ch.enqueue({ue.get_rnti(),
                    ue.get_buffer_manager().get_all_lcg_buffer_sizes()}, tti);
            }

            // 收 UL Grant: 本 RNTI 专属信道
            while (auto g = grant_ch.at(rnti).dequeue(tti)) {
                ul_grant grant = g.value();
                auto action = ue.handle_ul_grant(grant);
                if (action.is_retx) retx_count++;
                pdu_msg m;
                m.rnti = rnti;
                m.grant = grant;
                m.pdu = ue.get_last_pdu();
                pdu_ch.enqueue(m, tti);
            }

            // 收 PHICH: 本 RNTI 专属信道
            while (auto ph = phich_ch.at(rnti).dequeue(tti)) {
                ue.handle_harq_feedback(ph.value().pid, ph.value().ack);
            }
        });
    });

    // eNB 线程
    std::thread enb_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            while (auto s = sr_ch.dequeue(tti)) {
                scheduler.handle_sr(s.value().rnti, s.value().pending_bytes);
            }

            auto results = scheduler.schedule_ul(tti);
            for (const auto& res : results) {
                grant_ch.at(res.rnti).enqueue(res.grant, tti);
            }

            // 【简化】单 UE 场景每 TTI 至多一条 PDU, 原 std::async 逐消息
            // 建线程属过度设计 (线程创建开销远大于处理本身), 改为顺序处理;
            // 多 UE 并行接收的演示保留在场景2/5。
            while (auto m = pdu_ch.dequeue(tti)) {
                pdu_msg msg = m.value();
                enb_handle_bsr_pdu(enb_bsr, scheduler, msg.rnti, msg.pdu);
                auto rx = enb_harq.receive_tb(msg.rnti, msg.grant);
                if (rx.discarded) fail_count++;
                phich_msg pm;
                pm.rnti = msg.rnti;
                pm.pid = msg.grant.pid;
                pm.ack = rx.crc_ok;
                phich_ch.at(msg.rnti).enqueue(pm, tti);
                scheduler.handle_ul_crc(msg.rnti, pm.pid,
                                        rx.discarded ? true : rx.crc_ok);
            }
        });
    });

    clock.run();
    ue_thread.join();
    enb_thread.join();

    metrics_collector::instance().set_simulation_tti(MAX_TTI);

    auto m = ue.get_metrics();
    print_ue_metrics(m);
    print_enb_rx_stats(enb_bsr, enb_harq);
    std::cout << "\n  HARQ重传统计 (UE 桩视角):\n";
    std::cout << "    总重传次数: " << retx_count.load() << "\n";
    std::cout << "    丢弃次数(达到最大重传): " << fail_count.load() << "\n";
    std::cout << "    平均每包重传: " << std::fixed << std::setprecision(3)
              << m.avg_harq_retx << "\n";

    auto harq_stats = ue.get_harq_manager().get_stats();
    std::cout << "    UE桩 BLER: " << std::fixed << std::setprecision(2)
              << harq_stats.bler * 100 << "%\n";
    std::cout << "    UE桩 ACK: " << harq_stats.total_ack
              << ", NACK: " << harq_stats.total_nack << "\n";
}

// ============================================================================
// 场景4: 增强功能演示 (UE 桩自适应SR, eNB 接收侧统计)
// ============================================================================

void scenario4_enhanced_features() {
    print_separator("场景4: 增强功能 (UE桩自适应SR, eNB接收侧统计) [方案C: 多线程 + 4-TTI时延]");

    mac_logger::instance().set_level(log_level::WARNING);

    constexpr uint32_t MAX_TTI = 500;
    uint16_t rnti = 0x0001;

    // UE 桩 (独立线程)
    ue_context ue(rnti);
    ue.setup_lcid(2, 0, 8);
    ue.setup_lcid(3, 1, 10);

    // eNB 侧核心模块
    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);
    register_ue_at_enb(scheduler, enb_bsr, enb_harq, rnti);
    enb_harq.set_ul_snr(rnti, 2000); // 好信道

    std::cout << ">>> 仿真开始: " << MAX_TTI << " TTI, 单UE, PF调度\n";
    std::cout << ">>> 演示 UE 桩自适应 SR 周期, 四信道各+"
              << CHANNEL_PROPAGATION_TTI << " TTI可见时延\n\n";

    // ---- 方案C: 中央TTI时钟(屏障同步, 2个worker: UE线程+eNB线程) + 四信道 ----
    tti_clock clock(MAX_TTI, 2);
    timed_channel<sr_msg>            sr_ch;
    per_rnti_channel_map<ul_grant>   grant_ch;   // PDCCH: RNTI 点对点
    timed_channel<pdu_msg>           pdu_ch;
    per_rnti_channel_map<phich_msg>  phich_ch;   // PHICH: RNTI 点对点
    grant_ch.add_rnti(rnti);
    phich_ch.add_rnti(rnti);

    // UE 线程
    std::thread ue_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
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

            // UE 桩: 自适应 SR 周期调整 (高流量缩短SR周期, 低流量延长以省功耗)
            ue.get_sr_manager().adjust_sr_period(traffic_rate);
            ue.run_tti(tti);

            // SR 仅在实际发送时入队 (防 PENDING 期间每 TTI 重复上报)
            if (ue.get_sr_manager().take_sr_transmitted()) {
                sr_ch.enqueue({ue.get_rnti(),
                    ue.get_buffer_manager().get_all_lcg_buffer_sizes()}, tti);
            }

            // 收 UL Grant: 本 RNTI 专属信道
            while (auto g = grant_ch.at(rnti).dequeue(tti)) {
                ul_grant grant = g.value();
                ue.handle_ul_grant(grant);
                pdu_msg m;
                m.rnti = rnti;
                m.grant = grant;
                m.pdu = ue.get_last_pdu();
                pdu_ch.enqueue(m, tti);
            }

            // 收 PHICH: 本 RNTI 专属信道
            while (auto ph = phich_ch.at(rnti).dequeue(tti)) {
                ue.handle_harq_feedback(ph.value().pid, ph.value().ack);
            }

            if (tti % 100 == 99) {
                auto sr_stats = ue.get_sr_manager().get_stats();
                std::cout << "  TTI=" << tti
                          << ", SR period=" << sr_stats.current_sr_period << "ms"
                          << ", SR sent=" << sr_stats.total_sr_sent
                          << ", SR success=" << sr_stats.total_sr_success << "\n";
            }
        });
    });

    // eNB 线程
    std::thread enb_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            while (auto s = sr_ch.dequeue(tti)) {
                scheduler.handle_sr(s.value().rnti, s.value().pending_bytes);
            }

            auto results = scheduler.schedule_ul(tti);
            for (const auto& res : results) {
                grant_ch.at(res.rnti).enqueue(res.grant, tti);
            }

            // 【简化】单 UE 场景顺序处理即可 (原 std::async 逐消息建线程
            // 属过度设计); 多 UE 并行接收保留在场景2/5
            while (auto m = pdu_ch.dequeue(tti)) {
                pdu_msg msg = m.value();
                enb_handle_bsr_pdu(enb_bsr, scheduler, msg.rnti, msg.pdu);
                auto rx = enb_harq.receive_tb(msg.rnti, msg.grant);
                phich_msg pm;
                pm.rnti = msg.rnti;
                pm.pid = msg.grant.pid;
                pm.ack = rx.crc_ok;
                phich_ch.at(msg.rnti).enqueue(pm, tti);
                scheduler.handle_ul_crc(msg.rnti, pm.pid,
                                        rx.discarded ? true : rx.crc_ok);
            }
        });
    });

    clock.run();
    ue_thread.join();
    enb_thread.join();

    metrics_collector::instance().set_simulation_tti(MAX_TTI);
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

    // P1: 调度实时性度量 - 展示每次 TTI 调度决策耗时的 P50/P99
    auto lat = scheduler.get_sched_latency_stats();
    std::cout << "\n  调度实时性 (schedule_ul 单次耗时, 微秒):\n";
    std::cout << "    采样数: " << lat.count << "\n";
    std::cout << "    min=" << lat.min_us << "  max=" << lat.max_us
              << "  avg=" << lat.avg_us << "\n";
    std::cout << "    P50=" << lat.p50_us << "  P99=" << lat.p99_us << "\n";
}

// ============================================================================
// 场景5: 华为增强型比例公平调度 (EPF)
//   演示: 差异化 QoS 权重 (VoIP/视频/尽力而为) + 弱信道 UE 不饿死
// ============================================================================

void scenario5_epf() {
    print_separator("场景5: 华为增强型比例公平调度 (EPF) [方案C: 多线程 + 4-TTI时延]");

    mac_logger::instance().set_level(log_level::WARNING);

    constexpr int NOF = 5;
    constexpr uint32_t MAX_TTI = 1000;
    // 5 个 UE, 差异化业务 + 差异化信道
    //   UE1: VoIP  (GBR, 最高权重), 好信道
    //   UE2: 视频  (高权重),        好信道
    //   UE3: 尽力而为(基础权重),    好信道
    //   UE4: 尽力而为(基础权重),    好信道
    //   UE5: 尽力而为(基础权重),    弱信道 (差SNR, 验证不饿死)
    uint16_t rntis[NOF] = {0x0001, 0x0002, 0x0003, 0x0004, 0x0005};
    qos_class qos[NOF]  = {qos_class::VOIP, qos_class::VIDEO, qos_class::BE,
                           qos_class::BE,    qos_class::BE};
    int32_t   snr[NOF]  = {2000, 2000, 2000, 2000, 200}; // UE5 弱信道 2dB

    // eNB 侧核心模块 (eNB 线程独占访问)
    ul_scheduler scheduler(sched_algorithm::EPF, 100);
    // 可配置公平性因子: 偏向公平 (alpha=1.0 标准PF), 信道感知 beta=0.5
    epf_params epf;
    epf.alpha = 1.0f;
    epf.beta = 0.5f;
    epf.gamma = 1.0f;
    epf.min_prb_ratio = 0.10f;   // 饿死保护: 弱信道UE保底 10% PRB
    epf.starve_tti = 200;        // 超过 200 TTI 未调度视为濒临饿死
    scheduler.configure_epf(epf);

    enb_bsr_manager enb_bsr;
    enb_ul_harq_manager enb_harq(4);

    // 每个 UE 一个独立线程
    std::vector<std::unique_ptr<ue_context>> ues;
    for (int i = 0; i < NOF; i++) {
        ues.push_back(std::make_unique<ue_context>(rntis[i]));
        ues.back()->setup_lcid(2, 0, 8);
        register_ue_at_enb(scheduler, enb_bsr, enb_harq, rntis[i]); // 事务化注册
        scheduler.set_ue_qos(rntis[i], qos[i]); // 注入差异化 QoS 权重
        enb_harq.set_ul_snr(rntis[i], snr[i]);
    }

    std::cout << ">>> 仿真开始: " << MAX_TTI << " TTI, 5个UE(每UE1线程), EPF调度\n";
    std::cout << ">>> QoS: UE1=VoIP(W=3) UE2=Video(W=2) UE3-5=BE(W=1)\n";
    std::cout << ">>> UE5 弱信道(2dB), 验证饿死保护 (min_prb=10%)\n";
    std::cout << ">>> 四信道各+" << CHANNEL_PROPAGATION_TTI << " TTI可见时延\n\n";

    // ---- 方案C: 中央TTI时钟(屏障同步) + 四条空口信道 ----
    // 屏障 worker 数 = 5 个 UE 线程 + 1 个 eNB 线程
    tti_clock clock(MAX_TTI, NOF + 1);
    timed_channel<sr_msg>            sr_ch;     // UE->eNB (PUCCH, 载荷含sr_msg)
    per_rnti_channel_map<ul_grant>   grant_ch;  // eNB->UE (PDCCH, RNTI点对点)
    timed_channel<pdu_msg>           pdu_ch;    // UE->eNB (PUSCH, 载荷含rnti)
    per_rnti_channel_map<phich_msg>  phich_ch;  // eNB->UE (PHICH, RNTI点对点)
    // 每 UE 专属 grant/PHICH 信道, 杜绝共享 FIFO 的"错领"缺陷
    for (int i = 0; i < NOF; i++) {
        grant_ch.add_rnti(rntis[i]);
        phich_ch.add_rnti(rntis[i]);
    }

    // 统计各 UE 调度次数 (eNB 线程递增, 汇总打印用原子量)
    std::vector<std::atomic<uint32_t>> sched_count(NOF);
    for (int i = 0; i < NOF; i++) sched_count[i].store(0);

    // UE 工作线程
    std::vector<std::thread> ue_threads;
    for (int i = 0; i < NOF; i++) {
        ue_threads.emplace_back([&, i]() {
            ue_context& ue = *ues[i];
            uint16_t rnti = rntis[i];
            run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
                // 各 UE 持续产生数据, VoIP/视频产生更频繁 (业务特征)
                uint32_t period = (qos[i] == qos_class::VOIP) ? 2u
                                : (qos[i] == qos_class::VIDEO) ? 4u : 5u;
                if (tti % period == static_cast<uint32_t>(i % period)) {
                    uint32_t bytes = (qos[i] == qos_class::VOIP) ? 100u   // 语音小包
                                    : (qos[i] == qos_class::VIDEO) ? 800u // 视频大包
                                    : 500u;                                // BE
                    ue.data_arrived(2, bytes);
                }
                ue.run_tti(tti);

                // SR 仅在实际发送时入队 (防 PENDING 期间每 TTI 重复上报)
                if (ue.get_sr_manager().take_sr_transmitted()) {
                    sr_ch.enqueue({ue.get_rnti(),
                        ue.get_buffer_manager().get_all_lcg_buffer_sizes()}, tti);
                }

                // 收 UL Grant: 只取本 RNTI 专属信道 (PDCCH 按 RNTI 加扰)
                while (auto g = grant_ch.at(rnti).dequeue(tti)) {
                    ul_grant grant = g.value();
                    ue.handle_ul_grant(grant);
                    pdu_msg m;
                    m.rnti = rnti;
                    m.grant = grant;
                    m.pdu = ue.get_last_pdu();
                    pdu_ch.enqueue(m, tti);
                }

                // 收 PHICH: 只取本 RNTI 专属信道
                while (auto ph = phich_ch.at(rnti).dequeue(tti)) {
                    ue.handle_harq_feedback(ph.value().pid, ph.value().ack);
                }
            });
        });
    }

    // eNB 调度线程
    std::thread enb_thread([&]() {
        run_tti_worker(clock, MAX_TTI, [&](uint32_t tti) {
            while (auto s = sr_ch.dequeue(tti)) {
                scheduler.handle_sr(s.value().rnti, s.value().pending_bytes);
            }

            auto results = scheduler.schedule_ul(tti);
            for (const auto& res : results) {
                grant_ch.at(res.rnti).enqueue(res.grant, tti);
                // 递增对应 UE 的调度计数 (eNB 线程内, 按 rnti 定位索引)
                for (int i = 0; i < NOF; i++) {
                    if (rntis[i] == res.rnti) { sched_count[i]++; break; }
                }
            }

            std::vector<std::future<void>> tasks;
            while (auto m = pdu_ch.dequeue(tti)) {
                pdu_msg msg = m.value();
                tasks.emplace_back(std::async(std::launch::async, [&, msg]() {
                    enb_handle_bsr_pdu(enb_bsr, scheduler, msg.rnti, msg.pdu);
                    auto rx = enb_harq.receive_tb(msg.rnti, msg.grant);
                    phich_msg pm;
                    pm.rnti = msg.rnti;
                    pm.pid = msg.grant.pid;
                    pm.ack = rx.crc_ok;
                    phich_ch.at(msg.rnti).enqueue(pm, tti);
                    scheduler.handle_ul_crc(msg.rnti, pm.pid,
                                            rx.discarded ? true : rx.crc_ok);
                }));
            }
            for (auto& t : tasks) t.get();

            // 每 200 TTI 打印一次 EPF 度量快照 (eNB 线程内访问 scheduler 安全)
            // 注意: 快照在 mark_tti_done 语义之后打印 (原实现在屏障汇报后),
            // 此处放在 body 末尾仅用于展示, 不影响调度时序
            if (tti % 200 == 199) {
                std::cout << "  TTI=" << (tti + 1) << " EPF 度量快照:\n";
                for (int i = 0; i < NOF; i++) {
                    std::cout << "    UE" << (i + 1)
                              << " QoS=" << static_cast<int>(qos[i])
                              << " metric=" << std::fixed << std::setprecision(1)
                              << scheduler.get_epf_metric(rntis[i], tti)
                              << " sched=" << sched_count[i].load() << "\n";
                }
            }
        });
    });

    clock.run();
    for (auto& t : ue_threads) t.join();
    enb_thread.join();

    metrics_collector::instance().set_simulation_tti(MAX_TTI);
    std::cout << "\n>>> 总调度次数 (公平性/不饿死验证):\n";
    for (int i = 0; i < NOF; i++) {
        const char* qname = (qos[i] == qos_class::VOIP) ? "VoIP"
                          : (qos[i] == qos_class::VIDEO) ? "Video" : "BE";
        const char* ch = (snr[i] < 1000) ? "弱" : "好";
        std::cout << "  UE" << (i + 1) << " [" << qname << "," << ch << "信道]"
                  << " 调度=" << sched_count[i].load()
                  << " 次, 吞吐=" << std::fixed << std::setprecision(2)
                  << ues[i]->get_metrics().ul_throughput_kbps << " kbps\n";
    }
    for (int i = 0; i < NOF; i++) print_ue_metrics(ues[i]->get_metrics());
    print_enb_rx_stats(enb_bsr, enb_harq);
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    // strict 3GPP 模式接线: 命令行 --strict-3gpp (或环境变量 UL_MAC_STRICT=1)
    // 开启后禁用所有非标准增强 (自适应SR周期 / Padding BSR差分抑制 等),
    // 用于与"增强模式"做教学对比。协议本身要求的机制不受影响。
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--strict-3gpp") g_strict_3gpp_mode = true;
    }
    if (!g_strict_3gpp_mode) {
        const char* env = std::getenv("UL_MAC_STRICT");
        if (env && std::string(env) == "1") g_strict_3gpp_mode = true;
    }
    if (g_strict_3gpp_mode) {
        std::cout << ">>> strict 3GPP 模式已开启: 非标准增强 (自适应SR / Padding BSR抑制) 已禁用\n";
    }

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
    metrics_collector::instance().reset();

    scenario5_epf();

    print_separator("最终系统性能指标");
    metrics_collector::instance().print_summary();

    std::cout << "\n演示完成。eNB侧上行MAC接收链路核心机制 (方案B: BSR随PUSCH上报):\n";
    std::cout << "  - BSR上报 (ue_context→mac_pdu_packer→PUSCH): UE打包BSR CE进MAC PDU\n";
    std::cout << "  - BSR解码 (enb_bsr_manager): eNB解PUSCH PDU取BSR CE → 解码 → per-UE LCG视图\n";
    std::cout << "  - SR (PUCCH 1-bit): 仅作调度提示, 与BSR空口分离\n";
    std::cout << "  - 上行调度 (ul_scheduler): SR/BSR驱动 + PRB分配 + HARQ PID管理\n";
    std::cout << "  - HARQ接收 (enb_ul_harq_manager): IR软合并 + CRC + PHICH反馈\n";
    std::cout << "  - 调度算法: PF/RR/EPF(华为增强型比例公平) 三种\n";
    std::cout << "  - EPF: QoS权重 + 信道感知 + 饿死保护 (可配置 alpha/beta/gamma)\n";
    std::cout << "  - UE桩增强: 自适应SR (发送端行为)\n";

    return 0;
}
