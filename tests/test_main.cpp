// =============================================================================
// test_main.cpp - UL MAC Manager 轻量级自动测试框架 + 测试用例
//
// 不依赖外部测试框架 (如 Google Test), 自行实现轻量级 assert 宏。
// 每个 TEST 宏在静态初始化期执行, 失败抛 std::runtime_error 由 runner 捕获。
// 测试覆盖: LCP令牌桶 / MCS-TBS / HARQ / SR / BSR / 延迟统计 / 死锁修复 / 线程安全
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cassert>
#include <thread>
#include <atomic>
#include <stdexcept>

#include "ul_mac/lcg_buffer.h"
#include "ul_mac/ue_sr_manager.h"
#include "ul_mac/ue_bsr_manager.h"
#include "ul_mac/ue_ul_harq_manager.h"
#include "ul_mac/enb_ul_harq_manager.h"
#include "ul_mac/enb_bsr_manager.h"
#include "ul_mac/enb_ul_scheduler.h"
#include "ul_mac/mac_pdu.h"
#include "ul_mac/ue_context.h"
#include "ul_mac/mac_logger.h"
#include "ul_mac/metrics_collector.h"

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
// 轻量级测试框架基础设施
// ============================================================================

static int test_count = 0;
static int test_passed = 0;
static int test_failed = 0;

#define TEST(name) \
    static void name(); \
    struct name##_runner { \
        name##_runner() { \
            test_count++; \
            std::cout << "[ RUN      ] " << #name << std::endl; \
            try { \
                name(); \
                test_passed++; \
                std::cout << "[       OK ] " << #name << std::endl; \
            } catch (const std::exception& e) { \
                test_failed++; \
                std::cout << "[  FAILED  ] " << #name << ": " << e.what() << std::endl; \
            } \
        } \
    } name##_instance; \
    static void name()

#define EXPECT_EQ(a, b) \
    do { \
        auto _a = (a); auto _b = (b); \
        if (!(_a == _b)) { \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": EXPECT_EQ failed: " + std::to_string(_a) + " != " + std::to_string(_b)); \
        } \
    } while (0)

#define EXPECT_TRUE(a) \
    do { \
        if (!(a)) { \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": EXPECT_TRUE failed"); \
        } \
    } while (0)

#define EXPECT_FALSE(a) \
    do { \
        if (a) { \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": EXPECT_FALSE failed"); \
        } \
    } while (0)

#define EXPECT_GE(a, b) \
    do { \
        auto _a = (a); auto _b = (b); \
        if (!(_a >= _b)) { \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": EXPECT_GE failed: " + std::to_string(_a) + " < " + std::to_string(_b)); \
        } \
    } while (0)

#define EXPECT_LE(a, b) \
    do { \
        auto _a = (a); auto _b = (b); \
        if (!(_a <= _b)) { \
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": EXPECT_LE failed: " + std::to_string(_a) + " > " + std::to_string(_b)); \
        } \
    } while (0)

// 静态初始化: 在所有 TEST 之前设置日志级别为 ERROR, 减少日志噪音
// (同一翻译单元内, 静态对象按声明顺序初始化, 此处先于所有 test runner)
static int _log_level_init = []() {
    mac_logger::instance().set_level(log_level::ERROR);
    return 0;
}();

// ============================================================================
// LCP 令牌桶测试
// ============================================================================

// 测试1: pbr>0 时, 第一阶段受令牌桶限制
// 两个通道(优先级1和2), 各自 token_count=10, 消耗15字节:
//   第一阶段: 高优先级取 min(10,1000,15)=10, 低优先级取 min(10,1000,5)=5
//   若无令牌桶限制, 高优先级会在第二阶段取走全部15字节
TEST(test_lcp_token_bucket_basic) {
    lcg_buffer_manager buf_mgr;

    // 两个通道, 不同优先级, pbr=10 bytes/ms, bsd=100ms -> 桶容量=1000
    buf_mgr.setup_lcid(1, 0, 1, 10, 100);  // lcid=1, lcg=0, pri=1
    buf_mgr.setup_lcid(2, 1, 2, 10, 100);  // lcid=2, lcg=1, pri=2

    // 令牌桶步进 1ms: 每个通道获得 10*1=10 令牌
    buf_mgr.step_token_buckets(1);

    // 设置缓冲区
    buf_mgr.update_buffer_state(1, 1000);
    buf_mgr.update_buffer_state(2, 1000);
    buf_mgr.update_old_buffer();  // old = new = 1000

    // 消耗 15 字节
    // 第一阶段: A(pri1) 取 min(10,1000,15)=10, B(pri2) 取 min(10,1000,5)=5
    // 第二阶段: remaining=0
    buf_mgr.consume_data(15);
    buf_mgr.update_old_buffer();  // 将消耗后的 new_buffer 同步到 old_buffer

    uint32_t lcg0 = buf_mgr.get_buffer_state_lcg(0);  // 1000-10=990
    uint32_t lcg1 = buf_mgr.get_buffer_state_lcg(1);  // 1000-5=995

    EXPECT_EQ(lcg0, 990u);  // 高优先级受 token_count=10 限制, 只取 10
    EXPECT_EQ(lcg1, 995u);  // 低优先级取剩余 5 (令牌桶让低优先级也能获得资源)
}

// 测试2: step_token_buckets 后令牌补充
// 初始 token_count=0, 第一阶段无法取数据, 全靠第二阶段(纯优先级)
// 补充令牌后, 第一阶段可按 PBR 分配资源给各通道
TEST(test_lcp_token_bucket_refill) {
    lcg_buffer_manager buf_mgr;

    buf_mgr.setup_lcid(1, 0, 1, 10, 100);  // pbr=10
    buf_mgr.setup_lcid(2, 1, 2, 10, 100);  // pbr=10

    buf_mgr.update_buffer_state(1, 1000);
    buf_mgr.update_buffer_state(2, 1000);
    buf_mgr.update_old_buffer();

    // 初始 token_count=0, 消耗 20 字节:
    //   第一阶段: 都取 0 (无令牌)
    //   第二阶段: A 取 20, B 取 0 (纯优先级)
    buf_mgr.consume_data(20);
    buf_mgr.update_old_buffer();

    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(0), 980u);   // A 取了 20
    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(1), 1000u);  // B 没取到

    // 重新填充缓冲区
    buf_mgr.update_buffer_state(1, 1000);
    buf_mgr.update_buffer_state(2, 1000);
    buf_mgr.update_old_buffer();

    // 令牌桶步进 1ms: token_count=10
    buf_mgr.step_token_buckets(1);

    // 再次消耗 20: 第一阶段 A 取 10, B 取 10 (令牌补充后两通道都能取到)
    buf_mgr.consume_data(20);
    buf_mgr.update_old_buffer();

    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(0), 990u);   // A 取 10
    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(1), 990u);   // B 取 10 (令牌补充后能取到)
}

// 测试3: pbr=0 时退化为纯优先级排序
// pbr=0 的通道跳过第一阶段, 全部在第二阶段按优先级分配
TEST(test_lcp_no_pbr_fallback) {
    lcg_buffer_manager buf_mgr;

    // pbr=0, 无 PBR 限制
    buf_mgr.setup_lcid(1, 0, 1, 0, 100);  // pri=1, pbr=0
    buf_mgr.setup_lcid(2, 1, 2, 0, 100);  // pri=2, pbr=0

    buf_mgr.update_buffer_state(1, 1000);
    buf_mgr.update_buffer_state(2, 1000);
    buf_mgr.update_old_buffer();

    // 消耗 20: 第一阶段都跳过(pbr=0), 第二阶段 A 取 20, B 取 0
    buf_mgr.consume_data(20);
    buf_mgr.update_old_buffer();

    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(0), 980u);   // 高优先级取 20
    EXPECT_EQ(buf_mgr.get_buffer_state_lcg(1), 1000u);  // 低优先级没取到 (纯优先级)
}

// ============================================================================
// MCS/TBS 查找表测试
// ============================================================================

// 测试4: SNR->MCS 映射的区间正确性
// ul_scheduler 默认 ul_snr=200 (2dB), 根据阈值表 SNR>=200 -> MCS 7
// (ul_snr 无公开 setter, 此处验证默认 SNR 下的映射点)
TEST(test_mcs_snr_mapping) {
    ul_scheduler sched(sched_algorithm::PROPORTIONAL_FAIR, 100);
    sched.add_ue(0x0001);
    sched.handle_bsr(0x0001, 0, 30);  // 设置缓冲区, 使调度器产生授权

    auto results = sched.schedule_ul(0);

    uint8_t mcs = 0;
    bool found = false;
    for (const auto& r : results) {
        if (r.rnti == 0x0001) {
            mcs = r.grant.mcs;
            found = true;
        }
    }
    EXPECT_TRUE(found);       // 调度器应为该 UE 产生授权
    EXPECT_EQ(mcs, 7u);       // 默认 snr=200(2dB) -> MCS 7 (SNR>=200 阈值)
}

// 测试5: TBS 随 MCS 和 PRB 单调递增
// 固定 SNR(->MCS 固定), 增大 BSR(->增大 req_bytes->增大 n_prb), TBS 应单调递增
TEST(test_tbs_monotonic) {
    ul_scheduler sched(sched_algorithm::PROPORTIONAL_FAIR, 100);
    sched.add_ue(0x0001);

    // 小缓冲区
    sched.handle_bsr(0x0001, 0, 10);  // bsr_index=10 -> 46 bytes
    auto r1 = sched.schedule_ul(0);
    uint32_t tbs1 = 0;
    for (const auto& r : r1) if (r.rnti == 0x0001) tbs1 = r.grant.tbs;

    // 中等缓冲区
    sched.handle_bsr(0x0001, 0, 40);  // bsr_index=40 -> 1528 bytes
    auto r2 = sched.schedule_ul(1);
    uint32_t tbs2 = 0;
    for (const auto& r : r2) if (r.rnti == 0x0001) tbs2 = r.grant.tbs;

    // 大缓冲区
    sched.handle_bsr(0x0001, 0, 60);  // bsr_index=60 -> 21956 bytes
    auto r3 = sched.schedule_ul(2);
    uint32_t tbs3 = 0;
    for (const auto& r : r3) if (r.rnti == 0x0001) tbs3 = r.grant.tbs;

    EXPECT_TRUE(tbs1 > 0);          // 小缓冲区也有授权
    EXPECT_TRUE(tbs2 >= tbs1);      // TBS 随缓冲区非递减
    EXPECT_TRUE(tbs3 >= tbs2);      // PRB 饱和时 TBS 封顶, 允许相等
}

// ============================================================================
// HARQ 测试
// ============================================================================

// 测试6: 新传时 NDI 翻转、RV=0、tx_nb=1
TEST(test_harq_new_tx) {
    ul_harq_manager mgr(0x0001);
    ul_harq_config cfg;
    cfg.max_harq_tx = 4;
    cfg.harq_rtt_ttis = 8;
    mgr.init(cfg);

    ul_grant g;
    g.pid = 0;
    g.tbs = 1000;
    g.ndi = true;          // NDI=1 (与初始 cur_ndi_=false 不同 -> 新传)
    g.ndi_present = true;
    g.rv = 0;              // 新传 RV=0
    g.tti_tx = 0;

    auto action = mgr.new_grant_ul(g, false);
    EXPECT_TRUE(action.is_new_tx);
    EXPECT_FALSE(action.is_retx);
    EXPECT_EQ(action.rv, 0u);     // 新传 RV=0
    EXPECT_EQ(action.tx_nb, 1u);  // 第一次传输
}

// 测试7: 重传时 NDI 不变、RV 序列 {0,2,3,1}
// 新传 RV=0, 后续重传 RV 依次为 2, 3, 1 (由 IRV 计数器驱动)
TEST(test_harq_retx) {
    ul_harq_manager mgr(0x0001);
    ul_harq_config cfg;
    cfg.max_harq_tx = 4;
    cfg.harq_rtt_ttis = 0;  // 即时反馈, 简化测试
    mgr.init(cfg);

    // 新传: RV=0, tx_nb=1
    ul_grant g1;
    g1.pid = 0; g1.tbs = 1000; g1.ndi = true; g1.ndi_present = true;
    g1.rv = 0; g1.tti_tx = 0;
    auto a1 = mgr.new_grant_ul(g1, false);
    EXPECT_TRUE(a1.is_new_tx);
    EXPECT_EQ(a1.rv, 0u);
    EXPECT_EQ(a1.tx_nb, 1u);

    // 重传1: PHICH NACK, NDI 不变 (ndi=true 与 cur_ndi_=true 相同 -> 重传)
    ul_grant g2;
    g2.pid = 0; g2.tbs = 1000; g2.ndi = true; g2.ndi_present = true;
    g2.rv = -1;  // 由 UE 内部 IRV 计算
    g2.phich_available = true; g2.hi_value = false;  // NACK
    g2.tti_tx = 1;
    auto a2 = mgr.new_grant_ul(g2, false);
    EXPECT_TRUE(a2.is_retx);       // NDI 不变 -> 重传 (非新传)
    EXPECT_EQ(a2.rv, 2u);          // RV 序列第2项
    EXPECT_EQ(a2.tx_nb, 2u);

    // 重传2: RV=3
    ul_grant g3;
    g3.pid = 0; g3.tbs = 1000; g3.ndi = true; g3.ndi_present = true;
    g3.rv = -1; g3.phich_available = true; g3.hi_value = false;
    g3.tti_tx = 2;
    auto a3 = mgr.new_grant_ul(g3, false);
    EXPECT_TRUE(a3.is_retx);
    EXPECT_EQ(a3.rv, 3u);          // RV 序列第3项

    // 重传3: RV=1
    ul_grant g4;
    g4.pid = 0; g4.tbs = 1000; g4.ndi = true; g4.ndi_present = true;
    g4.rv = -1; g4.phich_available = true; g4.hi_value = false;
    g4.tti_tx = 3;
    auto a4 = mgr.new_grant_ul(g4, false);
    EXPECT_TRUE(a4.is_retx);
    EXPECT_EQ(a4.rv, 1u);          // RV 序列第4项
}

// 测试8: 达到最大重传次数后 TB 被丢弃
TEST(test_harq_max_retx_discard) {
    ul_harq_manager mgr(0x0001);
    ul_harq_config cfg;
    cfg.max_harq_tx = 2;  // 最大 2 次传输 (1 新传 + 1 重传)
    cfg.harq_rtt_ttis = 0;
    mgr.init(cfg);

    // 新传: tx_nb=1
    ul_grant g1;
    g1.pid = 0; g1.tbs = 1000; g1.ndi = true; g1.ndi_present = true;
    g1.rv = 0; g1.tti_tx = 0;
    auto a1 = mgr.new_grant_ul(g1, false);
    EXPECT_EQ(a1.tx_nb, 1u);

    // 重传: tx_nb=2 (current_tx_nb=1 < max=2, 不丢弃)
    ul_grant g2;
    g2.pid = 0; g2.tbs = 1000; g2.ndi = true; g2.ndi_present = true;
    g2.rv = -1; g2.phich_available = true; g2.hi_value = false;
    g2.tti_tx = 1;
    auto a2 = mgr.new_grant_ul(g2, false);
    EXPECT_TRUE(a2.is_retx);
    EXPECT_EQ(a2.tx_nb, 2u);

    // 再次 NACK: current_tx_nb=2 >= max=2 -> 丢弃
    ul_grant g3;
    g3.pid = 0; g3.tbs = 1000; g3.ndi = true; g3.ndi_present = true;
    g3.rv = -1; g3.phich_available = true; g3.hi_value = false;
    g3.tti_tx = 2;
    auto a3 = mgr.new_grant_ul(g3, false);
    EXPECT_TRUE(a3.is_discarded);  // 达到最大重传, TB 被丢弃
}

// 测试9: RTT 延迟内不处理反馈
// 配置 harq_rtt_ttis=8, 在 RTT 内发送 PHICH ACK, 反馈不应被处理
// (last_feedback 仍为 NACK); RTT 后再发 ACK, last_feedback 变为 ACK
TEST(test_harq_rtt_delay) {
    ul_harq_manager mgr(0x0001);
    ul_harq_config cfg;
    cfg.max_harq_tx = 10;  // 高阈值, 避免干扰
    cfg.harq_rtt_ttis = 8;
    mgr.init(cfg);

    // 新传 (tti=0, tx_tti_=0)
    ul_grant g1;
    g1.pid = 0; g1.tbs = 1000; g1.ndi = true; g1.ndi_present = true;
    g1.rv = 0; g1.tti_tx = 0;
    mgr.new_grant_ul(g1, false);

    // RTT 内 (tti=5, 5-0=5 < 8) 发送 PHICH ACK, 非自适应 (ndi_present=false)
    ul_grant g2;
    g2.pid = 0; g2.ndi_present = false;
    g2.phich_available = true; g2.hi_value = true;  // ACK
    g2.tti_tx = 5;
    mgr.new_grant_ul(g2, false);
    // 反馈未处理, harq_feedback_ 保持 false (新传后的默认值)

    auto info1 = mgr.get_process_info(0);
    EXPECT_TRUE(info1.last_feedback == harq_feedback::NACK);  // ACK 未被处理

    // RTT 后 (tti=13, 13-5=8 >= 8) 再发 PHICH ACK
    ul_grant g3;
    g3.pid = 0; g3.ndi_present = false;
    g3.phich_available = true; g3.hi_value = true;  // ACK
    g3.tti_tx = 13;
    mgr.new_grant_ul(g3, false);
    // 反馈已处理, harq_feedback_ = true (ACK)

    auto info2 = mgr.get_process_info(0);
    EXPECT_TRUE(info2.last_feedback == harq_feedback::ACK);  // ACK 已被处理
}

// 测试10: 连续 NACK 触发早期终止
// 配置 max_harq_tx=10 (避免 max retx 干扰), 连续 3 次 NACK + tx_nb>=2 -> 丢弃
TEST(test_harq_early_termination) {
    ul_harq_manager mgr(0x0001);
    ul_harq_config cfg;
    cfg.max_harq_tx = 10;  // 高阈值, 隔离早期终止逻辑
    cfg.harq_rtt_ttis = 0;
    mgr.init(cfg);

    // 新传: tx_nb=1
    ul_grant g1;
    g1.pid = 0; g1.tbs = 1000; g1.ndi = true; g1.ndi_present = true;
    g1.rv = 0; g1.tti_tx = 0;
    mgr.new_grant_ul(g1, false);

    // 重传: tx_nb=2 (此时 consecutive_nack=0, 不触发早期终止)
    ul_grant g2;
    g2.pid = 0; g2.tbs = 1000; g2.ndi = true; g2.ndi_present = true;
    g2.rv = -1; g2.phich_available = true; g2.hi_value = false;
    g2.tti_tx = 1;
    auto a2 = mgr.new_grant_ul(g2, false);
    EXPECT_TRUE(a2.is_retx);
    EXPECT_EQ(a2.tx_nb, 2u);

    // 累积连续 NACK (通过 handle_harq_feedback 更新 consecutive_nack)
    mgr.handle_harq_feedback(0, harq_feedback::NACK);
    mgr.handle_harq_feedback(0, harq_feedback::NACK);
    mgr.handle_harq_feedback(0, harq_feedback::NACK);  // consecutive_nack=3

    // 再次 PHICH NACK: current_tx_nb=2 >= 2 && consecutive_nack=3 >= 3 -> 早期终止
    ul_grant g3;
    g3.pid = 0; g3.tbs = 1000; g3.ndi = true; g3.ndi_present = true;
    g3.rv = -1; g3.phich_available = true; g3.hi_value = false;
    g3.tti_tx = 2;
    auto a3 = mgr.new_grant_ul(g3, false);
    EXPECT_TRUE(a3.is_discarded);  // 早期终止触发, TB 被丢弃
}

// ============================================================================
// SR (调度请求) 测试
// ============================================================================

// 测试11: SR 状态机 IDLE -> PENDING -> (TRANSMITTING 瞬态) -> IDLE
// step() 内部 state 先置 TRANSMITTING 再回到 PENDING, 外部观察到 PENDING
TEST(test_sr_state_machine) {
    sr_manager sr(0x0001);
    sr_config cfg;
    cfg.enabled = true;
    cfg.dsr_transmax = 8;
    cfg.sr_period = 10;
    sr.init(cfg, nullptr, nullptr);

    // 初始状态: IDLE
    EXPECT_TRUE(sr.get_state() == sr_state::IDLE);

    // BSR 触发 SR: IDLE -> PENDING
    sr.start();
    EXPECT_TRUE(sr.get_state() == sr_state::PENDING);

    // step: 发送 SR (内部经 TRANSMITTING 后回到 PENDING 等待响应)
    sr.step(0);
    EXPECT_TRUE(sr.get_state() == sr_state::PENDING);  // 发送后回到 PENDING
    EXPECT_EQ(sr.get_sr_counter(), 1u);                // SR 计数+1

    // 收到上行授权: SR 成功, PENDING -> IDLE
    sr.notify_ul_grant_received();
    EXPECT_TRUE(sr.get_state() == sr_state::IDLE);

    auto stats = sr.get_stats();
    EXPECT_EQ(stats.total_sr_sent, 1u);       // 发送了 1 次
    EXPECT_EQ(stats.total_sr_success, 1u);    // 成功 1 次
}

// 测试12: 自适应 SR 周期调整 (高流量 -> 短周期)
TEST(test_sr_adaptive_period) {
    sr_manager sr(0x0001);
    sr_config cfg;
    cfg.enabled = true;
    cfg.sr_period = 40;  // 初始周期 40ms
    sr.init(cfg, nullptr, nullptr);

    uint32_t initial_period = sr.get_stats().current_sr_period;
    EXPECT_EQ(initial_period, 40u);

    // 高流量 (1000 bytes/TTI) -> 应缩短 SR 周期
    // 多次调用以克服 EMA 平滑
    for (int i = 0; i < 20; i++) {
        sr.adjust_sr_period(1000.0);
    }

    uint32_t adjusted_period = sr.get_stats().current_sr_period;
    // 高流量 -> 周期应缩短 (严格小于初始值)
    EXPECT_TRUE(adjusted_period < initial_period);
    EXPECT_GE(adjusted_period, 5u);  // 不低于最小周期 5ms
}

// ============================================================================
// BSR (缓冲区状态报告) 测试
// ============================================================================

// 测试13: 新数据到达触发 Regular BSR
TEST(test_bsr_trigger_regular) {
    lcg_buffer_manager buf_mgr;
    bsr_manager bsr_mgr(0x0001, buf_mgr);
    bsr_mgr.init(bsr_config(), nullptr, nullptr);

    // 配置逻辑通道
    buf_mgr.setup_lcid(1, 0, 1);

    // 新数据到达 (LCG0 之前无数据, 现在有 -> 触发 Regular BSR)
    buf_mgr.update_buffer_state(1, 1000);
    bsr_mgr.step(0);  // step 内检查触发条件

    EXPECT_TRUE(bsr_mgr.get_trigger_type() == bsr_trigger_type::REGULAR);
}

// 测试14: 预测性 BSR 线性回归 (递增趋势 -> 预测值 > 当前值)
TEST(test_bsr_predict_linear) {
    lcg_buffer_manager buf_mgr;
    bsr_manager bsr_mgr(0x0001, buf_mgr);
    bsr_mgr.init(bsr_config(), nullptr, nullptr);

    buf_mgr.setup_lcid(1, 0, 1);

    // 递增趋势: 100, 200, 300, 400, 500
    for (uint32_t i = 1; i <= 5; i++) {
        buf_mgr.update_buffer_state(1, i * 100);
        bsr_mgr.step(i);
    }
    // 趋势走平: 500 (不再增长), 使线性回归预测值高于当前值
    buf_mgr.update_buffer_state(1, 500);
    bsr_mgr.step(6);

    uint32_t current = buf_mgr.get_total_buffer_state();  // 500
    uint32_t predicted = bsr_mgr.predict_buffer_demand();

    // 递增趋势 -> 预测值应大于当前值
    EXPECT_TRUE(predicted > current);
}

// 测试15: 差分 BSR (无变化时 Padding BSR 被跳过)
TEST(test_bsr_differential) {
    lcg_buffer_manager buf_mgr;
    bsr_manager bsr_mgr(0x0001, buf_mgr);
    bsr_mgr.init(bsr_config(), nullptr, nullptr);
    bsr_mgr.set_differential_enabled(true);

    buf_mgr.setup_lcid(1, 0, 1);
    buf_mgr.update_buffer_state(1, 1000);
    buf_mgr.update_old_buffer();  // old = 1000

    // 第一次 Padding BSR: 缓冲区有变化 (从 0 到非 0), 应生成成功
    bsr_ce bsr1;
    bool result1 = bsr_mgr.generate_padding_bsr(100, bsr1);
    EXPECT_TRUE(result1);  // 第一次生成成功

    // 缓冲区无变化, 再次生成 Padding BSR: 差分检查应跳过
    bsr_ce bsr2;
    bool result2 = bsr_mgr.generate_padding_bsr(100, bsr2);
    EXPECT_FALSE(result2);  // 差分 BSR: 无变化, 跳过
}

// ============================================================================
// 延迟统计测试
// ============================================================================

// 测试16: P50/P90/P99 计算正确性
TEST(test_latency_stats) {
    metrics_collector::instance().reset();

    // 记录 100 个样本: 0, 1, 2, ..., 99
    for (uint32_t i = 0; i < 100; i++) {
        metrics_collector::instance().record_latency(i);
    }

    auto stats = metrics_collector::instance().get_latency_stats();
    // 排序后: [0, 1, ..., 99]
    // p50 = latencies[100*50/100] = latencies[50] = 50
    // p90 = latencies[100*90/100] = latencies[90] = 90
    // p99 = latencies[100*99/100] = latencies[99] = 99
    EXPECT_EQ(stats.p50, 50u);
    EXPECT_EQ(stats.p90, 90u);
    EXPECT_EQ(stats.p99, 99u);
    EXPECT_EQ(stats.min, 0u);
    EXPECT_EQ(stats.max, 99u);
}

// 测试17: 延迟样本被正确记录
TEST(test_latency_recording) {
    metrics_collector::instance().reset();

    metrics_collector::instance().record_latency(10);
    metrics_collector::instance().record_latency(20);
    metrics_collector::instance().record_latency(30);

    auto stats = metrics_collector::instance().get_latency_stats();
    EXPECT_EQ(stats.latencies.size(), 3u);  // 3 个样本被记录
}

// ============================================================================
// 死锁修复验证
// ============================================================================

// 测试18: TB 丢弃后调度器 pending_retx 位图被清除 (HARQ进程泄漏修复)
// 验证: 多进程并发 NACK 不会被单值覆盖丢失; ACK 后对应进程位图与 busy 标志清除
TEST(test_deadlock_fix_discard_clears_retx) {
    ul_scheduler scheduler(sched_algorithm::PROPORTIONAL_FAIR, 100);
    scheduler.add_ue(0x0001);

    // 模拟 CRC 失败: PID=3 与 PID=7 同时 NACK, 位图两位置位 (验证不互相覆盖)
    scheduler.handle_ul_crc(0x0001, 3, false);  // crc_ok=false
    scheduler.handle_ul_crc(0x0001, 7, false);  // crc_ok=false
    auto ctx1 = scheduler.get_ue_context(0x0001);
    EXPECT_TRUE(ctx1.has_value());
    if (!ctx1.has_value()) return;
    EXPECT_TRUE(ctx1->pending_retx[3]);
    EXPECT_TRUE(ctx1->pending_retx[7]);

    // 模拟 TB 丢弃后的修复: 主动通知调度器清除重传标志 (对应 main.cpp 中 is_discarded)
    scheduler.handle_ul_crc(0x0001, 3, true);  // crc_ok=true -> 释放PID=3
    auto ctx2 = scheduler.get_ue_context(0x0001);
    EXPECT_TRUE(ctx2.has_value());
    if (!ctx2.has_value()) return;
    EXPECT_FALSE(ctx2->pending_retx[3]);   // 已清除
    EXPECT_TRUE(ctx2->pending_retx[7]);     // PID=7 仍待重传, 未被覆盖
    EXPECT_FALSE(ctx2->harq_pid_busy[3]);   // 进程释放
    EXPECT_TRUE(ctx2->harq_pid_busy[7]);    // 进程仍占用
}

// ============================================================================
// 线程安全测试
// ============================================================================

// 测试19: get_all_process_info() 在并发访问下不崩溃且返回正确数量
// 一个写线程不断调用 new_grant_ul, 多个读线程并发调用 get_all_process_info
TEST(test_get_all_process_info) {
    ul_harq_manager mgr(0x0001);
    mgr.init(ul_harq_config());

    std::atomic<int> errors{0};
    std::vector<std::thread> threads;

    // 写线程: 不断触发新传/重传
    threads.emplace_back([&]() {
        ul_grant g;
        g.pid = 0; g.tbs = 100; g.ndi = true; g.ndi_present = true; g.rv = 0;
        for (uint32_t tti = 0; tti < 500; tti++) {
            g.tti_tx = tti;
            mgr.new_grant_ul(g, false);
        }
    });

    // 读线程: 并发调用 get_all_process_info
    for (int t = 0; t < 3; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < 500; i++) {
                auto infos = mgr.get_all_process_info();
                if (infos.size() != MAX_HARQ_PROCESSES) {
                    errors++;
                }
            }
        });
    }

    // 等待所有线程完成 (确保 expectations 在 join 后执行, 避免线程泄漏)
    for (auto& t : threads) t.join();

    EXPECT_EQ(errors.load(), 0);  // 无崩溃, 数量始终正确
}

// ============================================================================
// eNB 接收端 HARQ 测试 (enb_ul_harq_manager)
// ============================================================================

// 辅助: 构造上行授权
static ul_grant make_ul_grant(uint32_t pid, bool ndi, uint8_t mcs,
                              uint32_t tbs, uint32_t tti) {
    ul_grant g;
    g.pid = pid;
    g.ndi = ndi;
    g.ndi_present = true;
    g.mcs = mcs;
    g.tbs = tbs;
    g.tti_tx = tti;
    g.rv = 0;
    return g;
}

// 测试20: 软合并增益 - 新传NACK, 一次重传ACK
// 默认 ul_snr=200(2dB), MCS=7 选择阈值200, 解码阈值=200+100=300
//   新传: eff=200 < 300 -> NACK
//   重传: eff=200+200(IR)=400 >= 300 -> ACK (体现HARQ软合并价值)
TEST(test_enb_harq_soft_combine_gain) {
    enb_ul_harq_manager harq(4); // max_harq_tx=4
    harq.add_ue(0x0001);
    // ul_snr 默认200, 无需显式设置

    // 新传 (NDI=true)
    ul_grant g1 = make_ul_grant(0, true, 7, 100, 0);
    auto r1 = harq.receive_tb(0x0001, g1);
    EXPECT_TRUE(r1.is_new_tb);       // 新TB
    EXPECT_TRUE(!r1.crc_ok);         // NACK
    EXPECT_EQ(r1.tx_count, 1u);
    EXPECT_TRUE(!r1.soft_combined);  // 新传无软合并
    EXPECT_TRUE(!r1.discarded);
    EXPECT_TRUE(harq.get_process_state(0x0001, 0) == harq_state::RETX_PENDING);
    EXPECT_TRUE(harq.has_retx_pending(0x0001));

    // 重传 (NDI保持true)
    ul_grant g2 = make_ul_grant(0, true, 7, 100, 8);
    auto r2 = harq.receive_tb(0x0001, g2);
    EXPECT_TRUE(!r2.is_new_tb);      // 重传
    EXPECT_TRUE(r2.crc_ok);          // ACK (软合并后解码成功)
    EXPECT_EQ(r2.tx_count, 2u);
    EXPECT_TRUE(r2.soft_combined);   // 发生软合并
    EXPECT_TRUE(harq.get_process_state(0x0001, 0) == harq_state::INACTIVE);
    EXPECT_TRUE(!harq.has_retx_pending(0x0001));
    EXPECT_TRUE(harq.get_phich(0x0001, 0)); // 最近反馈=ACK
}

// 测试21: NDI翻转判定新TB, 软buffer刷新
TEST(test_enb_harq_ndi_toggle_new_tb) {
    enb_ul_harq_manager harq(4);
    harq.add_ue(0x0001);
    harq.set_ul_snr(0x0001, 2000); // 高SNR, MCS=15 阈值1100, 一次解码成功

    // 新传 NDI=true, eff=2000>=1100 -> ACK
    auto r1 = harq.receive_tb(0x0001, make_ul_grant(0, true, 15, 200, 0));
    EXPECT_TRUE(r1.crc_ok);
    EXPECT_TRUE(r1.is_new_tb);

    // NDI翻转=true->false, 视为新TB, combined_count重置为1
    // eff_snr应回到ul_snr(2000), 而非累积
    auto r2 = harq.receive_tb(0x0001, make_ul_grant(0, false, 15, 200, 8));
    EXPECT_TRUE(r2.is_new_tb);
    EXPECT_EQ(r2.tx_count, 1u);          // 新TB, 计数重置
    EXPECT_EQ(r2.eff_snr_x100, 2000);    // 软buffer已刷新, 无IR累积

    auto stats = harq.get_ue_stats(0x0001);
    EXPECT_EQ(stats.total_new_tb, 2u);   // 两次新TB
    EXPECT_EQ(stats.total_soft_combine, 0u); // 无重传
}

// 测试22: 达到最大重传次数后丢弃TB
// ul_snr=-500, MCS=10 解码阈值=500+100=600, IR增益2dB/次, 3次重传仍不够 -> 丢弃
TEST(test_enb_harq_max_retx_discard) {
    enb_ul_harq_manager harq(3); // max_harq_tx=3
    harq.add_ue(0x0001);
    harq.set_ul_snr(0x0001, -500);

    auto r1 = harq.receive_tb(0x0001, make_ul_grant(0, true, 10, 150, 0));
    EXPECT_TRUE(!r1.crc_ok); EXPECT_EQ(r1.tx_count, 1u); EXPECT_TRUE(!r1.discarded);

    auto r2 = harq.receive_tb(0x0001, make_ul_grant(0, true, 10, 150, 8));
    EXPECT_TRUE(!r2.crc_ok); EXPECT_EQ(r2.tx_count, 2u); EXPECT_TRUE(!r2.discarded);

    auto r3 = harq.receive_tb(0x0001, make_ul_grant(0, true, 10, 150, 16));
    EXPECT_TRUE(!r3.crc_ok); EXPECT_EQ(r3.tx_count, 3u);
    EXPECT_TRUE(r3.discarded);  // 第3次达到max, 丢弃
    EXPECT_TRUE(harq.get_process_state(0x0001, 0) == harq_state::INACTIVE);
    EXPECT_TRUE(!harq.has_retx_pending(0x0001));

    auto stats = harq.get_ue_stats(0x0001);
    EXPECT_EQ(stats.total_nack, 3u);
    EXPECT_EQ(stats.total_discard, 1u);
}

// 测试23: 多UE独立HARQ进程 - 弱信号UE需重传, 强信号UE一次成功
TEST(test_enb_harq_multi_ue_independent) {
    enb_ul_harq_manager harq(4);
    harq.add_ue(0x0001);
    harq.add_ue(0x0002);
    // UE1: ul_snr=200 (MCS7边缘, 需1次重传)
    // UE2: ul_snr=2000 (MCS15, 一次成功)
    harq.set_ul_snr(0x0002, 2000);

    auto r1a = harq.receive_tb(0x0001, make_ul_grant(0, true, 7, 100, 0));
    auto r2a = harq.receive_tb(0x0002, make_ul_grant(0, true, 15, 200, 0));
    EXPECT_TRUE(!r1a.crc_ok);  // UE1 NACK
    EXPECT_TRUE(r2a.crc_ok);   // UE2 ACK

    auto s1 = harq.get_ue_stats(0x0001);
    auto s2 = harq.get_ue_stats(0x0002);
    EXPECT_EQ(s1.total_nack, 1u); EXPECT_EQ(s1.total_ack, 0u);
    EXPECT_EQ(s2.total_ack, 1u);  EXPECT_EQ(s2.total_nack, 0u);

    // UE1重传后ACK (进程0), UE2发新TB到进程1 (NDI无关, 不同PID)
    auto r1b = harq.receive_tb(0x0001, make_ul_grant(0, true, 7, 100, 8));
    EXPECT_TRUE(r1b.crc_ok);
    EXPECT_EQ(harq.get_retx_pid(0x0001), -1); // 无待重传

    EXPECT_EQ(harq.get_nof_ues(), 2u);
}

// ============================================================================
// eNB 侧 BSR 解码测试 (enb_bsr_manager)
// ============================================================================
// 镜像 UE 侧 bsr_manager 编码器, 验证 eNB 解码 + per-UE LCG 缓冲区视图

// 辅助: 构造 Short/Truncated BSR CE (单LCG报告)
static bsr_ce make_short_bsr(bsr_format fmt, uint8_t lcg_id, uint8_t buf_idx) {
    bsr_ce b;
    b.format = fmt;
    b.reports.push_back(bsr_report(lcg_id, buf_idx));
    return b;
}

// 辅助: 构造 Long BSR CE (多LCG报告, 仅包含 buffer>0 的 LCG, 与UE编码器一致)
static bsr_ce make_long_bsr(const std::vector<std::pair<uint8_t, uint8_t>>& lcg_idx_pairs) {
    bsr_ce b;
    b.format = bsr_format::LONG_BSR;
    for (const auto& p : lcg_idx_pairs) {
        b.reports.push_back(bsr_report(p.first, p.second));
    }
    return b;
}

// 测试24: Short BSR 解码 - 仅更新报告的LCG, 其它LCG不受影响
TEST(test_enb_bsr_short_decode) {
    enb_bsr_manager bsr;
    bsr.add_ue(0x0001);

    // 初始: 所有LCG=0
    EXPECT_EQ(bsr.get_total_buffer(0x0001), 0u);

    // Short BSR: LCG=1, idx=10 → bsr_index_to_bytes(10)
    auto b1 = make_short_bsr(bsr_format::SHORT_BSR, 1, 10);
    EXPECT_TRUE(bsr.receive_bsr(0x0001, b1));

    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 1), bsr_index_to_bytes(10));
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), 0u);    // LCG0 未报告, 保持0
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 2), 0u);    // LCG2 未报告, 保持0
    EXPECT_EQ(bsr.get_total_buffer(0x0001), bsr_index_to_bytes(10));

    // 再来一个 Short BSR: LCG=2, idx=20
    // 验证 LCG1 仍保留之前值 (Short BSR 不清零其它LCG)
    auto b2 = make_short_bsr(bsr_format::SHORT_BSR, 2, 20);
    EXPECT_TRUE(bsr.receive_bsr(0x0001, b2));

    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 1), bsr_index_to_bytes(10)); // LCG1 保留
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 2), bsr_index_to_bytes(20));
    EXPECT_EQ(bsr.get_total_buffer(0x0001),
              bsr_index_to_bytes(10) + bsr_index_to_bytes(20));

    auto stats = bsr.get_ue_stats(0x0001);
    EXPECT_EQ(stats.total_bsr_rx, 2u);
    EXPECT_EQ(stats.short_count, 2u);
    EXPECT_EQ(stats.long_count, 0u);
    EXPECT_EQ(stats.truncated_count, 0u);
}

// 测试25: Long BSR 解码 - 清零所有LCG后填充报告的LCG
TEST(test_enb_bsr_long_decode_clears_unreported) {
    enb_bsr_manager bsr;
    bsr.add_ue(0x0001);

    // 先用 Short BSR 在 LCG0/LCG1 设值
    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 0, 15));
    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 1, 20));
    EXPECT_EQ(bsr.get_total_buffer(0x0001),
              bsr_index_to_bytes(15) + bsr_index_to_bytes(20));

    // Long BSR 仅报告 LCG2(idx=30), LCG3(idx=40)
    // 解码语义: Long BSR 先清零所有LCG, 再填充
    auto long_bsr = make_long_bsr({{2, 30}, {3, 40}});
    EXPECT_TRUE(bsr.receive_bsr(0x0001, long_bsr));

    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), 0u);    // 清零 (Long BSR未报告)
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 1), 0u);    // 清零
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 2), bsr_index_to_bytes(30));
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 3), bsr_index_to_bytes(40));
    EXPECT_EQ(bsr.get_total_buffer(0x0001),
              bsr_index_to_bytes(30) + bsr_index_to_bytes(40));

    auto stats = bsr.get_ue_stats(0x0001);
    EXPECT_EQ(stats.long_count, 1u);
    EXPECT_EQ(stats.short_count, 2u);
    EXPECT_EQ(stats.total_bsr_rx, 3u);
}

// 测试26: Truncated BSR 解码 - 行为与 Short BSR 一致 (仅更新单LCG)
TEST(test_enb_bsr_truncated_decode) {
    enb_bsr_manager bsr;
    bsr.add_ue(0x0001);

    // Truncated BSR: LCG=3, idx=25 → bytes=256
    auto b = make_short_bsr(bsr_format::TRUNCATED_BSR, 3, 25);
    EXPECT_TRUE(bsr.receive_bsr(0x0001, b));

    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 3), 256u); // bsr_index_to_bytes(25) = 256
    EXPECT_EQ(bsr.get_total_buffer(0x0001), 256u);

    auto stats = bsr.get_ue_stats(0x0001);
    EXPECT_EQ(stats.truncated_count, 1u);
    EXPECT_EQ(stats.short_count, 0u);
}

// 测试27: 多UE独立LCG缓冲区视图 - 互不干扰
TEST(test_enb_bsr_multi_ue_independent) {
    enb_bsr_manager bsr;
    bsr.add_ue(0x0001);
    bsr.add_ue(0x0002);
    EXPECT_EQ(bsr.get_nof_ues(), 2u);

    // UE1: Long BSR 报告 LCG0(idx=10), LCG1(idx=20)
    bsr.receive_bsr(0x0001, make_long_bsr({{0, 10}, {1, 20}}));
    // UE2: Long BSR 报告 LCG2(idx=30), LCG3(idx=50)
    bsr.receive_bsr(0x0002, make_long_bsr({{2, 30}, {3, 50}}));

    EXPECT_EQ(bsr.get_total_buffer(0x0001),
              bsr_index_to_bytes(10) + bsr_index_to_bytes(20));
    EXPECT_EQ(bsr.get_total_buffer(0x0002),
              bsr_index_to_bytes(30) + bsr_index_to_bytes(50));

    // UE1 LCG视图
    auto v1 = bsr.get_ue_lcg_view(0x0001);
    EXPECT_EQ(v1[0], bsr_index_to_bytes(10));
    EXPECT_EQ(v1[1], bsr_index_to_bytes(20));
    EXPECT_EQ(v1[2], 0u);
    EXPECT_EQ(v1[3], 0u);

    // UE2 LCG视图
    auto v2 = bsr.get_ue_lcg_view(0x0002);
    EXPECT_EQ(v2[0], 0u);
    EXPECT_EQ(v2[1], 0u);
    EXPECT_EQ(v2[2], bsr_index_to_bytes(30));
    EXPECT_EQ(v2[3], bsr_index_to_bytes(50));

    // 全局统计: 2 个 Long BSR
    auto g = bsr.get_stats();
    EXPECT_EQ(g.total_bsr_rx, 2u);
    EXPECT_EQ(g.long_count, 2u);

    // 移除 UE1 后不影响 UE2
    bsr.remove_ue(0x0001);
    EXPECT_EQ(bsr.get_nof_ues(), 1u);
    EXPECT_EQ(bsr.get_total_buffer(0x0001), 0u);  // UE1 已移除
    EXPECT_EQ(bsr.get_total_buffer(0x0002),
              bsr_index_to_bytes(30) + bsr_index_to_bytes(50)); // UE2 不受影响
}

// 测试28: 量化映射正确性 - 验证 bsr_index_to_bytes 双向一致性
// eNB 解码出的字节数 = UE 编码时使用的索引对应的字节数
TEST(test_enb_bsr_quantization_mapping) {
    enb_bsr_manager bsr;
    bsr.add_ue(0x0001);

    // 验证边界索引: 0, 1, 63
    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 0, 0));
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), bsr_index_to_bytes(0)); // 0
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), 0u);

    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 0, 1));
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), bsr_index_to_bytes(1)); // 10

    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 0, 63));
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), bsr_index_to_bytes(63)); // 25000

    // 端到端: UE 端 bytes_to_bsr_index(1500) → 编码 idx, eNB 解码 idx → bytes
    // 协议语义 (TS 36.321 §6.1.3.1): BSR 上报缓冲区为区间下界, 解码值 <= 原值
    uint32_t orig_bytes = 1500;
    uint8_t enc_idx = bytes_to_bsr_index(orig_bytes);
    bsr.receive_bsr(0x0001, make_short_bsr(bsr_format::SHORT_BSR, 0, enc_idx));
    uint32_t decoded_bytes = bsr.get_ul_buffer(0x0001, 0);
    // 解码值 <= 原值 (向下取整, 符合协议下界语义), 且等于对应索引的字节数
    EXPECT_TRUE(decoded_bytes <= orig_bytes);
    EXPECT_EQ(decoded_bytes, bsr_index_to_bytes(enc_idx));
    // 区间下界性质: 不存在更小的索引其字节数 > 原值
    if (enc_idx > 0) {
        EXPECT_TRUE(bsr_index_to_bytes(enc_idx - 1) <= orig_bytes);
    }
}

// 测试29: 未注册UE与空BSR的容错
TEST(test_enb_bsr_error_handling) {
    enb_bsr_manager bsr;

    // 未注册UE: 接收BSR返回false, 查询返回0
    EXPECT_FALSE(bsr.receive_bsr(0x9999, make_short_bsr(bsr_format::SHORT_BSR, 0, 10)));
    EXPECT_EQ(bsr.get_ul_buffer(0x9999, 0), 0u);
    EXPECT_EQ(bsr.get_total_buffer(0x9999), 0u);

    bsr.add_ue(0x0001);

    // 空reports的BSR: 返回false
    bsr_ce empty_bsr;
    empty_bsr.format = bsr_format::SHORT_BSR;
    EXPECT_FALSE(bsr.receive_bsr(0x0001, empty_bsr));

    // 非法 lcg_id (>=4) 被跳过, 不崩溃
    bsr_ce bad_bsr;
    bad_bsr.format = bsr_format::SHORT_BSR;
    bad_bsr.reports.push_back(bsr_report(5, 10)); // 非法 lcg_id=5
    bad_bsr.reports.push_back(bsr_report(0, 10)); // 合法 lcg_id=0
    EXPECT_TRUE(bsr.receive_bsr(0x0001, bad_bsr)); // 整体返回true (有合法报告)
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 0), bsr_index_to_bytes(10)); // LCG0 已更新
    EXPECT_EQ(bsr.get_ul_buffer(0x0001, 5 % 4), 0u); // 非法LCG不影响视图

    // reset_ue 只清缓冲区视图, 不清统计
    auto stats_before = bsr.get_ue_stats(0x0001);
    bsr.reset_ue(0x0001);
    EXPECT_EQ(bsr.get_total_buffer(0x0001), 0u);
    auto stats_after = bsr.get_ue_stats(0x0001);
    EXPECT_EQ(stats_after.total_bsr_rx, stats_before.total_bsr_rx);
}

// ============================================================================
// MAC PDU 组包/解包 测试 (P1: 新增模块, 岗位核心技能)
// 参考 3GPP TS 36.321 Section 6.1.2 / 6.2.1
// ============================================================================

// 测试30: Short BSR CE 组包 → 解包 往返一致
TEST(test_mac_pdu_short_bsr_roundtrip) {
    uint8_t buf[64];
    auto bsr = make_short_bsr(bsr_format::SHORT_BSR, 1, 20); // LCG1, idx=20
    auto pr = mac_pdu_packer::pack_bsr_only(buf, 8, bsr);
    EXPECT_TRUE(pr.ok);
    EXPECT_EQ(pr.bsr_bytes, 2u);       // 1B 子头 + 1B BSR
    EXPECT_TRUE(pr.padding_bytes >= 6u); // 剩余为 padding

    auto ur = mac_pdu_unpacker::unpack(buf, pr.written);
    EXPECT_TRUE(ur.ok);
    EXPECT_TRUE(ur.bsr.format == bsr_format::SHORT_BSR);
    EXPECT_EQ(ur.bsr.reports.size(), 1u);
    EXPECT_EQ(ur.bsr.reports[0].lcg_id, 1);
    EXPECT_EQ(ur.bsr.reports[0].buffer_size, 20);
    EXPECT_EQ(ur.sdus.size(), 0u);    // 无 SDU
}

// 测试31: Long BSR CE 组包 → 解包 多LCG一致
TEST(test_mac_pdu_long_bsr_roundtrip) {
    uint8_t buf[64];
    auto long_bsr = make_long_bsr({{0, 10}, {1, 20}, {2, 30}, {3, 40}});
    auto pr = mac_pdu_packer::pack_bsr_only(buf, 16, long_bsr);
    EXPECT_TRUE(pr.ok);
    EXPECT_EQ(pr.bsr_bytes, 4u);      // 1B 子头 + 3B BSR

    auto ur = mac_pdu_unpacker::unpack(buf, pr.written);
    EXPECT_TRUE(ur.ok);
    EXPECT_TRUE(ur.bsr.format == bsr_format::LONG_BSR);
    EXPECT_EQ(ur.bsr.reports.size(), 4u);
    // 校验每个 LCG 的索引还原正确
    for (uint8_t i = 0; i < 4; ++i) {
        bool found = false;
        for (const auto& r : ur.bsr.reports) {
            if (r.lcg_id == i) { EXPECT_EQ(r.buffer_size, static_cast<uint8_t>((i + 1) * 10)); found = true; break; }
        }
        EXPECT_TRUE(found);
    }
}

// 测试32: SDU 复用 + Padding 填充
TEST(test_mac_pdu_sdu_mux_and_padding) {
    uint8_t buf[64];
    auto bsr = make_short_bsr(bsr_format::SHORT_BSR, 0, 5);
    std::vector<lcid_sdu> sdus = {
        {lcid_sdu{3, 10}},
        {lcid_sdu{1, 20}},
    };
    auto pr = mac_pdu_packer::pack(buf, 40, bsr, sdus);
    EXPECT_TRUE(pr.ok);
    EXPECT_EQ(pr.bsr_bytes, 2u);
    EXPECT_EQ(pr.sdu_bytes, 30u);      // 10 + 20
    EXPECT_EQ(pr.padding_bytes, 4u);    // 40 - 2(BSR) - 12(SDU1:1+1+10) - 22(SDU2:1+1+20) = 4

    auto ur = mac_pdu_unpacker::unpack(buf, pr.written);
    EXPECT_TRUE(ur.ok);
    EXPECT_EQ(ur.sdus.size(), 2u);
    EXPECT_EQ(ur.sdus[0].lcid, 3);
    EXPECT_EQ(ur.sdus[0].size, 10u);
    EXPECT_EQ(ur.sdus[1].lcid, 1);
    EXPECT_EQ(ur.sdus[1].size, 20u);
    EXPECT_EQ(ur.bsr.reports[0].lcg_id, 0);
}

// 测试33: 子头 E 标志链 (多个 SDU 连续)
TEST(test_mac_pdu_multi_sdu_e_chain) {
    uint8_t buf[128];
    std::vector<lcid_sdu> sdus = {
        {lcid_sdu{0, 5}},
        {lcid_sdu{2, 7}},
        {lcid_sdu{4, 3}},
    };
    auto pr = mac_pdu_packer::pack(buf, 64, bsr_ce(), sdus); // 仅 SDU, 无 BSR
    EXPECT_TRUE(pr.ok);
    EXPECT_EQ(pr.sdu_bytes, 15u);  // 5+7+3

    auto ur = mac_pdu_unpacker::unpack(buf, pr.written);
    EXPECT_TRUE(ur.ok);
    EXPECT_EQ(ur.sdus.size(), 3u);
    EXPECT_EQ(ur.sdus[0].lcid, 0); EXPECT_EQ(ur.sdus[0].size, 5u);
    EXPECT_EQ(ur.sdus[1].lcid, 2); EXPECT_EQ(ur.sdus[1].size, 7u);
    EXPECT_EQ(ur.sdus[2].lcid, 4); EXPECT_EQ(ur.sdus[2].size, 3u);
}

// 测试34: 空 PDU / 越界保护 不崩溃
TEST(test_mac_pdu_empty_and_oob) {
    // 空输入
    auto ur1 = mac_pdu_unpacker::unpack(nullptr, 0);
    EXPECT_FALSE(ur1.ok);

    // 极小 grant: 仅够 BSR 子头+CE, 无 padding 空间
    uint8_t buf[8];
    auto bsr = make_short_bsr(bsr_format::SHORT_BSR, 0, 1);
    auto pr = mac_pdu_packer::pack(buf, 2, bsr, {});
    EXPECT_TRUE(pr.ok);
    EXPECT_EQ(pr.bsr_bytes, 2u);

    // 截断的 PDU (声明长度大于实际): 解包不越界
    uint8_t bad[4] = {0};
    auto ur2 = mac_pdu_unpacker::unpack(bad, 4);
    EXPECT_TRUE(ur2.ok); // 容错, 不崩溃
}

// ============================================================================
// main: 输出测试总结, 失败返回非 0
// ============================================================================
int main() {
// 将 Windows 控制台输入/输出代码页切到 UTF-8，避免中文乱码。
// 在非 Windows 平台此段被忽略，不影响原有行为。
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::cout << "\n========== Test Summary ==========\n";
    std::cout << "Passed: " << test_passed << " / " << test_count
              << ", Failed: " << test_failed << std::endl;
    std::cout << "==================================\n";
    if (test_failed > 0) {
        return 1;  // 有失败, 返回非 0
    }
    return 0;
}
