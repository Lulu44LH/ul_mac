// =============================================================================
// mac_pdu.h - MAC PDU 组包/解包 (上行, 对应 3GPP TS 36.321 Section 6.1.2)
//
// 实现 UL-SCH MAC PDU 的编解码:
//   - MAC subheader: R/R/E/LCID (1B) [ + F/L (1~2B, L 字段存在时) ]
//   - MAC CE: 仅实现 BSR CE (Short / Long / Truncated) 的编解码
//   - MAC SDU: 逻辑通道数据复用 (按 LCP 优先级)
//   - Padding: 剩余字节以 Padding CE (LCID=31) 或 Padding subheader 填充
//
// 设计说明 (演示级):
//   - 聚焦"组包/解包状态机与字节布局正确性", 不实现 RLC/MAC 层间真实 SDU 来源
//   - LCID 编码遵循 TS 36.321 Table 6.2.1-1 (LTE UL-SCH):
//     Truncated BSR=28, Short BSR=29, Long BSR=30, Padding=31 (均无 L 字段)
//   - 末个子头 E 位清 0: 若 PDU 恰好结束于某载荷 (无 Padding), 组包器
//     将最后一个子头的 E 位置 0, 符合 TS 36.321 §6.2.1 子头链接语义
//
// 关键参考:
//   - 3GPP TS 36.321 Section 6.1.2 (MAC PDU 格式) / Section 6.2.1 (LCID)
//   - srsRAN_4G/lib/src/mac/mac_pdu.cc (UL/DL PDU 编解码)
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include <cstdint>
#include <vector>

namespace ul_mac {

/// MAC subheader 中的 LCID 取值 (TS 36.321 Table 6.2.1-1, 上行常用值)
enum class mac_lcid : uint8_t {
    // 0-10: 逻辑信道 (数据)
    MIN_DATA_LCID = 0,
    MAX_DATA_LCID = 10,
    // 控制元素 CE —— 取值依据 3GPP TS 36.321 Table 6.2.1-1 (UL-SCH),
    // 与 srsRAN_4G lib/include/srsran/mac/pdu.h 的 ul_sch_lcid 一致
    TRUNCATED_BSR = 28,   ///< Truncated BSR (无 L 字段)
    SHORT_BSR     = 29,   ///< Short BSR (无 L 字段)
    LONG_BSR      = 30,   ///< Long BSR (无 L 字段)
    PADDING       = 31,   ///< Padding (无 L 字段)
};

/// 单个待组包的 SDU (逻辑通道数据)
struct lcid_sdu {
    uint8_t  lcid;        ///< 逻辑通道ID (0-10)
    uint32_t size;        ///< 该SDU的字节数
    // 真实系统此处为 payload 指针; 演示级仅记录 size 与来源 LCID
};

/// 组包结果 (供 eNB 解包校验 / 日志)
struct mac_pdu_pack_result {
    size_t written = 0;   ///< 实际写入字节数
    size_t sdu_bytes = 0; ///< 承载的 SDU 总字节数
    size_t bsr_bytes = 0; ///< 承载的 BSR CE 字节数
    size_t padding_bytes = 0; ///< Padding 字节数
    bool   ok = false;
};

/// MAC PDU 组包器
///
/// 行为 (LCP 逻辑简化版):
///   1. 若提供 BSR CE, 优先放入 (BSR 是控制信令, 优先级最高)
///   2. 按 LCID 优先级 (priority 小=高) 顺序复用 SDU, 直到 grant 用尽
///   3. 剩余空间 >= 2 字节用 Padding CE (LCID=31 + 1B subheader)
///      若仅剩 1 字节, 用单字节 Padding subheader (无后续字节)
class mac_pdu_packer {
public:
    /// 组包到 buf (至少 grant_bytes 字节可用)
    /// @param buf         输出缓冲
    /// @param grant_bytes 授权字节数 (= TBS)
    /// @param bsr         可选 BSR CE ( Short/Long/Truncated ), 空 = 不携带
    /// @param sdus        待复用 SDU 列表 (调用方已按 LCP 排好序或乱序均可, 内部按优先级)
    /// @return 组包统计
    static mac_pdu_pack_result pack(uint8_t* buf, size_t grant_bytes,
                                    const bsr_ce& bsr,
                                    const std::vector<lcid_sdu>& sdus);

    /// 仅 BSR CE 的组包 (无数据 SDU) - 便捷封装
    static mac_pdu_pack_result pack_bsr_only(uint8_t* buf, size_t grant_bytes,
                                             const bsr_ce& bsr);
};

/// MAC PDU 解包结果
struct mac_pdu_unpack_result {
    bool ok = false;
    bsr_ce bsr;                                  ///< 解析出的 BSR CE (无则 format=UNKNOWN)
    std::vector<lcid_sdu> sdus;                  ///< 解析出的 SDU 列表
    size_t padding_bytes = 0;
    size_t consumed = 0;                         ///< 已解析字节数
};

/// MAC PDU 解包器 (eNB 侧, 对应 UE 上行发送)
///
/// 解析子头链:
///   - 每个子头 1B: [R|R|E|LCID]
///   - 若 LCID 为数据(0-10) 或 需要 L 字段的 CE, 紧跟 F(1b)+L(7b 或 15b)
///   - E=1 表示后面还有子头; E=0 表示后面是 SDU/Padding 的剩余部分
class mac_pdu_unpacker {
public:
    /// 解包 buf (长度 len)
    static mac_pdu_unpack_result unpack(const uint8_t* buf, size_t len);
};

} // namespace ul_mac
