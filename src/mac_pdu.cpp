// =============================================================================
// mac_pdu.cpp - MAC PDU 组包/解包实现 (上行, TS 36.321 Section 6.1.2)
// 详见 mac_pdu.h 头部说明
// =============================================================================

#include "ul_mac/mac_pdu.h"
#include <algorithm>
#include <cstring>

namespace ul_mac {

// ---------------------------------------------------------------------------
// 内部辅助: BSR CE 字节布局
// ---------------------------------------------------------------------------
// Short / Truncated BSR: 1 字节 = [ LCG_ID(2bit) | BUFFER_INDEX(6bit) ]
//   Short BSR 报告 1 个 LCG; Truncated BSR 同样 1 字节格式, 仅语义不同
// Long BSR: 3 字节, 每 LCG 6bit, 顺序 LCG0..LCG3 (各占 6bit, 共 24bit)
//   字节0: [LCG0_hi2 | LCG1(6bit 完整)]  -> 实际布局: LCG0 占 bit7-6, LCG1 占 bit5-0
//   标准布局 (TS 36.321 §6.1.3.1 Long BSR):
//     Octet1: LCG0[5:0] 的低6位 -> LCG0(6) ; Octet2: LCG1(6); Octet3: LCG2(6)+LCG3(6) 拆分
//   这里采用逐 LCG 6bit 紧凑排列 (LCG0 在最高位):
//     bit: [ LCG0(6) | LCG1(6) | LCG2(6) | LCG3(6) ] 共 24bit, 跨 3 字节

namespace {

// 编码单个 BSR CE 到 out, 返回写入字节数 (0=不支持的格式)
size_t encode_bsr_ce(const bsr_ce& bsr, uint8_t* out) {
    switch (bsr.format) {
        case bsr_format::SHORT_BSR:
        case bsr_format::TRUNCATED_BSR: {
            if (bsr.reports.empty()) return 0;
            const auto& r = bsr.reports[0];
            uint8_t lcg = static_cast<uint8_t>(r.lcg_id & 0x03);
            uint8_t idx = static_cast<uint8_t>(r.buffer_size & 0x3F);
            out[0] = static_cast<uint8_t>((lcg << 6) | idx);
            return 1;
        }
        case bsr_format::LONG_BSR: {
            // 24bit: LCG0..3 各 6bit, LCG0 在高位
            uint32_t bits = 0;
            for (int i = 0; i < 4; ++i) {
                uint8_t idx = 0;
                for (const auto& r : bsr.reports) {
                    if (r.lcg_id == static_cast<uint8_t>(i)) {
                        idx = static_cast<uint8_t>(r.buffer_size & 0x3F);
                        break;
                    }
                }
                bits = (bits << 6) | idx;
            }
            out[0] = static_cast<uint8_t>((bits >> 16) & 0xFF);
            out[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
            out[2] = static_cast<uint8_t>(bits & 0xFF);
            return 3;
        }
        default:
            return 0;
    }
}

// 解码 BSR CE: 已知格式与首字节位置, 写入 bsr
void decode_bsr_ce(mac_lcid lcid, const uint8_t* p, bsr_ce& bsr) {
    if (lcid == mac_lcid::LONG_BSR) {
        bsr.format = bsr_format::LONG_BSR;
        bsr.reports.clear();
        uint32_t bits = (static_cast<uint32_t>(p[0]) << 16) |
                        (static_cast<uint32_t>(p[1]) << 8) |
                        static_cast<uint32_t>(p[2]);
        for (int i = 0; i < 4; ++i) {
            uint8_t idx = static_cast<uint8_t>((bits >> (6 * (3 - i))) & 0x3F);
            bsr.reports.emplace_back(static_cast<uint8_t>(i), idx);
        }
    } else { // SHORT_BSR / TRUNCATED_BSR
        bsr.format = (lcid == mac_lcid::TRUNCATED_BSR)
                         ? bsr_format::TRUNCATED_BSR
                         : bsr_format::SHORT_BSR;
        bsr.reports.clear();
        uint8_t lcg = static_cast<uint8_t>((p[0] >> 6) & 0x03);
        uint8_t idx = static_cast<uint8_t>(p[0] & 0x3F);
        bsr.reports.emplace_back(lcg, idx);
    }
}

// LCID 是否为数据信道 (需要 L 字段)
bool is_data_lcid(uint8_t lcid) {
    return lcid <= static_cast<uint8_t>(mac_lcid::MAX_DATA_LCID);
}

// 子头: R/R/E/LCID (1B)
uint8_t make_subheader(uint8_t lcid, bool e) {
    return static_cast<uint8_t>((lcid & 0x1F) | (e ? 0x20 : 0x00));
}

// 是否需要 L 字段 (数据 LCID 需要; CE/Padding 不需要)
bool needs_length_field(uint8_t lcid) { return is_data_lcid(lcid); }

} // anonymous namespace

// ---------------------------------------------------------------------------
// 组包
// ---------------------------------------------------------------------------
mac_pdu_pack_result mac_pdu_packer::pack(uint8_t* buf, size_t grant_bytes,
                                         const bsr_ce& bsr,
                                         const std::vector<lcid_sdu>& sdus) {
    mac_pdu_pack_result res;
    if (buf == nullptr || grant_bytes == 0) return res;
    size_t pos = 0;

    // 1) BSR CE 优先 (控制信令优先级最高)
    size_t bsr_len = encode_bsr_ce(bsr, buf + pos + 1); // 预留 1B 子头
    if (bsr_len > 0 && pos + 1 + bsr_len <= grant_bytes) {
        buf[pos] = make_subheader(static_cast<uint8_t>(
            bsr.format == bsr_format::LONG_BSR ? mac_lcid::LONG_BSR
            : bsr.format == bsr_format::TRUNCATED_BSR ? mac_lcid::TRUNCATED_BSR
                                                     : mac_lcid::SHORT_BSR),
            /*e=*/true);
        pos += 1 + bsr_len;
        res.bsr_bytes = 1 + bsr_len;
    }

    // 2) SDU 复用 (按调用方传入顺序; 调用方应已按 LCP 优先级排序)
    //    为避免越界, 每次至少需 1B 子头 + 1B L 字段 + 1B 数据
    for (const auto& sdu : sdus) {
        if (pos + 3 > grant_bytes) break;             // 不足以放下子头+L+至少1B
        if (sdu.lcid > static_cast<uint8_t>(mac_lcid::MAX_DATA_LCID)) break;
        size_t avail = grant_bytes - pos - 2;         // 扣 1B 子头 + 1B L
        if (avail == 0) break;
        size_t n = std::min(static_cast<size_t>(sdu.size), avail);
        // 子头: LCID + E(暂置1, 末尾统一修正)
        buf[pos] = make_subheader(sdu.lcid, /*e=*/true);
        buf[pos + 1] = static_cast<uint8_t>(n & 0x7F); // F=0, 7bit L (演示级仅支持 <128B SDU)
        pos += 2 + n;
        res.sdu_bytes += n;
    }

    // 3) Padding: 剩余空间处理
    size_t remain = grant_bytes - pos;
    if (remain >= 2) {
        // Padding CE: 子头(LCID=31) + (remain-1) 字节的无用数据
        buf[pos] = make_subheader(static_cast<uint8_t>(mac_lcid::PADDING), /*e=*/false);
        // 剩余 remain-1 字节无需初始化 (演示级填 0)
        std::memset(buf + pos + 1, 0, remain - 1);
        pos += remain;
        res.padding_bytes = remain;
    } else if (remain == 1) {
        // 仅 1 字节剩余: 单字节 Padding subheader (E=0, LCID=31)
        buf[pos] = make_subheader(static_cast<uint8_t>(mac_lcid::PADDING), /*e=*/false);
        pos += 1;
        res.padding_bytes = 1;
    }

    res.written = pos;
    res.ok = (pos == grant_bytes); // 组包应恰好填满 grant
    return res;
}

mac_pdu_pack_result mac_pdu_packer::pack_bsr_only(uint8_t* buf, size_t grant_bytes,
                                                 const bsr_ce& bsr) {
    return pack(buf, grant_bytes, bsr, {});
}

// ---------------------------------------------------------------------------
// 解包
// ---------------------------------------------------------------------------
mac_pdu_unpack_result mac_pdu_unpacker::unpack(const uint8_t* buf, size_t len) {
    mac_pdu_unpack_result res;
    if (buf == nullptr || len == 0) return res;

    size_t pos = 0;
    bool has_more = true;
    while (pos < len && has_more) {
        uint8_t sh = buf[pos];
        uint8_t lcid_val = sh & 0x1F;
        bool e = (sh & 0x20) != 0;
        pos += 1;

        mac_lcid lcid = static_cast<mac_lcid>(lcid_val);

        if (lcid == mac_lcid::PADDING) {
            // Padding: 剩余全部为填充, 解析结束
            res.padding_bytes += (len - pos);
            has_more = false;
            break;
        }

        if (needs_length_field(lcid_val)) {
            // 数据 SDU: 1B L 字段 (F=0, 7bit)
            if (pos >= len) break;
            uint8_t l = buf[pos] & 0x7F;
            pos += 1;
            if (pos + l > len) break; // 越界保护
            lcid_sdu sdu;
            sdu.lcid = lcid_val;
            sdu.size = l;
            res.sdus.push_back(sdu);
            pos += l;
        } else {
            // CE (BSR 等): 按 LCID 解码
            if (lcid == mac_lcid::LONG_BSR) {
                if (pos + 3 > len) break;
                decode_bsr_ce(lcid, buf + pos, res.bsr);
                pos += 3;
            } else if (lcid == mac_lcid::SHORT_BSR ||
                       lcid == mac_lcid::TRUNCATED_BSR) {
                if (pos + 1 > len) break;
                decode_bsr_ce(lcid, buf + pos, res.bsr);
                pos += 1;
            } else {
                // 未知 CE: 跳过 1 字节 (演示级容错)
                if (pos + 1 > len) break;
                pos += 1;
            }
        }

        has_more = e;
    }

    res.consumed = pos;
    res.ok = true;
    return res;
}

} // namespace ul_mac
