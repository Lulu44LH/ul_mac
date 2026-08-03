// =============================================================================
// ue_bsr_manager.cpp - 上行缓冲区状态报告(BSR)管理器实现
//
// 实现基于3GPP TS 36.321 Section 5.4.5 的BSR过程
// 参考 srsRAN_4G/srsue/src/stack/mac/proc_bsr.cc
// =============================================================================

#include "ul_mac/ue_bsr_manager.h"
#include <cmath>
#include <algorithm>

namespace ul_mac {

// MAC控制元素大小常量
// 对应 srsRAN proc_bsr.cc generate_bsr() 中的 CE_SUBHEADER_LEN 和 ce_size
static constexpr uint32_t CE_SUBHEADER_LEN = 1;  ///< MAC CE子头长度
static constexpr uint32_t SHORT_BSR_SIZE = 1;    ///< Short BSR: 1字节(2-bit LCG ID + 6-bit缓冲区大小)
static constexpr uint32_t LONG_BSR_SIZE = 3;     ///< Long BSR: 3字节(4个LCG各6-bit缓冲区大小, 共24bit)

bsr_manager::bsr_manager(uint16_t rnti, lcg_buffer_manager& buffer_mgr)
    : rnti_(rnti)
    , buffer_mgr_(buffer_mgr)
    , initialized_(false)
    , triggered_type_(bsr_trigger_type::NONE)
    , periodic_timer_counter_(0)
    , retx_timer_counter_(0)
    , periodic_timer_running_(false)
    , retx_timer_running_(false)
    , sr_proc_(nullptr)
    , tx_callback_(nullptr)
    , history_idx_(0)
    , differential_enabled_(true)
{
    buffer_history_.fill(0);
    last_reported_bsr_.fill(0);
}

void bsr_manager::init(const bsr_config& config,
                        sr_manager* sr_proc,
                        bsr_tx_callback tx_cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    bsr_cfg_ = config;
    sr_proc_ = sr_proc;
    tx_callback_ = tx_cb;
    initialized_ = true;
    triggered_type_ = bsr_trigger_type::NONE;

    // 启动定时器 (对应 srsRAN proc_bsr.cc init() 中的定时器配置)
    if (config.periodic_timer > 0) {
        periodic_timer_running_ = true;
        periodic_timer_counter_ = config.periodic_timer;
    }

    LOG_INFO("BSR", rnti_, 0,
        "BSR initialized: periodic_timer=" + std::to_string(config.periodic_timer) +
        "ms, retx_timer=" + std::to_string(config.retx_timer) + "ms");
}

void bsr_manager::set_config(const bsr_config& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    bsr_cfg_ = config;

    if (config.periodic_timer > 0) {
        periodic_timer_running_ = true;
        periodic_timer_counter_ = config.periodic_timer;
        LOG_INFO("BSR", rnti_, 0,
            "BSR periodic timer configured: " + std::to_string(config.periodic_timer) + "ms");
    } else {
        periodic_timer_running_ = false;
    }
}

void bsr_manager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    periodic_timer_running_ = false;
    retx_timer_running_ = false;
    triggered_type_ = bsr_trigger_type::NONE;
    LOG_DEBUG("BSR", rnti_, 0, "BSR state reset");
}

void bsr_manager::set_trigger(bsr_trigger_type type) {
    // 对应 srsRAN proc_bsr.cc set_trigger() 方法
    triggered_type_ = type;

    // 【关键】当Regular BSR被触发时, 自动触发SR过程
    // 对应 srsRAN proc_bsr.cc: "Triggering SR procedure"
    if (type == bsr_trigger_type::REGULAR && sr_proc_) {
        LOG_DEBUG("BSR", rnti_, 0, "Regular BSR triggered, triggering SR procedure");
        sr_proc_->start();
    }
}

bool bsr_manager::check_regular_bsr_trigger() {
    // 对应 srsRAN proc_bsr.cc step() 中的触发条件检查
    //
    // Regular BSR触发条件 (3GPP TS 36.321 Section 5.4.4):
    //   1. 当UL缓冲区为空且有新数据到达 (check_new_data)
    //   2. 当有数据属于比当前缓冲区中数据更高优先级的LCG (check_highest_priority_channel)
    //   3. retxBSR-Timer超时且有数据待发送

    bool trigger = false;

    // 条件1: 新数据到达
    if (buffer_mgr_.check_new_data()) {
        LOG_DEBUG("BSR", rnti_, 0, "Regular BSR trigger: new data arrived in empty LCG");
        trigger = true;
    }
    // 条件2: 高优先级数据到达
    else if (buffer_mgr_.check_highest_priority_channel()) {
        LOG_DEBUG("BSR", rnti_, 0, "Regular BSR trigger: higher priority data arrived");
        trigger = true;
    }

    return trigger;
}

void bsr_manager::handle_timer_expiry(uint32_t tti) {
    // 对应 srsRAN proc_bsr.cc timer_expired() 方法

    // periodicBSR-Timer超时
    if (periodic_timer_counter_ == 0 && periodic_timer_running_) {
        if (triggered_type_ == bsr_trigger_type::NONE) {
            set_trigger(bsr_trigger_type::PERIODIC);
            LOG_DEBUG("BSR", rnti_, tti, "Periodic BSR triggered by timer expiry");
        }
        // 重启周期定时器
        periodic_timer_counter_ = bsr_cfg_.periodic_timer;
    }

    // retxBSR-Timer超时
    if (retx_timer_counter_ == 0 && retx_timer_running_) {
        // 仅当有数据待发送时才触发Regular BSR
        // 对应 srsRAN proc_bsr.cc: "Triger Regular BSR if UE has available data"
        if (buffer_mgr_.check_any_channel_has_data()) {
            set_trigger(bsr_trigger_type::REGULAR);
            LOG_DEBUG("BSR", rnti_, tti, "Regular BSR triggered by retxBSR-Timer expiry");
        }
        // 重启重传定时器
        retx_timer_counter_ = bsr_cfg_.retx_timer;
    }
}

void bsr_manager::step(uint32_t tti) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return;

    // 1. 递减定时器计数器
    if (periodic_timer_running_ && periodic_timer_counter_ > 0) {
        periodic_timer_counter_--;
    }
    if (retx_timer_running_ && retx_timer_counter_ > 0) {
        retx_timer_counter_--;
    }

    // 2. 处理定时器超时
    handle_timer_expiry(tti);

    // 3. 检查是否需要触发Regular BSR
    // 对应 srsRAN proc_bsr.cc step() 中的触发检查
    if (check_regular_bsr_trigger()) {
        LOG_INFO("BSR", rnti_, tti, "Triggering Regular BSR");
        set_trigger(bsr_trigger_type::REGULAR);
    }

    // 4. 更新缓冲区状态 (将new_buffer复制到old_buffer)
    // 对应 srsRAN proc_bsr.cc step() 中的 update_old_buffer()
    buffer_mgr_.update_old_buffer();

    // 【优化】记录流量历史 (用于预测性BSR)
    uint32_t current_buffer = buffer_mgr_.get_total_buffer_state();
    buffer_history_[history_idx_ % buffer_history_.size()] = current_buffer;
    history_idx_++;
}

bsr_format bsr_manager::select_bsr_format(uint32_t pdu_space,
                                            uint32_t nof_lcg_with_data) {
    // 【优化】BSR格式选择算法
    //
    // 标准3GPP选择逻辑 (对应 srsRAN proc_bsr.cc generate_bsr()):
    //   - 空间足够Long BSR且多个LCG有数据 -> Long BSR
    //   - 空间足够Long BSR但仅1个LCG有数据 -> Short BSR (节省空间)
    //   - 空间不足Long BSR但够Short BSR, 多个LCG有数据 -> Truncated BSR
    //   - 空间不足Long BSR但够Short BSR, 仅1个LCG有数据 -> Short BSR
    //
    // 注意: Padding BSR空间足够时优先用Long BSR (3GPP TS 36.321 §5.4.5:
    // 填充空间本就要浪费, 不如携带更完整的缓冲区信息)

    uint32_t long_bsr_total = CE_SUBHEADER_LEN + LONG_BSR_SIZE;
    uint32_t short_bsr_total = CE_SUBHEADER_LEN + SHORT_BSR_SIZE;

    if (pdu_space >= long_bsr_total) {
        // 空间足够发送Long BSR
        if (triggered_type_ != bsr_trigger_type::PADDING && nof_lcg_with_data <= 1) {
            // Regular/Periodic BSR且仅1个LCG有数据, 用Short BSR
            return bsr_format::SHORT_BSR;
        } else {
            return bsr_format::LONG_BSR;
        }
    } else if (pdu_space >= short_bsr_total) {
        // 空间只能容纳Short/Truncated BSR
        if (nof_lcg_with_data > 1) {
            return bsr_format::TRUNCATED_BSR;
        } else {
            return bsr_format::SHORT_BSR;
        }
    }

    // 空间不足, 无法发送任何BSR
    return bsr_format::SHORT_BSR; // 返回默认, 外部会检查空间
}

bool bsr_manager::generate_bsr(bsr_ce& bsr, uint32_t pdu_space) {
    // 对应 srsRAN proc_bsr.cc generate_bsr() 方法
    //
    // 生成BSR MAC控制元素:
    //   1. 统计有数据的LCG数量
    //   2. 选择BSR格式
    //   3. 填充各LCG的缓冲区大小索引

    // 获取各LCG的缓冲区大小
    auto lcg_sizes = buffer_mgr_.get_all_lcg_buffer_sizes();

    // 统计有数据的LCG数量
    uint32_t nof_lcg_with_data = 0;
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        if (lcg_sizes[i] > 0) {
            nof_lcg_with_data++;
        }
    }

    // 【优化】差分BSR: 仅Padding BSR场景下生效
    // 比较当前各LCG的BSR索引与上次报告值, 若全部未变化则跳过本次发送
    // 原理: Padding BSR利用剩余填充空间携带缓冲区状态, 若状态未变则无需重复上报
    // 注意: Regular/Periodic BSR不受此优化影响 (3GPP标准要求必须发送)
    if (differential_enabled_ && triggered_type_ == bsr_trigger_type::PADDING) {
        bool any_change = false;
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            uint8_t cur_idx = bytes_to_bsr_index(lcg_sizes[i]);
            if (cur_idx != last_reported_bsr_[i]) {
                any_change = true;
                break;
            }
        }
        if (!any_change) {
            LOG_DEBUG("BSR", rnti_, 0,
                "Differential BSR: no LCG buffer index changed, skipping padding BSR");
            return false;
        }
    }

    // 选择BSR格式
    bsr.format = select_bsr_format(pdu_space, nof_lcg_with_data);

    // 检查是否有足够空间
    uint32_t required_space;
    if (bsr.format == bsr_format::LONG_BSR) {
        required_space = CE_SUBHEADER_LEN + LONG_BSR_SIZE;
    } else {
        required_space = CE_SUBHEADER_LEN + SHORT_BSR_SIZE;
    }

    if (pdu_space < required_space) {
        return false; // 空间不足
    }

    // 填充BSR报告
    bsr.reports.clear();

    if (bsr.format == bsr_format::SHORT_BSR) {
        // Short BSR: 只报告最高优先级LCG
        uint32_t target_lcg = 0;
        if (nof_lcg_with_data == 1) {
            // 仅1个LCG有数据, 找到它
            for (uint32_t i = 0; i < NOF_LCGS; i++) {
                if (lcg_sizes[i] > 0) {
                    target_lcg = i;
                    break;
                }
            }
        } else {
            // 多个LCG有数据但空间只够Short BSR, 报告最高优先级
            target_lcg = buffer_mgr_.find_max_priority_lcg_with_data();
        }
        uint8_t idx = bytes_to_bsr_index(lcg_sizes[target_lcg]);
        bsr.reports.push_back(bsr_report(static_cast<uint8_t>(target_lcg), idx));

    } else if (bsr.format == bsr_format::TRUNCATED_BSR) {
        // Truncated BSR: 只报告最高优先级LCG (与Short BSR类似)
        uint32_t target_lcg = buffer_mgr_.find_max_priority_lcg_with_data();
        uint8_t idx = bytes_to_bsr_index(lcg_sizes[target_lcg]);
        bsr.reports.push_back(bsr_report(static_cast<uint8_t>(target_lcg), idx));

    } else {
        // Long BSR: 报告所有有数据的LCG
        for (uint32_t i = 0; i < NOF_LCGS; i++) {
            if (lcg_sizes[i] > 0) {
                uint8_t idx = bytes_to_bsr_index(lcg_sizes[i]);
                bsr.reports.push_back(bsr_report(static_cast<uint8_t>(i), idx));
            }
        }
    }

    // 生成BSR后, 重启周期定时器 (非Truncated BSR)
    // 对应 srsRAN proc_bsr.cc: "Restart or Start Periodic timer"
    if (bsr_cfg_.periodic_timer > 0 && bsr.format != bsr_format::TRUNCATED_BSR) {
        periodic_timer_running_ = true;
        periodic_timer_counter_ = bsr_cfg_.periodic_timer;
    }

    // 重置触发状态
    // 3GPP TS 36.321: 含Truncated BSR的PDU不取消已触发的BSR (下次授权仍需补发完整BSR)
    if (bsr.format != bsr_format::TRUNCATED_BSR) {
        triggered_type_ = bsr_trigger_type::NONE;
    }

    // 更新统计
    stats_.total_bsr_sent++;
    switch (bsr.format) {
        case bsr_format::SHORT_BSR:     stats_.short_count++;     break;
        case bsr_format::LONG_BSR:      stats_.long_count++;      break;
        case bsr_format::TRUNCATED_BSR: stats_.truncated_count++; break;
    }
    metrics_collector::instance().record_bsr_tx(bsr.format);

    // 【优化】更新上次报告的BSR索引 (用于差分BSR比较)
    // 记录本次实际报告的各LCG缓冲区索引, 供下次Padding BSR差分判断使用
    for (uint32_t i = 0; i < NOF_LCGS; i++) {
        last_reported_bsr_[i] = bytes_to_bsr_index(lcg_sizes[i]);
    }

    return true;
}

bool bsr_manager::need_to_send_bsr_on_ul_grant(uint32_t grant_size,
                                                  uint32_t total_data,
                                                  bsr_ce& bsr) {
    // 对应 srsRAN proc_bsr.cc need_to_send_bsr_on_ul_grant() 方法
    //
    // 3GPP TS 36.321 Section 5.4.5:
    //   - 当有触发的Periodic或Regular BSR时, 在UL授权中发送
    //   - 如果UL授权可以容纳所有待发送数据, 取消所有BSR触发
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return false;

    if (triggered_type_ == bsr_trigger_type::PERIODIC ||
        triggered_type_ == bsr_trigger_type::REGULAR) {
        // 检查授权是否可以容纳所有数据
        // 对应 srsRAN proc_bsr.cc: "All triggered BSRs shall be cancelled if grant can accommodate all pending data"
        if (grant_size >= total_data) {
            // 授权足够, 取消BSR触发
            triggered_type_ = bsr_trigger_type::NONE;
            return false;
        }

        // 生成BSR
        // 注意: 先保存触发类型, generate_bsr()内部会将triggered_type_清为NONE
        bsr_trigger_type trigger_before = triggered_type_;
        bool result = generate_bsr(bsr, grant_size);
        if (result) {
            LOG_INFO("BSR", rnti_, 0,
                "Sending " + bsr_format_to_string(bsr.format) +
                " on UL grant (grant=" + std::to_string(grant_size) +
                "B, data=" + std::to_string(total_data) + "B)");

            // 更新触发类型统计
            if (trigger_before == bsr_trigger_type::REGULAR) {
                stats_.regular_count++;
            } else {
                stats_.periodic_count++;
            }

            if (tx_callback_) {
                tx_callback_(rnti_, bsr);
            }
        }
        return result;
    }

    return false;
}

bool bsr_manager::generate_padding_bsr(uint32_t nof_padding_bytes, bsr_ce& bsr) {
    // 对应 srsRAN proc_bsr.cc generate_padding_bsr() 方法
    //
    // 当UL授权中有剩余填充空间时, 利用填充空间发送Padding BSR
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return false;

    // 检查填充空间是否足够容纳BSR
    if (nof_padding_bytes < CE_SUBHEADER_LEN + SHORT_BSR_SIZE) {
        return false; // 填充空间不足
    }

    // 设置Padding触发 (不覆盖已有的Regular/Periodic触发,
    // 已触发的BSR可直接利用填充空间发送)
    if (triggered_type_ == bsr_trigger_type::NONE) {
        triggered_type_ = bsr_trigger_type::PADDING;
    }

    bool result = generate_bsr(bsr, nof_padding_bytes);
    if (result) {
        stats_.padding_count++;
        LOG_DEBUG("BSR", rnti_, 0,
            "Sending padding " + bsr_format_to_string(bsr.format) +
            " (padding=" + std::to_string(nof_padding_bytes) + "B)");

        if (tx_callback_) {
            tx_callback_(rnti_, bsr);
        }
    }
    return result;
}

void bsr_manager::update_bsr_tti_end(const bsr_ce& bsr) {
    // 对应 srsRAN proc_bsr.cc update_bsr_tti_end() 方法
    // 在TTI结束后更新缓冲区状态

    // 启动retxBSR-Timer (如果有数据被发送)
    // 对应 srsRAN: retxBSR-Timer在BSR发送后启动
    if (bsr_cfg_.retx_timer > 0 && !bsr.reports.empty()) {
        retx_timer_running_ = true;
        retx_timer_counter_ = bsr_cfg_.retx_timer;
    }
}

uint32_t bsr_manager::predict_buffer_demand() const {
    // 【优化】预测性BSR算法 (深化版)
    //
    // 基于历史缓冲区大小预测未来需求
    // 深化点: 从简单加权平均升级为线性回归预测, 真正捕捉缓冲区的增长/下降趋势
    //
    // 线性回归模型: y = a*x + b
    //   x = 时间索引 (0, 1, ..., count-1), 按时间顺序排列
    //   y = 缓冲区大小
    //   预测下一时刻: y_pred = a * count + b
    //   - slope(a) > 0: 缓冲区正在增长, 预测值高于当前值, 提前申请更多资源
    //   - slope(a) < 0: 缓冲区正在下降, 预测值低于当前值, 避免过度申请资源

    std::lock_guard<std::mutex> lock(mutex_);

    // 历史样本不足时无法拟合, 直接返回当前缓冲区状态
    if (history_idx_ < 2) {
        return buffer_mgr_.get_total_buffer_state();
    }

    uint32_t count = std::min(history_idx_, static_cast<uint32_t>(buffer_history_.size()));

    // 最小二乘法求解线性回归系数
    //   slope = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
    //   intercept = (Σy - slope*Σx) / n
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (uint32_t i = 0; i < count; i++) {
        // 按时间顺序取历史样本: 最旧的样本x=0, 最新的样本x=count-1
        uint32_t idx = (history_idx_ - count + i) % buffer_history_.size();
        double x = static_cast<double>(i);
        double y = static_cast<double>(buffer_history_[idx]);
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double n = static_cast<double>(count);
    double denominator = n * sum_x2 - sum_x * sum_x;

    // 分母为0表示所有x相同 (理论上不应发生, 因为x是连续索引), 退化为返回最新值
    if (std::abs(denominator) < 1e-10) {
        return buffer_mgr_.get_total_buffer_state();
    }

    double slope = (n * sum_xy - sum_x * sum_y) / denominator;
    double intercept = (sum_y - slope * sum_x) / n;

    // 预测下一个采样点 (x = count, 即历史序列之后的第一个点)
    double predicted = slope * count + intercept;

    // 缓冲区大小不可为负, 限制下界
    if (predicted < 0) predicted = 0;

    return static_cast<uint32_t>(predicted);
}

} // namespace ul_mac
