#include "WS2812.h"
#include "tim.h"
#include <cstring> // for memset

// 定义 WS2812 协议中 '0' 和 '1' 的高电平时间占总周期的比例
// 这些值是根据 WS2812 (800kHz) 数据手册的典型值调整的
constexpr float DUTY_CYCLE_HIGH = 0.64f; // '1' 码, ~0.8us
constexpr float DUTY_CYCLE_LOW = 0.32f;  // '0' 码, ~0.4us

// 在数据流末尾需要一段长的低电平来让 LED "锁存" 颜色数据
// 我们通过发送一串占空比为0的 PWM 波形来实现
// 50us / (1.25us/bit) = 40 bits. 我们用 50 bit 作为安全余量
constexpr uint16_t RESET_PULSE_BITS = 50;

namespace led {

// 定义全局实例
ws2812_strip main_led_strip;
    
// 指向当前活动实例的静态指针，用于中断回调
static ws2812_strip* active_instance = nullptr;

// 全局初始化函数，方便调用
void begin() {
    const uint16_t NUM_LEDS = 5; 
    main_led_strip.begin(&htim3, TIM_CHANNEL_3, NUM_LEDS);
}

ws2812_strip::~ws2812_strip() {
    // 释放动态分配的内存
    delete[] pixel_buffer_;
    delete[] pwm_buffer_;
}

void ws2812_strip::begin(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t num_leds) {
    htim_ = htim;
    channel_ = channel;
    num_leds_ = num_leds;
    
    // 设置回调实例
    active_instance = this;

    // 计算代表 '1' 和 '0' 的 CCR (Pulse) 值
    uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(htim_);
    pwm_val_high_ = static_cast<uint16_t>((timer_period + 1) * DUTY_CYCLE_HIGH);
    pwm_val_low_  = static_cast<uint16_t>((timer_period + 1) * DUTY_CYCLE_LOW);

    // 分配颜色缓冲区 (每个 LED 3 字节: G, R, B)
    if (pixel_buffer_) delete[] pixel_buffer_;
    pixel_buffer_ = new uint8_t[num_leds_ * 3];
    memset(pixel_buffer_, 0, num_leds_ * 3);

    // 分配 PWM 缓冲区 (每个颜色字节需要8个PWM周期，外加末尾的复位脉冲)
    pwm_buffer_size_ = (num_leds_ * 24) + RESET_PULSE_BITS;
    if (pwm_buffer_) delete[] pwm_buffer_;
    pwm_buffer_ = new uint16_t[pwm_buffer_size_];
    memset(pwm_buffer_, 0, pwm_buffer_size_ * sizeof(uint16_t));
    
    dma_busy_ = false;
}

void ws2812_strip::set_pixel_color(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= num_leds_) {
        return; // 越界检查
    }
    // WS2812 的数据格式是 GRB
    uint16_t pos = index * 3;
    pixel_buffer_[pos]     = g;
    pixel_buffer_[pos + 1] = r;
    pixel_buffer_[pos + 2] = b;
}

uint32_t ws2812_strip::get_pixel_color(uint16_t index) const {
    if (index >= num_leds_) {
        return 0;
    }
    uint16_t pos = index * 3;
    uint8_t g = pixel_buffer_[pos];
    uint8_t r = pixel_buffer_[pos + 1];
    uint8_t b = pixel_buffer_[pos + 2];
    return (static_cast<uint32_t>(g) << 16) | (static_cast<uint32_t>(r) << 8) | b;
}

void ws2812_strip::clear() {
    memset(pixel_buffer_, 0, num_leds_ * 3);
}

bool ws2812_strip::is_busy() const {
    return dma_busy_;
}

uint16_t ws2812_strip::get_num_pixels() const {
    return num_leds_;
}

void ws2812_strip::fill_pwm_buffer() {
    uint32_t buffer_pos = 0;
    for (uint16_t i = 0; i < num_leds_ * 3; ++i) {
        uint8_t color_byte = pixel_buffer_[i];
        for (int j = 7; j >= 0; --j) {
            if ((color_byte >> j) & 0x01) {
                pwm_buffer_[buffer_pos] = pwm_val_high_;
            } else {
                pwm_buffer_[buffer_pos] = pwm_val_low_;
            }
            buffer_pos++;
        }
    }
    // 复位脉冲部分在 begin() 中已经用 memset 设置为 0，无需再次填充
}


bool ws2812_strip::show() {
    if (dma_busy_) {
        return false; // 上次传输还未完成
    }
    
    dma_busy_ = true;
    
    // 1. 根据颜色缓冲区填充 PWM 缓冲区
    fill_pwm_buffer();
    
    // 2. 启动 DMA 传输
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start_DMA(
        htim_, 
        channel_, 
        reinterpret_cast<uint32_t*>(pwm_buffer_), 
        pwm_buffer_size_
    );
    
    if (status != HAL_OK) {
        dma_busy_ = false; // 启动失败
        return false;
    }
    
    return true;
}

void ws2812_strip::dma_cplt_callback() {
    // 停止 PWM 输出，防止 DMA 循环传输
    HAL_TIM_PWM_Stop_DMA(htim_, channel_);
    dma_busy_ = false;
}

} // namespace led

/**
 * @brief  HAL 库的 PWM 脉冲完成回调函数 (DMA 模式下即传输完成)
 * @note   这是一个 weak 函数，我们在这里重写它
 * @param  htim: 定时器句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    // 检查是否是我们的定时器触发的回调
    if (led::active_instance && htim->Instance == led::active_instance->get_timer_handle()->Instance) {
        led::active_instance->dma_cplt_callback();
    }
}