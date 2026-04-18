#ifndef WS2812_H
#define WS2812_H

#include "main.h" // 包含 STM32 HAL 库和 CubeMX 生成的定义
#include <cstdint>

namespace led {

class ws2812_strip {
public:
    ws2812_strip() = default;
    ~ws2812_strip();
    void begin(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t num_leds);
    void set_pixel_color(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    uint32_t get_pixel_color(uint16_t index) const;
    bool show();
    void clear();
    bool is_busy() const;
    uint16_t get_num_pixels() const;
    void dma_cplt_callback();
    TIM_HandleTypeDef* get_timer_handle() const { return htim_; }
private:
    void fill_pwm_buffer();
    TIM_HandleTypeDef *htim_ = nullptr;
    uint32_t channel_ = 0;
    uint16_t num_leds_ = 0;
    uint8_t *pixel_buffer_ = nullptr; 
    uint16_t *pwm_buffer_ = nullptr;   
    uint16_t pwm_buffer_size_ = 0;
    uint16_t pwm_val_high_ = 0;
    uint16_t pwm_val_low_ = 0;
    volatile bool dma_busy_ = false;
};
extern ws2812_strip main_led_strip;
void begin(); 

} // namespace led

#endif // WS2812_H