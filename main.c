#include "stm32f1xx_hal.h"
#include <u8g2.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// --- ADC 滤波参数 ---
#define ADC_SAMPLES 10

// --- 引脚定义 ---
#define OLED_CS_Pin         GPIO_PIN_2
#define OLED_CS_GPIO_Port   GPIOA
#define OLED_DC_Pin         GPIO_PIN_3
#define OLED_DC_GPIO_Port   GPIOA
#define OLED_RES_Pin        GPIO_PIN_4
#define OLED_RES_GPIO_Port  GPIOA

// --- 电池参数定义 ---
#define BAT_VOLTAGE_MAX 8.4f
#define BAT_VOLTAGE_MIN 6.0f

#ifndef M_PI
    #define M_PI 3.1415926535f
#endif

// --- 动画状态枚举 ---
typedef enum {
    ANIM_STATE_FILLING, ANIM_STATE_FILLED,
    ANIM_STATE_RELEASING, ANIM_STATE_RELEASED
} AnimationState;

// =================================================================
// --- 人形图标位图 (16x24 像素) ---
// =================================================================
static const unsigned char person_icon_xbm[] = {
    0x00, 0x0C, 0x00, 0x1E, 0x80, 0x3F, 0x40, 0x7F, 0x40, 0x7F, 0x80, 0x3F,
    0x00, 0x1E, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0xFE, 0x00, 0x54, 0x00, 0x38,
    0x00, 0xFE, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C,
    0x00, 0x0C, 0x00, 0x1E, 0x00, 0x1E, 0x00, 0x36, 0x00, 0x26, 0x00, 0x62,
    0x00, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// --- DS18B20 驱动 (非阻塞) ---
#define DWT_CONTROL *(volatile uint32_t *)0xE0001000
#define DWT_CYCCNT  *(volatile uint32_t *)0xE0001004
#define SCB_DEMCR   *(volatile uint32_t *)0xE000EDFC
#define DS18B20_PORT GPIOA
#define DS18B20_PIN     GPIO_PIN_1
typedef enum { TEMP_STATE_IDLE, TEMP_STATE_START_CONVERSION, TEMP_STATE_READ_DATA } TempState;
typedef struct { float current_temp; TempState state; uint32_t last_action_time; } TemperatureData;
TemperatureData temperature_data = {25.0f, TEMP_STATE_IDLE, 0};


// --- 粒子系统结构体和定义 ---
#define SPLASH_PARTICLE_COUNT 50
#define MAIN_PARTICLE_COUNT     35
typedef struct { float x, y; float vx, vy; } Particle;
Particle splash_particles[SPLASH_PARTICLE_COUNT];
Particle main_particles[MAIN_PARTICLE_COUNT];


// --- 函数声明 ---
void DWT_Init(void);
void DWT_Delay_us(uint32_t us);
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
void DS18B20_Manage_Temperature(void);
void SystemClock_Config(void);
void MX_SPI1_Init(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void drawDischargeAnimation(void);
void drawMainContent(uint8_t isDischarging, float temperature, uint8_t personNearby);
void drawBottomProgressBar(void);
float read_battery_voltage(void);
void reset_splash_particle(Particle *p);
void init_main_particles(void);
void update_and_draw_main_particles(void);
void reset_main_particle(Particle *p);
void updateAnimationProgress(uint32_t delta_ms);
void drawStatusBar(float voltage, uint8_t isCharging);
void reset_charging_particle(Particle *p);
void init_charging_particles(void);
void update_and_draw_charging_particles(void);
void drawChargingEstimate(float charger_voltage);
float get_percentage_from_voltage(float voltage);
float get_voltage_from_percentage(float percentage);


// --- 全局变量 ---
u8g2_t u8g2;
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;
AnimationState currentAnimationState = ANIM_STATE_RELEASED;
float animationProgress = 0.0f;
uint32_t lastAnimationUpdate = 0;
uint8_t isCharging = 0;

// --- 【修改】用于时间倒计时的变量 ---
uint32_t charge_session_start_time = 0;      // 本次充电会话开始的滴答时间
float    initial_remaining_minutes = 0.0f; // 【【名称修改】】本次充电开始时，预计还需多少分钟
// --- 【修改结束】---

// --- 【【核心修复：根据你的建议新增】】 ---
float last_known_battery_voltage = 0.0f; // 持续跟踪的、未充电时的真实电压
// --- 【【修复结束】】 ---


// --- 【【修改：根据你的测量值】】 ---
#define FULL_CHARGE_TIME_MINUTES 180.0f         // 3 hours * 60 min/hour
// --- 【【修改结束】】 ---

#define CHARGE_DISPLAY_PARTICLES_MS 5000        // 显示 5 秒粒子
#define CHARGE_DISPLAY_ESTIMATE_MS  3000        // 显示 3 秒预估时间
#define CHARGE_CYCLE_TOTAL_MS (CHARGE_DISPLAY_PARTICLES_MS + CHARGE_DISPLAY_ESTIMATE_MS)


// =================================================================
// --- 粒子动画函数 --- (无修改)
// =================================================================
void reset_splash_particle(Particle *p) {
    p->x = 64.0f; p->y = 32.0f;
    float angle = (rand() % 360) * (M_PI / 180.0f);
    float speed = 0.5f + (rand() % 100) / 100.0f;
    p->vx = cos(angle) * speed; p->vy = sin(angle) * speed;
}

void init_main_particles(void) {
    for (int i = 0; i < MAIN_PARTICLE_COUNT; i++) {
        reset_main_particle(&main_particles[i]);
    }
}

void reset_main_particle(Particle *p) {
    p->x = (float)(rand() % 128);
    p->y = (float)(rand() % 40);
    p->vx = ((rand() % 100) / 100.0f) - 0.5f;
    p->vy = ((rand() % 100) / 100.0f) * 0.5f;
}

void update_and_draw_main_particles(void) {
    const float bar_x = 14.0f, bar_y = 54.0f, bar_w = 100.0f;
    const float attraction_zone_top = 35.0f;
    const float attraction_strength = 0.08f;
    const float damping = 0.95f;
    for (int i = 0; i < MAIN_PARTICLE_COUNT; i++) {
        Particle *p = &main_particles[i];
        if (p->y > attraction_zone_top && p->y < bar_y) {
            float target_x = p->x;
            if (target_x < bar_x) target_x = bar_x;
            if (target_x > bar_x + bar_w) target_x = bar_x + bar_w;
            float target_y = bar_y;
            float ax = target_x - p->x; float ay = target_y - p->y;
            p->vx += ax * attraction_strength; p->vy += ay * attraction_strength;
            p->vx *= damping; p->vy *= damping;
        } else {
            p->vx += ((rand() % 100) / 500.0f) - 0.1f;
            p->vy += ((rand() % 100) / 500.0f);
        }

        if(p->vx > 1.5f) p->vx = 1.5f;
        if(p->vx < -1.5f) p->vx = -1.5f;
        if(p->vy > 2.0f) p->vy = 2.0f;
        if(p->vy < -2.0f) p->vy = -2.0f;

        p->x += p->vx; p->y += p->vy;
        if (p->y >= bar_y || p->x < 0 || p->x >= 128 || p->y < 0) {
            reset_main_particle(p);
        }
        u8g2_DrawPixel(&u8g2, (int)p->x, (int)p->y);
    }
}

void reset_charging_particle(Particle *p) {
    p->x = 64.0f + (float)(rand() % 64);
    p->y = 15.0f + (float)(rand() % 49);
    p->vx = -0.5f - ((rand() % 100) / 100.0f) * 0.5f;
    p->vy = ((rand() % 100) / 100.0f) - 0.5f;
}

void init_charging_particles(void) {
    for (int i = 0; i < MAIN_PARTICLE_COUNT; i++) {
        reset_charging_particle(&main_particles[i]);
    }
}

void update_and_draw_charging_particles(void) {
    const float target_x = 25.0f;
    const float target_y = 6.0f;
    const float attraction_strength = 0.05f;
    const float damping = 0.96f;

    for (int i = 0; i < MAIN_PARTICLE_COUNT; i++) {
        Particle *p = &main_particles[i];

        float ax = target_x - p->x;
        float ay = target_y - p->y;

        p->vx += ax * attraction_strength;
        p->vy += ay * attraction_strength;
        p->vx *= damping;
        p->vy *= damping;

        if(p->vx > 2.0f) p->vx = 2.0f;
        if(p->vx < -2.0f) p->vx = -2.0f;
        if(p->vy > 2.0f) p->vy = 2.0f;
        if(p->vy < -2.0f) p->vy = -2.0f;

        p->x += p->vx;
        p->y += p->vy;

        float dist_sq = (p->x - target_x)*(p->x - target_x) + (p->y - target_y)*(p->y - target_y);
        if (dist_sq < 10.0f || p->x < 0 || p->x >= 128 || p->y >= 64 || p->y < 0) {
            reset_charging_particle(p);
        }

        u8g2_DrawPixel(&u8g2, (int)p->x, (int)p->y);
    }
}

void updateAnimationProgress(uint32_t delta_ms) {
    if (currentAnimationState == ANIM_STATE_FILLING) {
        float delta_progress = (float)delta_ms / 2000.0f;
        animationProgress += delta_progress;
        if (animationProgress >= 1.0f) {
            animationProgress = 1.0f;
            currentAnimationState = ANIM_STATE_FILLED;
        }
    } else if (currentAnimationState == ANIM_STATE_RELEASING) {
        float delta_progress = (float)delta_ms / 500.0f;
        animationProgress -= delta_progress;
        if (animationProgress <= 0.0f) {
            animationProgress = 0.0f;
            currentAnimationState = ANIM_STATE_RELEASED;
        }
    }
}


// =================================================================
// --- 主函数 ---
// =================================================================
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_ADC1_Init();
    DWT_Init();

    u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    srand(HAL_GetTick());

    // --- 启动画面 --- (无修改)
    for (int i = 0; i < SPLASH_PARTICLE_COUNT; i++) {
        reset_splash_particle(&splash_particles[i]);
    }
    const char *line1 = "Lightning";
    u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tr);
    u8g2_uint_t line1_width = u8g2_GetStrWidth(&u8g2, line1);
    uint8_t line1_x = (128 - line1_width) / 2;
    uint8_t line1_y = 26;
    const char *line2 = "By WXL";
    u8g2_uint_t line2_width = u8g2_GetStrWidth(&u8g2, line2);
    uint8_t line2_x = (128 - line2_width) / 2;
    uint8_t line2_y = 48;
    uint32_t startTime = HAL_GetTick();
    uint32_t animationDuration = 5000;
    uint32_t frameDelay = 20;
    while (HAL_GetTick() - startTime < animationDuration) {
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tr);
        u8g2_SetFontMode(&u8g2, 1);
        u8g2_DrawStr(&u8g2, line1_x, line1_y, line1);
        u8g2_DrawStr(&u8g2, line2_x, line2_y, line2);
        for (int i = 0; i < SPLASH_PARTICLE_COUNT; i++) {
            Particle *p = &splash_particles[i];
            p->x += p->vx; p->y += p->vy;
            if (p->x < 0 || p->x >= 128 || p->y < 0 || p->y >= 64) {
                reset_splash_particle(p);
            }
            u8g2_DrawPixel(&u8g2, (int)p->x, (int)p->y);
        }
        u8g2_SendBuffer(&u8g2);
        HAL_Delay(frameDelay);
    }
    // --- 启动画面结束 ---

    init_main_particles();

    float realBatteryVoltage = 0.0f;
    float baselineBatteryVoltage = 0.0f;
    float smoothVoltage = 0.0f;

    uint8_t isDischarging = 0;
    int8_t last_isDischarging = -1;

    typedef enum { MOTION_STATE_LISTENING, MOTION_STATE_COOLDOWN } MotionState;
    MotionState motionState = MOTION_STATE_LISTENING;
    #define MOTION_DISPLAY_TIME_MS 2000
    #define MOTION_COOLDOWN_MS     2500
    uint32_t cooldownStartTime = 0;
    uint8_t personDetected = 0;
    uint8_t last_pin_state = 0;
    uint32_t first_high_time = 0;

    #define CAPACITOR_BOOST_THRESHOLD 0.06f
    #define CAPACITOR_BOOST_DISPLAY_TIME_MS 500
    uint8_t capacitorBoostActive = 0;
    uint32_t capacitorBoostStartTime = 0;

    uint32_t dischargeStopTime = 0;
    
    // 延长电击后的充电锁定时间，防止电压回弹误触
    #define CHARGE_LOCKOUT_AFTER_DISCHARGE_MS 2000 

    // --- 【【新增：SMA-3 滤波器变量】】 ---
    #define SMA_SIZE 3
    static float voltage_history[SMA_SIZE] = {0.0f};
    static int voltage_history_index = 0;
    static float sma3_voltage = 0.0f; // 3次平均滤波器的输出
    // --- 【【新增结束】】 ---

    uint32_t chargeDetectStartTime = 0;
    #define CHARGE_DEBOUNCE_TIME_MS 600

    lastAnimationUpdate = HAL_GetTick();

    while (1) {
        // --- 任务1: 电压读取与滤波 ---
        realBatteryVoltage = read_battery_voltage(); // 1. 获取瞬时(10样本均值)电压

        // --- 【【新增：SMA-3 滤波逻辑】】 ---
        // 2. 将新值存入历史记录
        voltage_history[voltage_history_index] = realBatteryVoltage;
        voltage_history_index++;
        if (voltage_history_index >= SMA_SIZE) {
            voltage_history_index = 0; // 环形缓冲
        }

        // 3. 计算 SMA(3) 平均值
        float voltage_sum = 0.0f;
        for (int i = 0; i < SMA_SIZE; i++) {
            voltage_sum += voltage_history[i];
        }
        sma3_voltage = voltage_sum / SMA_SIZE;
        // --- 【【新增结束】】 ---
        
        if (baselineBatteryVoltage == 0.0f && realBatteryVoltage > 0.1f) {
            baselineBatteryVoltage = realBatteryVoltage;
            smoothVoltage = realBatteryVoltage;
            last_known_battery_voltage = realBatteryVoltage;

            // --- 【【新增：初始化 SMA 滤波器】】 ---
            // 用第一个有效读数填满历史记录，避免从0开始
            for (int i = 0; i < SMA_SIZE; i++) {
                voltage_history[i] = realBatteryVoltage;
            }
            sma3_voltage = realBatteryVoltage; // 立即设置SMA输出
            // --- 【【新增结束】】 ---
        }

        // --- 【【修改：使用 SMA(3) 的结果进行 IIR 滤波】】 ---
        // 现在 smoothVoltage 是 10样本均值 -> SMA(3) -> IIR(0.7)
        smoothVoltage = (smoothVoltage * 0.7f) + (sma3_voltage * 0.3f); // <-- 更“稳定”的电压
        // --- 【【修改结束】】 ---

        // 必须在真正的待机状态下才更新 "last_known"
        if (isCharging == 0 && isDischarging == 0 && chargeDetectStartTime == 0) {
            last_known_battery_voltage = (last_known_battery_voltage * 0.9f) + (smoothVoltage * 0.1f);
        }


        // --- 任务2: DS18B20 温度读取 (不变) ---
        DS18B20_Manage_Temperature();

        // --- 任务3: 人体感应状态机 (不变) ---
        switch(motionState) {
            case MOTION_STATE_LISTENING: {
                uint8_t current_pin_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
                if (current_pin_state == 1) {
                    if (last_pin_state == 0) {
                        first_high_time = HAL_GetTick();
                    }
                    if (first_high_time > 0 && (HAL_GetTick() - first_high_time > 500)) {
                        personDetected = 1;
                        cooldownStartTime = HAL_GetTick();
                        motionState = MOTION_STATE_COOLDOWN;
                        first_high_time = 0;
                    }
                } else {
                    first_high_time = 0;
                }
                last_pin_state = current_pin_state;
                break;
            }
            case MOTION_STATE_COOLDOWN: {
                uint32_t elapsed = HAL_GetTick() - cooldownStartTime;
                personDetected = (elapsed < MOTION_DISPLAY_TIME_MS);
                if (elapsed > MOTION_COOLDOWN_MS) {
                    motionState = MOTION_STATE_LISTENING;
                    last_pin_state = 0;
                }
                break;
            }
        }

        // ================================================================
        // --- 【【【 START: 充放电状态机 】】】 ---
        // ================================================================

        // --- 任务4: 充放电逻辑与动画状态更新 ---

        // “快反”电压，用于电击检测 (有噪声)
        float voltageDrop = baselineBatteryVoltage - realBatteryVoltage;
        // float voltageGain = realBatteryVoltage - baselineBatteryVoltage; // <-- 已被 【【充电防抖修复】】 替代

        if (capacitorBoostActive == 0) {
            if (voltageDrop >= CAPACITOR_BOOST_THRESHOLD) {
                capacitorBoostActive = 1;
                capacitorBoostStartTime = HAL_GetTick();
            }
        } else {
            if (HAL_GetTick() - capacitorBoostStartTime > CAPACITOR_BOOST_DISPLAY_TIME_MS) {
                capacitorBoostActive = 0;
            }
        }

        // 1. 【优先处理】: 无论在什么状态，检测到电压暴跌 (电击)，立即进入放电状态
        if (voltageDrop >= 0.25f) 
        {
            isDischarging = 1;      // 强制进入放电
            isCharging = 0;         // 强制退出充电
            chargeDetectStartTime = 0; // 重置充电检测
            charge_session_start_time = 0; // 重置充电会话
            initial_remaining_minutes = 0.0f;
        }


        // 2. 【状态机】: 根据当前状态执行操作
        if (isDischarging) 
        {
            // (A) 处在放电状态: 检查是否停止
            isCharging = 0; // 再次确保充电已关闭
            chargeDetectStartTime = 0;
            
            if (voltageDrop < 0.1f) {
                isDischarging = 0;
                // 以【放电前】的真实电压为新基准
                baselineBatteryVoltage = last_known_battery_voltage; 
                dischargeStopTime = HAL_GetTick();
            }
        }
        // --- 【【【 START: 充电防抖修复 】】】 ---
        else if (isCharging)
        {
            // (B) 处在充电状态: 检查是否拔掉
            // (注意：电击导致的暴跌已在 步骤1 中被拦截)
            
            // 我们使用最稳定的 smoothVoltage 来计算增益，而不是用带噪声的 voltageGain
            float smoothVoltageGain = smoothVoltage - baselineBatteryVoltage;

            if (smoothVoltageGain < -0.02f) { // 使用平滑增益进行拔出检测
                isCharging = 0;

                charge_session_start_time = 0;
                initial_remaining_minutes = 0.0f; 

                baselineBatteryVoltage = realBatteryVoltage; // 以【拔掉后】的电压为新基准
                init_main_particles();
            } else {
                // 充电时，缓慢拉高基准电压
                // 让基准跟随平滑电压，而不是噪声电压
                baselineBatteryVoltage = (baselineBatteryVoltage * 0.99f) + (smoothVoltage * 0.01f);
            }
        }
        // --- 【【【 END: 充电防抖修复 】】】 ---
        else
        {
            // (C) 处在待机状态: 检查是否插入充电
            // (注意：放电检测已在 步骤1 中处理)
            
            // --- 【【【 START: 完美修复 (使用平滑电压) 】】】 ---
            // 使用 "smoothVoltage" (稳定值) 来计算增益，防止噪声误触发
            float smoothVoltageGain = smoothVoltage - baselineBatteryVoltage; 

            // 使用 "smoothVoltageGain" 替代 "voltageGain"
            if (smoothVoltageGain > 0.03f && (HAL_GetTick() - dischargeStopTime > CHARGE_LOCKOUT_AFTER_DISCHARGE_MS)) 
            // --- 【【【 END: 完美修复 】】】 ---
            {
                if (chargeDetectStartTime == 0) {
                    chargeDetectStartTime = HAL_GetTick();
                }
                else if (HAL_GetTick() - chargeDetectStartTime > CHARGE_DEBOUNCE_TIME_MS)
                {
                    isCharging = 1;

                    if (charge_session_start_time == 0) {
                        charge_session_start_time = HAL_GetTick();
                        float start_percent = get_percentage_from_voltage(last_known_battery_voltage);
                        start_percent = fmax(0.0f, fmin(100.0f, start_percent)); 
                        float remaining_percent = 100.0f - start_percent;
                        initial_remaining_minutes = (remaining_percent / 100.0f) * FULL_CHARGE_TIME_MINUTES; 
                    }
                    
                    init_charging_particles();
                    chargeDetectStartTime = 0;
                }
            }
            else
            {
                // 真正待机时，缓慢拉平基准电压 (吸收噪声)
                chargeDetectStartTime = 0;
                // --- 【【【 START: 噪声吸收修复 】】】 ---
                // 让基准电压更快地跟随噪声，防止`smoothVoltageGain`积累
                baselineBatteryVoltage = (baselineBatteryVoltage * 0.95f) + (realBatteryVoltage * 0.05f);
                // --- 【【【 END: 噪声吸收修复 】】】 ---
            }
        }

        // ================================================================
        // --- 【【【 END: 充放电状态机 】】】 ---
        // ================================================================


        if (isDischarging != last_isDischarging) {
            if (isDischarging) {
                currentAnimationState = ANIM_STATE_RELEASING;
            } else {
                currentAnimationState = ANIM_STATE_FILLING;
            }
            last_isDischarging = isDischarging;
        }

        // --- 任务 4.8: 更新动画进度 ---
        uint32_t now = HAL_GetTick();
        uint32_t delta_ms = now - lastAnimationUpdate;
        lastAnimationUpdate = now;
        updateAnimationProgress(delta_ms);


        // --- 任务5: 屏幕绘制 ---
        u8g2_ClearBuffer(&u8g2);

        // 1. 状态栏常驻
        drawStatusBar(smoothVoltage, isCharging);

        if (isCharging)
        {
            // --- 充电时：循环切换显示 ---
            uint32_t now_for_drawing = HAL_GetTick();
            uint32_t cycle_time = now_for_drawing % CHARGE_CYCLE_TOTAL_MS;

            if (cycle_time < CHARGE_DISPLAY_PARTICLES_MS) {
                // 阶段 1: 显示充电粒子
                update_and_draw_charging_particles();
            } else {
                // 阶段 2: 显示预估时间 (调用更新后的函数)
                drawChargingEstimate(smoothVoltage);
            }
        }
        else
        {
            // --- 不充电时：显示主内容 ---
            drawMainContent(isDischarging, temperature_data.current_temp, personDetected);
            if (!isDischarging) { update_and_draw_main_particles(); }
        }

        // 3. 底部进度条常驻
        drawBottomProgressBar();

        // 4. 电容增强提示常驻
        if (capacitorBoostActive) {
            u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tr);
            u8g2_SetFontMode(&u8g2, 1);
            u8g2_DrawStr(&u8g2, 2, 42, "+");
            u8g2_uint_t plus_width = u8g2_GetStrWidth(&u8g2, "+");
            u8g2_DrawStr(&u8g2, 127 - plus_width - 2, 42, "+");
        }

        u8g2_SendBuffer(&u8g2);
        HAL_Delay(10);
    }
}

// =================================================================
// --- 函数定义 ---
// =================================================================

// --- 【新增的SoC查找表】---
typedef struct {
    float voltage;
    float percentage; // 0.0f 到 100.0f
} VoltageSoC_t;

// 这是一个【示例】2S 18650 电池曲线，你必须通过实测来校准你的电池！
// 尤其是 6.6V, 8.2V, 8.4V 这几个关键拐点
static const VoltageSoC_t soc_table[] = {
    { 6.0f, 0.0f },   // 截止电压 (0%)
    { 6.6f, 10.0f },  // 底部快速上升区
    { 7.0f, 25.0f },
    { 7.4f, 50.0f },  // 中间平坦区
    { 7.8f, 75.0f },
    { 8.2f, 90.0f },  // 顶部快速上升区
    { 8.4f, 100.0f }  // 恒压点 (100%)
};
static const int soc_table_size = sizeof(soc_table) / sizeof(soc_table[0]);

/**
 * @brief   使用分段线性插值从查找表获取SoC百分比
 * @param   voltage: 当前电池电压
 * @return  估算的电量百分比 (0-100)
 */
float get_percentage_from_voltage(float voltage) {
    // 1. 处理边界情况
    if (voltage <= soc_table[0].voltage) {
        return soc_table[0].percentage;
    }
    if (voltage >= soc_table[soc_table_size - 1].voltage) {
        return soc_table[soc_table_size - 1].percentage;
    }

    // 2. 查找电压所在的区间
    for (int i = 1; i < soc_table_size; i++) {
        if (voltage <= soc_table[i].voltage) {
            // 找到区间 [i-1] 到 [i]
            const VoltageSoC_t* v_low = &soc_table[i-1];
            const VoltageSoC_t* v_high = &soc_table[i];

            // 3. 线性插值
            float v_range = v_high->voltage - v_low->voltage;
            float p_range = v_high->percentage - v_low->percentage;

            // 防止除以零
            if (v_range <= 0.001f) {
                return v_low->percentage;
            }

            float v_progress = (voltage - v_low->voltage) / v_range;
            return v_low->percentage + (v_progress * p_range);
        }
    }

    // 理论上不会到这里
    return soc_table[soc_table_size - 1].percentage;
}
// --- 【新增结束】---

// =================================================================
// --- 【【新增函数：从百分比反向查找电压 (非线性)】】 ---
// =================================================================
float get_voltage_from_percentage(float percentage) {
    // 1. 处理边界情况
    if (percentage <= soc_table[0].percentage) {
        return soc_table[0].voltage;
    }
    if (percentage >= soc_table[soc_table_size - 1].percentage) {
        return soc_table[soc_table_size - 1].voltage;
    }

    // 2. 查找百分比所在的区间
    for (int i = 1; i < soc_table_size; i++) {
        if (percentage <= soc_table[i].percentage) {
            // 找到区间 [i-1] 到 [i]
            const VoltageSoC_t* p_low = &soc_table[i-1];
            const VoltageSoC_t* p_high = &soc_table[i];

            // 3. 线性插值
            float p_range = p_high->percentage - p_low->percentage;
            float v_range = p_high->voltage - p_low->voltage;

            // 防止除以零
            if (p_range <= 0.001f) {
                return p_low->voltage;
            }

            // 根据百分比进度，计算电压
            float p_progress = (percentage - p_low->percentage) / p_range;
            return p_low->voltage + (p_progress * v_range);
        }
    }

    // 理论上不会到这里
    return soc_table[soc_table_size - 1].voltage;
}


// --- 【【最终版：基于固定总时间的非线性估算 + 电池动画】】 ---
void drawChargingEstimate(float charger_voltage) { // 传入的是 8.42V 的充电器电压
    char buf_min[20];
    char buf_volt[20];

    // --- 1. 计算剩余时间 ---
    float estimated_minutes_remaining = 0.0f;
    float elapsed_minutes = 0.0f;

    if (charge_session_start_time > 0) {
        uint32_t elapsed_ms = HAL_GetTick() - charge_session_start_time;
        elapsed_minutes = (float)elapsed_ms / 60000.0f;
        // 剩余时间 = 初始剩余时间 - 已过时间
        estimated_minutes_remaining = initial_remaining_minutes - elapsed_minutes;
        if (estimated_minutes_remaining < 0.0f) {
            estimated_minutes_remaining = 0.0f;
        }
    }
    sprintf(buf_min, "%d min", (int)estimated_minutes_remaining);

    // --- 2. 【【修改：基于固定总时间(180min)估算百分比和电压】】 ---

    float estimated_internal_voltage = 0.0f;
    float current_estimated_percentage = 0.0f;

    // 2a. 计算当前估算的 "百分比"
    float start_percent = get_percentage_from_voltage(last_known_battery_voltage);

    if (charge_session_start_time > 0) {
        // 百分比增加 = (已过时间 / 总时间) * 100
        float percentage_increase = (elapsed_minutes / FULL_CHARGE_TIME_MINUTES) * 100.0f;
        current_estimated_percentage = start_percent + percentage_increase;
    } else {
        // 如果不在充电会话中 (理论上不会进入此绘图函数, 但作为保险)
        current_estimated_percentage = start_percent;
    }
    current_estimated_percentage = fmax(0.0f, fmin(100.0f, current_estimated_percentage));

    // 2b. 从 "百分比" 反向推算 "估算电压" (使用非线性查找表)
    estimated_internal_voltage = get_voltage_from_percentage(current_estimated_percentage);

    // 确保估算电压不会超过充电器电压
    if (estimated_internal_voltage > charger_voltage) {
        estimated_internal_voltage = charger_voltage;
    }

    // 格式化这个 "估算" 的电压
    sprintf(buf_volt, "%d.%02dV", (int)estimated_internal_voltage, (int)((estimated_internal_voltage - (int)estimated_internal_voltage) * 100));
    // --- 【【修改结束】】 ---


    // --- 3. 绘制图标和文本 (新布局) ---

    // --- (A) 时间 (上半部分) ---
    u8g2_uint_t clock_x = 45;
    u8g2_uint_t clock_y = 24;
    u8g2_DrawCircle(&u8g2, clock_x, clock_y, 7, U8G2_DRAW_ALL); // 绘制时钟
    u8g2_DrawVLine(&u8g2, clock_x, clock_y - 4, 4);            // 时钟: 分针
    u8g2_DrawHLine(&u8g2, clock_x, clock_y, 3);                // 时钟: 时针

    u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tr); // 设置大字体
    u8g2_SetFontMode(&u8g2, 1);
    u8g2_DrawStr(&u8g2, clock_x + 12, 30, buf_min); // 绘制时间文本

    // --- (B) 【【已修改：使用估算的百分比】】 ---
    u8g2_uint_t bat_x = 42;
    u8g2_uint_t bat_y = 39;
    u8g2_DrawFrame(&u8g2, bat_x, bat_y, 20, 10);      // 绘制电池外框
    u8g2_DrawBox(&u8g2, bat_x + 20, bat_y + 3, 2, 4); // 绘制电池正极

    // 1. (我们已经在 2a 步骤中计算了: current_estimated_percentage)

    // 2. 计算填充宽度 (内部宽度 18px)
    uint8_t fill_width = (uint8_t)(current_estimated_percentage / 100.0f * 18.0f);

    // 3. 绘制填充
    if (fill_width > 0) {
         u8g2_DrawBox(&u8g2, bat_x + 1, bat_y + 1, fill_width, 8); // 内部高度 8px
    }

    // 4. 绘制动画 (在填充矩形之上 '清除' 像素)
    if (fill_width > 0) {
        u8g2_SetDrawColor(&u8g2, 0); // 0 = 擦除
        for (int x = 0; x < fill_width; x++) {
            int y_offset = (int)(sinf((float)x * 0.8f + (float)HAL_GetTick() * 0.008f) * 1.5f);
            u8g2_DrawVLine(&u8g2, bat_x + 1 + x, bat_y + 1 + 2 + y_offset, 4);
        }
        u8g2_SetDrawColor(&u8g2, 1); // 1 = 恢复绘制
    }

    // 5. 绘制电压文本
    u8g2_SetFont(&u8g2, u8g2_font_helvR10_tr);
    u8g2_SetFontMode(&u8g2, 1);
    u8g2_DrawStr(&u8g2, bat_x + 26, 48, buf_volt); // 绘制电压文本 (估算电压)
}
// --- 【替换结束】---


// --- drawMainContent (无修改) ---
void drawMainContent(uint8_t isDischarging, float temperature, uint8_t personNearby) {
    if (!isDischarging) {
        char buf[16];
        int temp_int = (int)temperature;
        int temp_frac = abs((int)((temperature - temp_int) * 10));
        sprintf(buf, "%d.%d", temp_int, temp_frac);
        u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tr);
        u8g2_uint_t temp_val_width = u8g2_GetStrWidth(&u8g2, buf);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
        u8g2_uint_t unit_c_width = u8g2_GetStrWidth(&u8g2, "C");
        u8g2_uint_t circle_part_width = 10;
        u8g2_uint_t icon_width = 16;
        u8g2_uint_t icon_space = 4;
        u8g2_uint_t temp_display_total_width = temp_val_width + circle_part_width + unit_c_width;
        u8g2_uint_t final_total_width = personNearby ? (icon_width + icon_space + temp_display_total_width) : temp_display_total_width;
        u8g2_uint_t start_x = (128 - final_total_width) / 2;
        u8g2_uint_t current_x = start_x;
        if (personNearby) {
            u8g2_DrawXBM(&u8g2, current_x, 20, icon_width, 24, person_icon_xbm);
            current_x += icon_width + icon_space;
        }
        u8g2_SetFont(&u8g2, u8g2_font_logisoso24_tr);
        u8g2_SetFontMode(&u8g2, 1);
        u8g2_DrawStr(&u8g2, current_x, 42, buf);
        current_x += temp_val_width;
        u8g2_DrawCircle(&u8g2, current_x + 5, 28, 3, U8G2_DRAW_ALL);
        u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
        u8g2_DrawStr(&u8g2, current_x + 10, 42, "C");
    } else {
        drawDischargeAnimation();
    }
}

// --- drawBottomProgressBar (无修改) ---
void drawBottomProgressBar(void) {
    u8g2_uint_t bar_x = 14, bar_y = 54, bar_w = 100, bar_h = 8;
    u8g2_DrawFrame(&u8g2, bar_x, bar_y, bar_w, bar_h);
    if (animationProgress > 0) {
        u8g2_uint_t fill_w_total = (u8g2_uint_t)((bar_w - 2) * animationProgress);
        u8g2_uint_t fill_w_half = fill_w_total / 2;
        u8g2_DrawBox(&u8g2, bar_x + 1, bar_y + 1, fill_w_half, bar_h - 2);
        u8g2_DrawBox(&u8g2, bar_x + bar_w - 1 - fill_w_half, bar_y + 1, fill_w_half, bar_h - 2);
        if (fill_w_total % 2 != 0) {
            u8g2_DrawPixel(&u8g2, bar_x + bar_w/2, bar_y + bar_h/2);
        }
    }
    if (currentAnimationState == ANIM_STATE_FILLED) {
        u8g2_DrawRFrame(&u8g2, bar_x - 2, bar_y - 2, bar_w + 4, bar_h + 4, 2);
    }
}

// --- read_battery_voltage (无修改) ---
#define CAL_FACTOR 0.004007f

float read_battery_voltage(void) {
    uint32_t adc_total = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            adc_total += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }
    uint16_t adc_value = adc_total / ADC_SAMPLES;
    float voltage = (float)adc_value * CAL_FACTOR;
    return (voltage < 0.1f) ? 0.0f : voltage;
}

// --- 【【修改：drawStatusBar 中的百分比估算逻辑】】 ---
void drawStatusBar(float voltage, uint8_t isCharging) {
    char buf[16];

    float percentage;

    if (isCharging) {
        // --- 【【修改：与 drawChargingEstimate 保持一致】】 ---
        float start_percent = get_percentage_from_voltage(last_known_battery_voltage);
        float elapsed_minutes = 0.0f;

        if (charge_session_start_time > 0) {
             uint32_t elapsed_ms = HAL_GetTick() - charge_session_start_time;
             elapsed_minutes = (float)elapsed_ms / 60000.0f;
        }

        // 百分比增加 = (已过时间 / 总时间) * 100
        // 使用 FULL_CHARGE_TIME_MINUTES (180.0f)
        float percentage_increase = (elapsed_minutes / FULL_CHARGE_TIME_MINUTES) * 100.0f;
        percentage = start_percent + percentage_increase;
        // --- 【【修改结束】】 ---

    } else {
        // 未充电时, 百分比和电压都基于 "smoothVoltage"
        percentage = get_percentage_from_voltage(voltage);
    }

    percentage = fmax(0.0f, fmin(100.0f, percentage));

    // --- 绘制部分 (无修改) ---
    uint8_t x_offset = isCharging ? 10 : 0;
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);
    u8g2_SetFontMode(&u8g2, 1);
    if (isCharging) { u8g2_DrawStr(&u8g2, 2, 10, "+"); }
    u8g2_DrawFrame(&u8g2, 2 + x_offset, 0, 26, 12);
    u8g2_DrawBox(&u8g2, 28 + x_offset, 3, 2, 6);
    uint8_t battery_fill = (uint8_t)(percentage / 100.0f * 22.0f);
    u8g2_DrawBox(&u8g2, 4 + x_offset, 2, battery_fill, 8);
    if (isCharging && battery_fill > 0) {
        u8g2_SetDrawColor(&u8g2, 0);
        for (int x = 0; x < battery_fill; x++) {
            int y_offset = (int)(sinf((float)x * 0.5f + (float)HAL_GetTick() * 0.008f) * 2.0f);
            u8g2_DrawVLine(&u8g2, 4 + x_offset + x, 4 + y_offset, 4);
        }
        u8g2_SetDrawColor(&u8g2, 1);
    }
    sprintf(buf, "%d%%", (int)percentage);
    u8g2_DrawStr(&u8g2, 34 + x_offset, 10, buf);
    int volts = (int)voltage;
    int fraction = (int)((voltage - volts) * 100);
    sprintf(buf, "%d.%02dV", volts, fraction);
    u8g2_DrawStr(&u8g2, 88, 10, buf);
}

// --- drawDischargeAnimation (无修改) ---
void drawDischargeAnimation(void) {
    int x[] = {64, 50, 78, 64, 55, 75, 64};
    int y[] = {15, 25, 25, 35, 35, 45, 55};
    int points = sizeof(x) / sizeof(int);
    for (int i = 0; i < points - 1; i++) {
        int jitter_x = rand() % 3 - 1;
        int jitter_y = rand() % 3 - 1;
        u8g2_DrawLine(&u8g2, x[i] + jitter_x, y[i] + jitter_y, x[i+1] + jitter_x, y[i+1] + jitter_y);
    }
}

// --- HAL, DS18B20, U8G2 底层驱动 --- (无修改)
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, OLED_CS_Pin|OLED_DC_Pin|OLED_RES_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = OLED_RES_Pin|OLED_DC_Pin|OLED_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void DS18B20_Manage_Temperature(void) {
    switch (temperature_data.state) {
        case TEMP_STATE_IDLE:
            if (DS18B20_Start()) {
                DS18B20_Write(0xCC); DS18B20_Write(0x44);
                temperature_data.state = TEMP_STATE_START_CONVERSION;
                temperature_data.last_action_time = HAL_GetTick();
            }
            break;
        case TEMP_STATE_START_CONVERSION:
            if (HAL_GetTick() - temperature_data.last_action_time >= 750) {
                temperature_data.state = TEMP_STATE_READ_DATA;
            }
            break;
        case TEMP_STATE_READ_DATA:
            if (DS18B20_Start()) {
                uint8_t temp_LSB, temp_MSB; int16_t temp_raw;
                DS18B20_Write(0xCC); DS18B20_Write(0xBE);
                temp_LSB = DS18B20_Read(); temp_MSB = DS18B20_Read();
                temp_raw = (temp_MSB << 8) | temp_LSB; // 修正: 应该是 LSB
                float new_temp = (float)temp_raw / 16.0f;
                if (new_temp > -55.0f && new_temp < 125.0f) {
                    temperature_data.current_temp = new_temp;
                }
            }
            temperature_data.state = TEMP_STATE_IDLE;
            break;
    }
}
void DWT_Init(void) { SCB_DEMCR |= 0x01000000; DWT_CONTROL |= 1; DWT_CYCCNT = 0; }
void DWT_Delay_us(uint32_t us) { uint32_t start_tick = DWT_CYCCNT; uint32_t delay_ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000); while ((DWT_CYCCNT - start_tick) < delay_ticks); }
static void Set_Pin_Output(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; GPIO_InitStruct.Pin = DS18B20_PIN; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct); }
static void Set_Pin_Input(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; GPIO_InitStruct.Pin = DS18B20_PIN; GPIO_InitStruct.Mode = GPIO_MODE_INPUT; GPIO_InitStruct.Pull = GPIO_PULLUP; HAL_GPIO_Init(DS18B20_PORT, &GPIO_InitStruct); }
uint8_t DS18B20_Start(void) { uint8_t Response = 0; Set_Pin_Output(); HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); DWT_Delay_us(480); Set_Pin_Input(); DWT_Delay_us(80); if (!(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN))) Response = 1; DWT_Delay_us(400); return Response; }
void DS18B20_Write(uint8_t data) { Set_Pin_Output(); for (int i = 0; i < 8; i++) { if ((data & (1 << i)) != 0) { Set_Pin_Output(); HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); DWT_Delay_us(1); Set_Pin_Input(); DWT_Delay_us(60); } else { Set_Pin_Output(); HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); DWT_Delay_us(60); Set_Pin_Input(); } } }
uint8_t DS18B20_Read(void) { uint8_t value = 0; Set_Pin_Input(); for (int i = 0; i < 8; i++) { Set_Pin_Output(); HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET); DWT_Delay_us(2); Set_Pin_Input(); if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)) { value |= 1 << i; } DWT_Delay_us(60); } return value; }

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { while(1); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { while(1); }
}

void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { while(1); }
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { while(1); }
}
void MX_SPI1_Init(void) { hspi1.Instance = SPI1; hspi1.Init.Mode = SPI_MODE_MASTER; hspi1.Init.Direction = SPI_DIRECTION_2LINES; hspi1.Init.DataSize = SPI_DATASIZE_8BIT; hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; hspi1.Init.CLKPhase = SPI_PHASE_1EDGE; hspi1.Init.NSS = SPI_NSS_SOFT; hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; hspi1.Init.TIMode = SPI_TIMODE_DISABLE; hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; hspi1.Init.CRCPolynomial = 10; HAL_SPI_Init(&hspi1); }
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) { GPIO_InitTypeDef GPIO_InitStruct = {0}; if(adcHandle->Instance==ADC1) { __HAL_RCC_ADC1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); GPIO_InitStruct.Pin = GPIO_PIN_0; GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); } }
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) { GPIO_InitTypeDef GPIO_InitStruct = {0}; if(spiHandle->Instance==SPI1) { __HAL_RCC_SPI1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7; GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); } }
void SysTick_Handler(void) { HAL_IncTick(); }
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) { switch (msg) { case U8X8_MSG_GPIO_AND_DELAY_INIT: break; case U8X8_MSG_DELAY_MILLI: HAL_Delay(arg_int); break; case U8X8_MSG_GPIO_CS: HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, arg_int); break; case U8X8_MSG_GPIO_DC: HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int); break; case U8X8_MSG_GPIO_RESET: HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int); break; default: u8x8_SetGPIOResult(u8x8, 1); break; } return 1; }

// --- 【【【 START: 编译修复 】】】 ---
// 修复了 u8g2_gpio_SetCS -> u8x8_gpio_SetCS 的拼写错误
uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            HAL_SPI_Transmit(&hspi1, (uint8_t *)arg_ptr, arg_int, 1000);
            break;
        case U8X8_MSG_BYTE_INIT:
            break;
        case U8X8_MSG_BYTE_SET_DC:
            u8x8_gpio_SetDC(u8x8, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER: 
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level); // <-- 已修复
            HAL_Delay(1U);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            HAL_Delay(1U);
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level); // <-- 已修复
            break;
        default:
            return 0;
    }
    return 1;
}
// --- 【【【 END: 编译修复 】】】 ---
