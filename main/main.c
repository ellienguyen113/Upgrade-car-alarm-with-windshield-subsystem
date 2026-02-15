#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>

typedef struct {
    gpio_num_t driver_seat;
    gpio_num_t passenger_seat;
    gpio_num_t driver_seatbelt;
    gpio_num_t passenger_seatbelt;
    gpio_num_t ignition_button;
}
input_pins_t;
typedef struct{
    gpio_num_t ignition_led;
    gpio_num_t engine_led;
    gpio_num_t buzzer;
}
output_pins_t;
typedef struct {
    bool dseat;
    bool pseat;
    bool dbelt;
    bool pbelt;
    bool ignition_button;
}
inputs_state_t;

//Initialize

const input_pins_t inputs = {
    .driver_seat = GPIO_NUM_4,
    .passenger_seat = GPIO_NUM_5,
    .driver_seatbelt = GPIO_NUM_6,
    .passenger_seatbelt = GPIO_NUM_7,
    .ignition_button = GPIO_NUM_12,
};
const output_pins_t outputs = {
    .ignition_led = GPIO_NUM_9,
    .engine_led = GPIO_NUM_10,
    .buzzer = GPIO_NUM_11
};

static void init_gpio(const input_pins_t *in, const output_pins_t *out){
    gpio_config_t io_conf = {0};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
    io_conf.pin_bit_mask =
    (1ULL << in->driver_seat)|
    (1ULL << in->passenger_seat)|
    (1ULL << in->driver_seatbelt)|
    (1ULL << in->passenger_seatbelt)|
    (1ULL << in->ignition_button);
    gpio_config(&io_conf);
 
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
    (1ULL << out->ignition_led) |
    (1ULL << out->engine_led) |
    (1ULL << out->buzzer);
    gpio_config(&io_conf);
}

static void read_inputs (inputs_state_t *state, const input_pins_t *pins){
    state -> dseat = (gpio_get_level(pins->driver_seat)==0);
    state -> pseat = (gpio_get_level(pins->passenger_seat)==0);
    state -> dbelt = (gpio_get_level(pins->driver_seatbelt)==0);
    state -> pbelt = (gpio_get_level(pins->passenger_seatbelt)==0);
    state -> ignition_button = (gpio_get_level(pins->ignition_button)==0);
}
typedef enum {
    wiper_off,
    wiper_low,
    wiper_high,
    wiper_intermittent
}
wiper_mode_t;
typedef enum{
    delay_short,
    delay_medium,
    delay_long
}
delay_mode_t;

#define THRESH_OFF_LOW     1023
#define THRESH_LOW_HIGH    2046
#define THRESH_HIGH_INT    3069
wiper_mode_t decode_wiper_mode(uint16_t wiper_mode){
    if (wiper_mode < THRESH_OFF_LOW){
        return wiper_off;
    }
    else if (wiper_mode < THRESH_LOW_HIGH) {
        return wiper_low;
    }
    else if (wiper_mode < THRESH_HIGH_INT) {
        return wiper_high;
    }
     else {
        return wiper_intermittent;
    }
}
//Read delay from pot2 when mode = intermittent
#define THRESH_SHORT 1365
#define THRESH_MED 2730
delay_mode_t decode_delay_mode(uint16_t delay_adc){
    if (delay_adc < THRESH_SHORT){
        return delay_short;
    }
    else if (delay_adc < THRESH_MED){
        return delay_medium;
    }
    else {
        return delay_long;
    }
}
#define ADC_CHANNEL_WIPER     ADC_CHANNEL_2
#define ADC_CHANNEL_DELAY    ADC_CHANNEL_3
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define BITWIDTH        ADC_BITWIDTH_12

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (15)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. 
#define SERVO_DUTY_OFF           ((8191*0.028)) // Set duty to 3.75%.
#define SERVO_DUTY_LOW           ((8191*0.075)) // Set duty to 9.25%.
#define SERVO_DUTY_HIGH          ((8191*0.1196)) // Set duty to 14.65%.

static void servo_pwm_init(void);
static void servo_pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel_cfg = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}
    static void servo_set_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
}

// LCD DISPLAY

static hd44780_t lcd = {
    .write_cb = NULL,
    .font = HD44780_FONT_5X8,
    .lines = 2,
    .pins = {
        .rs = GPIO_NUM_38,
        .e  = GPIO_NUM_37,
        .d4 = GPIO_NUM_36,
        .d5 = GPIO_NUM_35,
        .d6 = GPIO_NUM_48,
        .d7 = GPIO_NUM_47,
        .bl = HD44780_NOT_USED
    }
};
static void lcd_init_once(void){
    hd44780_init(&lcd);
    hd44780_clear(&lcd);
}
static const char *mode_show(wiper_mode_t m){
    switch (m){
        case wiper_off:          return "OFF";
        case wiper_low:          return "LOW";
        case wiper_high:         return "HIGH";
        case wiper_intermittent: return "INTERMITTENT";
        default:                 return "";
    }
}
static const char *delay_show(delay_mode_t d){
    switch (d){
        case delay_short:   return "SHORT 1S";
        case delay_medium:  return "MEDIUM 3S";
        case delay_long:    return "LONG 5S";
        default:            return "";
    }
}
// 2x20 LCD: Line1 shows mode, Line2 shows delay only in INT
static void lcd_show_mode(hd44780_t *lcd, wiper_mode_t mode, delay_mode_t dmode)
{
    hd44780_clear(lcd);

    // Line 1
    hd44780_gotoxy(lcd, 0, 0);
    hd44780_puts(lcd, "MODE: ");
    hd44780_puts(lcd, mode_show(mode));

    // Line 2
    hd44780_gotoxy(lcd, 0, 1);
    if (mode == wiper_intermittent) {
        hd44780_puts(lcd, "DELAY: ");
        hd44780_puts(lcd, delay_show(dmode));
    } else {
        // clear line 2 (20 spaces)
        hd44780_puts(lcd, "                    ");
    }
}

void app_main(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_WIPER, &config);
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_DELAY, &config);

    init_gpio(&inputs, &outputs);
    inputs_state_t input_state = {0};
    lcd_init_once();
    servo_pwm_init();

    bool ignition_enabled = false;
    bool engine_running   = false;
    bool last_ignit       = false;
    bool welcome_not_shown = true;
    //Wiper state 
    int duty = (int)SERVO_DUTY_OFF;
    int dir  = +1;                        // +1 up, -1 down
    TickType_t pause_until = 0;           // INT pause end tick
    bool int_reached_max = false;         // for INT pause scheduling
    wiper_mode_t last_mode = wiper_off;
    //Graceful stop state
    bool parking = false;                // finish cycle then park at 0
    bool reached_max_this_cycle = false; // must hit max once before coming back
    int latched_maxD = (int)SERVO_DUTY_LOW;
    int latched_step = 10;
    int latched_step_ms = 25;
    //Engine shutdown request
    bool shutdown_requested = false;
    // LCD cache to reduce flicker 
    static wiper_mode_t last_lcd_mode = (wiper_mode_t)(-1);
    static delay_mode_t last_lcd_delay = (delay_mode_t)(-1);
    while (1) {
        read_inputs(&input_state, &inputs);
        // STAGE 1: ENGINE NOT RUNNING
        if (!engine_running) {
            if (input_state.dseat && welcome_not_shown) {
                printf("Welcome to enhanced alarm system model 218-W26\n");
                welcome_not_shown = false;
            }
            if (input_state.dseat && input_state.pseat && input_state.dbelt && input_state.pbelt) {
                ignition_enabled = true;
                gpio_set_level(outputs.ignition_led, 1);
            } else {
                ignition_enabled = false;
                gpio_set_level(outputs.ignition_led, 0);
            }
            if (input_state.ignition_button && !last_ignit) {
                if (ignition_enabled) {
                    engine_running = true;
                    gpio_set_level(outputs.engine_led, 1);
                    gpio_set_level(outputs.ignition_led, 0);
                    printf("Engine started\n");
                    // reset shutdown request when starting
                    shutdown_requested = false;
                } else {
                    gpio_set_level(outputs.buzzer, 1);
                    printf("Ignition inhibited\n");
                    if (!input_state.dseat) printf("Driver seat not occupied\n");
                    if (!input_state.pseat) printf("Passenger seat not occupied\n");
                    if (!input_state.dbelt) printf("Driver's seatbelt not fastened\n");
                    if (!input_state.pbelt) printf("Passenger's seatbelt not fastened\n");
                }
            }
            // Park servo at 0 while engine is off
            servo_set_duty((uint32_t)SERVO_DUTY_OFF);
            last_ignit = input_state.ignition_button;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        // STAGE 2: ENGINE RUNNING
        // shutdown request on rising edge of ignition button
        if (input_state.ignition_button && !last_ignit) {
            shutdown_requested = true;
        }
        int wiper_bits = 0, delay_bits = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_WIPER, &wiper_bits));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_DELAY, &delay_bits));

        wiper_mode_t mode  = decode_wiper_mode((uint16_t)wiper_bits);
        delay_mode_t dmode = decode_delay_mode((uint16_t)delay_bits);

        // Update LCD only when needed (2x20)
        if (mode != last_lcd_mode || (mode == wiper_intermittent && dmode != last_lcd_delay)) {
            lcd_show_mode(&lcd, mode, dmode);
            last_lcd_mode  = mode;
            last_lcd_delay = dmode;
        }
        bool off_requested = (mode == wiper_off) || shutdown_requested;
        // If mode changed (and we're not parking), reset cycle/pause for immediate response
        if (!parking && mode != last_mode) {
            int_reached_max = false;
            pause_until = 0;
            dir = +1;
            last_mode = mode;
        }
        // Determine normal sweep params based on current mode 
        int minD = (int)SERVO_DUTY_OFF;
        int maxD = (int)SERVO_DUTY_LOW;
        int step = 10;
        int step_ms = 25;

        if (mode == wiper_low) {
            maxD = (int)SERVO_DUTY_LOW;   // 0 <-> 60
            step = 10;
            step_ms = 25;
        } else if (mode == wiper_high) {
            maxD = (int)SERVO_DUTY_HIGH;  // 0 <-> 90
            step = 15;
            step_ms = 20;
        } else if (mode == wiper_intermittent) {
            maxD = (int)SERVO_DUTY_LOW;   // INT sweep uses 0 <-> 60
            step = 10;
            step_ms = 20;
        } else {
            // mode == wiper_off
            maxD = (int)SERVO_DUTY_LOW;
            step = 10;
            step_ms = 20;
        }
        // INT hesitation behavior
        TickType_t now = xTaskGetTickCount();
        if (mode == wiper_intermittent && pause_until != 0 && now < pause_until) {
            // remain stationary at 0 during hesitation
            duty = minD;
            servo_set_duty((uint32_t)duty);
            // If OFF/shutdown requested during hesitation: remain stationary; shutdown can occur now
            if (off_requested) {
                parking = false;
                reached_max_this_cycle = false;
                int_reached_max = false;
                pause_until = 0;
                if (shutdown_requested) {
                    engine_running = false;
                    shutdown_requested = false;
                    gpio_set_level(outputs.engine_led, 0);
                    gpio_set_level(outputs.ignition_led, 0);
                }
            }
            last_ignit = input_state.ignition_button;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        // --- Start graceful park if OFF/shutdown requested while mid-sweep ---
        if (off_requested) {
            if (duty == minD) {
                // already parked
                parking = false;
                reached_max_this_cycle = false;
                if (shutdown_requested) {
                    engine_running = false;
                    shutdown_requested = false;
                    gpio_set_level(outputs.engine_led, 0);
                    gpio_set_level(outputs.ignition_led, 0);
                }
                servo_set_duty((uint32_t)minD);
                last_ignit = input_state.ignition_button;
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            } else if (!parking) {
                // latch the current sweep params at the moment OFF/shutdown is requested
                parking = true;
                latched_maxD = maxD;
                latched_step = step;
                latched_step_ms = step_ms;
                // must reach max once to "complete the cycle"
                reached_max_this_cycle = (duty >= maxD);
            }
        }
        int use_maxD, use_step, use_stepms;
        if (parking) {
            use_maxD = latched_maxD;
            use_step = latched_step;
            use_stepms = latched_step_ms;
        } else {
            use_maxD   = maxD;
            use_step   = step;
            use_stepms = step_ms;
}   
        // During parking, if we haven't reached max yet, force direction up to max first
        if (parking && !reached_max_this_cycle) {
            dir = +1;
        }
        // One tick of motion
        duty += dir * use_step;
        if (duty >= use_maxD) {
            duty = use_maxD;
            dir = -1;
            reached_max_this_cycle = true;
            if (!parking && mode == wiper_intermittent) {
                int_reached_max = true;
            }
        } else if (duty <= minD) {
            duty = minD;
            dir = +1;
            if (parking) {
                // parked after completing cycle
                parking = false;
                reached_max_this_cycle = false;
                int_reached_max = false;
                pause_until = 0;

                if (shutdown_requested) {
                    engine_running = false;
                    shutdown_requested = false;
                    gpio_set_level(outputs.engine_led, 0);
                    gpio_set_level(outputs.ignition_led, 0);
                }
            } else {
                // schedule INT pause after completing 0->max->0
                if (mode == wiper_intermittent && int_reached_max) {
                    int pause_ms = 1000;
                    if (dmode == delay_short) pause_ms = 1000;
                    else if (dmode == delay_medium) pause_ms = 3000;
                    else pause_ms = 5000;

                    pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(pause_ms);
                    int_reached_max = false;
                }
            }
        }
        servo_set_duty((uint32_t)duty);
        last_ignit = input_state.ignition_button;
        vTaskDelay(pdMS_TO_TICKS(use_stepms));
    }
}
