#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include <freertos/task.h>
#include <sys/time.h>
#include <hd44780.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>

typedef struct {
    adc_channel_t wiper_channel;
    adc_channel_t delay_channel;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    uint32_t num_samples;
}
adc_config_t;
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

typedef struct{
    ledc_timer_t timer;
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    gpio_num_t gpio;
    ledc_timer_bit_t resolution;
    uint32_t freq_hz;

    uint32_t duty_min;
    uint32_t duty_max;
    uint32_t duty_mid;
}
servo_pwm_t;

typedef struct {
    bool dseat;
    bool pseat;
    bool dbelt;
    bool pbelt;
    bool ignition_button;
}
inputs_state_t;

//Initialize
const adc_config_t adc_cfg = {
    .wiper_channel = ADC_CHANNEL_1,
    .delay_channel = ADC_CHANNEL_2,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
    .num_samples = 1000
};

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
const servo_pwm_t steering_servo = {
    .timer = LEDC_TIMER_0,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .gpio = GPIO_NUM_5,
    .resolution = LEDC_TIMER_13_BIT,
    .freq_hz     = 50,
}

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
typedef enum {
    wiper_off,
    wiper_low,
    wiper_high,
    wiper_intermitten
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
wiper_mode_t decode_wiper_mode(uint16_t mode_adc){
    if (mode_adc < THRESH_OFF_LOW){
        return wiper_off;
    }
    else if (mode_adc < THRESH_LOW_HIGH) {
        return wiper_low;
    }
    else if (mode_adc < THRESH_HIGH_INT) {
        return wiper_high;
    }
     else {
        return wiper_intermitten;
    }
}
//Read delay from pot2 when mode = intermittent
#define THRESH_SHORT 1365
#define THRESH_MED 2730
delay_mode_t decode_delay_mode(uint16_t delay_adc){
    if (delay_adc < SHORT_DELAY){
        return delay_short;
    }
    else if (delay_adc < THRESH_MED){
        return delay_medium;
    }
    else {
        return delay_long;
    }
}

static void servo_pwm_init(void);
static void servo_pwm_init(const servo_pwm_t *cfg)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = cfg ->speed_mode,
        .duty_resolution  = cfg->resolution,
        .timer_num        = cfg->timer,
        .freq_hz          = cfg->freq_hz, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t channel_cfg = {
        .speed_mode     = cfg->speed_mode,
        .channel        = cfg->channel,
        .timer_sel      = cfg->timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = cfg->gpio,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}
#define SERVO_DUTY_OFF   ((uint32_t)(8191 * 0.028f))
#define SERVO_DUTY_LOW   ((uint32_t)(8191 * 0.075f))
#define SERVO_DUTY_HIGH  ((uint32_t)(8191 * 0.1196f))

static void servo_set_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
}
static adc_oneshot_unit_handle_t adc1_handle;
static void init_adc(const adc_config_t *cfg)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = cfg-> ADC_ATTEN,
        .bitwidth = cfg ->BITWIDTH
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, cfg->pot_channel, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, cfg->mode_channel, &chan_config));
}

static void read_inputs (inputs_state_t *state, const input_pins_t *pins){
    state -> dseat = (gpio_get_level(pins->driver_seat)==0);
    state -> pseat = (gpio_get_level(pins->passenger_seat)==0);
    state -> dbelt = (gpio_get_level(pins->driver_seatbelt)==0);
    state -> pbelt = (gpio_get_level(pins->passenger_seatbelt)==0);
    state -> ignition_button = (gpio_get_level(pins->ignition_button)==0);
}
void app_main(void)
{
    int pot_bits;
    int mode_bits;

    init_gpio(&inputs, &outputs);
    init_adc(&adc_cfg);
    servo_pwm_init(&steering_servo);
    inputs_state_t input_state = {0};
    lcd_init_once(); 

    bool ignition_enabled= false;  //Indicates ignition readiness
    bool engine_running = false;   //Indicates engine running
    bool last_ignit = false;
    bool welcome_not_shown = true;  //Ensures welcome message prints once
    while(1){
        read_inputs(&input_state, &inputs);
        // STAGE 1: ENGINE NOT RUNNING
        if (!engine_running){
            //Welcome message
            if (input_state.dseat && welcome_not_shown) {
                printf("Welcome to enhanced alarm system model 218-W26\n");
                welcome_not_shown = false;
            }
            // Ignition enabled
            if (inputs.dseat&& pseat && inputs.dbelt && inputs.pbelt){
                ignition_enabled = true;
                gpio_set_level(outputs.ignition_led,1);
            }
            else {
                ignition_enabled = false;
                gpio_set_level(outputs.ignition_led,0);
            }
            //Ignition pressed
            if (inputs.ignition_button && !last_ignit){
                // Case 1: All safety conditions met
                if (ignition_enabled){
                    engine_running = true;
                    gpio_set_level(outputs.engine_led,1);
                    gpio_set_level(outputs.ignition_led,0);
                    printf("Engine started\n");
                }
                // Case 2: Safety conditions not met
                else {
                    gpio_set_level(outputs.buzzer, 1);
                    printf("Ignition inhibited\n");

                if (!inputs.dseat){
                    printf("Driver seat not occupied\n");
                }
                    
                if (!inputs.pseat){
                    printf("Passenger seat not occupied\n");
                }
                if (!inputs.dbelt){
                    printf("Driver's seatbelt not fastened\n");
                }
                if (!inputs.pbelt){
                    printf("Passenger's seatbelt not fastened\n");
                }
            }
            uint16_t mode_adc;
            uint16_t delay_adc;
            adc_oneshot_read
            (adc1_handle, adc_cfg.wiper_channel, &mode_adc);
            wiper_mode_t wiper_mode = decode_wiper_mode(mode_adc);
            adc_oneshot_read
            (adc1_handle, adc_cfg.delay_channel, &delay_adc);
            delay_mode_t delay_mode = decode_delay_mode(delay_adc);
            
            //STAGE 2: ENGINE RUNNING
            else{   
                //STOP ENGINE
                if (inputs.ignition && !last_ignit){
                    engine_running = false;
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    gpio_set_level(ENGINE_LED, 0);
                    printf("car stopped\n");
                }
                //WINDSHIELD 
                switch(wiper_mode){
                    case wiper_off:
                    servo_set_duty(SERVO_DUTY_OFF);
                    break;
                    case wiper_low:
                    servo_set_duty(SERVO_DUTY_LOW);
                    break;
                    case wiper_high:
                    servo_set_duty(SERVO_DUTY_HIGH);
                    break;
                    case wiper_intermittent:
                    servo_set_duty(SERVO_DUTY_LOW);
                    handle_delay(delay_mode);
                    lcd_show_intermittent(&lcd, delay);
                    break;
                }
            }
                last_ignit = input_state.ignition_button;
                vTaskDelay(25/portTICK_PERIOD_MS);
            }
        }
    }
}
// LCD DISPLAY

static const uint8_t char_data[] =
{
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00
};

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

static const char *delay_show(delay_mode_t d){
    switch (d){
        case delay_short:
        return "SHORT DELAY 1S";
        case delay_medium:
        return "MEDIUM DELAY 3S";
        case delay_long:
        return "LONG DELAY 5S";
    }
}
static void lcd_intermisttent (hd44780_t *lcd, delay_mode_t delay){
    hd44780_clear(lcd);
    hd44780_gotoxy(lcd, 0,0);
    hd44780_puts(lcd, "INTERMITTENT")

    hd44780_gotoxy(lcd, 0,1);
    hd44780_puts(lcd, "DELAY: ")
    hd44780_puts(lcd, "delay_show(delay)")
}
