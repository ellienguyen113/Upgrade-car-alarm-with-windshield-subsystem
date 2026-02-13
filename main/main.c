
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

static const char *delay_show(delay_mode_t d){
    switch (d){
        case delay_short:
        return "SHORT DELAY 1S";
        case delay_medium:
        return "MEDIUM DELAY 3S";
        case delay_long:
        return "LONG DELAY 5S";
        default: return "";
    }
};
static void lcd_intermittent (hd44780_t *lcd, delay_mode_t delay){
    hd44780_clear(lcd);
    hd44780_gotoxy(lcd, 0,0);
    hd44780_puts(lcd, "INTERMITTENT");

    hd44780_gotoxy(lcd, 0,1);
    hd44780_puts(lcd, "DELAY: ");
    hd44780_puts(lcd, delay_show(delay));
}

void app_main(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config, &adc_handle);  // Populate unit handle
   
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Channel config
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_WIPER, &config);
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_DELAY, &config);

    init_gpio(&inputs, &outputs);
    inputs_state_t input_state = {0};
    lcd_init_once();
    servo_pwm_init();

    bool ignition_enabled= false;  //Indicates ignition readiness
    bool engine_running = false;   //Indicates engine running
    bool last_ignit = false;
    bool welcome_not_shown = true;  //Ensures welcome message prints once
    bool wiper_active = false;
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
            if (input_state.dseat&& input_state.pseat && input_state.dbelt && input_state.pbelt){
                ignition_enabled = true;
                gpio_set_level(outputs.ignition_led,1);
            }
            else {
                ignition_enabled = false;
                gpio_set_level(outputs.ignition_led,0);
            }
        
            //Ignition pressed
            if (input_state.ignition_button && !last_ignit){
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
                    if (!input_state.dseat){
                        printf("Driver seat not occupied\n");
                    }
                    
                    if (!input_state.pseat){
                        printf("Passenger seat not occupied\n");
                    }
                    if (!input_state.dbelt){
                        printf("Driver's seatbelt not fastened\n");
                    }
                    if (!input_state.pbelt){
                        printf("Passenger's seatbelt not fastened\n");
                    }
                }
            }
    
            //STAGE 2: ENGINE RUNNING
            else {   
                //STOP ENGINE
                if (input_state.ignition_button && !last_ignit){
                    engine_running = false;
                    if (wiper_active){
                        servo_set_duty(SERVO_DUTY_HIGH);
                        vTaskDelay(300 / portTICK_PERIOD_MS);
                        servo_set_duty(SERVO_DUTY_OFF);
                    }
                    servo_set_duty(SERVO_DUTY_OFF);
                    gpio_set_level(outputs.engine_led,0);
                }            
                //WINDSHIELD 
                int wiper_bits = 0, delay_bits = 0;
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_WIPER, &wiper_bits));
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_DELAY, &delay_bits));

                wiper_mode_t mode = decode_wiper_mode((uint16_t)wiper_bits);
                delay_mode_t dmode = decode_delay_mode((uint16_t)delay_bits);

                // If mode changed, reset cycle state so it reacts immediately
                if (mode != last_mode) {
                    int_reached_max = false;
                    pause_until = 0;
                    dir = +1;
                    last_mode = mode;
                }

                // OFF: park at 0 degree
                if (mode == wiper_off) {
                    duty = SERVO_DUTY_OFF;
                    dir = +1;
                    int_reached_max = false;
                    pause_until = 0;
                    servo_set_duty((uint32_t)duty);
                    vTaskDelay(50/portTICK_PERIOD_MS);
                    continue;
                }

                // Intermittent pause handling (non-blocking, so switching modes still works)
                TickType_t now = xTaskGetTickCount();
                if (mode == wiper_intermittent && pause_until != 0 && now < pause_until) {
                    duty = (int)SERVO_DUTY_OFF;
                    servo_set_duty((uint32_t)duty);
                    vTaskDelay(50/portTICK_PERIOD_MS);  // check knob frequently during pause
                    continue;
                } 
                else if (mode == wiper_intermittent) {
                    pause_until = 0; // pause finished 
                }
                // Choose bounds and speed
                int minD = SERVO_DUTY_OFF;
                int maxD;
                int step;
                int step_ms;

                if (mode == wiper_low) {
                    maxD = SERVO_DUTY_LOW;   // 0 <-> 60
                    step = 10;
                    step_ms = 25;
                } 
                else if (mode == wiper_high) {
                    maxD = SERVO_DUTY_HIGH;  // 0 <-> 90
                    step = 15;
                    step_ms = 20;
                } 
                else { // intermittent sweep uses 0 <-> 60
                    maxD = SERVO_DUTY_LOW;
                    step = 10;
                    step_ms = 20;
                }

                // One tick of back-and-forth motion
                duty += dir * step;

                if (duty >= maxD) {
                    duty = maxD;
                    dir = -1;
                    if (mode == wiper_intermittent){
                        int_reached_max = true;
                    }
                } 
                else if (duty <= minD) {
                    duty = minD;
                    dir = +1;

                    // If intermittent: after completing a full 0->max->0 cycle, start pause
                    if (mode == wiper_intermittent && int_reached_max) {
                        int pause_ms;
                    if (dmode == delay_short) {
                        pause_ms = 1000;
                    } 
                    else if (dmode == delay_medium) {
                        pause_ms = 3000;
                    }
                    else {
                        pause_ms = 5000;
                    }

                    pause_until = xTaskGetTickCount() + pdMS_TO_TICKS(pause_ms);
                    int_reached_max = false;  // reset for next wipe
                }
            }

            servo_set_duty((uint32_t)duty);
            vTaskDelay(pdMS_TO_TICKS(step_ms));
            last_ignit = input_state.ignition_button;
            vTaskDelay(25/portTICK_PERIOD_MS);
            }
        }
    }
}
    