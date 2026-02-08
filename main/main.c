#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#define POT_CHANNEL ADC_CHANNEL_7
#define MODE_CHANNEL ADC_CHANNEL_2

#define ADC_ATTEN ADC_ATTEN_DB_12
#define BITWIDTH ADC_BITWIDTH_12
#define NUM_SAMPLES 1000

#define D_SEAT GPIO_NUM_4
#define P_SEAT GPIO_NUM_5
#define D_SEATBELT GPIO_NUM_6
#define P_SEATBELT GPIO_NUM_7

#define IGNITION_LED GPIO_NUM_9
#define ENGINE_LED GPIO_NUM_10

#define BUZZER GPIO_NUM_11
#define IGNITION GPIO_NUM_12

#define SHORT_DELAY GPIO_NUM_15
#define MED_DELAY GPIO_NUM_16
#define LONG_DELAY GPIO_NUM_17

#define POT_OFF  1023
#define POT_LOW  2046
#define POT_HI 3069
#define POT_INT 4095

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          5
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits

//Set the PWM signal frequency required by servo motor
#define LEDC_FREQUENCY          50 // Frequency in Hertz. 

//Calculate the values for the minimum (0.75ms) and maximum (2.25) servo pulse widths
#define LEDC_DUTY_MIN (230)         // Set duty to 2.8%
#define LEDC_DUTY_MAX (980)        // Set duty to 11.96%.
#define LEDC_DUTY_LOW (480)   // 60 degree

static void example_ledc_init(void);

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
ledc_channel_config(&ledc_channel);
}

void app_main(void)
{
    int pot_bits;
    int mode_bits;
  
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };
    //GPIO8, 3
    adc_oneshot_config_channel
    (adc1_handle, POT_CHANNEL, &config);

    adc_oneshot_config_channel
    (adc1_handle, MODE_CHANNEL, &config);

    gpio_reset_pin(D_SEAT);
    gpio_set_direction(D_SEAT, GPIO_MODE_INPUT);
    gpio_pullup_en(D_SEAT);

    gpio_reset_pin(P_SEAT);
    gpio_set_direction(P_SEAT, GPIO_MODE_INPUT);
    gpio_pullup_en(P_SEAT);

    gpio_reset_pin(D_SEATBELT);
    gpio_set_direction(D_SEATBELT, GPIO_MODE_INPUT);
    gpio_pullup_en(D_SEATBELT);

    gpio_reset_pin(P_SEATBELT);
    gpio_set_direction(P_SEATBELT, GPIO_MODE_INPUT);
    gpio_pullup_en(P_SEATBELT);

    gpio_reset_pin(IGNITION);
    gpio_set_direction(IGNITION, GPIO_MODE_INPUT);
    gpio_pullup_en(IGNITION);

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER, 0);

    gpio_reset_pin (IGNITION_LED);
    gpio_set_direction(IGNITION_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(IGNITION_LED,0);

    gpio_reset_pin(ENGINE_LED);
    gpio_set_direction(ENGINE_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ENGINE_LED,0);
    
    gpio_reset_pin(LEFT_LAMP);
    gpio_set_direction(LEFT_LAMP,GPIO_MODE_OUTPUT);
    gpio_set_level(LEFT_LAMP,0);

    gpio_reset_pin(RIGHT_LAMP);
    gpio_set_direction(RIGHT_LAMP,GPIO_MODE_OUTPUT);
    gpio_set_level(RIGHT_LAMP,0);

    bool d_seat, p_seat; // Seat occupancy states
    bool d_belt, p_belt; // Seatbelt states

    bool ignit = false;                     //Ignition button state
    bool ignition_enabled= false;  //Indicates ignition readiness
    bool engine_running = false;   //Indicates engine running

    bool mode = false;
    bool s_delay, m_delay, l_delay;
    bool last_ignit = false;

    bool welcome_not_shown = true;  //Ensures welcome message prints once

    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 2.8% (0 degrees)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    while(1){

        //Read all inputs (active-low: pressed = 0)
        d_seat = (gpio_get_level(D_SEAT) == 0);
        p_seat = (gpio_get_level(P_SEAT) == 0);
        d_belt = (gpio_get_level(D_SEATBELT) == 0);
        p_belt = (gpio_get_level(P_SEATBELT) == 0);
        ignit = (gpio_get_level(IGNITION) == 0);

        adc_oneshot_read
        (adc1_handle, POT_CHANNEL, &pot_bits);
        
        adc_oneshot_read
        (adc1_handle, MODE_CHANNEL,&mode_bits);

        // STAGE 1: ENGINE NOT RUNNING
        if (!engine_running){
            //Welcome message
            if (d_seat && welcome_not_shown) {
                printf("Welcome to enhanced alarm system model 218-W26\n");
                welcome_not_shown = false;
            }

            // Ignition enabled
            if (d_seat && p_seat && d_belt && p_belt){
                ignition_enabled = true;
                gpio_set_level(IGNITION_LED,1);
            }
            else {
                ignition_enabled = false;
                gpio_set_level(IGNITION_LED,0);
            }

            //Ignition pressed
            if (ignit && !last_ignit){
                // Case 1: All safety conditions met
                if (ignition_enabled){
                    engine_running = true;
                    gpio_set_level(ENGINE_LED,1);
                    gpio_set_level(IGNITION_LED,0);
                    printf("Engine started\n");
                }
                // Case 2: Safety conditions not met
                else {
                    gpio_set_level(BUZZER, 1);
                    printf("Ignition inhibited\n");

                if (!d_seat){
                    printf("Driver seat not occupied\n");
                }
                    
                if (!p_seat){
                    printf("Passenger seat not occupied\n");
                }
                if (!d_belt){
                    printf("Driver's seatbelt not fastened\n");
                }
                if (!p_belt){
                    printf("Passenger's seatbelt not fastened\n");
                }
                
                //delay_ms(500);
                //gpio_set_level(BUZZER,0);
            }
        }
    }
    //STAGE 2: ENGINE RUNNING
    else{   
            //STOP ENGINE
            if (ignit && !last_ignit){
                engine_running = false;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                gpio_set_level(ENGINE_LED, 0);
                printf("car stopped\n");

            }

            //WINDSHIELD SUBSYSTEM
            //OFF mode
            if (pot_bits > 0 && pot_bits < 1023) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(700 /portTICK_PERIOD_MS);  
            }
            //LOW mode
            else if (pot_bits >1023 && pot_bits < 2046) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_LOW);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(700 /portTICK_PERIOD_MS);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }

            //HIGH mode 
            else if (pot_bits >1023 && pot_bits < 2046) {
                
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MAX);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                vTaskDelay(700 /portTICK_PERIOD_MS);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            else {
                if (){
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_MIN);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
                //INTERMITENT MODE
                if (mode_bits >= 0 && mode_bits < 1365){
                    mode = true;
                    vTaskDelay(1000/portTICK_PERIOD_MS);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_LOW);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
                else if (mode_bits >= 1365 && mode_bits < 2730){
                    mode = true;
                    vTaskDelay(3000/portTICK_PERIOD_MS);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_LOW);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
                else {
                    mode = true;
                    vTaskDelay(5000/portTICK_PERIOD_MS);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_LOW);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    }
                }
            }
            last_ignit=ignit;
            vTaskDelay(25/portTICK_PERIOD_MS);
        }
    }

