#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "freertos/task.h"

// Definição da GPIO utilizada no controle das placas
#define PDIG1 GPIO_NUM_18
#define PWM1 GPIO_NUM_4
#define PDIG2 GPIO_NUM_19
#define PWM2 GPIO_NUM_21

// Definição dos parâmetros do PWM
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do PWM a ser gerado (VALOR DO TOP)
#define TOP_PWM 8191 // Definindo o TOP do PWM (2^13 - 1)
#define LEDC_FREQUENCY (4000) // Freq do PWM em Hz
#define LEDC_CHANNEL_PWM1 LEDC_CHANNEL_0
#define LEDC_CHANNEL_PWM2 LEDC_CHANNEL_1


void PWM_init(){
    // Configurando o timer do LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // PWM1
    ledc_channel_config_t ledc_channel_pwm1 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_PWM1,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM1,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_pwm1));

    // PWM2
    ledc_channel_config_t ledc_channel_pwm2 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_PWM2,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM2,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_pwm2));
}

void PWM_change(int pwm_channel, float duty_percent){
    if(duty_percent<0.0f){
        duty_percent = 0.0f;
    };
    if(duty_percent>1.0f){
        duty_percent = 1.0f;
    };

    uint32_t duty_int = (uint32_t)(duty_percent*TOP_PWM);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, pwm_channel, duty_int));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, pwm_channel));
}


void app_main(void){
    // Inicializando os PDIG em nível baixo
    ESP_ERROR_CHECK(gpio_set_direction(PDIG1, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PDIG2, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PDIG1, 0));
    ESP_ERROR_CHECK(gpio_set_level(PDIG2, 0));

    PWM_init();

    bool rise_pwm1 = false;
    bool rise_pwm2 = false;

    float duty_percent1 = 0.0;
    float duty_percent2 = 0.0;

    while(1){
        // PWM 1
        if(rise_pwm1){
            duty_percent1+=0.01;
            if(duty_percent1 >= 1.0f){
                rise_pwm1=false;
            }
        }
        else{
            duty_percent1-=0.01;
            if(duty_percent1 <= 0.0f){
                rise_pwm1=true;
            }
        }
        PWM_change(LEDC_CHANNEL_PWM1, duty_percent1);

        // PWM 2
        if(rise_pwm2){
            duty_percent2+=0.01;
            if(duty_percent2 >= 1.0f){
                rise_pwm2=false;
            }
        }
        else{
            duty_percent2-=0.01;
            if(duty_percent2 <= 0.0f){
                rise_pwm2=true;
            }
        }
        PWM_change(LEDC_CHANNEL_PWM2, duty_percent2);

        // Indicando a rotação com LEDs
        ESP_ERROR_CHECK(gpio_set_level(PDIG1, rise_pwm1));
        ESP_ERROR_CHECK(gpio_set_level(PDIG2, rise_pwm2));

        vTaskDelay(30/portTICK_PERIOD_MS);
    }
}