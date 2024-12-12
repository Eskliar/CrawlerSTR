#include "servo.h"

// Variables globales
int current_pos[2] = {0, 0}; // [hombro, codo]

// Función para configurar el ángulo de un servo
void set_servo_pulse(int channel, int pulse) {
    // Convertir el pulso a duty cycle
    int duty = (pulse * ((1 << LEDC_DUTY_RES) - 1)) / 20000;
    // Configurar el duty cycle en el canal especificado
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

// Función para inicializar los servos
void init_servo() {
    // Configurar temporizador para LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = SERVO_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configurar canal para el servo del hombro
    ledc_channel_config_t ledc_channel_shoulder = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_SHOULDER_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SHOULDER_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_shoulder);

    // Configurar canal para el servo del codo
    ledc_channel_config_t ledc_channel_elbow = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_ELBOW_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ELBOW_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_elbow);
}



void process_move_shoulder(int angle) {
    switch (angle) {
        case 0:
            current_pos[0] = SHOULDER_MIN_PULSE;
            break;
        case 45:
            current_pos[0] = SHOULDER_MID_PULSE;
            break;
        case 90:
            current_pos[0] = SHOULDER_MAX_PULSE;
            break;
        default:
            break;
    }
    set_servo_pulse(LEDC_SHOULDER_CHANNEL, current_pos[0]);
}

void process_move_elbow(int angle) {
    switch (angle) {
        case 0:
            current_pos[1] = ELBOW_MIN_PULSE;
            break;
        case 45:
            current_pos[1] = ELBOW_MID_PULSE;
            break;
        case 90:
            current_pos[1] = ELBOW_MAX_PULSE;
            break;
        default:
            break;
    }
    set_servo_pulse(LEDC_ELBOW_CHANNEL, current_pos[1]);
}

void set_pos(int shoulder, int elbow){
    current_pos[0] = shoulder;
    current_pos[1] = elbow;
}

