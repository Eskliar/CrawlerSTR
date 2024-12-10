#pragma once
#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"    

// Definiciones de pines
#define SHOULDER_PIN    GPIO_NUM_13
#define ELBOW_PIN       GPIO_NUM_14

// Definiciones de pulsos para el hombro y el codo
#define SHOULDER_MAX_PULSE 800
#define SHOULDER_MID_PULSE 650
#define SHOULDER_MIN_PULSE 400

#define ELBOW_MAX_PULSE 2100
#define ELBOW_MID_PULSE 1800
#define ELBOW_MIN_PULSE 1400

// Configuración de LEDC para PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Resolución de ciclo de trabajo de 13 bits
#define LEDC_SHOULDER_CHANNEL   LEDC_CHANNEL_0
#define LEDC_ELBOW_CHANNEL      LEDC_CHANNEL_1
#define SERVO_FREQUENCY                  50

// Declaraciones de funciones (prototipos)
void init_servo(void);
void set_pos(int shoulder, int elbow);
void set_servo_pulse(int channel, int pulse);
void process_move_elbow(int angle);
void process_move_shoulder(int angle);


#endif // SERVO_H
