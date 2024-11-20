#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Estructura para almacenar el estado del encoder
typedef struct {
    int pin_out;               // Pin de salida con interrupción
    volatile int32_t count;    // Contador de pulsos
    volatile int64_t last_pulse_time;  // Marca de tiempo del último pulso
} encoder_t;


// Definir los pines de salida de los encoders
#define ENCODER1_OUT 16
#define ENCODER2_OUT 17


// Nueva enumeración para la dirección
typedef enum {
    DIRECTION_STOPPED = 0,     // Sin movimiento
    DIRECTION_FORWARD = 1,     // Movimiento hacia adelante
    DIRECTION_BACKWARD = -1    // Movimiento hacia atrás
} direction_t;
//Estructura para enviar a la task
typedef struct {
    encoder_t *encoder1;
    encoder_t *encoder2;
} encoders_params_t;

// Prototipo adicional
direction_t get_movement_direction();

// Prototipos de funciones
void encoder_init(encoder_t *encoder, int pin_out);
int32_t encoder_get_count(encoder_t *encoder);
void encoder_reset_count(encoder_t *encoder);
void encoder_isr_handler(void *arg);
void tarea_verificar_variable(void *param);
float encoder_get_reward(encoder_t *encoder);
float get_reward(encoder_t *encoder1, encoder_t *encoder2);


#endif // ENCODER_H