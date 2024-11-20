#include "encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"  // Para obtener marca de tiempo en microsegundos

// Variables globales para comparar pulsos
static volatile int64_t encoder1_last_time = 0;
static volatile int64_t encoder2_last_time = 0;
static volatile direction_t movement_direction = DIRECTION_STOPPED;


static volatile int last_encoder_triggered = 0; // 1 para encoder1, 2 para encoder2
static volatile int64_t last_trigger_time = 0; // Tiempo de la última interrupción
// ISR: Incrementa el contador cada vez que se genera un pulso
void encoder_isr_handler(void *arg) {
    encoder_t *encoder = (encoder_t *)arg;
    int64_t now = esp_timer_get_time();  // Marca de tiempo en microsegundos
    // Incrementar el contador del encoder correspondiente
    encoder->count++;
    // Determinar el orden de las interrupciones
    if (encoder->pin_out == ENCODER1_OUT) {
        if (last_encoder_triggered == 2 && (now - last_trigger_time) < 500000) { 
            // Si el último fue el encoder 2 y la diferencia de tiempo es pequeña
            movement_direction = DIRECTION_FORWARD; 
        }
        last_encoder_triggered = 1;
    } else if (encoder->pin_out == ENCODER2_OUT) {
        if (last_encoder_triggered == 1 && (now - last_trigger_time) < 500000) { 
            // Si el último fue el encoder 1 y la diferencia de tiempo es pequeña
            movement_direction = DIRECTION_BACKWARD; 
        }
        last_encoder_triggered = 2;
    }
    // Actualizar el tiempo del último pulso
    last_trigger_time = now;
    encoder->last_pulse_time = now; // Registrar el tiempo del pulso
}

void tarea_verificar_variable(void *param) {
    encoders_params_t *encoders = (encoders_params_t *)param;
    while (1) {
        int32_t count1 = encoder_get_count(encoders->encoder1);
        int32_t count2 = encoder_get_count(encoders->encoder2);
        //printf("Verificando encoders: Encoder1 = %ld, Encoder2 = %ld\n", count1, count2);
        int64_t now = esp_timer_get_time();
        int64_t diferencia = (now - last_encoder_triggered) / 1000000;
        if (diferencia >= 5) {  // 5 segundos
            movement_direction = DIRECTION_STOPPED;
            encoder_reset_count(encoders->encoder1);
            encoder_reset_count(encoders->encoder2);
        }
        // Espera 5 segundos
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


void encoder_init(encoder_t *encoder, int pin_out) {
    encoder->pin_out = pin_out;
    encoder->count = 0;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Flanco de bajada
        .mode = GPIO_MODE_INPUT,        // Configurar como entrada
        .pin_bit_mask = (1ULL << pin_out),
        .pull_up_en = GPIO_PULLUP_ENABLE, // Habilitar resistencia pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE // Deshabilitar pull-down
    };
    gpio_config(&io_conf);

    // Instalar servicio de interrupciones si no está instalado
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Agregar el controlador de interrupciones
    gpio_isr_handler_add(pin_out, encoder_isr_handler, (void *)encoder);
}

// Devuelve el valor del contador del encoder
int32_t encoder_get_count(encoder_t *encoder) {
    return encoder->count;
}

// Resetea el contador del encoder a cero
void encoder_reset_count(encoder_t *encoder) {
    encoder->count = 0;
}

direction_t get_movement_direction() {
    return movement_direction;
}

float encoder_get_reward(encoder_t *encoder){
    int32_t real_value= (encoder->count)/2;
    float reward = real_value >= 5 ? 1.0 : (float)real_value / 5.0; //Acotado a 5 como max (rendijas de una vuelta)
    return reward;
}

float get_reward(encoder_t *encoder1, encoder_t *encoder2){
    printf("Obteniendo recompensas...\n");
    float reward_encoder1=encoder_get_reward(encoder1);
    printf("Recompensa encoder1: %f \n",reward_encoder1);
    float reward_encoder2=encoder_get_reward(encoder2);
    printf("Recompensa encoder2: %f \n",reward_encoder2);
    float reward_final= (reward_encoder1 + reward_encoder2)/2;
    printf("Recompensa total: %f \n",reward_final);
    encoder_reset_count(encoder1);
    encoder_reset_count(encoder2);
    return reward_final;
}