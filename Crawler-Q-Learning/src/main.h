#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "servo.h"
#include "encoder.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "cJSON.h"

// Definiciones de constantes
#define ROW_NUM 9 // Número de estados (3 posiciones para servo1 y 3 para servo2)
#define COL_NUM 9 // Número de estados (3 posiciones para servo1 y 3 para servo2)
#define SERVO_NUM 2 // Número de servos
#define MAX_POSITION 90 // Máxima posición (grados) para los servos
#define MIN_POSITION 0 // Mínima posición (grados) para los servos
#define NUM_ESTADOS 9 // Número total de estados
#define ACT_NUM 4 // Número de acciones (2 acciones por cada servo: +45 y -45 grados)
#define FRONT_LEARN 0 // Estado de aprendizaje hacia adelante
#define BACK_LEARN 1 // Estado de aprendizaje hacia atrás
#define MAX_INTENTOS 6 // Número máximo de intentos
#define ACTION_SERVO1_FORWARD 0  // Mover servo 1 hacia adelante
#define ACTION_SERVO1_BACKWARD 1  // Mover servo 1 hacia atras
#define ACTION_SERVO2_FORWARD 2 // Mover servo 2 hacia adelante
#define ACTION_SERVO2_BACKWARD 3 // Mover servo 2 hacia atras

// Configuración del AP
#define AP_SSID "MiESP32_AP"
#define AP_PASSWORD "123456789"
#define MAX_STA_CONN 4
#define TAG "HTTP_GET"

// Declaraciones de variables globales
extern char *response_data;
extern size_t response_data_size;
extern int estadoCrawler;
extern int estadoAprendiendoEjecutando;
extern int learn;
extern encoder_t encoder1;
extern encoder_t encoder2;
extern SemaphoreHandle_t xMutex;

// Declaraciones de funciones
void wifi_init_softap(void);
int http_get(const char *url);
void calcular_matriz_Q(float Q[ROW_NUM][COL_NUM], int estado, int accion, float recompensa, float tasa_aprendizaje, float factor_descuento);
void calcular_matriz_R(float R[ROW_NUM][COL_NUM], int estado, int accion, float recompensa);

esp_err_t http_event_handler(esp_http_client_event_t *evt);
void http_post(const char *url, const char *post_data);
int process_get_response(const char *response);
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt);
void enviarDatosMatriz(float matriz[9][9]);
int obtenerEstadoCrawler();
int obtenerDireccionCrawler();
void enviarEstadoCrawler();

typedef struct {
    float Q[NUM_ESTADOS][NUM_ESTADOS]; // Matriz Q para estados y transiciones
    float R[NUM_ESTADOS][NUM_ESTADOS]; // Matriz R para recompensas
    float epsilon; // Tasa de exploración
    float alpha;   // Tasa de aprendizaje
    float gamma;   // Factor de descuento
} Q_Agent;

void q_agent_init(Q_Agent *agent);
int q_agent_select_action(Q_Agent *agent, int state);
void q_agent_update(Q_Agent *agent, int state, int action, int next_state);
void mover_servos(int next_state);
void encoder_signal(Q_Agent *agent, int current_state, int next_state, encoder_t *encoder1, encoder_t *encoder2); // simula la señal del encoder
void print_q_matrix(Q_Agent *agent); // Nueva función para imprimir la matriz Q
void print_r_matrix(Q_Agent *agent);
void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position); // Nueva función para el movimiento continuo de arrastre
void simu_mover_servos(int next_state, int accion);
void simu_encoder_signal(Q_Agent *agent, int current_state, int next_state);
int obtener_siguiente_estado(int current_state, int action);
bool accion_valida(int current_state, int action);
int* get_valid_actions(int servo1_pos, int servo2_pos, int* size);

#endif // MAIN_H