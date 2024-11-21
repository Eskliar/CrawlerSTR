#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "servo.c"
#include "encoder.C" 
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"


//---------CORRECCIONES FUTURAS----------
/*
-establecer estados para no tener que hacer
    int next_state = (servo1_new_position / 45) + (servo2_new_position / 45) * 3;
*/


#define SERVO_NUM 2 // Dos servos
#define ROW_NUM 9 // 9 estados (3 posiciones para servo1 y 3 para servo2)
#define ACT_NUM 3 // 5 acciones por cada servo: adelante, atrás, sin movimiento, y dos más si se definen
#define MAX_POSITION 90 // máxima posición (grados)
#define MIN_POSITION 0 

//estan al cuete por ahora
#define ACTION_SERVO1_FORWARD 0  // Mover servo hacia adelante
#define ACTION_SERVO2_FORWARD 1  // Mover servo hacia atras
#define ACTION_SERVO1_BACKWARD 2 // no move


//---------------MAIN ENCODER---------------------

// Configuración del AP
#define AP_SSID "MiESP32_AP"
#define AP_PASSWORD "123456789"
#define MAX_STA_CONN 4

encoder_t encoder1, encoder2;  // Instancias de los dos encoders


// Configura el modo AP del ESP32
void wifi_init_softap() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .password = AP_PASSWORD,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();

    printf("Punto de acceso inicializado: SSID:%s\n", AP_SSID);
}

// Maneja eventos HTTP
esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ERROR) {
        //printf("Error en la solicitud HTTP\n");
    }
    return ESP_OK;
}

// Enviar una solicitud HTTP POST
void http_post(const char *url, const char *post_data) {
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        printf("POST enviado con éxito, código de respuesta: %d\n", 
               esp_http_client_get_status_code(client));
    } else {
        //printf("Error en el POST: %s\n", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

void enviarDatosMatriz(int matriz[9][9]) {
    char buffer[1024]; // Buffer para el string JSON
    int offset = 0;    // Offset para ir escribiendo en el buffer

    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "{ \"matriz\": [");

    for (int i = 0; i < 9; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "[");
        for (int j = 0; j < 9; j++) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d", matriz[i][j]);
            if (j < 8) {
                offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",");
            }
        }
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "]");
        if (i < 8) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, ",");
        }
    }

    snprintf(buffer + offset, sizeof(buffer) - offset, "]}");

    //printf("Datos JSON generados: %s\n", buffer);

    // Llamar a http_post con el JSON generado
    http_post("http://192.168.4.2:8000/api/recibir_dato/", buffer);
}


//-------------FIN MAIN ENCODER--------------------------



// // Desplazamiento para cada servo por acción
// const int action_displacements[SERVO_NUM][ACT_NUM] = {
//     { 45,  0, -45,  0,  0 }, // Efecto en Servo 1
//     {  0, 45,   0, -45,  0 }  // Efecto en Servo 2
// };

// Desplazamiento para cada servo por acción
const int action_displacements[ACT_NUM] = {
    { 45, -45,  0 } // Efecto en Servo
};

encoder_t encoder1, encoder2;  // Instancias de los dos encoders

typedef struct {
    float Q[SERVO_NUM][ROW_NUM][ACT_NUM]; // matriz Q tridimensional
    float R[SERVO_NUM][ROW_NUM][ACT_NUM]; // matriz R tridimensional (recompensas)
    float epsilon; // tasa de exploración
    float alpha; // tasa de aprendizaje
    float gamma; // factor de descuento
} Q_Agent;

Q_Agent agent;
bool crawler_listo = false; // Indica si el aprendizaje ha finalizado

// definición de funciones
void q_agent_init(Q_Agent *agent);
int q_agent_select_action(Q_Agent *agent, int servo, int state);
void q_agent_update(Q_Agent *agent, int servo, int state, int action, int next_state);
void mover_servos(int servo1_position, int servo2_position);
void encoder_signal(Q_Agent *, int,  int,  int,  encoder_t *, encoder_t *); // simula la señal del encoder
void print_q_matrix(Q_Agent *agent); // Nueva función para imprimir la matriz Q
void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position); // Nueva función para el movimiento continuo de arrastre


// Proceso de aprendizaje
void tarea_q_learning(void *param){
    int current_state = 0; // estado inicial
    int servo1_position = 0;
    int servo2_position = 0;
    int cont = 0;
    
    while (!crawler_listo && (cont < 100)) {
        // Seleccionar acción para cada servo
        int servo1_action = q_agent_select_action(&agent, 0, current_state);
        int servo2_action = q_agent_select_action(&agent, 1, current_state);

        int servo1_new_position = servo1_position + action_displacements[servo1_action];
        int servo2_new_position = servo2_position + action_displacements[servo2_action];

        // Restringir posiciones a los límites
        servo1_new_position = servo1_new_position > MAX_POSITION ? MAX_POSITION :
                            (servo1_new_position < MIN_POSITION ? MIN_POSITION : servo1_new_position);

        servo2_new_position = servo2_new_position > MAX_POSITION ? MAX_POSITION :
                            (servo2_new_position < MIN_POSITION ? MIN_POSITION : servo2_new_position);

        // Calcular nuevo estado
        int next_state = (servo1_new_position / 45) + (servo2_new_position / 45) * 3;

        // Simular recompensas del encoder
        // int reward1 = encoder_signal();
        // int reward2 = encoder_signal();

        // Mover los servos
        // mover_servos(servo1_new_position, servo2_new_position);
        process_move_shoulder(servo1_new_position);
        process_move_elbow(servo2_new_position);

        // Actualizar la matriz R con recompensas
        encoder_signal(&agent, 0, current_state, servo1_action, &encoder1, &encoder2);
        encoder_signal(&agent, 1, current_state, servo2_action, &encoder1, &encoder2);
        
        
        // Actualizar las matrices Q para ambos servos
        q_agent_update(&agent, 0, current_state, servo1_action, next_state);
        q_agent_update(&agent, 1, current_state, servo2_action, next_state);

        // Actualizar estado y posiciones
        current_state = next_state;
        servo1_position = servo1_new_position;
        servo2_position = servo2_new_position;

        // Imprimir matriz Q
        print_q_matrix(&agent);

        vTaskDelay(pdMS_TO_TICKS(1000)); // espera 1 segundo entre ciclos
        cont++;
    }

    // Una vez finalizado el aprendizaje, cambiar a modo de arrastre continuo
    printf("Proceso de aprendizaje completado.\n");
    mover_servos_continuamente(servo1_position, servo2_position);


}


//-----main-------------------------------------------------------------

void app_main() {


    //SERVOS----------------------------------------------
    init_servo();
    current_pos[0] = SHOULDER_MID_PULSE;
    current_pos[1] = ELBOW_MID_PULSE;
    set_servo_angle(LEDC_SHOULDER_CHANNEL, current_pos[0]);
    set_servo_angle(LEDC_ELBOW_CHANNEL, current_pos[1]);

    //ENCODER + AP----------------------------------------
    uart_set_baudrate(UART_NUM_0, 115200);
    nvs_flash_init();  // Inicializa NVS
    wifi_init_softap();  // Inicia AP

    esp_task_wdt_deinit();  // Desactiva el watchdog para las tareas TESTEANDO


    // Inicializar los encoders
    encoder_init(&encoder1, ENCODER1_OUT);
    encoder_init(&encoder2, ENCODER2_OUT);
    encoders_params_t encoders = {
        .encoder1 = &encoder1,
        .encoder2 = &encoder2,
    };

    //Provisorio, dummy. Ajustar para que sea la matriz de verdad
    // int matriz[9][9] = {//Dummy. Enviar la que en verdad es
    //     {1,  2,  3,  4,  5,  6,  7,  8,  9},
    //     {10, 11, 12, 13, 14, 15, 16, 17, 18},
    //     {19, 20, 21, 22, 23, 24, 25, 26, 27},
    //     {28, 29, 30, 31, 32, 33, 34, 35, 36},
    //     {37, 38, 39, 40, 41, 42, 43, 44, 45},
    //     {46, 47, 48, 49, 50, 51, 52, 53, 54},
    //     {55, 56, 57, 58, 59, 60, 61, 62, 63},
    //     {64, 65, 66, 67, 68, 69, 70, 71, 72},
    //     {73, 74, 75, 76, 77, 78, 79, 80, 81}
    //     };



    //Q-learning---------------------------------------

    q_agent_init(&agent);

    // xTaskCreate(tarea_encoders, "Tarea Encoders", 4096, NULL, 1, NULL);

    xTaskCreate(tarea_q_learning, "Tarea Q-Learning", 4096, NULL, 2, NULL);
    xTaskCreate(tarea_verificar_variable,      // Función de la tarea
    "VerificarVariableTask",       // Nombre de la tarea
    2048,                          // Tamaño del stack
    (void *)&encoders,             // Parámetro de entrada
    1,                             // Prioridad
    NULL);

    // void tarea_encoders(void *param) 
    //     {

    //         xTaskCreate(tarea_verificar_variable,      // Función de la tarea
    //             "VerificarVariableTask",       // Nombre de la tarea
    //             2048,                          // Tamaño del stack
    //             (void *)&encoders,             // Parámetro de entrada
    //             1,                             // Prioridad
    //             NULL);
            // vTaskDelay(pdMS_TO_TICKS(5000));
            // while(1){
            //     int32_t count1 = encoder_get_count(&encoder1);
            //     int32_t count2 = encoder_get_count(&encoder2);
            //     printf("Pulsos Encoder 1: %ld, Pulsos Encoder 2: %ld \n", count1, count2);

            //     direction_t dir = get_movement_direction();
            //     if (dir == DIRECTION_FORWARD) {
            //         printf("Hacia adelante\n");
            //     } else if (dir == DIRECTION_BACKWARD) {
            //         printf("Hacia atrás\n");
            //     } else {
            //         printf("Detenido\n");
            //     }
                
            //     char post_data[100];
            //     snprintf(post_data, sizeof(post_data), 
            //             "{\"encoder1\": %ld , \"encoder2\": %ld }", count1, count2);
            //     //9x9. Función para enviar datos al servidor
                
            //     //Obtiene recompensa encoders
            //     printf("Reward: %f\n", get_reward(&encoder1,&encoder2));

                
            //     //
            //     vTaskDelay(pdMS_TO_TICKS(1000));
            //     enviarDatosMatriz(matriz);
            // }

        // }   
}

void q_agent_init(Q_Agent *agent) {
    for (int servo = 0; servo < SERVO_NUM; servo++) {
        for (int s = 0; s < ROW_NUM; s++) {
            for (int a = 0; a < ACT_NUM; a++) {
                agent->Q[servo][s][a] = 0; // Inicializa la matriz Q en ceros
                agent->R[servo][s][a] = 0; // Inicializa la matriz R en ceros
            }
        }
    }
    agent->epsilon = 0.1; // exploración
    agent->alpha = 0.1; // tasa de aprendizaje
    agent->gamma = 0.9; // factor de descuento
}

int q_agent_select_action(Q_Agent *agent, int servo, int state) {
    if (((float)rand() / RAND_MAX) < agent->epsilon) {
        return rand() % ACT_NUM; // exploración
    } else {
        float max_q = agent->Q[servo][state][0];
        int action = 0;
        for (int a = 1; a < ACT_NUM; a++) {
            if (agent->Q[servo][state][a] > max_q) {
                max_q = agent->Q[servo][state][a];
                action = a;
            }
        }
        return action; // Retorna la mejor acción (del vector de acciones 0-5)
    }
}

// void q_agent_update(Q_Agent *agent, int servo, int state, int action, int reward, int next_state) {
//     float old_q = agent->Q[servo][state][action];
//     float max_q_next = agent->Q[servo][next_state][0];

//     for (int a = 1; a < ACT_NUM; a++) {
//         if (agent->Q[servo][next_state][a] > max_q_next) {
//             max_q_next = agent->Q[servo][next_state][a];
//         }
//     }
//     // Actualización de Q
//     agent->Q[servo][state][action] = old_q + agent->alpha * (reward + agent->gamma * max_q_next - old_q);
// }

void q_agent_update(Q_Agent *agent, int servo, int state, int action, int next_state) {
    float old_q = agent->Q[servo][state][action];
    float max_q_next = agent->Q[servo][next_state][0];

    // Buscar el valor Q máximo en el próximo estado
    for (int a = 1; a < ACT_NUM; a++) {
        if (agent->Q[servo][next_state][a] > max_q_next) {
            max_q_next = agent->Q[servo][next_state][a];
        }
    }
    // Usar la recompensa desde la matriz R
    float reward = agent->R[servo][state][action];
    // Actualizar la tabla Q
    agent->Q[servo][state][action] = old_q + agent->alpha * (reward + agent->gamma * max_q_next - old_q);
}

void print_q_matrix(Q_Agent *agent) {
    for (int servo = 0; servo < SERVO_NUM; servo++) {
        printf("Matriz Q para Servo %d:\n", servo + 1);
        for (int s = 0; s < ROW_NUM; s++) {
            for (int a = 0; a < ACT_NUM; a++) {
                printf("%.2f ", agent->Q[servo][s][a]);
            }
            printf("\n");
        }
        printf("\n");
    }
}

void mover_servos(int servo1_position, int servo2_position) {
    // Aquí debería ir el código para mover los servos reales utilizando PWM
    printf("Servo 1: %d grados, Servo 2: %d grados\n", servo1_position, servo2_position);
}

// int encoder_signal() {
//     // Simula una recompensa aleatoria
//     return rand() % 2; // Retorna 0 o 1
// }

void encoder_signal(Q_Agent *agent, int servo, int state, int action, encoder_t *encoder1, encoder_t *encoder2) {
    // Leer el valor de recompensa desde los encoders
    float reward = get_reward(&encoder1,&encoder2);
    // Actualizar la matriz R con la recompensa obtenida
    agent->R[servo][state][action] = reward;
    
    // printf("Actualizada recompensa en R[%d][%d][%d]: %.2f\n", servo, state, action, reward);
}



void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position) {
    //aca iria el boton de start stop-----
    while (1) {
        printf("Movimiento continuo: Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
