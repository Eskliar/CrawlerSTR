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


//---------CORRECCIONES FUTURAS----------
/*
-establecer estados para no tener que hacer
    int next_state = (servo1_new_position / 45) + (servo2_new_position / 45) * 3;
*/

#define ROW_NUM 9 // 9 estados (3 posiciones para servo1 y 3 para servo2)
#define COL_NUM 9 // 9 estados (3 posiciones para servo1 y 3 para servo2)

#define SERVO_NUM 2 // Dos servos
#define MAX_POSITION 90 // máxima posición (grados)
#define MIN_POSITION 0 

#define NUM_ESTADOS 9
#define ACT_NUM 4 // 2 acciones por cada servo: +45 -45

//estan al cuete por ahora
#define ACTION_SERVO1_FORWARD 0  // Mover servo 1 hacia adelante
#define ACTION_SERVO1_BACKWARD 1  // Mover servo 1 hacia atras
#define ACTION_SERVO2_FORWARD 2 // Mover servo 2 hacia adelante
#define ACTION_SERVO2_BACKWARD 3 // Mover servo 2 hacia atras


//---------------MAIN ENCODER---------------------

// Configuración del AP
#define AP_SSID "MiESP32_AP"
#define AP_PASSWORD "123456789"
#define MAX_STA_CONN 4

encoder_t encoder1, encoder2;  // Instancias de los dos encoders


// Configura el modo AP del ESP32---------------------------------------
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

// Maneja eventos HTTP-------------------------------------

esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ERROR) {
        //printf("Error en la solicitud HTTP\n");
    }
    return ESP_OK;
}

// Enviar una solicitud HTTP POST---------------------------

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

//envia datos de la matriz al servidor-----------------

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


//-------------FIN MAIN ENCODER-------------------------------------------------------



//-----------------------------------Estructuras y Definición de Matrices------------------

encoder_t encoder1, encoder2;  // Instancias de los dos encoders

typedef struct {
    float Q[NUM_ESTADOS][NUM_ESTADOS]; // Matriz Q para estados y transiciones
    float R[NUM_ESTADOS][NUM_ESTADOS]; // Matriz R para recompensas
    float epsilon; // Tasa de exploración
    float alpha;   // Tasa de aprendizaje
    float gamma;   // Factor de descuento

} Q_Agent;

Q_Agent agent;


//-----------------------------------------------------


bool crawler_listo = false; // Indica si el aprendizaje ha finalizado

// definición de funciones
void q_agent_init(Q_Agent *agent);
int q_agent_select_action(Q_Agent *agent, int state);
void q_agent_update(Q_Agent *agent, int state, int action, int next_state);
//void mover_servos(int servo1_position, int servo2_position);
//void encoder_signal(Q_Agent *, int,  int,  int,  encoder_t *, encoder_t *); // simula la señal del encoder
void print_q_matrix(Q_Agent *agent); // Nueva función para imprimir la matriz Q
void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position); // Nueva función para el movimiento continuo de arrastre
void simu_mover_servos(int next_state, int accion);
void simu_encoder_signal(Q_Agent *agent, int current_state, int next_state);
int obtener_siguiente_estado(int current_state, int action);
bool accion_valida(int current_state, int action);


// Proceso de aprendizaje----------------------------------------------------APRENDIZAJE--------------------------------------------------

void tarea_q_learning(void *param) {
    int current_state = 0;  // Estado inicial
    int next_state = 0;
    int action = 0;
    int cont = 0;

    // Número máximo de iteraciones para el aprendizaje
    int max_iterations = 1000;

    // Se asume que se quiere entrenar por un número determinado de iteraciones
    while (cont < max_iterations) {
        // 1. Seleccionar acción en función de la política epsilon-greedy
        action = q_agent_select_action(&agent, current_state);

        // 2. Ejecutar la acción (mover los servos)
        next_state = obtener_siguiente_estado(current_state, action);  // Determina el siguiente estado

        // Validar que el estado no se sale de los límites y que la acción es válida
        if (!accion_valida(current_state, action)) {
            printf("Acción no válida desde el estado %d con acción %d\n", current_state, action);
            continue;  // Saltamos esta iteración si la acción no es válida
        }

        // 3. Mover servos según el estado siguiente (simular el movimiento)
        // mover_servos(next_state);
        simu_mover_servos(next_state, action);

        // 4. Obtener la recompensa (basado en los encoders)
        // encoder_signal(&agent, current_state, next_state, &encoder1, &encoder2);
        simu_encoder_signal(&agent, current_state, next_state);
        
        // 5. Actualizar la matriz Q
        q_agent_update(&agent, current_state, action, next_state);

        // 6. Actualizar el estado actual para el siguiente ciclo
        current_state = next_state;

        // 7. Mostrar la matriz Q para depuración (opcional)
        print_q_matrix(&agent);

        // Incrementar el contador de iteraciones
        cont++;

        // 8. Controlar el tiempo de ejecución con FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(500));  // Espera de medio segundo entre ciclos de aprendizaje
    }

    // 9. Cuando se termine el aprendizaje, podemos salir del bucle
    crawler_listo = true;  // Señalamos que el aprendizaje ha terminado
    printf("Proceso de aprendizaje completado.\n");
}



//---------------------------------------------------------------------FIN APRENDIZAJE-------------------------------------------------------------

void app_main() {


    //SERVOS----------------------------------------------
    init_servo();
    set_pos(SHOULDER_MID_PULSE,ELBOW_MID_PULSE);
    set_servo_angle(LEDC_SHOULDER_CHANNEL, SHOULDER_MID_PULSE);
    set_servo_angle(LEDC_ELBOW_CHANNEL, ELBOW_MID_PULSE);

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


    //Q-learning---------------------------------------

    q_agent_init(&agent);
    xTaskCreate(tarea_q_learning, "Tarea Q-Learning", 4096, NULL, 2, NULL);
    xTaskCreate(tarea_verificar_variable,      // Función de la tarea
    "VerificarVariableTask",       // Nombre de la tarea
    2048,                          // Tamaño del stack
    (void *)&encoders,             // Parámetro de entrada
    1,                             // Prioridad
    NULL);

}

//----------------------------FUNCIONES--------------------------------------------------

// Función para transformar el par de ángulos en un estado
int get_estado(int servo1_pos, int servo2_pos) {
    int estado = 0;
    if (servo1_pos == 45) estado += 3;
    if (servo1_pos == 90) estado += 6;
    if (servo2_pos == 45) estado += 1;
    if (servo2_pos == 90) estado += 2;
    return estado;
}

// Función para obtener las acciones válidas basadas en las posiciones de los servos
int* get_valid_actions(int servo1_pos, int servo2_pos, int* size) {
    static int valid_actions[4]; // Array estático para almacenar las acciones válidas
    int action_count = 0;

    // Verificar si se puede mover el servo 1 hacia arriba (sumar 45 grados)
    if (servo1_pos + 45 <= MAX_POSITION) {
        valid_actions[action_count++] = ACTION_SERVO1_FORWARD;
    }

    // Verificar si se puede mover el servo 1 hacia abajo (restar 45 grados)
    if (servo1_pos - 45 >= MIN_POSITION) {
        valid_actions[action_count++] = ACTION_SERVO1_BACKWARD;
    }

    // Verificar si se puede mover el servo 2 hacia arriba (sumar 45 grados)
    if (servo2_pos + 45 <= MAX_POSITION) {
        valid_actions[action_count++] = ACTION_SERVO2_FORWARD;
    }

    // Verificar si se puede mover el servo 2 hacia abajo (restar 45 grados)
    if (servo2_pos - 45 >= MIN_POSITION) {
        valid_actions[action_count++] = ACTION_SERVO2_BACKWARD;
    }

    *size = sizeof(valid_actions)/sizeof(valid_actions[0]);

    return valid_actions;  // Devuelve las acciones válidas
}


// Función para inicializar las matrices Q y R
void q_agent_init(Q_Agent *agent) {
    for (int i = 0; i < NUM_ESTADOS; i++) {
        for (int j = 0; j < NUM_ESTADOS; j++) {
            agent->Q[i][j] = 0.0f;  // Inicializa la matriz Q con ceros
            agent->R[i][j] = 0.0f;  // Inicializa la matriz R con ceros
        }
    }
    agent->epsilon = 0.1; // exploración
    agent->alpha = 0.1; // tasa de aprendizaje
    agent->gamma = 0.9; // factor de descuento
}


// Función para seleccionar la acción usando la política epsilon-greedy, considerando solo acciones válidas
int q_agent_select_action(Q_Agent *agent, int estado) {
    // Obtener las posiciones actuales de los servos
    int servo1_pos = estado / 3 * 45;  // Dividiendo el estado para obtener la posición de servo 1
    int servo2_pos = (estado % 3) * 45;  // Calculando la posición de servo 2
    int num_valid_actions;

    // Obtener las acciones válidas
    int* valid_actions = get_valid_actions(servo1_pos, servo2_pos, &num_valid_actions);

    if ((float)rand() / RAND_MAX < agent->epsilon) {
        // Exploración: Seleccionar una acción aleatoria entre las acciones válidas
        return valid_actions[rand() % num_valid_actions];
    } else {
        // Explotación: Seleccionar la mejor acción (basado en la Q-matriz para las acciones válidas)
        float max_q = -1e6; // Valor mínimo
        int best_action = valid_actions[0];
        for (int i = 0; i < num_valid_actions; i++) {
            int action = valid_actions[i];
            if (agent->Q[estado][action] > max_q) {
                max_q = agent->Q[estado][action];
                best_action = action;
            }
        }
        return best_action;  // Devolver la acción con el mayor valor Q
    }
}

int obtener_siguiente_estado(int current_state, int action) {
    int servo1_pos = (current_state / 3) * 45;  // Decodificar servo1 desde el estado
    int servo2_pos = (current_state % 3) * 45;  // Decodificar servo2 desde el estado

    // Aplicar la acción al servo correspondiente
    if (action == 0) {  // Servo 1 hacia adelante
        servo1_pos += 45;
        if (servo1_pos > MAX_POSITION) servo1_pos = MAX_POSITION;
    } else if (action == 1) {  // Servo 1 hacia atrás
        servo1_pos -= 45;
        if (servo1_pos < MIN_POSITION) servo1_pos = MIN_POSITION;
    } else if (action == 2) {  // Servo 2 hacia adelante
        servo2_pos += 45;
        if (servo2_pos > MAX_POSITION) servo2_pos = MAX_POSITION;
    } else if (action == 3) {  // Servo 2 hacia atrás
        servo2_pos -= 45;
        if (servo2_pos < MIN_POSITION) servo2_pos = MIN_POSITION;
    }

    // Convertir las nuevas posiciones a un estado
    return (servo1_pos / 45) * 3 + (servo2_pos / 45);
}


bool accion_valida(int current_state, int action) {
    int servo1_pos = (current_state / 3) * 45;
    int servo2_pos = (current_state % 3) * 45;

    // Validar la acción para el servo 1
    if ((action == 0 && servo1_pos == MAX_POSITION) || (action == 1 && servo1_pos == MIN_POSITION)) {
        return false;  // Si se intenta mover el servo fuera de límites
    }

    // Validar la acción para el servo 2
    if ((action == 2 && servo2_pos == MAX_POSITION) || (action == 3 && servo2_pos == MIN_POSITION)) {
        return false;  // Si se intenta mover el servo fuera de límites
    }

    return true;  // Acción válida
}



// Función para actualizar la matriz Q con la recompensa obtenida
//esta funcion solo debería fijarse en los estados posibles, ya que los demás deberían tener siempre 0
void q_agent_update(Q_Agent *agent, int estado, int accion, int siguiente_estado) {
    float old_q = agent->Q[estado][siguiente_estado];
    float max_q_next = agent->Q[siguiente_estado][0];

    for (int a = 1; a < NUM_ESTADOS; a++) {
        if (agent->Q[siguiente_estado][a] > max_q_next) {
            max_q_next = agent->Q[siguiente_estado][a];
        }
    }
    // Usar la recompensa desde la matriz R
    float reward = agent->R[estado][siguiente_estado];
    // Actualizar la tabla Q
    agent->Q[estado][siguiente_estado] = old_q + agent->alpha * (reward + agent->gamma * max_q_next - old_q);
}

void print_q_matrix(Q_Agent *agent) {
    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            printf("%.2f ", agent->Q[r][c]);
        }
        printf("\n");
    }
}

// Función para mover los servos según el estado 
void simu_mover_servos(int estado, int accion)
{
    int servo1_pos = estado / 3 * 45;  // Dividiendo el estado para obtener la posición de servo 1
    int servo2_pos = (estado % 3) * 45;  // Calculando la posición de servo 2

    // Ejecutar la acción correspondiente
    switch (accion) {
        case ACTION_SERVO1_FORWARD:
            servo1_pos += 45; // Mover servo 1 hacia arriba
            break;
        case ACTION_SERVO1_BACKWARD:
            servo1_pos -= 45; // Mover servo 1 hacia abajo
            break;
        case ACTION_SERVO2_FORWARD:
            servo2_pos += 45; // Mover servo 2 hacia arriba
            break;
        case ACTION_SERVO2_BACKWARD:
            servo2_pos -= 45; // Mover servo 2 hacia abajo
            break;
    }

    // Limitar las posiciones de los servos a entre 0 y 90 grados
    //ya está implmentado en el get_valid_actions, pero le da una capa de prevencion
    if (servo1_pos > 90) servo1_pos = 90;
    if (servo1_pos < 0) servo1_pos = 0;
    if (servo2_pos > 90) servo2_pos = 90;
    if (servo2_pos < 0) servo2_pos = 0;

    // Imprimir las nuevas posiciones de los servos
    printf("Moviendo servo 1 a %d grados, servo 2 a %d grados\n", servo1_pos, servo2_pos);
}
//el estado debería llegar bien
/*void mover_servos(int estado) {
    int servo1_pos = 0;
    int servo2_pos = 0;
    switch (estado) {
        case 0: servo1_pos = 0; servo2_pos = 0; break;
        case 1: servo1_pos = 0; servo2_pos = 45; break;
        case 2: servo1_pos = 0; servo2_pos = 90; break;
        case 3: servo1_pos = 45; servo2_pos = 0; break;
        case 4: servo1_pos = 45; servo2_pos = 45; break;
        case 5: servo1_pos = 45; servo2_pos = 90; break;
        case 6: servo1_pos = 90; servo2_pos = 0; break;
        case 7: servo1_pos = 90; servo2_pos = 45; break;
        case 8: servo1_pos = 90; servo2_pos = 90; break;
    }
    process_move_shoulder(servo1_pos);
    process_move_elbow(servo2_pos);

    // Aquí deberías poner el código que mueve físicamente los servos
    // usando los ángulos decodificados (servo1_pos, servo2_pos)
    printf("Moviendo servo 1 a %d y servo 2 a %d\n", servo1_pos, servo2_pos);
}*/

// Función para mover los servos según la acción
/*void mover_servos(int estado, int accion) {
    int servo1_pos = estado / 3 * 45;  // Dividiendo el estado para obtener la posición de servo 1
    int servo2_pos = (estado % 3) * 45;  // Calculando la posición de servo 2

    // Ejecutar la acción correspondiente
    switch (accion) {
        case ACTION_SERVO1_FORWARD:
            servo1_pos += 45; // Mover servo 1 hacia arriba
            break;
        case ACTION_SERVO1_BACKWARD:
            servo1_pos -= 45; // Mover servo 1 hacia abajo
            break;
        case ACTION_SERVO2_FORWARD:
            servo2_pos += 45; // Mover servo 2 hacia arriba
            break;
        case ACTION_SERVO2_BACKWARD:
            servo2_pos -= 45; // Mover servo 2 hacia abajo
            break;
    }

    // Limitar las posiciones de los servos a entre 0 y 90 grados
    //ya está implmentado en el get_valid_actions, pero le da una capa de prevencion
    if (servo1_pos > 90) servo1_pos = 90;
    if (servo1_pos < 0) servo1_pos = 0;
    if (servo2_pos > 90) servo2_pos = 90;
    if (servo2_pos < 0) servo2_pos = 0;

    // Mover los servos físicamente a las nuevas posiciones
    process_move_shoulder(servo1_pos);
    process_move_elbow(servo2_pos);

    // Imprimir las nuevas posiciones de los servos
    printf("Moviendo servo 1 a %d grados, servo 2 a %d grados\n", servo1_pos, servo2_pos);
}*/



/*void encoder_signal(Q_Agent *agent, int state, int next_state, encoder_t *encoder1, encoder_t *encoder2) {
    // Leer el valor de recompensa desde los encoders
    float reward = get_reward(encoder1,encoder2);
    // Actualizar la matriz R con la recompensa obtenida
    agent->R[state][next_state] = reward;
    
    // printf("Actualizada recompensa en R[%d][%d][%d]: %.2f\n", servo, state, action, reward);
}*/

void simu_encoder_signal(Q_Agent *agent, int state, int next_state) {
    //simula un reward random como si leyera el encoder
    // Actualizar la matriz R con la recompensa obtenida
    srand(time(NULL)); // Inicializa la semilla
    // Genera un número aleatorio entre 0 y 1
    float random = (float)rand() / RAND_MAX;
    agent->R[state][next_state] = random;
    
    // printf("Actualizada recompensa en R[%d][%d][%d]: %.2f\n", servo, state, action, reward);
}


void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position) {
    //aca iria el boton de start stop-----
    while (1) {
        printf("Movimiento continuo: Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
