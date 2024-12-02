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

#include "cJSON.h" // Librería para procesar JSON (incluida en ESP-IDF)


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
#define TAG "HTTP_GET"

encoder_t encoder1, encoder2;  // Instancias de los dos encoders

SemaphoreHandle_t xMutex = NULL;

//agregar a codigo final------------------
char *response_data = NULL;
size_t response_data_size = 0;
int estadoCrawler=-1; //-1 Detenido, 1 Empezado(haciendo algo)
int estadoAprendiendoEjecutando=-1; //-1 Detenido, 0 Aprendiendo, 1 Ejecutando
//------------

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

// Función para manejar la respuesta del GET
//NUEVO HTTP, AGREGAR A CODIGO FINAL -------------------------------------------------------------------
int process_get_response(const char *response) {
    // Parsear el JSON
    cJSON *json = cJSON_Parse(response);
    if (json == NULL) {
        printf("Error al parsear la respuesta JSON\n");
        return 0;
    }
    int respuesta=0;

    // Extraer el valor de "start"
    cJSON *start_item = cJSON_GetObjectItem(json, "start");
    if (cJSON_IsBool(start_item)) {
        bool start = cJSON_IsTrue(start_item);
        printf("Estado de start: %s\n", start ? "true" : "false");
        if(start){
            respuesta=1;
        }
    } else {
        printf("No se encontró el campo 'start' o no es un booleano\n");
    }

    // Liberar la memoria del objeto JSON
    cJSON_Delete(json);
    return respuesta;
}

//NUEVO HTTP, AGREGAR A CODIGO FINAL -------------------------------------------------------------------
// Método HTTP GET mejorado
// Método HTTP GET con soporte JSON
esp_err_t client_event_get_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA, longitud de datos: %d\n", evt->data_len);
        // Redimensionar el buffer para almacenar los nuevos datos
        response_data = realloc(response_data, response_data_size + evt->data_len + 1); // +1 para el terminador nulo
        if (response_data == NULL)
        {
            printf("Error al redimensionar el buffer de respuesta.\n");
            return ESP_FAIL;
        }

        // Copiar los nuevos datos al buffer
        memcpy(response_data + response_data_size, evt->data, evt->data_len);
        response_data_size += evt->data_len;
        response_data[response_data_size] = '\0'; // Asegurarse de que sea una cadena terminada en nulo
        break;

    default:
        break;
    }
    return ESP_OK;
}


int http_get(const char *url)
{
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = client_event_get_handler, // Usar el manejador de eventos
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    int estado = estadoCrawler; // Por defecto, asumimos error

    // Realizar la solicitud HTTP GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        int status_code = esp_http_client_get_status_code(client);
        printf("GET exitoso, código de respuesta: %d\n", status_code);

        if (response_data != NULL)
        {
            printf("Respuesta completa del servidor: %s\n", response_data);

            // Analizar el contenido JSON usando cJSON
            cJSON *json_response = cJSON_Parse(response_data);
            if (json_response == NULL)
            {
                printf("Error al analizar JSON.\n");
            }
            else
            {
                // Extraer valor del campo "start"
                cJSON *start = cJSON_GetObjectItemCaseSensitive(json_response, "start");
                if (cJSON_IsBool(start))
                {
                    estado = cJSON_IsTrue(start) ? 1 : -1; // 1 si es `true`, 0 si es `false`
                    printf("Valor de 'start': %s\n", estado ? "true" : "false");
                }
                else
                {
                    printf("El campo 'start' no es booleano o no existe.\n");
                }
                cJSON_Delete(json_response); // Libera la memoria del objeto JSON
            }

            free(response_data); // Libera el buffer de respuesta
            response_data = NULL;
            response_data_size = 0;
        }
        else
        {
            printf("No se recibió respuesta del servidor.\n");
        }
    }
    else
    {
        printf("Error en el GET: %s\n", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return estado;
}


//envia datos de la matriz al servidor-----------------

void enviarDatosMatriz(float matriz[9][9]) {
    char buffer[1024]; // Buffer para el string JSON
    int offset = 0;    // Offset para ir escribiendo en el buffer

    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "{ \"matriz\": [");

    for (int i = 0; i < 9; i++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "[");
        for (int j = 0; j < 9; j++) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.2f", matriz[i][j]);
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


//AGREGAR CODIGO -------------------------------------------------
int obtenerEstadoCrawler(){
    printf("Solicitando estado de la variable start...\n");
    int estado=http_get("http://192.168.4.2:8000/get_start_state/"); //0 - Detener ...... 1 - Empezar
    return estado;
}

//AGREGAR A CODIGO FINAL ----------------------------------------------------
void enviarEstadoCrawler(){
    char buffer[128];
    int length = snprintf(buffer, sizeof(buffer), "{ \"estado\": %d }", estadoAprendiendoEjecutando);
    if (length < 0 || length >= sizeof(buffer)) {
        printf("Error al generar el JSON.\n");
        return;
    }
    // Imprimir el JSON
    printf("JSON generado:\n%s\n", buffer);
    http_post("http://192.168.4.2:8000/api/recibir_estado/", buffer);
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
void print_r_matrix(Q_Agent *agent);
void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position); // Nueva función para el movimiento continuo de arrastre
void simu_mover_servos(int next_state, int accion);
void simu_encoder_signal(Q_Agent *agent, int current_state, int next_state);
int obtener_siguiente_estado(int current_state, int action);
bool accion_valida(int current_state, int action);


// Proceso de aprendizaje----------------------------------------------------HILO APRENDIZAJE--------------------------------------------------

void tarea_q_learning(void *param) {
    int current_state = 0;  // Estado inicial
    int next_state = 0;
    int action = 0;
    int cont = 0;

    // Número máximo de iteraciones para el aprendizaje
    int max_iterations = 100;

    // Se asume que se quiere entrenar por un número determinado de iteraciones
    while (cont < max_iterations) {

        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            // 1. Seleccionar acción en función de la política epsilon-greedy
            action = q_agent_select_action(&agent, current_state);
            xSemaphoreGive(xMutex);
	    }	

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

        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            // 4. Obtener la recompensa (basado en los encoders)
            // encoder_signal(&agent, current_state, next_state, &encoder1, &encoder2);
            simu_encoder_signal(&agent, current_state, next_state);
            
            // 5. Actualizar la matriz Q
            q_agent_update(&agent, current_state, action, next_state);
            xSemaphoreGive(xMutex);
	    }	

        // 6. Actualizar el estado actual para el siguiente ciclo
        current_state = next_state;

        // 7. Mostrar la matriz Q para depuración (opcional)
        print_q_matrix(&agent);

        if(cont*20%100 == 0)
        {
            enviarDatosMatriz(agent.Q);
        }

        // Incrementar el contador de iteraciones
        cont++;

        // 8. Controlar el tiempo de ejecución con FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera de medio segundo entre ciclos de aprendizaje
    }

    // 9. Cuando se termine el aprendizaje, podemos salir del bucle
    crawler_listo = true;  // Señalamos que el aprendizaje ha terminado
    // enviarDatosMatriz(agent.Q);
    enviarDatosMatriz(agent.Q);
    printf("Proceso de aprendizaje completado.\n");
    mover_servos_continuamente(0,0);
}

//---------------------------------------------------------------------FIN APRENDIZAJE-------------------------------------------------------------

// Proceso de aprendizaje----------------------------------------------------HILO WIFI--------------------------------------------------

// void tarea_http_wifi(void *param) {

//     // Configuración de Wi-Fi
//     wifi_init_softap();
//     esp_task_wdt_deinit();
//     while (1) {
//         // Aquí podrías hacer solicitudes HTTP o esperar para recibir peticiones
//         // int estado = obtenerEstadoCrawler();
//         // if (estado == 1) {
//         //     // Procesar si se solicita que empiece el aprendizaje
//         //     printf("Inicio de aprendizaje.\n");
//         //     vTaskDelay(pdMS_TO_TICKS(1000));
//         // }
//         enviarDatosMatriz(agent.Q);
//         vTaskDelay(pdMS_TO_TICKS(5000));  // Esperar entre consultas
//     }
// }

// ----------------------------------------------------FIN HILO WIFI--------------------------------------------------

// void app_main() {


//     //SERVOS----------------------------------------------
//      init_servo();
//      set_pos(SHOULDER_MID_PULSE,ELBOW_MID_PULSE);
//      set_servo_angle(LEDC_SHOULDER_CHANNEL, SHOULDER_MID_PULSE);
//      set_servo_angle(LEDC_ELBOW_CHANNEL, ELBOW_MID_PULSE);

//     // //ENCODER + AP----------------------------------------
//      uart_set_baudrate(UART_NUM_0, 115200);
//      nvs_flash_init();  // Inicializa NVS
//      wifi_init_softap();  // Inicia AP

//      esp_task_wdt_deinit();  // Desactiva el watchdog para las tareas TESTEANDO


//     // // Inicializar los encoders
//      encoder_init(&encoder1, ENCODER1_OUT);
//      encoder_init(&encoder2, ENCODER2_OUT);
//      encoders_params_t encoders = {
//          .encoder1 = &encoder1,
//          .encoder2 = &encoder2,
//      };


//     //Q-learning---------------------------------------

//     q_agent_init(&agent);
//     xTaskCreate(tarea_q_learning, "Tarea Q-Learning", 4096, NULL, 2, NULL);
//     // xTaskCreate(tarea_verificar_variable,      // Función de la tarea
//     // "VerificarVariableTask",       // Nombre de la tarea
//     // 2048,                          // Tamaño del stack
//     // (void *)&encoders,             // Parámetro de entrada
//     // 1,                             // Prioridad
//     // NULL);

// }

void app_main() {

    // Inicialización de hardware (Wi-Fi, encoders, etc.)
    init_servo();
    set_pos(SHOULDER_MID_PULSE, ELBOW_MID_PULSE);
    set_servo_angle(LEDC_SHOULDER_CHANNEL, SHOULDER_MID_PULSE);
    set_servo_angle(LEDC_ELBOW_CHANNEL, ELBOW_MID_PULSE);

    uart_set_baudrate(UART_NUM_0, 115200);
    nvs_flash_init(); // Inicializa NVS

    wifi_init_softap();
    esp_task_wdt_deinit();

    encoder_init(&encoder1, ENCODER1_OUT);
    encoder_init(&encoder2, ENCODER2_OUT);
    encoders_params_t encoders = {
        .encoder1 = &encoder1,
        .encoder2 = &encoder2,
    };

    xMutex = xSemaphoreCreateMutex();

    q_agent_init(&agent);

    srand(time(NULL)); // Inicializa la semilla
    // Crear tareas en los dos núcleos:

    // Tarea para comunicación HTTP y Wi-Fi (Núcleo 0)
    xTaskCreate(
        tarea_q_learning,            // Función de la tarea
        "Tarea_HTTP_WiFi",          // Nombre de la tarea
        4096,                       // Tamaño del stack
        NULL,                       // Parámetro de entrada
        2,                          // Prioridad
        NULL                      // Handle de la tarea
                                    // Núcleo al que se asigna (Core 0)
    );

    // // Tarea para el aprendizaje Q-Learning (Núcleo 1)
    // xTaskCreatePinnedToCore(
    //     tarea_q_learning,           // Función de la tarea
    //     "Tarea_Q_Learning",         // Nombre de la tarea
    //     4096,                       // Tamaño del stack
    //     NULL,                       // Parámetro de entrada
    //     2,                          // Prioridad
    //     NULL,                       // Handle de la tarea
    //     1                            // Núcleo al que se asigna (Core 1)
    // );
}

// void tarea_q_learning(void *param) {
//     int current_state = 0;  // Estado inicial
//     int next_state = 0;
//     int action = 0;
//     int cont = 0;
//     q_agent_init(&agent);

//     while (cont < 100) {
//         // Seleccionar acción, ejecutar, actualizar y simular movimiento
//         action = q_agent_select_action(&agent, current_state);
//         next_state = obtener_siguiente_estado(current_state, action);
//         simu_mover_servos(next_state, action);
//         simu_encoder_signal(&agent, current_state, next_state);
//         q_agent_update(&agent, current_state, action, next_state);
//         current_state = next_state;

//         // Mostrar y enviar datos de la matriz Q
//         if (cont % 10 == 0) {
//             enviarDatosMatriz(agent.Q);
//         }

//         cont++;
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Controlar la frecuencia de aprendizaje
//     }
//     printf("Proceso de aprendizaje completado.\n");
// }


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

    print_r_matrix(agent);

}

void print_q_matrix(Q_Agent *agent) {
    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            printf("%.2f ", agent->Q[r][c]);
        }
        printf("\n");
    }
}

void print_r_matrix(Q_Agent *agent) {
    printf("\n");
    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            printf("%.2f ", agent->R[r][c]);
        }
        printf("\n");
    }
}

// Función para mover los servos según el estado 
void simu_mover_servos(int estado, int accion)
{
    int servo1_pos = estado / 3 * 45;  // Dividiendo el estado para obtener la posición de servo 1
    int servo2_pos = (estado % 3) * 45;  // Calculando la posición de servo 2

    // // Ejecutar la acción correspondiente
    // switch (accion) {
    //     case ACTION_SERVO1_FORWARD:
    //         servo1_pos += 45; // Mover servo 1 hacia arriba
    //         break;
    //     case ACTION_SERVO1_BACKWARD:
    //         servo1_pos -= 45; // Mover servo 1 hacia abajo
    //         break;
    //     case ACTION_SERVO2_FORWARD:
    //         servo2_pos += 45; // Mover servo 2 hacia arriba
    //         break;
    //     case ACTION_SERVO2_BACKWARD:
    //         servo2_pos -= 45; // Mover servo 2 hacia abajo
    //         break;
    // }

    // // Limitar las posiciones de los servos a entre 0 y 90 grados
    // //ya está implmentado en el get_valid_actions, pero le da una capa de prevencion
    // if (servo1_pos > 90) servo1_pos = 90;
    // if (servo1_pos < 0) servo1_pos = 0;
    // if (servo2_pos > 90) servo2_pos = 90;
    // if (servo2_pos < 0) servo2_pos = 0;
    // process_move_shoulder(servo1_pos);
    // process_move_elbow(servo2_pos);

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
    // srand(time(NULL)); // Inicializa la semilla
    // Genera un número aleatorio entre 0 y 1
    float random = (float)rand() / RAND_MAX;
    agent->R[state][next_state] = random;
    
    printf("Actualizada recompensa en R[%d][%d]: %.2f\n",state, next_state, random);
}


// void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position) {
//     //aca iria el boton de start stop-----
//     while (1) {
//         printf("Movimiento continuo: Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);
//         printf("Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position) {
    // Encontrar el estado con el mayor valor en la matriz Q
    int best_state = 0;
    float max_q_value = -1e6; // Inicializar con un valor muy bajo para encontrar el máximo

    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            if (agent.Q[r][c] > max_q_value) {
                max_q_value = agent.Q[r][c];
                best_state = c; // Guardar la columna (estado destino) con el mayor valor
            }
        }
    }

    // Decodificar las posiciones del mejor estado
    int servo1_best_position = (best_state / 3) * 45; // Fila del estado mejor
    int servo2_best_position = (best_state % 3) * 45; // Columna del estado mejor

    printf("Mejor estado encontrado: %d con Q-value: %.2f\n", best_state, max_q_value);
    printf("Moviendo continuamente entre posición inicial Servo 1: %d, Servo 2: %d y mejor posición Servo 1: %d, Servo 2: %d\n",
           servo1_initial_position, servo2_initial_position,
           servo1_best_position, servo2_best_position);

    while (1) {
        // Movimiento de inicial a mejor posición
        printf("Moviendo hacia la mejor posición...\n");
        while (servo1_initial_position != servo1_best_position || servo2_initial_position != servo2_best_position) {
            if (servo1_initial_position < servo1_best_position) {
                servo1_initial_position += 45; // Incrementar hacia la mejor posición
            } else if (servo1_initial_position > servo1_best_position) {
                servo1_initial_position -= 45; // Decrementar hacia la mejor posición
            }

            if (servo2_initial_position < servo2_best_position) {
                servo2_initial_position += 45; // Incrementar hacia la mejor posición
            } else if (servo2_initial_position > servo2_best_position) {
                servo2_initial_position -= 45; // Decrementar hacia la mejor posición
            }

            // Imprimir el estado actual del movimiento
            printf("Moviendo Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);

            // Simular tiempo de movimiento
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // Movimiento de regreso a la posición inicial
        printf("Moviendo de regreso a la posición inicial...\n");
        while (servo1_initial_position != 0 || servo2_initial_position != 0) {
            if (servo1_initial_position > 0) {
                servo1_initial_position -= 45; // Decrementar hacia la posición inicial
            } else if (servo1_initial_position < 0) {
                servo1_initial_position += 45; // Incrementar hacia la posición inicial
            }

            if (servo2_initial_position > 0) {
                servo2_initial_position -= 45; // Decrementar hacia la posición inicial
            } else if (servo2_initial_position < 0) {
                servo2_initial_position += 45; // Incrementar hacia la posición inicial
            }

            // Imprimir el estado actual del movimiento
            printf("Moviendo Servo 1 a %d, Servo 2 a %d\n", servo1_initial_position, servo2_initial_position);

            // Simular tiempo de movimiento
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

