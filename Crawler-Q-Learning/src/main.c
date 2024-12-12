#include <main.h>

Q_Agent agent; // Instancia del agente Q-Learning

/***********************************************************/

//Inicialización de variables globales

char *response_data = NULL;
size_t response_data_size = 0;
int estadoCrawler=-1; //-1 Detenido, 1 Empezado(haciendo algo)
int estadoAprendiendoEjecutando=-1; //-1 Detenido, 0 Aprendiendo, 1 Ejecutando
int learn=FRONT_LEARN; //aprende para adelante (0) o atras (1)
encoder_t encoder1; // Definición de encoder1
encoder_t encoder2; // Definición de encoder2
SemaphoreHandle_t xMutex; // Definición de xMutex

/***********************************************************/

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

/***********************************************************/

// Maneja eventos HTTP-------------------------------------

esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ERROR) {
        //printf("Error en la solicitud HTTP\n");
    }
    return ESP_OK;
}

/***********************************************************/

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

/***********************************************************/

// Función para manejar la respuesta del GET

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

/***********************************************************/

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

/***********************************************************/

// Método HTTP GET con soporte JSON
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
                    estado = cJSON_IsTrue(start) ? 1 : -1; // 1 si es `true`, -1 si es `false`
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
            // estado=-1;
        }
    }
    else
    {
        printf("Error en el GET: %s\n", esp_err_to_name(err));
        // estado=-1;
    }

    esp_http_client_cleanup(client);
    return estado;
}

/*******funcion para tiempo************ */
double dwalltime() {
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (double)time.tv_sec + (double)time.tv_nsec / 1.0e9;
}

/***********************************************************/

//Envia datos de la matriz al servidor
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

/***********************************************************/

//Obtiene el estado del boton start
int obtenerEstadoCrawler(){
    printf("Solicitando estado de la variable start...\n");
    int estado=http_get("http://192.168.4.2:8000/get_start_state/"); //-1 - Detener ...... 1 - Empezar
    return estado;
}

/***********************************************************/

//Obtiene la dirección de la variable direccion
int obtenerDireccionCrawler(){
    printf("Solicitando direccion de la variable direccion...\n");
    int direccion=http_get("http://192.168.4.2:8000/api/get_direccion_crawler/"); //-1 - Detener ...... 1 - Empezar
    if(direccion==1){
        direccion=0;
    }
    else if (direccion==(-1))
    {
        direccion=1;
    }
    return direccion;
}

/***********************************************************/

//Envia el estado del crawler
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

/***********************************************************/

// Variable que indica si el aprendizaje ha finalizado
bool crawler_listo = false; 

/****************************************************************************************/
/************ FIN DE DECLARACIÓN DE FUNCIONES QUE TRABAJAN CON EL SERVIDOR **************/
/****************************************************************************************/


/***************** FUNCIONES PARA EL ALGORTIRMO DE APRENDIZAJE Q ************************/

// Función para transformar el par de ángulos en un estado
int get_estado(int servo1_pos, int servo2_pos) {
    int estado = 0;
    if (servo1_pos == 45) estado += 3;
    if (servo1_pos == 90) estado += 6;
    if (servo2_pos == 45) estado += 1;
    if (servo2_pos == 90) estado += 2;
    return estado;
}

/***********************************************************/

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


/***********************************************************/

// Función para inicializar las matrices Q y R
void q_agent_init(Q_Agent *agent) {
    for (int i = 0; i < NUM_ESTADOS; i++) {
        for (int j = 0; j < NUM_ESTADOS; j++) {
            agent->Q[i][j] = 0.0f;  // Inicializa la matriz Q con ceros
            agent->R[i][j] = 0.0f;  // Inicializa la matriz R con ceros
        }
    }
    agent->epsilon = 0.9; // exploración (exploración vs explotación)
    agent->alpha = 0.5; // tasa de aprendizaje (recompensa inmediata)
    agent->gamma = 0.7; // factor de descuento (valor medio-alto prioriza el largo plazo)
}

/***********************************************************/

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

/***********************************************************/

// Función para obtener el siguiente estado basado en la acción
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

/***********************************************************/

// Función para verificar si la acción es válida
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

/***********************************************************/

// Función para actualizar la matriz Q con la recompensa obtenida, esta funcion solo debería fijarse en los estados posibles, ya que los demás deberían tener siempre 0
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

/***********************************************************/

// Función para imprimir la matriz Q
void print_q_matrix(Q_Agent *agent) {
    printf("Q Matrix \n");
    printf("R matrix \n");
    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            printf("%.2f ", agent->Q[r][c]);
        }
        printf("\n");
    }
}

void print_r_matrix(Q_Agent *agent) {
    printf("\n");
    printf("R matrix \n");
    for (int r = 0; r < ROW_NUM; r++) {
        for (int c = 0; c < COL_NUM; c++) {
            printf("%.2f ", agent->R[r][c]);
        }
        printf("\n");
    }
}

/***********************************************************/

// Función de simulación para mover los servos según el estado
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

/***********************************************************/

// Función para mover los servos según el estado
void mover_servos(int estado) {
    // int servo1_pos = 0;
    // int servo2_pos = 0;
    // switch (estado) {
    //     case 0: servo1_pos = 0; servo2_pos = 0; break;
    //     case 1: servo1_pos = 0; servo2_pos = 45; break;
    //     case 2: servo1_pos = 0; servo2_pos = 90; break;
    //     case 3: servo1_pos = 45; servo2_pos = 0; break;
    //     case 4: servo1_pos = 45; servo2_pos = 45; break;
    //     case 5: servo1_pos = 45; servo2_pos = 90; break;
    //     case 6: servo1_pos = 90; servo2_pos = 0; break;
    //     case 7: servo1_pos = 90; servo2_pos = 45; break;
    //     case 8: servo1_pos = 90; servo2_pos = 90; break;
    // }

    int servo1_pos = estado / 3 * 45;  // Dividiendo el estado para obtener la posición de servo 1
    int servo2_pos = (estado % 3) * 45;  // Calculando la posición de servo 2
    process_move_shoulder(servo1_pos);
    process_move_elbow(servo2_pos);

    // Aquí deberías poner el código que mueve físicamente los servos
    // usando los ángulos decodificados (servo1_pos, servo2_pos)
    printf("Moviendo servo 1 a %d y servo 2 a %d\n", servo1_pos, servo2_pos);
}

/***********************************************************/

// Función que calcula la recompenza en base a la lectura los encoders
void encoder_signal(Q_Agent *agent, int state, int next_state, encoder_t *encoder1, encoder_t *encoder2) {
    // Leer el valor de recompensa desde los encoders
    float reward = get_reward(encoder1,encoder2);
    // Actualizar la matriz R con la recompensa obtenida
    agent->R[state][next_state] = reward;

    printf("Actualizada recompensa en R[%d][%d]: %.2f\n", state, next_state, reward);
}

/***********************************************************/

// Función para simular la señal del encoder
void simu_encoder_signal(Q_Agent *agent, int state, int next_state) {
    //simula un reward random como si leyera el encoder
    // Actualizar la matriz R con la recompensa obtenida
    // srand(time(NULL)); // Inicializa la semilla
    // Genera un número aleatorio entre 0 y 1
    float random = (float)rand() / RAND_MAX;
    agent->R[state][next_state] = random;

    printf("Actualizada recompensa en R[%d][%d]: %.2f\n",state, next_state, random);
}

/***********************************************************/

// Función para mover los servos continuamente
void mover_servos_continuamente(int servo1_initial_position, int servo2_initial_position) {

    int intentos = 0;//variable para contar los intentos de ejecución de lo aprendido
    int best_state = 0;
    float max_q_value = 0;

    if(learn == FRONT_LEARN){
        // Encontrar el estado con el mayor valor en la matriz Q
        max_q_value = -1e6; // Inicializar con un valor muy bajo para encontrar el máximo

        for (int r = 0; r < ROW_NUM; r++) {
            for (int c = 0; c < COL_NUM; c++) {
                if (agent.Q[r][c] > max_q_value) {
                    max_q_value = agent.Q[r][c];
                    best_state = c; // Guardar la columna (estado destino) con el mayor valor
                }
            }
        }
    }
    else if(learn == BACK_LEARN){
        // Encontrar el estado con el menor valor en la matriz Q
        max_q_value = 1e6; // Inicializar con un valor muy alto para encontrar el minimo

        for(int r = 0; r < ROW_NUM; r++) {
            for (int c = 0; c < COL_NUM; c++) {
                if (agent.Q[r][c] < max_q_value) {
                    max_q_value = agent.Q[r][c];
                    best_state = r; // Guardar la fila (estado anterior a destino) con el menor valor
                }
            }
        }
    }

    // Decodificar el estado -> pos de servos
    //esto podría implementarse como una función
    int servo1_best_position = (best_state / 3) * 45; // Fila del estado mejor
    int servo2_best_position = (best_state % 3) * 45; // Columna del estado mejor

    printf("Mejor estado encontrado: %d con Q-value: %.2f\n", best_state, max_q_value);
    // printf("Moviendo continuamente entre posición inicial Servo 1: %d, Servo 2: %d y mejor posición Servo 1: %d, Servo 2: %d\n",
    //        servo1_initial_position, servo2_initial_position,
    //        servo1_best_position, servo2_best_position);

    while(intentos<MAX_INTENTOS){
        // inicia en pos inicial
        printf("Iniciando en posición inicial...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        process_move_elbow(45);//lo mismo de arriba
        vTaskDelay(pdMS_TO_TICKS(1000));
        process_move_elbow(servo2_initial_position);
        vTaskDelay(pdMS_TO_TICKS(1000));
        process_move_shoulder(45);//para que el movimiento no sea de golpe (ya que no se lleva registro de donde esta el brazo en ese momento)
        vTaskDelay(pdMS_TO_TICKS(1000));
        process_move_shoulder(servo1_initial_position);
        estadoCrawler=obtenerEstadoCrawler();

        while(estadoCrawler!=(1)){
            estadoCrawler=obtenerEstadoCrawler();
            learn=obtenerDireccionCrawler();
            printf("Esperando a que se presione el botón START en el servidor para comenzar a aprender...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        estadoAprendiendoEjecutando=1;
        enviarEstadoCrawler();
        if(learn==FRONT_LEARN){
            while (estadoCrawler==1) {
                // Movimiento de inicial a mejor posición
                printf("Moviendo hacia la mejor posición...\n");
                while (servo1_initial_position != servo1_best_position) {
                    if (servo1_initial_position < servo1_best_position) {
                        servo1_initial_position += 45; // Incrementar hacia la mejor posición
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else if (servo1_initial_position > servo1_best_position) {
                        servo1_initial_position -= 45; // Decrementar hacia la mejor posición
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                while(servo2_initial_position != servo2_best_position){
                    if (servo2_initial_position < servo2_best_position) {
                        servo2_initial_position += 45; // Incrementar hacia la mejor posición
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else if (servo2_initial_position > servo2_best_position) {
                        servo2_initial_position -= 45; // Decrementar hacia la mejor posición
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                
                vTaskDelay(pdMS_TO_TICKS(2000));
                if(estadoCrawler!=1) break;
                printf("Moviendo de regreso a la posición inicial...\n");
                while (servo1_initial_position != 45) {
                    if (servo1_initial_position > 45) {
                        servo1_initial_position -= 45; // Decrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } 
                    else if (servo1_initial_position < 45) {
                        servo1_initial_position += 45; // Incrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                while(servo2_initial_position != 0){
                    if (servo2_initial_position > 0) {
                        servo2_initial_position -= 45; // Decrementar hacia la posición inicial
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } 
                    else if (servo2_initial_position < 0) {
                        servo2_initial_position += 45; // Incrementar hacia la posición inicial
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
            estadoCrawler=obtenerEstadoCrawler();
            enviarEstadoCrawler();
            }
        }
        else if(learn==BACK_LEARN){
            while (estadoCrawler==1) {
                // Movimiento de inicial a mejor posición
                printf("Moviendo hacia la mejor posición...\n");
                while (servo2_initial_position != servo2_best_position) {
                    if (servo2_initial_position < servo2_best_position) {
                        servo2_initial_position += 45; // Incrementar hacia la mejor posición
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else if (servo2_initial_position > servo2_best_position) {
                        servo2_initial_position -= 45; // Decrementar hacia la mejor posición
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                while(servo1_initial_position != servo1_best_position){
                    if (servo1_initial_position < servo1_best_position) {
                        servo1_initial_position += 45; // Incrementar hacia la mejor posición
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else if (servo1_initial_position > servo1_best_position) {
                        servo1_initial_position -= 45; // Decrementar hacia la mejor posición
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                
                vTaskDelay(pdMS_TO_TICKS(2000));
                if(estadoCrawler!=1) break;
                printf("Moviendo de regreso a la posición inicial...\n");
                while (servo1_initial_position != 90) {
                    if (servo1_initial_position > 90) {
                        servo1_initial_position -= 45; // Decrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } 
                    else if (servo1_initial_position < 90) {
                        servo1_initial_position += 45; // Incrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                while (servo2_initial_position != 90) {
                    if (servo2_initial_position > 90) {
                        servo2_initial_position -= 45; // Decrementar hacia la posición inicial
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } 
                    else if (servo2_initial_position < 90) {
                        servo2_initial_position += 45; // Incrementar hacia la posición inicial
                        process_move_elbow(servo2_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                if(estadoCrawler!=1) break;
                while(servo1_initial_position != 0){
                    if (servo1_initial_position > 0) {
                        servo1_initial_position -= 45; // Decrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } 
                    else if (servo1_initial_position < 0) {
                        servo1_initial_position += 45; // Incrementar hacia la posición inicial
                        process_move_shoulder(servo1_initial_position);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
            estadoCrawler=obtenerEstadoCrawler();
            enviarEstadoCrawler();
            }
            // printf("Servo 1: %d Servo 2: %d\n", servo1_initial_position, servo2_initial_position);
        vTaskDelay(pdMS_TO_TICKS(2000));
        }

        estadoAprendiendoEjecutando=-1;
        enviarEstadoCrawler();

        printf("Moviendo a la posición inicial...\n");
        while (servo1_initial_position != 45) {
            if (servo1_initial_position > 45) {
                servo1_initial_position -= 45; // Decrementar hacia la posición inicial
                process_move_shoulder(servo1_initial_position);
                vTaskDelay(pdMS_TO_TICKS(1000));
            } 
            else if (servo1_initial_position < 45) {
                servo1_initial_position += 45; // Incrementar hacia la posición inicial
                process_move_shoulder(servo1_initial_position);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        while (servo2_initial_position != 0) {
            if (servo2_initial_position > 0) {
                servo2_initial_position -= 45; // Decrementar hacia la posición inicial
                process_move_elbow(servo2_initial_position);
                vTaskDelay(pdMS_TO_TICKS(1000));
            } 
            else if (servo2_initial_position < 0) {
                servo2_initial_position += 45; // Incrementar hacia la posición inicial
                process_move_elbow(servo2_initial_position);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

    intentos++;
    }

}

/****************************************************************************************/
/******* FIN DE DECLARACIÓN DE FUNCIONES PARA EL ALGORTIRMO DE APRENDIZAJE Q ************/
/****************************************************************************************/




/**************** INICIO APP_MAIN **************************/

void app_main(void){

    // Inicialización de hardware (Wi-Fi, encoders, etc.)
    init_servo();
    set_pos(SHOULDER_MID_PULSE, ELBOW_MID_PULSE);
    set_servo_pulse(LEDC_SHOULDER_CHANNEL, SHOULDER_MID_PULSE);
    set_servo_pulse(LEDC_ELBOW_CHANNEL, ELBOW_MID_PULSE);

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
    
    
    //--------------------------------------------Q-LEARNING----------------------------------
    while(1){
        q_agent_init(&agent);
        int current_state = 0;  // Estado inicial
        int next_state = 0;
        int action = 0;
        int cont = 0;//contador para las iteraciones
        // int max_iterations = 100;// Número de iteraciones para el aprendizaje
        float inicio;//para el calculo del tiempo

        estadoAprendiendoEjecutando=-1;
        enviarEstadoCrawler();
        estadoCrawler=obtenerEstadoCrawler();

        while(estadoCrawler!=(1)){
            estadoCrawler=obtenerEstadoCrawler();
            learn=obtenerDireccionCrawler();
            printf("Esperando a que se presione el botón START en el servidor para comenzar a aprender...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        estadoAprendiendoEjecutando=0;
        enviarEstadoCrawler();
        inicio = dwalltime();
        while ((estadoCrawler==1)) {

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
            mover_servos(next_state);
            // simu_mover_servos(next_state, action);

            if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
                // 4. Obtener la recompensa (basado en los encoders)
                encoder_signal(&agent, current_state, next_state, &encoder1, &encoder2);
                // simu_encoder_signal(&agent, current_state, next_state);
        
                // 5. Actualizar la matriz Q
                q_agent_update(&agent, current_state, action, next_state);
                xSemaphoreGive(xMutex);
            }	

            // 6. Actualizar el estado actual para el siguiente ciclo
            current_state = next_state;

            // 7. Mostrar la matriz Q para depuración (opcional)
            //print_q_matrix(&agent);

            // if(cont*20%100 == 0)
            // {
                enviarDatosMatriz(agent.Q);
            // }

            // Incrementar el contador de iteraciones
            cont++;

            if(cont*20%100 == 0)
            {
                agent.epsilon = agent.epsilon * 0.99; // decrementa la exploración cada 5 iteraciones
            }

            // 8. Controlar el tiempo de ejecución con FreeRTOS
            enviarEstadoCrawler();
            vTaskDelay(pdMS_TO_TICKS(1000));  // Espera de medio segundo entre ciclos de aprendizaje
            estadoCrawler=obtenerEstadoCrawler();
        }
        // 9. Cuando se termine el aprendizaje, podemos salir del bucle

        learn=obtenerDireccionCrawler();
        crawler_listo = true;  // Señalamos que el aprendizaje ha terminado
        enviarDatosMatriz(agent.Q);
        printf("Proceso de aprendizaje completado.\n");
        estadoAprendiendoEjecutando=-1;//estado ejecutando aprendido
        enviarEstadoCrawler();

        //---calculo del tiempo-----------
        float fin = dwalltime();
        float tiempo_aprendizaje = fin - inicio;;
        printf("Tiempo de ejecución de un episodio: %.2f segundos\n", tiempo_aprendizaje);
        //-------------------------------

        if(learn==FRONT_LEARN){
            mover_servos_continuamente(45,0);//Ccomienza a moverse segun lo aprendido, a partir de un estado inicial (posiciones de servos)
        }
        if(learn==BACK_LEARN){
            mover_servos_continuamente(0,90);//Ccomienza a moverse segun lo aprendido, a partir de un estado inicial (posiciones de servos)
        }
    }
}
