#include <unity.h>
#include <encoder.h>
#include <servo.h>
#include "main.h"
#include "driver/gpio.h"
#include "esp_timer.h" 



encoder_t encoder1;
encoder_t encoder2;
static volatile int64_t encoder1_last_time = 0;
static volatile int64_t encoder2_last_time = 0;
static volatile direction_t movement_direction = DIRECTION_STOPPED;
static volatile int last_encoder_triggered = 0; // 1 para encoder1, 2 para encoder2
static volatile int64_t last_trigger_time = 0;
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
        printf("Interrupción en encoder 1\n");
        last_encoder_triggered = 1;
    } else if (encoder->pin_out == ENCODER2_OUT) {
        if (last_encoder_triggered == 1 && (now - last_trigger_time) < 500000) { 
            // Si el último fue el encoder 1 y la diferencia de tiempo es pequeña
            movement_direction = DIRECTION_BACKWARD;
            printf("Interrupción en encoder 2\n"); 
        }
        last_encoder_triggered = 2;
    }
    
    // Actualizar el tiempo del último pulso
    last_trigger_time = now;
    encoder->last_pulse_time = now; // Registrar el tiempo del pulso
}
int32_t encoder_get_count(encoder_t *encoder) {
    return encoder->count;
}
void encoder_reset_count(encoder_t *encoder) {
    encoder->count = 0;
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

float calcular_disparidad(int contador1, int contador2) {
    int diferencia = abs((contador1/2) - (contador2/2)); // Diferencia absoluta entre ambos contadores
    int maximo = (contador1 > contador2) ? contador1 : contador2; // Mayor de los dos valores

    if (maximo == 0) {
        return 0.0; // Evitar división por cero
    }

    return ((float)diferencia / maximo) * 100.0; // Disparidad en porcentaje
}

float encoder_get_reward(encoder_t *encoder){
    printf("Contador real: %ld \n",encoder_get_count(encoder));
    int32_t real_value= (encoder->count)/2;
    printf("Contador final (divido por 2): %ld \n", real_value);
    float reward = real_value >= 3 ? 1.0 : (float)real_value / 3.0; //Acotado a 5 como max (rendijas de una vuelta)
    return reward;
}

/******************************************************************************/



/*---------------------TEST UNITARIOS ENCODERS-----------------------------*/
void test_encoder_init(){
    TEST_ASSERT_EQUAL(0,encoder_get_count(&encoder1));
    //TEST_ASSERT_EQUAL(0,encoder_get_count(&encoder2));
}

void test_encoder_get_count(){
    encoder1.count=5;
    encoder2.count=5;
    TEST_ASSERT_EQUAL(5,encoder_get_count(&encoder1));
    TEST_ASSERT_EQUAL(5,encoder_get_count(&encoder2));
}

void test_encoder_reset_count(){
    encoder_reset_count(&encoder1);
    encoder_reset_count(&encoder2);
    TEST_ASSERT_EQUAL(0,encoder_get_count(&encoder1));
    TEST_ASSERT_EQUAL(0,encoder_get_count(&encoder2));
}
void test_calcular_disparidad(){
    int valor1=3;
    int valor2=6;
    int resultado=calcular_disparidad(valor1,valor2);
    TEST_ASSERT_EQUAL(50,resultado);
}
void test_encoder_get_reward(){
    encoder1.count=10;
    float reward=encoder_get_reward(&encoder1);
    TEST_ASSERT_EQUAL_FLOAT(0.5,reward);
}
/*------------------ FIN TEST UNITARIOS ENCODERS -----------------------------*/



// Define the current_pos array to store servo positions
extern int current_pos[2];

/*--------------------- TEST UNITARIOS SERVOS --------------------------------*/
void test_set_servo_pulse(void) {
    // Prueba para verificar la configuración del pulso del servo
    process_move_shoulder(45);
    set_servo_pulse(LEDC_SHOULDER_CHANNEL, SHOULDER_MID_PULSE);
    TEST_ASSERT_EQUAL(SHOULDER_MID_PULSE, current_pos[0]);
    process_move_elbow(45);
    set_servo_pulse(LEDC_ELBOW_CHANNEL, ELBOW_MID_PULSE);
    TEST_ASSERT_EQUAL(ELBOW_MID_PULSE, current_pos[1]);
}
void test_process_move_shoulder(void) {
    // Prueba para verificar el movimiento del hombro
    process_move_shoulder(0);
    TEST_ASSERT_EQUAL(SHOULDER_MIN_PULSE, current_pos[0]);
    process_move_shoulder(45);
    TEST_ASSERT_EQUAL(SHOULDER_MID_PULSE, current_pos[0]);
    process_move_shoulder(90);
    TEST_ASSERT_EQUAL(SHOULDER_MAX_PULSE, current_pos[0]);
}
void test_process_move_elbow(void) {
    // Prueba para verificar el movimiento del codo
    process_move_elbow(0);
    TEST_ASSERT_EQUAL(ELBOW_MIN_PULSE, current_pos[1]);
    process_move_elbow(45);
    TEST_ASSERT_EQUAL(ELBOW_MID_PULSE, current_pos[1]);
    process_move_elbow(90);
    TEST_ASSERT_EQUAL(ELBOW_MAX_PULSE, current_pos[1]);
}
void test_set_pos(void) {
    // Prueba para verificar la configuración de la posición
    set_pos(SHOULDER_MID_PULSE, ELBOW_MID_PULSE);
    TEST_ASSERT_EQUAL(SHOULDER_MID_PULSE, current_pos[0]);
    TEST_ASSERT_EQUAL(ELBOW_MID_PULSE, current_pos[1]);
}
/*------------------ FIN TEST UNITARIOS SERVOS -----------------------------*/

// /*------------------ TEST UNITARIOS Q LEARNING -----------------------------*/

// // Prototipos de funciones
// void test_q_agent_init(void);
// void test_q_agent_select_action(void);
// void test_obtener_siguiente_estado(void);
// void test_get_valid_actions(void);

// // Inicializa el agente Q para las pruebas
// Q_Agent agent;


// // Prueba para inicializar el agente Q
// void test_q_agent_init(void) {
//     TEST_ASSERT_EQUAL_FLOAT(0.9, agent.epsilon);
//     TEST_ASSERT_EQUAL_FLOAT(0.5, agent.alpha);
//     TEST_ASSERT_EQUAL_FLOAT(0.7, agent.gamma);

//     for (int i = 0; i < NUM_ESTADOS; i++) {
//         for (int j = 0; j < NUM_ESTADOS; j++) {
//             TEST_ASSERT_EQUAL_FLOAT(0.0f, agent.Q[i][j]);
//             TEST_ASSERT_EQUAL_FLOAT(0.0f, agent.R[i][j]);
//         }
//     }
// }

// // Prueba para seleccionar una acción válida
// void test_q_agent_select_action(void) {
//     // Establecer un estado conocido
//     int estado = 0; // Estado inicial

//     // Asignar valores Q para las acciones válidas
//     agent.Q[estado][ACTION_SERVO1_FORWARD] = 1.0f;
//     agent.Q[estado][ACTION_SERVO1_BACKWARD] = 0.5f;

//     // Llamar a la función para seleccionar acción
//     int action = q_agent_select_action(&agent, estado);

//     // Verificar que se selecciona la acción con el mayor valor Q
//     TEST_ASSERT_TRUE(action == ACTION_SERVO1_FORWARD || action == ACTION_SERVO1_BACKWARD);
// }

// // Prueba para obtener el siguiente estado basado en la acción
// void test_obtener_siguiente_estado(void) {
//     int current_state = 0; // Servo1 en 0 grados y Servo2 en 0 grados

//     // Probar avance del servo 1
//     int next_state = obtener_siguiente_estado(current_state, ACTION_SERVO1_FORWARD);
//     TEST_ASSERT_EQUAL(3, next_state); // Servo1 debería estar en 45 grados

//     // Probar retroceso del servo 1
//     next_state = obtener_siguiente_estado(current_state, ACTION_SERVO1_BACKWARD);
//     TEST_ASSERT_EQUAL(0, next_state); // Debería permanecer en 0 grados

//     // Probar avance del servo 2
//     next_state = obtener_siguiente_estado(current_state, ACTION_SERVO2_FORWARD);
//     TEST_ASSERT_EQUAL(1, next_state); // Servo2 debería estar en 45 grados

//     // Probar retroceso del servo 2
//     next_state = obtener_siguiente_estado(current_state, ACTION_SERVO2_BACKWARD);
//     TEST_ASSERT_EQUAL(0, next_state); // Debería permanecer en 0 grados
// }

// // Prueba para obtener acciones válidas basadas en las posiciones de los servos
// void test_get_valid_actions(void) {
//     int size;
    
//     // int* actions = get_valid_actions(90, 90, &size); // Ambos servos en su posición máxima

//     // TEST_ASSERT_EQUAL(0, size); // No debería haber acciones válidas

//     // actions = get_valid_actions(45, 45, &size); // Ambos servos en posición media

//     // TEST_ASSERT_TRUE(sSize > 0); // Debería haber acciones válidas disponibles
// }

// /*---------------- FIN TEST UNITARIOS Q LEARNING -----------------------------*/

// /*------------------- TEST UNITARIOS SERVIDOR -------------------------------*/

// // Mocks para las funciones de HTTP
// esp_err_t mock_http_post(const char *url, const char *post_data) {
//     // Simular un POST exitoso
//     return ESP_OK; 
// }

// esp_err_t mock_http_get(const char *url) {
//     // Simular un GET exitoso y devolver un estado ficticio
//     if (strcmp(url, "http://192.168.4.2:8000/get_start_state/") == 0) {
//         return 1; // Simular que el crawler ha comenzado
//     }
//     return -1; // Simular error para otras URL
// }

// // Reemplazar las funciones originales por los mocks
// #define http_post mock_http_post
// #define http_get mock_http_get

// // Prototipos de funciones
// void test_http_post(void);
// void test_http_get(void);
// void test_enviarEstadoCrawler(void);
// void test_enviarDatosMatriz(void);

// // Prueba para la función http_post
// void test_http_post(void) {
//     const char *url = "http://192.168.4.2:8000/api/recibir_estado/";
//     const char *post_data = "{ \"estado\": 1 }";

//     esp_err_t result = http_post(url, post_data);
    
//     TEST_ASSERT_EQUAL(ESP_OK, result); // Verificar que el POST fue exitoso
// }

// // Prueba para la función http_get
// void test_http_get(void) {
//     const char *url = "http://192.168.4.2:8000/get_start_state/";
    
//     int estado = http_get(url);
    
//     TEST_ASSERT_EQUAL(1, estado); // Verificar que el estado devuelto es 1 (empezar)
// }

// // Prueba para enviar el estado del crawler
// void test_enviarEstadoCrawler(void) {
//     estadoAprendiendoEjecutando = 1; // Estado a enviar

//    // enviarEstadoCrawler(); // Llamar a la función

//     // Aquí puedes verificar que se haya llamado a http_post con los datos correctos.
//     // Esto puede requerir un mock más sofisticado si necesitas capturar los datos enviados.
// }

// // Prueba para enviar datos de la matriz al servidor
// void test_enviarDatosMatriz(void) {
//     float matriz[9][9] = { {0} }; // Inicializar matriz con ceros

//    // enviarDatosMatriz(matriz); // Llamar a la función

//     // Verificar que se haya llamado a http_post con los datos correctos.
// }


// /*---------------- FIN TEST UNITARIOS SERVIDOR -------------------------------*/




void setUp(void){
    encoder_init(&encoder1, ENCODER1_OUT);
    //encoder_init(&encoder2, ENCODER2_OUT);
    init_servo();
  //  q_agent_init(&agent);
}
void tearDown(void){

}


int runUnityTests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_encoder_init);
    RUN_TEST(test_encoder_get_count);
    RUN_TEST(test_encoder_reset_count);
    RUN_TEST(test_calcular_disparidad);
    RUN_TEST(test_encoder_get_reward);
    RUN_TEST(test_set_servo_pulse);
    RUN_TEST(test_process_move_shoulder);
    RUN_TEST(test_process_move_elbow);
    RUN_TEST(test_set_pos);
    // RUN_TEST(test_q_agent_init);
    // RUN_TEST(test_q_agent_select_action);
    // RUN_TEST(test_obtener_siguiente_estado);
    // RUN_TEST(test_get_valid_actions);
    // RUN_TEST(test_http_post);
    // RUN_TEST(test_http_get);
    // RUN_TEST(test_enviarEstadoCrawler);
    // RUN_TEST(test_enviarDatosMatriz);
    return UNITY_END();
}

int app_main(void) {
    return runUnityTests();
}