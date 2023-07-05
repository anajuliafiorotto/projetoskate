// Includes
#include <ProjetoSkate_inferencing.h>
#include <Arduino_LSM9DS1.h> // Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_lsm9ds1/

// Definindo as Constantes
#define LED_R_PIN 22   // Pino do LED vermelho
#define LED_G_PIN 23   // Pino do LED verde
#define LED_B_PIN 24   // Pino do LED azul
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f

// Variavel privada
static bool debug_nn = false; // Mude para true para ver os valores raw do sinal

// Função para retornar a classe em português
const char* getClasse(int index) {
    switch (index) {
        case 0:
            return "Andando";
        case 1:
            return "Flip";
        case 2:
            return "Ollie";
        case 3:
            return "Parado";
        default:
            return "Desconhecido";
    }
}

// Arduino setup

void setup()
{
    Serial.begin(115200);
    // comente a linha abaixo para nao esperar a conexao USB (needed for native USB)
    // while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
        digitalWrite(LED_B_PIN, LOW); // Acende o LED Azul
        delay(400);
        digitalWrite(LED_B_PIN, HIGH); // Apaga o LED Azul
    }
    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME deve ser igual a 6 (3 eixos do acelerometro e 3 eixos do giroscopio)\n");
        return;
    }
}

// Retorna o sinal dos numeros
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

// Capta os dados e exibe a inteface
void loop()
{
    ei_printf("\nComecando a inferencia em 3 segundos...\n");
    delay(2900);

     // Acender o LED enquanto a amostra está sendo capturada
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, LOW);
    delay(100);

    ei_printf("Capturando a Amostra...\n");

    // Aloca um buffer para os valores que vamos ler do IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
        // Determina o proximo tick (e para em seguida)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]); // Le os dados do Acelerometro e coloca na posicao correta do Buffer
        IMU.readGyroscope(buffer[ix + 3], buffer[ix + 4], buffer[ix + 5]); // Le os dados do Giroscopio e coloca na posicao correta do Buffer

        for (int i = 0; i < 6; i++) {
            if (i < 3 && fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) { // Verifica se os dados do Acelerometro estao dentro da range permitida (2.0f)
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }
        // Converte os dados de G para m/s^2
        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Apagar o LED depois da captura
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
    delay(100);

    // Tranforma o buffer em um sinal que podemos classificar
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Falhou em criar o sinal a partir do buffer (%d)\n", err);
        return;
    }

    // Roda o classificador
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Falhou em rodar o classificador (%d)\n", err);
        return;
    }

    // Imprime as predicoes
    ei_printf("Predicoes ");
    ei_printf("(DSP: %d ms., Classificacao: %d ms.)",
        result.timing.dsp, result.timing.classification);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", getClasse(ix), result.classification[ix].value);
    }

    // Printa a classe detectada
    float max_value = -1;
    int max_index = -1;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > max_value) {
            max_value = result.classification[ix].value;
            max_index = ix;
        }
    }

 // Acender o LED dependendo do movimento
    switch (max_index) {
    case 1:  // Classe "Flip" -> Verde
        digitalWrite(LED_R_PIN, HIGH);
        digitalWrite(LED_G_PIN, LOW);
        digitalWrite(LED_B_PIN, HIGH);
        break;
    case 2:  // Classe "Ollie" -> Azul
        digitalWrite(LED_R_PIN, HIGH);
        digitalWrite(LED_G_PIN, HIGH);
        digitalWrite(LED_B_PIN, LOW);
        delay(500);
        break;
    case 3:  // Classe "Parado" -> Roxo
        digitalWrite(LED_R_PIN, LOW);
        digitalWrite(LED_G_PIN, HIGH);
        digitalWrite(LED_B_PIN, LOW);
        break;
    case 0:  // Classe "Andando" -> Vermelho
    default:
        digitalWrite(LED_R_PIN, LOW);
        digitalWrite(LED_G_PIN, HIGH);
        digitalWrite(LED_B_PIN, HIGH);
        break;
    }

    if (max_index != -1) {
        ei_printf("Classe detectada: %s\n", getClasse(max_index));
    }

    delay(1000); // Aguarda 1 segundo antes da próxima inferência
    // Apaga o LED
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
}
