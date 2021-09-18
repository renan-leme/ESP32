// Adicionando referencias para chamadas da biblioteca C (printf/scanf)
#include <stdio.h>
// Adicionando referencias para as configuracoes da SDK
#include "sdkconfig.h"
// Adicionando recursos do FreeRTOS para gerenciamento de tarefas (usaremos para delay)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Adicionando recursos para referencia de informacoes do sistema esp32
#include "esp_system.h"
// Adicionando recursos para visualizar informacoes da Flash (SPI)
#include "esp_spi_flash.h"
// Adicionando biblioteca de driver pra manipular os sinais de entrada e saida (GPIO)
#include "driver/gpio.h"
// Adicionando biblioteca de driver de timer da espressif para esp32
#include "driver/timer.h"

// 2o - Defines para referencia de dados a serem usados no programa
// Entenda o define como "apelido" - define apelido original
#define LED_PLACA       GPIO_NUM_2
#define BOTAO_1         GPIO_NUM_22
#define LED_CONTROLE    GPIO_NUM_21

#define TIMER_DIVIDER   (16)                                // divisor de clock de hw em fator 16x
#define TIMER_SCALE     (TIMER_BASE_CLK / TIMER_DIVIDER)    // timer_base_clock/16 converte em segundos
// timer_base_clk eh 80 MHz por padrao

// 3o - Variaveis globais (evitem se possivel, mas usem com cuidado)
// uint32_t = unsigned int 32 bits - variavel inteira, sem sinal, com dimensao de 32 bits
uint32_t contador = 0;
bool led_controle = 0;

// Estrutura de referencia para setup e informacao do timer
typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} esp_timer_info_t;

// Estrutura de referencia para eventos de timer ("opcional")
typedef struct {
   esp_timer_info_t info;
   uint64_t timer_counter_value;
} esp_timer_event_t;

// 4o - Prototipos de funcoes presentes no codigo (quando nao usado .h a parte)

// 5o - Implementacao dos metodos e tarefas -----------------------------------

// Callback para tratamento da interrupcao do botao
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    // Verifica se o botao 1 foi a fonte da interrupcao
    // 1 - Como fazer debounce na leitura da interrupcao de gpio?
    if (BOTAO_1 == (uint32_t) arg) //typecast para tipo uint32_t - inteiro usado na definicao dos gpios
    {
        if (gpio_get_level((uint32_t) arg) == 0) 
        {
            contador++;
        }
    }
}

// Callback para tratamento de interrupcao de Timer
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    esp_timer_info_t *info = (esp_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
    
    // Carrega valores de evento de timer = podemos usar isso mais tarde! Por hora eh soh referencia!
    esp_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };
    
    // Se nao temos auto_reload configurado
    if (!info->auto_reload) {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    // Pisca led na ocorrencia de interrupcao de timer -- ACAO no determinado tempo (ou qtd de eventos) desejado
    gpio_set_level(LED_CONTROLE, led_controle);
    led_controle = !led_controle; //inverte o estado do LED pra proxima vez

    return high_task_awoken == pdTRUE; // Retorna se precisamos "ceder" ao final da rotina de interrupcao 
    // Util em troca de contexto
}


// Rotina de execucao principal de codigo (CPU_APP - ESP32)
void app_main(void) {
    // Exibe a mensagem no terminal
    printf("Inicializando Esquenta ESP32... \n");

    // Obtemos informacoes do nosso chip:
    // criamos um tipo para armazenar as informacoes
    esp_chip_info_t chip_info;
    // carregamos as informacoes
    esp_chip_info(&chip_info);

    printf("Executando %s com %d CPU Cores - WiFi %s %s \n", 
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    
    printf("Revisao de Silicio: %d \n", chip_info.revision);

    printf("%d MB de Flash %s \n", 
        (spi_flash_get_chip_size() / (1024*1024)),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa");

    // Vamos configurar o uso do LED (GPIO 2)
    // "Metodo discreto" de configuracao de GPIOs
    //gpio_reset_pin(LED_PLACA);
    //gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    // Metodo de configuracao de pinos usando o gpio_config_t
    // Primeiramente configuramos o botao
    gpio_config_t button_config = {
        .intr_type = GPIO_INTR_NEGEDGE, // interrupcao em borda negativa 1->0
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOTAO_1),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    // configura o gpio
    gpio_config(&button_config);

    // Vamos configurar "os leds"
    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE, //sem interrupcao
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PLACA) | (1ULL << LED_CONTROLE),
        .pull_down_en = 0
    };
    gpio_config(&led_config);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // Estamos configurando uma rotina de tratamento de interrupcao de baixa prioridade
    gpio_isr_handler_add(BOTAO_1, gpio_isr_handler, (void*) BOTAO_1);

    // Vamos configurar nosso timer para execucao
    timer_config_t config = {
        .divider = TIMER_DIVIDER,        // fator de escala - divisor por 16
        .counter_dir = TIMER_COUNT_UP,   // timer como contador crescente
        .counter_en = TIMER_PAUSE,       // timer comeca parado
        .alarm_en = TIMER_ALARM_EN,      // timer com alarme (a.k.a interrupcao "chique")
        .auto_reload = true,             // queremos auto-reload! por hora...
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    uint32_t intervalo_em_segundos = 5;

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); // comeca a contar em 0
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, intervalo_em_segundos * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0); // habilita interrupcao do timer

    // calloc - aloca memoria para um vetor de elementos e inicializa todo mundo em 0
    esp_timer_info_t *timer_info = calloc(1, sizeof(esp_timer_info_t));
    timer_info->timer_group = TIMER_GROUP_0;
    timer_info->timer_idx = TIMER_0;
    timer_info->auto_reload = true;
    // 5 segundos pra comecar os alarmes ou geracao de interrupcao
    timer_info->alarm_interval = intervalo_em_segundos; 
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

    // Ate agora TUDO foi perfumaria e ritual de configuracao
    // A partir desse exato momento, o TIMER comeca a contar independente da CPU, e gerando interrupcoes assim em diante
    // Conforme configurado
    timer_start(TIMER_GROUP_0, TIMER_0);

    uint8_t estado_led = 0;

    while(1) 
    {
        vTaskDelay( 1000 / portTICK_PERIOD_MS ); // delay de 30ms na API do FreeRTOS, pra debounce de tecla
        printf("Contador: %d\n", contador);
        gpio_set_level(LED_PLACA, estado_led);
        estado_led = !estado_led; // 0->1, 1->0, 1->0...
    }
}

// while(1){
//         vTaskDelay(30 / portTICK_PERIOD_MS);
//         if (botao_1_anterior == 1 && gpio_get_level(BOTAO_1) == 0)
//         {
//             contador++;
//         }
//         printf("Valor do contador: %d\n", contador);
//         botao_1_anterior = gpio_get_level(BOTAO_1);
// }