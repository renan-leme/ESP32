// 1o - Biblioteca de referencias para o desenvolvimento do codigo

// Adicionando referencias para chamadas da biblioteca C (printf/scanf)
#include <stdio.h>
// Adicionando referencias para as configuracoes da SDK
#include "sdkconfig.h"
// Adicionando recurso do FreeRTOS para gerenciamento de tarefas (usaremos para delay)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// Adicionando recursos para referencia de informacoes para o sistema ESP32
#include "esp_system.h"
// Adicionando recursos para visualizacoes de informacoes da flash (SPI)
#include "esp_spi_flash.h"
// Adicionando recursos para manipular os sinais de entrada e saida (GPIO)
#include "driver/gpio.h"

// 2o - Deines para referencia de dados a serem usados mo programa
// Entenda o define como um 'apelido' - define apelido original
#define LED_PLACA    GPIO_NUM_2
#define BOTAO_1      GPIO_NUM_22
#define LED_CONTROLE GPIO_NUM_21

// 3o - Variaveis globais (evitem se possivel, mas usem com cuidado)
// uint32_t = unsigned int 32 bits - variavel inteira, sem sinal, com dimensao de 32 bits
uint32_t contador = 0; 
uint32_t botao_1_anterior = 1;
uint32_t estado_led = 0; 

// 4o - Prototipos de funcoes presentes no codigo (quando nao usado .h a parte)

// 5o Implementacao dos metodos a tarefas
// Callback para tratamento de interrupção de botão
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    //Verifica se o botao 1 foi a fonte da interrupcao
    //Como fazer debounce na leitura de interrupcao?
    if (BOTAO_1 == (uint32_t) arg)
    {
        if (gpio_get_level((uint32_t) arg) ==0)
        {
            contador++;
        }
    }
}

void app_main(void) {
    // Exibe a mensagem no terminal
    printf("Inicializando Esquenta ESP32...\n");

    // Obtemos informações do nosso chip:
    // Criamos um tipo para armazenar as informações
    esp_chip_info_t chip_info;
    // Carregamos as informacoes
    esp_chip_info(&chip_info);
    printf("Executando %s com %d CPU Cotes - WiFi%s%s \n",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/Ble" : ""
    );

    printf("Revisão de Silicio: %d \n", chip_info.revision);

    printf("%d MB de Flash %s \n", 
        spi_flash_get_chip_size()/(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa"
    );

    // Vamos configurar o uso do LED (GPIO2)
    // Metodo discreto de configuracao de GPIOs
    //gpio_reset_pin(LED_PLACA);
    //gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    // Metodo de configuracao de pinos usando o gpio_config_t
    gpio_config_t button_config = {
        .intr_type = GPIO_INTR_NEGEDGE, //Sem interrupção neste momento
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOTAO_1),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    //Configura o GPIO:
    gpio_config(&button_config);

    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE, //Sem interrupção neste momento
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PLACA) | (1ULL << LED_CONTROLE),
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    //Configura o GPIO:
    gpio_config(&led_config);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); //Estamos configurando a rotina de tratamento de interrupcao de baixa prioridade
    gpio_isr_handler_add(BOTAO_1, gpio_isr_handler, (void*) BOTAO_1);

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Valor do contador: %d\n", contador);
        gpio_set_level(LED_PLACA, estado_led);
        estado_led = !estado_led;
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