// 1o - Biblioteca de referencias para o desenvolvimento do codigo

// Adicionando referencias para chamadas da biblioteca C (printf/scanf)
#include <stdio.h>
// Adicionando referencias para asc onfiguracoes da SDK
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
#define LED_PLACA GPIO_NUM_2

// 3o - Variaveis globais (evitem se possivel, mas usem com cuidado)
// uint32_t = unsigned int 32 bits - variavel inteira, sem sinal, com dimensao de 32 bits
uint32_t contador = 0;  

// 4o - Prototipos de funcoes presentes no codigo (quando nao usado .h a parte)

// 5o Implementacao dos metodos a tarefas

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
    gpio_reset_pin(LED_PLACA);
    gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    // Tudo o que foi feito até aqui será executado umaunica vez quando o microcontrolador iniciar

    //Laco de execucao continua - ficara em execucao ate expressamente usarmos break, ou chp desligar

    while(1) {
        printf("Contador: %d \n", contador);
        // Desliga led?
        gpio_set_level(LED_PLACA, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // espera 1000ms = 1 s
        // Liga led?
        gpio_set_level(LED_PLACA, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // espera 1000ms = 1 s
        contador ++; // incrementa o contador a cada 2 segundos

        if (contador == 100){
           contador = 0; 
        } // incrementa o contador a cada 2 segundos
    }
}