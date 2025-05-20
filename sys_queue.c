#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "lib/ws2812.pio.h"
#include "pico/bootrom.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

// Definição de botões
#define btn_a 5
#define btn_b 6

// Definição de parâmetros para a matriz de LEDS
#define NUM_PIXELS 25 // Número de LEDs na matriz 
#define IS_RGBW false // Define se os LEDs são RGBW ou apenas RGB
#define WS2812_PIN 7 // Pino onde os LEDs WS2812 estão conectados

// Definição de pinos do LED RGB
#define led_pin_red 13

// Definição de parâmetros para o Display 
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Definição do buzzer
#define buzzer_pin_l 10
#define buzzer_pin_r 21

// Definição do ADC
#define ADC_JOYSTICK_X 26
#define ADC_JOYSTICK_Y 27

typedef struct{
    uint16_t volum_chuva;
    uint16_t lvl_agua;
} joystick_data_t;

QueueHandle_t xQueueJoystickData;

// Variáveis Globais
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)

// Tarefa para o Joystick
void vJoystickTask(void *params){
    adc_gpio_init(ADC_JOYSTICK_Y);
    adc_gpio_init(ADC_JOYSTICK_X);
    adc_init();

    joystick_data_t joydata;

    while(true){
        adc_select_input(0);
        joydata.volum_chuva = adc_read()/40.95;

        adc_select_input(1);
        joydata.lvl_agua = adc_read()/40.95;

        xQueueSend(xQueueJoystickData, &joydata, 0);
        vTaskDelay(pdTICKS_TO_MS(50));
    }
}

// Tarefa para o LED RGB
void vLedsTask(void *params){
    // Ativando as GPIOs do LED RGB
    gpio_init(led_pin_red);
    gpio_set_dir(led_pin_red, GPIO_OUT);

    joystick_data_t joydata;

    while (true){
        if(xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) && (joydata.lvl_agua >= 70 || joydata.volum_chuva >= 80)){
            gpio_put(led_pin_red, true); // Liga o vermelho
            vTaskDelay(pdTICKS_TO_MS(100));
            gpio_put(led_pin_red, false); // Desliga o vermelho
            vTaskDelay(pdTICKS_TO_MS(100));
        } else {
            gpio_put(led_pin_red, false); // Desliga o vermelho
        }
    }
}

// Tarefa para a Matriz de LEDS
void vMatrizTask(void *params){
    // Configuração da matriz de leds
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    // Buffer para desenhar na Matriz
    bool led_buffer[] = { 
        1, 0, 0, 0, 1, 
        0, 1, 0, 1, 0, 
        0, 0, 1, 0, 0, 
        0, 1, 0, 1, 0, 
        1, 0, 0, 0, 1
    };
    
    joystick_data_t joydata;

    while (true){
        if(xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) && (joydata.lvl_agua >= 70 || joydata.volum_chuva >= 80)){
            for (int i = 0; i < NUM_PIXELS; i++) {
                if (led_buffer[i]) {
                    pio_sm_put_blocking(pio0, 0, 0x001000 << 8u);
                } else {
                    pio_sm_put_blocking(pio0, 0, 0 << 8u); // Apaga os outros LEDs
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            for (int i = 0; i < NUM_PIXELS; i++) {
                pio_sm_put_blocking(pio0, 0, 0 << 8u); // Apaga os outros LEDs
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            for (int i = 0; i < NUM_PIXELS; i++) {
                    pio_sm_put_blocking(pio0, 0, 0 << 8u); // Apaga os outros LEDs
            }   
        }
    }
}

// Tarefa para o Display
void vDisplayTask(void *params){
    // Configuração do display
    i2c_init(I2C_PORT, 400 * 1000);                               // I2C Initialisation. Using it at 400Khz.
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    joystick_data_t joydata;
    bool cor = true;

    char chuva[25];
    char agua[25];    
    
    while (true){
        if(xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) && (joydata.lvl_agua >= 70 || joydata.volum_chuva >= 80)){
            // Converte os valores numéricos para string
            sprintf(chuva, "Vol chuva: %d%%", joydata.volum_chuva);
            sprintf(agua, "Niv agua: %d%%", joydata.lvl_agua);

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_draw_string(&ssd, "Modo Alerta", 1, 5);
            ssd1306_draw_string(&ssd, chuva, 1, 20);
            ssd1306_draw_string(&ssd, agua, 1, 35);
            ssd1306_draw_string(&ssd, "ALERTA", 5, 50);
            ssd1306_send_data(&ssd);    // Atualiza o display
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            // Converte os valores numéricos para string
            sprintf(chuva, "Vol chuva: %d%%", joydata.volum_chuva);
            sprintf(agua, "Niv agua: %d%%", joydata.lvl_agua);

            ssd1306_fill(&ssd, !cor);                          // Limpa o display
            ssd1306_draw_string(&ssd, "Modo Normal", 1, 5);
            ssd1306_draw_string(&ssd, chuva, 1, 20);
            ssd1306_draw_string(&ssd, agua, 1, 35);
            ssd1306_send_data(&ssd);    // Atualiza o display
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Tarefa para o Buzzer
void vBuzzerTask(void *params){
    // Configuração do PWM do Buzzer
    gpio_set_function(buzzer_pin_l, GPIO_FUNC_PWM);
    uint slice_buz_1 = pwm_gpio_to_slice_num(buzzer_pin_l);
    pwm_set_clkdiv(slice_buz_1, 40);
    pwm_set_wrap(slice_buz_1, 12500);
    pwm_set_enabled(slice_buz_1, true);  

    gpio_set_function(buzzer_pin_r, GPIO_FUNC_PWM);
    uint slice_buz_2 = pwm_gpio_to_slice_num(buzzer_pin_r);
    pwm_set_clkdiv(slice_buz_2, 40);
    pwm_set_wrap(slice_buz_2, 25000);
    pwm_set_enabled(slice_buz_2, true);  
    
    joystick_data_t joydata;

    while (true){
        if(xQueueReceive(xQueueJoystickData, &joydata, portMAX_DELAY) && (joydata.lvl_agua >= 70 || joydata.volum_chuva >= 80)){
            pwm_set_gpio_level(buzzer_pin_l, 60);  
            pwm_set_gpio_level(buzzer_pin_r, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            pwm_set_gpio_level(buzzer_pin_l, 0); 
            pwm_set_gpio_level(buzzer_pin_r, 60); 
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            pwm_set_gpio_level(buzzer_pin_l, 0);  
            pwm_set_gpio_level(buzzer_pin_r, 0);         
        }
    }
}

// Trecho para modo BOOTSEL com botão B
void gpio_irq_handler(uint gpio, uint32_t events){
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if(current_time - last_time > 200000){
        last_time = current_time;
        
        if (gpio == btn_b){
            reset_usb_boot(0,0);
        }
    }

}

// Função principal
int main(){
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(btn_b);
    gpio_set_dir(btn_b, GPIO_IN);
    gpio_pull_up(btn_b);

    gpio_set_irq_enabled_with_callback(btn_b, GPIO_IRQ_EDGE_FALL,true, &gpio_irq_handler);
    // Fim do trecho para modo BOOTSEL com botão B

    // Criação da fila
    xQueueJoystickData = xQueueCreate(10, sizeof(joystick_data_t));

    // Criação das tasks
    xTaskCreate(vJoystickTask, "Joystick Task", 256, NULL, 1, NULL);
    xTaskCreate(vLedsTask, "LED Task", 512, NULL, 1, NULL);
    xTaskCreate(vMatrizTask, "Matriz LED Task", 512, NULL, 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 512, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);

    // Inicializa o agendador
    vTaskStartScheduler();
    panic_unsupported();
}
