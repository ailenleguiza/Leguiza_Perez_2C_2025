/*! @mainpage Leguiza_Perez_EMG
 *
 * @section genDesc General Description
 *
 * Este proyecto ejemplifica el uso del módulo de comunicación 
 * Bluetooth Low Energy (BLE), junto con el cálculo de la FFT 
 * de una señal EMG adquirida desde un canal analógico.
 * Permite graficar en una aplicación móvil la FFT de la señal.
 *
 * @section changelog Changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 22/10/2025 | Código adaptado a EMG real con buffer circular  |
 *
 * @author Florencia Ailen Leguiza Scandizzo
 *         María de los Ángeles Perez
 *
 */

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "ble_mcu.h"
#include "delay_mcu.h"

#include "fft.h"
#include "iir_filter.h"

#include "analog_io_mcu.h"
/*#include "uart_mcu.h"*/
#include "timer_mcu.h"

/*==================[macros and definitions]=================================*/
#define CONFIG_BLINK_PERIOD 500
#define LED_BT	            LED_1
#define BUFFER_SIZE         256     // Ventana FFT
#define EMG_BUFFER_LEN      1024    // Buffer circular
#define SAMPLE_FREQ	        220     // Hz
#define ADC_CH_EMG          CH1

/*==================[internal data definition]===============================*/
static float emg_buffer[EMG_BUFFER_LEN];   // buffer circular para EMG
static uint16_t write_index = 0;
static uint16_t sample_count = 0;

static float emg_window[BUFFER_SIZE];       // ventana para FFT
static float emg_filt[BUFFER_SIZE];
static float emg_fft[BUFFER_SIZE/2];
static float emg_filt_fft[BUFFER_SIZE/2];
static float f[BUFFER_SIZE/2];

TaskHandle_t emg_task_handle = NULL;

/*==================[internal functions declaration]=========================*/
/**
 * @brief Función que se ejecuta al recibir datos por BLE.
 *
 * @param data      Puntero a array de datos recibidos
 * @param length    Longitud del array de datos recibidos
 */
void read_data(uint8_t * data, uint8_t length){
	if(data[0] == 'R'){
        xTaskNotifyGive(emg_task_handle);
    }
}

/**
 * @brief Escribe una muestra en el buffer circular EMG.
 *
 * @param sample Valor leído del ADC
 */
static void CircularBufferWrite(float sample){
    emg_buffer[write_index] = sample;
    write_index = (write_index + 1) % EMG_BUFFER_LEN;
    if(sample_count < EMG_BUFFER_LEN){
        sample_count++;
    }
}

/**
 * @brief Copia las últimas BUFFER_SIZE muestras desde el buffer circular
 *        hacia la ventana de procesamiento para FFT.
 */
static void CircularBufferReadWindow(float *window){
    /*uint16_t start = (write_index + EMG_BUFFER_LEN - BUFFER_SIZE) % EMG_BUFFER_LEN;
    for(int i=0; i<BUFFER_SIZE; i++){
        window[i] = emg_buffer[(start + i) % EMG_BUFFER_LEN];
    }*/
    // Verificar que tenemos suficientes muestras
    if(sample_count < BUFFER_SIZE) {
        return; // O manejar el error
    }
    
    // Calcular inicio de las últimas BUFFER_SIZE muestras
    uint16_t start = (write_index - BUFFER_SIZE + EMG_BUFFER_LEN) % EMG_BUFFER_LEN;
    
    for(int i = 0; i < BUFFER_SIZE; i++){
        window[i] = emg_buffer[(start + i) % EMG_BUFFER_LEN];
    }
}

/**
 * @brief Tarea principal EMG: adquiere muestras, filtra, calcula FFT y envía por BLE.
 */
static void EMGTask(void *pvParameter){
    char msg[64];
    float sample;
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Tomar BUFFER_SIZE muestras desde buffer circular
        CircularBufferReadWindow(emg_window);

        // Filtros
        HiPassFilter(emg_window, emg_filt, BUFFER_SIZE);
        LowPassFilter(emg_filt, emg_filt, BUFFER_SIZE);

        // FFT
        FFTMagnitude(emg_window, emg_fft, BUFFER_SIZE);
        FFTMagnitude(emg_filt, emg_filt_fft, BUFFER_SIZE);
        FFTFrequency(SAMPLE_FREQ, BUFFER_SIZE, f);

        // Envío BLE
        for(int i=0; i<BUFFER_SIZE/2; i++){
            sprintf(msg, "*HX%2.2fY%2.2f,X%2.2fY%2.2f*\n", f[i], emg_fft[i], f[i], emg_filt_fft[i]);
            BleSendString(msg);
            /* agregar linea para poder graficar la fft filtrada tambien copiando lo de arriba per ocambiando */
        }
    }
}

/**
 * @brief Timer ISR: lee ADC y guarda en buffer circular
 */
void EMG_TimerISR(void *param){
    uint16_t adc_val;
    AnalogInputReadSingle(ADC_CH_EMG, &adc_val);
    CircularBufferWrite((float)adc_val);
}

/*==================[external functions definition]==========================*/
void app_main(void){
    // Inicializaciones
    LedsInit();
    FFTInit();
    LowPassInit(SAMPLE_FREQ, 30, ORDER_2);
    HiPassInit(SAMPLE_FREQ, 1, ORDER_2);

    // BLE
    ble_config_t ble_configuration = {
        "ESP_EMG",
        read_data
    };
    BleInit(&ble_configuration);

    // ADC
    analog_input_config_t adc_config = {
        .input = ADC_CH_EMG,
        .mode = ADC_SINGLE,
        .func_p = NULL,
        .param_p = NULL,
        .sample_frec = SAMPLE_FREQ
    };
    AnalogInputInit(&adc_config);

    // Timer para muestreo EMG
    timer_config_t emg_timer = {
        .timer = TIMER_A,
        .period = 1000000 / SAMPLE_FREQ, // en us
        .func_p = EMG_TimerISR,
        .param_p = NULL
    };
    TimerInit(&emg_timer);
    TimerStart(TIMER_A);

    // Tarea EMG
    xTaskCreate(&EMGTask, "EMG", 2048, NULL, 5, &emg_task_handle);

    // Loop principal: indica estado BLE
    while(1){
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        switch(BleStatus()){
            case BLE_OFF:
                LedOff(LED_BT);
            break;
            case BLE_DISCONNECTED:
                LedToggle(LED_BT);
            break;
            case BLE_CONNECTED:
                LedOn(LED_BT);
            break;
        }
    }
}
