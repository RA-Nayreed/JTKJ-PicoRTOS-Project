/*
 * JTKJ Module2 Final Project - Tier 1
 * Authors: Rezwan Ahmad Nayreed, Joni Lahtinen and Aati Samuel Strömmer	
 * Date: 2025-11-10
 * Hardware: RP2040 + TKJ HAT (ICM42670), two buttons
 * Summary: IMU-based Morse transmitter over USB CDC using FreeRTOS
 */


#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <tusb.h>
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

#define CDC_ITF_TX 1
#define QUEUE_LEN  32

typedef enum { WAITING, RECORDING } programState;

static volatile programState state = WAITING;
static QueueHandle_t xMorseQueue;
// Task Prototypes 
static void imu_task(void *arg);
static void print_task(void *arg);
static void usb_task(void *arg);
static void gpio_callback(uint gpio, uint32_t events);
static inline void enqueue_symbol_from_isr(char c);

// Helpers
static inline void display_symbol(char c) {
    // send to CDC1 (data)
    char out[2] = { c, '\0' };
    if (tud_cdc_n_connected(CDC_ITF_TX)) {
        tud_cdc_n_write_str(CDC_ITF_TX, out);
        tud_cdc_n_write_flush(CDC_ITF_TX);
    }
    char dbg[2] = { c, '\0' };
    usb_serial_print(dbg);
}

// Tasks
static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;

    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
        usb_serial_print("IMU Initialized! Press Button 2 to start recording.\n");
    } else {
        usb_serial_print("IMU FAILED to initialize.\n");
    }

    for (;;) {
        if (state == RECORDING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

                // --- detection logic ---
                if (az < -0.8f) {
                    char dash = '-';
                    (void)xQueueSend(xMorseQueue, &dash, 0);
                    vTaskDelay(pdMS_TO_TICKS(800));  // flood control
                } else if (ay < -0.8f) {
                    char dot = '.';
                    (void)xQueueSend(xMorseQueue, &dot, 0);
                    vTaskDelay(pdMS_TO_TICKS(800));  // flood control
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300)); // idle sleep / polling cadence
    }
}

static void print_task(void *arg) {
    (void)arg;
    char c;
    for (;;) {
        // block until a symbol arrives
        if (xQueueReceive(xMorseQueue, &c, portMAX_DELAY) == pdTRUE) {
            // We always print what is produced, independent of current state;
            // alternatively, gate by state if you want silence when paused.
            display_symbol(c);
        }
    }
}

static void usb_task(void *arg) {
    (void)arg;
    while (1) {
        tud_task(); // TinyUSB polling
        // Optional: vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// GPIO / ISR 
static inline void enqueue_symbol_from_isr(char c) {
    BaseType_t hpw = pdFALSE;
    (void)xQueueSendFromISR(xMorseQueue, &c, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void gpio_callback(uint gpio, uint32_t events) {
    if ((gpio == BUTTON1) && (events & GPIO_IRQ_EDGE_RISE)) {
        if (state == RECORDING) {
            enqueue_symbol_from_isr(' ');
        }
    } else if ((gpio == BUTTON2) && (events & GPIO_IRQ_EDGE_RISE)) {
        state = (state == RECORDING) ? WAITING : RECORDING;
    }
}


int main(void) {
    // Initializations
    init_hat_sdk();
    sleep_ms(300);

    init_button1();
    init_button2();

    // one global IRQ callback in Pico SDK
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    xMorseQueue = xQueueCreate(QUEUE_LEN, sizeof(char));

    // Create tasks
    xTaskCreate(imu_task,   "IMU",   2048, NULL, 2, NULL);
    xTaskCreate(print_task, "Print", 1024, NULL, 2, NULL);

    TaskHandle_t hUsb;
    xTaskCreate(usb_task, "USB", 2048, NULL, 3, &hUsb);
#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
#endif

    tusb_init();
    usb_serial_init();

    vTaskStartScheduler();
    return 0;
}