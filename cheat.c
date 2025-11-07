#include <stdio.h>
#include <string.h>

/* --- Pico and FreeRTOS Headers --- */
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

/* --- Project-Specific Headers --- */
#include "tkjhat/sdk.h"
#include <tusb.h> // Required for USB
#include "usbSerialDebug/helper.h" // Required for dual-port serial

/* --- Definitions --- */
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1 // We send data to the Serial Client on Interface 1

/* --- Global Variables (for Synchronization) --- */

// This is our state machine
typedef enum {
    IDLE,       // Not recording
    RECORDING   // Recording IMU data
} programState;

// 'volatile' is critical here because this variable is shared
// between an ISR (btn_fxn) and a task (imu_task).
volatile programState state = IDLE; // Start in the IDLE state

// This is our synchronization mechanism (Req 2.5.4)
// It will hold pointers to our morse symbols (e.g., ".")
QueueHandle_t xMorseQueue;

/* --- Task Prototypes --- */
static void imu_task(void *arg);
static void sender_task(void *arg);
static void usb_task(void *arg);

/* --- Interrupt Handler (for Button) --- */

/**
 * @brief Interrupt Service Routine for Button 1 (Req 2.5.2)
 *
 * This function is called on a rising edge (press) of BUTTON1.
 * It sends a "space" symbol to the queue.
 */
static void btn_fxn(uint gpio, uint32_t eventMask) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Assume no task woken

    if (gpio == BUTTON1) {
        const char* space = " ";
        
        // Send the "space" pointer to the queue FROM THE ISR
        // This is interrupt-safe and wakes up the sender_task
        xQueueSendFromISR(xMorseQueue, &space, &xHigherPriorityTaskWoken);
    }
    else if (gpio == BUTTON2) {
        // NEW: Toggle the global recording state
        if (state == IDLE) {
            state = RECORDING;
        } else {
            state = IDLE;
        }
    }
    
    // If xHigherPriorityTaskWoken was set to pdTRUE by xQueueSendFromISR,
    // perform a context switch.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* --- Task Implementations --- */

/**
 * @brief Task 1: IMU Sensor Reader (Req 2.5.1, 2.5.5)
 *
 * Initializes and polls the IMU. Detects "flat" and "side"
 * positions and sends "." or "-" symbols to the queue.
 */
static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;

    // Initialize the IMU (from examples/hat_imu_ex)
    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
        // Send a debug message to USB port 0 (Serial Monitor)
        usb_serial_print("IMU Initialized! Press Button 2 to start recording.\n"); // CHANGED
    } else {
        usb_serial_print("IMU FAILED to initialize.\n");
    }

    // This is the main task loop
    for (;;) {
        // CHANGED: Only read and process data if we are in the RECORDING state
        if (state == RECORDING) {
            // Read sensor data
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                
                // This is our position detection logic (Req 2.5.5, 2.5.7)
                
                if (az < -0.9) {
                    // Device is FLAT (Z-axis pointing down at -1g)
                    const char* dash = "-";
                    xQueueSend(xMorseQueue, &dash, 0); // Send "dash" to queue
                    vTaskDelay(pdMS_TO_TICKS(400)); // Delay to prevent flooding
                } 
                else if (ay < -0.9) {
                    // Device is 90-DEGREES (Y-axis pointing down at -1g)
                    const char* dot = ".";
                    xQueueSend(xMorseQueue, &dot, 0); // Send "dot" to queue
                    vTaskDelay(pdMS_TO_TICKS(400)); // Delay to prevent flooding
                }
            }
        } // END of "if (state == RECORDING)"

        // Wait 100ms before the next loop iteration.
        // If IDLE, this task just sleeps, consuming minimal CPU.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Task 2: Serial Sender (Req 2.5.1, 2.5.8)
 *
 * Waits for symbols to arrive on the queue and sends them
 * to the workstation's Serial Client via USB (CDC 1).
 */
static void sender_task(void *arg) {
    (void)arg;
    
    char* symbol_to_send; // This will hold the pointer from the queue

    for (;;) {
        // This line blocks the task (0% CPU) until an item
        // is received from the queue. (Req 2.5.4)
        if (xQueueReceive(xMorseQueue, &symbol_to_send, portMAX_DELAY) == pdPASS) {
            
            // We received a symbol! Send it to the Serial Client.
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                tud_cdc_n_write_str(CDC_ITF_TX, symbol_to_send);
                tud_cdc_n_write_flush(CDC_ITF_TX);
            }

            // Also send it to our debug monitor (CDC 0)
            usb_serial_print(symbol_to_send);
        }
    }
}

/**
 * @brief Task 3: TinyUSB Background Task (Required)
 *
 * This task must run for the usb_serial_debug library to work.
 * It handles all low-level USB events.
 */
static void usb_task(void *arg) {
    (void)arg;
    while (1) {
        // This is the tinyusb task function
        tud_task();
        // DO NOT add vTaskDelay() here. tud_task() manages its own waiting.
    }
}


/* --- Main Function (Entry Point) --- */

int main() {
    // DO NOT call stdio_init_all(); it is disabled.
    
    // Initialize the HAT hardware (I2C, etc.)
    init_hat_sdk();
    sleep_ms(300); // Wait for hardware to settle

    // Initialize Buttons and attach our interrupt (Req 2.5.2)
    init_button1();
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &btn_fxn);

    // NEW: Initialize Button 2 and attach the SAME interrupt handler
    init_button2();
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_RISE, true, &btn_fxn);
    
    // Create the Queue (Req 2.5.4)
    // It can hold 10 "char*" pointers
    xMorseQueue = xQueueCreate(10, sizeof(char*));

    /* --- Create all our tasks --- */
    TaskHandle_t hImu, hSender, hUsb;

    xTaskCreate(imu_task, "IMU_Task", DEFAULT_STACK_SIZE, NULL, 2, &hImu);
    xTaskCreate(sender_task, "Sender_Task", DEFAULT_STACK_SIZE, NULL, 2, &hSender);
    
    // The USB task needs a high priority
    xTaskCreate(usb_task, "USB_Task", DEFAULT_STACK_SIZE, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
        // Pin the USB task to Core 0
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif

    // Initialize the USB stack (MUST be done just before scheduler)
    tusb_init();
    usb_serial_init(); // Initializes our debug print helper

    // Start FreeRTOS
    vTaskStartScheduler();

    // This line should never be reached
    while (1);
    return 0;
}