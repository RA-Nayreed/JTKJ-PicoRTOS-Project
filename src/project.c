#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "tkjhat/sdk.h"
#include <inttypes.h>
// Copied and pasted these from main.c

#include <tusb.h>

/* --- Definitions --- */
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1 // We send data to the Serial Client on Interface 1


// This file to be used for the project itself. Main.c was exercise session 2.
// Although the tasks are missing from it because it was never pushed to the repository.

typedef enum {
    WAITING,
    DISPLAYING,
    DATA_READY,
    RECORDING
} programState;

char current_character;
// char current_str[]; // string of all the characters received so far (TIER 2)
programState state = WAITING;

// Prototypes
void displayOutput(char current_char);
void detectMovement();
void button_interrupt_space(uint gpio, uint32_t eventMask);
void button_interrupt_record(uint gpio, uint32_t eventMask);
void sensor_task();
void button_task();



// I'm guessing we should make a statemachine task that uses switch statements
// to determine which state we're in?




void detectMovement() {
    /*
    Detect movement from the device and save the corresponding character to current_character.
    */
    float ax, ay, az, gx, gy, gz, t;
      if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

        if (az < -0.8) {
            current_character = '-';
        }
        else if (ay < -0.8) {
            current_character = '.';
        }
      }
}


void button_interrupt_space(uint gpio, uint32_t eventMask) {
    /*
    Button interruption function
    */
    current_character = ' ';
}

void button_interrupt_record(uint gpio, uint32_t eventMask) {
    /*
    Button interruption function for button 2
    */
    if (state == RECORDING)
        state = WAITING;
    else if (state == WAITING)
        state = RECORDING;
}


// Terminal communication
void displayOutput(char current_char) {
    /*
    Displays the current morse code character.
    */
    printf("%c\n", current_char);
    state = WAITING;
}


// Task functions
void sensor_task(void *arg) {
    /*
    Detects movement and conerts it to dot or dash
        - detectMovement()
    */
   while(1) {
    detectMovement();
    state = DATA_READY;
    vTaskDelay(pdMS_TO_TICKS(1000));
   }

}

void button_task(void *arg) {
    /*
    Prints the current character on the console
        - displayOutput()
        - button interrupt, 
    */
   while(1) {
    displayOutput(current_character);
    vTaskDelay(pdMS_TO_TICKS(1000));

   }

}






static void usb_task(void *arg) {
    (void)arg;
    while (1) {
        tud_task();
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    /*
    Apparently "on the Pico SDK, only one global IRQ callback function can be registered using
    gpio_set_irq_enabled_with_callback().""
    */
    if (gpio == BUTTON1) {
        button_interrupt_space(gpio, events);
    } else if (gpio == BUTTON2) {
        button_interrupt_record(gpio, events);
    }
}







// Main function for initializing everything
int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(2000);

    TaskHandle_t hUsb;

    // Button Interrupts
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);
    
    // Task Creation
    xTaskCreate(sensor_task, "Sensor Task", 256, NULL, 1, NULL);
    xTaskCreate(button_task, "Button Task", 256, NULL, 1, NULL);


    xTaskCreate(usb_task, "USB_Task", DEFAULT_STACK_SIZE, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
        // Pin the USB task to Core 0
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif

    tusb_init();
    vTaskStartScheduler(); // Start FreeRTOS
    return 0;
}
