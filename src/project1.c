#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "tkjhat/sdk.h"
#include <inttypes.h>

// FIX 1: Add includes for TinyUSB and the debug helper library
#include <tusb.h>
#include "usbSerialDebug/helper.h"

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
char current_str[10];// string of all the characters received so far (TIER 2). Note: Arbitrary length so we won't get screwed by memory.
programState state = WAITING;

// Prototypes
void displayOutput(char current_char);
void detectMovement();
void button_interrupt_space(uint gpio, uint32_t eventMask);
void button_interrupt_record(uint gpio, uint32_t eventMask);
void sensor_task();
void button_task();
void translate_letter(); // TODO: turns current_str[] into an alphabetic letter and calls displayOutput()
void add_to_str(char current_char); // TODO: adds character to the end of current_str[] 


void detectMovement() {
    /*
    Detect movement from the device and save the corresponding character to current_character.
    */
    float ax, ay, az, gx, gy, gz, t;
      if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

        if (az < -0.8) {
            add_to_str('-');
        }
        else if (ay < -0.8) {
            add_to_str('.'); //These could be printed for debugging too
        }
      }
}


void add_to_str(char current_char){
    /*
    Checks if current_str has space for a character and adds it.
    If current_str is full, starts from the beginning <- can be adjusted depending on intended behavior.
    */
    int len = strlen(current_str);
    if (len < sizeof(current_str)-1){
        current_str[len] = current_char;
        current_str[len+1] = '\0';
    } else{
        current_str[0] = current_char;
        current_str[1] = '\0';
    }
}

void create_letter(){
    /* 
    Takes morse code from current_str[] and compares it to morse alphabet. 
    Stores result in char letter, calls displayOutput with it.
    */
   char letter;
   // comparisons go here
   displayOutput(letter);
   current_str[0] = '\0';
}

void button_interrupt_space(uint gpio, uint32_t eventMask) {
    /*
    Button interruption function
    */
    //current_character = ' ';

    //TODO: create_letter();

}

void button_interrupt_record(uint gpio, uint32_t eventMask) {
    /*
    Button interruption function for button 2
    */
    if (state == RECORDING)
        state = WAITING;
    else if (state == WAITING)
        state = RECORDING;char current_character;
}


// Terminal communication
void displayOutput(char current_char) {
    /*
    Displays the current morse code character.
    */
    
    // FIX 2: Replace printf() with the correct USB functions
    // printf("%c\n", current_char); // This will not work without stdio_init_all()

    // Create a 2-byte buffer: [character, null-terminator]
    char buf[2] = { current_char, '\0' }; // These were borken: they were taking global current_character instead of local current_char

    // Send the character to the "data" serial port (CDC1)
    if (tud_cdc_n_connected(CDC_ITF_TX)) {
        tud_cdc_n_write_str(CDC_ITF_TX, buf);
        tud_cdc_n_write_flush(CDC_ITF_TX);
    }

    // (Optional) Send to the "debug" serial port (CDC0) for logging
    char debug_buf[2] = { current_char, '\0' }; // These were borken: they were taking global current_character instead of local current_char
    usb_serial_print(debug_buf); // This prints to CDC0

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

/*void button_task(void *arg) {
    
    Prints the current character on the console
        - displayOutput()
        - button interrupt, 
        ! I have no idea what this task actually does beyond constantly printing. Completely unrelated to buttons.
         ALL BUTTON STUFF IS HANDLED BY INTERRUPT RIGHT NOW! Should likely be deleted for TIER 2
    
   while(1) {
    displayOutput(current_character); 
    vTaskDelay(pdMS_TO_TICKS(1000));

   }

} */






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
    // FIX 3: DO NOT call stdio_init_all(). This conflicts with tusb_init().
    // stdio_init_all();
    
    init_hat_sdk();
    sleep_ms(300); // Wait for HAT I2C to be ready

    // FIX 4: Initialize the HAT sensors before using them
    init_button1(); // Initialize button 1 pin
    init_button2(); // Initialize button 2 pin
    if (init_ICM42670() == 0) { // Initialize IMU
        ICM42670_start_with_default_values();
    }
    

    TaskHandle_t hUsb;

    // Button Interrupts
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    //Other button call here was redundant
    
    // Task Creation
    xTaskCreate(sensor_task, "Sensor Task", 2048, NULL, 1, NULL); // Increased stack
    xTaskCreate(button_task, "Button Task", 1024, NULL, 1, NULL); // Increased stack


    xTaskCreate(usb_task, "USB_Task", DEFAULT_STACK_SIZE, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
        // Pin the USB task to Core 0
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif

    // FIX 5: Initialize TinyUSB and the helper library
    // This MUST be done just before starting the scheduler
    tusb_init();
    usb_serial_init(); 
    
    vTaskStartScheduler(); // Start FreeRTOS
    
    // Should never reach here
    return 0;
}