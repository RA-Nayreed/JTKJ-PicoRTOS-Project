/*
 * JTKJ Module 2 Final Project - Tier 2-3
 * Authors: Rezwan Ahmad Nayreed, Joni Lahtinen and Aati Samuel Strömmer
 * This is a morse Pico-based morse code communicator. 
 * Button 1 is used for separating characters (short press) or adding spaces (double press).
 * Button 2 is used for starting/stopping recording and sending message to terminal.
 * Text sent into the device is translated to/from morse code and displayed on the screen.
 * Included SDK has updated scrolling function for long messages.
 * Parts of the program were generated using Gemini AI and manually curated/debugged by the authors.
*/

/*
Documentation progress:
- FreeRTOS tasks
    - usb_tx_task
    - usb_rx_task
- Main

Remove from list when finished
*/

#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <tusb.h>
#include <string.h>
#include <math.h> 
#include "tkjhat/sdk.h"
#include "usbSerialDebug/helper.h"

// ==========================================
// --- Configuration & Constants ---
// ==========================================

#define CDC_ITF_TX 1  
#define QUEUE_LEN  256 
#define MSG_BUFFER_SIZE 256 

#define DEBOUNCE_MS     50   
#define LONG_PRESS_MS   800  
#define BTN1_DEBOUNCE   250  

// ==========================================
// --- Data Structures ---
// ==========================================

typedef enum {
    STATE_IDLE,       
    STATE_RECORDING,  
    STATE_SENDING,    
    STATE_RECEIVING,  
    STATE_DISPLAYING  
} SystemState;

typedef struct {
    char buffer[MSG_BUFFER_SIZE];
    uint16_t length;
    bool complete;    
} MorseMessage;

// ==========================================
// --- Global Variables ---
// ==========================================

static volatile SystemState g_state = STATE_IDLE;
static MorseMessage g_tx_message = {0}; 
static MorseMessage g_rx_message = {0}; 

static volatile uint32_t btn2_press_time = 0;
static volatile uint32_t btn1_last_press_time = 0;

// Maps
const char *morse_map[36] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-",
    ".-..", "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-",
    ".--", "-..-", "-.--", "--..", ".----", "..---", "...--", "....-", ".....",
    "-....", "--...", "---..", "----.", "-----"
};
const char alphabet_map[36] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z', '1', '2', '3', '4',
    '5', '6', '7', '8', '9', '0'
};

// Handles
static QueueHandle_t xMorseTxQueue;   
static QueueHandle_t xMorseRxQueue;   
static QueueHandle_t xDisplayQueue;   
static QueueHandle_t xBuzzerQueue;    
static SemaphoreHandle_t xStateMutex; 
static SemaphoreHandle_t xTxMsgMutex; 

// Prototypes
static void imu_task(void *arg);/*NAYREED*/
static void usb_tx_task(void *arg);/*AATI*/
static void usb_rx_task(void *arg); /*NAYREED, JONI, AATI*/
static void display_task(void *arg);/*NAYREED*/
static void buzzer_task(void *arg);/*NAYREED*/
static void state_manager_task(void *arg);/*JONI*/
static void tinyusb_task(void *arg); /*AATI*/
static void gpio_callback(uint gpio, uint32_t events);/*JONI*/

// ==========================================
// --- Helper Functions ---
// ==========================================

static void change_state(SystemState new_state) {
    /*
    At core, this is a state machine.
    Mutex prevents other tasks from changing the state at the same time
    A short state-name message is sent to the display task via queue so the UI updates when the state changes.
    */
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) { // thread safety
        g_state = new_state;
        xSemaphoreGive(xStateMutex);
        
        char msg[32];
        switch(new_state) {
            case STATE_IDLE:       snprintf(msg, 32, "IDLE"); break;
            case STATE_RECORDING:  snprintf(msg, 32, "RECORDING"); break;
            case STATE_SENDING:    snprintf(msg, 32, "SENDING..."); break;
            case STATE_RECEIVING:  snprintf(msg, 32, "RECEIVING"); break;
            case STATE_DISPLAYING: snprintf(msg, 32, "DISPLAY"); break;
        }
        xQueueSend(xDisplayQueue, msg, 0);
    }
}

static void append_to_tx_message(char c) {
    /*
    Adds a character to the global TX message buffer
    */
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {    // thread safety
        if (g_tx_message.length < MSG_BUFFER_SIZE - 1) {                // check if there is space in the buffer
            g_tx_message.buffer[g_tx_message.length++] = c;             // append character
            g_tx_message.buffer[g_tx_message.length] = '\0';            // null terminator added
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void send_tx_message(void) {
    /*
    Finalizes message. Ensures newline termination and marks it as complete
    */
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {    // thread safety
        // Ensure proper newline termination
        if (g_tx_message.length > 0) {                                  // non-empty message
             if (g_tx_message.buffer[g_tx_message.length-1] != '\n') {  // check if last char is newline
                 if (g_tx_message.length < MSG_BUFFER_SIZE - 1) {       // check space
                     g_tx_message.buffer[g_tx_message.length++] = '\n'; // add new line and null terminator, string ends
                     g_tx_message.buffer[g_tx_message.length] = '\0';
                 }
             }
             g_tx_message.complete = true;  // mark as complete
        } else {
             // Empty message, sends just a newline
             g_tx_message.buffer[0] = '\n';
             g_tx_message.buffer[1] = '\0';
             g_tx_message.length = 1;
             g_tx_message.complete = true;
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void clear_tx_message(void) {
    /*
    Clears the message buffer
    */
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(g_tx_message.buffer, 0, MSG_BUFFER_SIZE);                // clear buffer
        g_tx_message.length = 0;
        g_tx_message.complete = false;                                  // mark as not complete   
        xSemaphoreGive(xTxMsgMutex);
    }
}

static const char* get_morse_from_char(char c) {
    /*
    Translates letters to morse code using predefined maps (A -> .-)
    */
    if (c >= 'a' && c <= 'z') c -= 32;                                  // Convert to uppercase using an ASCII trick
    for (int i = 0; i < 36; i++) {                                      // 36 letters/numbers. Loop finds the index
        if (alphabet_map[i] == c) return morse_map[i];                  // Returns the correct morse symbol based on index
    }
    return NULL; // no character found
}

static char get_char_from_morse(const char* morse_str) {
    /*
    Translates morse code to letters using predefined maps (.- -> A)
    */
    for (int i = 0; i < 36; i++) {
        if (strcmp(morse_str, morse_map[i]) == 0) {                     // compare input to what is in the map, (checks if they're the same)
            return alphabet_map[i];                                     // return the correct letter based on index
        }
    }
    return '?'; // Unknown pattern
}

static void play_feedback_tone(uint16_t freq_hz, uint16_t duration_ms) {
    /*
    Makes the buzzer play a feedback tone via queue
    */
    uint32_t cmd = (freq_hz << 16) | duration_ms;                       // pack frequency and duration into a single value: cmd (16+16 bits)
    xQueueSend(xBuzzerQueue, &cmd, 0);                                  // sends cmd to buzzer queue
}

static void display_text(const char* text) {
    /*
    Displays text on the LCD via queue
    */
    if (text && strlen(text) > 0) {                                     // ensure text isn't null and has length 
        xQueueSend(xDisplayQueue, text, 0);                             // send text to display queue
    }
}

// ==========================================
// --- Interrupt Handler ---
// ==========================================

static void gpio_callback(uint gpio, uint32_t events) {
    /*
    GPIO interrupt handler for buttons

    BUTTON1 (rising edge):
        - Debounce
        - Sends ' ' if recording

    BUTTON2:
        - Falling edge → record press time
        - Rising edge → measure duration
            * Long press → '\n' (end message)
            * Short press → 'T' (toggle)
*/
    BaseType_t hpw = pdFALSE;                                           // tracks if sending to a queue unblocked a higher-priority task
    uint32_t now = to_ms_since_boot(get_absolute_time());               // save current time in ms in a variable
    
    if (gpio == BUTTON1 && (events & GPIO_IRQ_EDGE_RISE)) {             // button 1 caused the interrupt
        if (now - btn1_last_press_time > BTN1_DEBOUNCE) {               // ignore presses that were too quick (debounce)
            btn1_last_press_time = now;                                 // update last press time
            if (g_state == STATE_RECORDING) {
                char space = ' ';
                xQueueSendFromISR(xMorseTxQueue, &space, &hpw);         // send space to TX queue if in recording state
            }
        }
    }
    else if (gpio == BUTTON2) {                                         // button 2 caused the interrupt
        if (events & GPIO_IRQ_EDGE_FALL) {
            btn2_press_time = now;
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            uint32_t press_duration = now - btn2_press_time;            // calculate how long button was pressed and check if it was a long press
            if (press_duration >= LONG_PRESS_MS) {
                if (g_state == STATE_RECORDING) {
                    char end_msg = '\n';  
                    xQueueSendFromISR(xMorseTxQueue, &end_msg, &hpw);   // if long press and recording, send newline to end msg
                }
            } else if (press_duration >= DEBOUNCE_MS) {                 // (debounce < duration < long press) --> short press
                char toggle = 'T';  
                xQueueSendFromISR(xMorseTxQueue, &toggle, &hpw);        // if short press, send toggle command to TX queue
            }
            portYIELD_FROM_ISR(hpw);                                    // Checks hpw flag from xQueueSendFromISR,
        }                                                               // if a higher-priority task is ready, portYIELD_FROM_ISR causes it to run immediately
    }
}

// ==========================================
// --- FreeRTOS Tasks ---
// ==========================================

static void imu_task(void *arg) {
    /*
    IMU input task
    Detects dot/dash gestures and sends them to the TX queue
    */
    (void)arg;                                                           // unused FreeRTOS task argument
    float ax, ay, az, gx, gy, gz, t;                                     // sensor data variables
    
    vTaskDelay(pdMS_TO_TICKS(500)); 

    if (init_ICM42670() == 0) {                                          // initialize IMU
        ICM42670_start_with_default_values();
        usb_serial_print("IMU OK.\n");
    } else {
        usb_serial_print("IMU FAIL.\n");
        vTaskDelete(NULL); 
        return;
    }
    
    for (;;) {
        if (g_state == STATE_RECORDING) {                                 // if STATE_RECORDING and IMU data read successfully
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                if (az < -0.8f && fabsf(ax) < 0.3f && fabsf(ay) < 0.3f) { // dot detected
                    char dot = '.';
                    xQueueSend(xMorseTxQueue, &dot, 0);                   // send dot to queue
                    play_feedback_tone(1000, 100);  
                    vTaskDelay(pdMS_TO_TICKS(800)); 
                }
                else if (ay < -0.8f && fabsf(ax) < 0.3f) {                // dash detected
                    char dash = '-';
                    xQueueSend(xMorseTxQueue, &dash, 0);                  // send dash to queue
                    play_feedback_tone(1000, 300);  
                    vTaskDelay(pdMS_TO_TICKS(800)); 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));  
    }
}

static void usb_tx_task(void *arg) {
    (void)arg;
    char symbol;
    char current_morse[8] = {0}; 
    uint8_t morse_len = 0;

    for (;;) {
        if (xQueueReceive(xMorseTxQueue, &symbol, portMAX_DELAY) == pdTRUE) {
            
            // Command Processing
            if (symbol == 'T') { 
                if (g_state == STATE_IDLE || g_state == STATE_RECEIVING || g_state == STATE_DISPLAYING) {
                    // Stop any incoming audio playback immediately
                    xQueueReset(xMorseRxQueue);
                    xQueueReset(xBuzzerQueue);
                    clear_tx_message();
                    change_state(STATE_RECORDING);
                    play_feedback_tone(2000, 100); 
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                } 
                else if (g_state == STATE_RECORDING) {
                    change_state(STATE_IDLE);
                    play_feedback_tone(1500, 100); 
                }
                continue;
            }
            else if (symbol == '\n') { 
                morse_len = 0;
                memset(current_morse, 0, sizeof(current_morse));
                send_tx_message();
                change_state(STATE_SENDING);
            }
            
            // Morse Translation Logic (IMU -> Text)
            if (g_state == STATE_RECORDING) {
                if (symbol == '.' || symbol == '-') {
                    if (morse_len < 7) {
                        current_morse[morse_len++] = symbol;
                        current_morse[morse_len] = '\0';
                        usb_serial_print(current_morse); 
                        usb_serial_print("\n");
                    } else {
                        morse_len = 0;
                        memset(current_morse, 0, sizeof(current_morse));
                    }
                } 
                else if (symbol == ' ') {
                    if (morse_len == 0) {
                        append_to_tx_message(' ');
                    } else {
                        char translated_char = '?'; 
                        for (int i = 0; i < 36; i++) {
                            if (strcmp(current_morse, morse_map[i]) == 0) {
                                translated_char = alphabet_map[i];
                                break;
                            }
                        }
                        append_to_tx_message(translated_char);
                        char lcd_buf[2] = {translated_char, '\0'};
                        display_text(lcd_buf);
                    }
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                }
            }
        }
        
        // Sending Logic
        if (g_state == STATE_SENDING && g_tx_message.complete) {
            usb_serial_print("\n[SENDING]\n");
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                tud_cdc_n_write(CDC_ITF_TX, g_tx_message.buffer, g_tx_message.length);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                play_feedback_tone(2500, 200); 
            }
            clear_tx_message();
            change_state(STATE_IDLE);
        }
    }
}

/**
 * Task: USB RX (Dual Mode)
 * Handles both Text Input (translates to Morse) AND Raw Morse Input (translates to Text).
 */
static void usb_rx_task(void *arg) {
    (void)arg;
    char rx_buf[64];
    
    // Buffer to hold incoming raw Morse code (dots/dashes) until a space is found
    char incoming_morse_buf[8]; 
    int incoming_morse_idx = 0;
    memset(incoming_morse_buf, 0, sizeof(incoming_morse_buf));

    for (;;) {
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rx_buf, sizeof(rx_buf) - 1);
            
            if (count > 0) {
                rx_buf[count] = '\0';
                change_state(STATE_RECEIVING); 
                
                for (uint32_t i = 0; i < count; i++) {
                    char c = rx_buf[i];

                    // --- CASE 1: Raw Morse Input (. or -) ---
                    if (c == '.' || c == '-') {
                        // Play sound immediately
                        xQueueSend(xMorseRxQueue, &c, 0);
                        
                        // Accumulate in local buffer
                        if (incoming_morse_idx < 7) {
                            incoming_morse_buf[incoming_morse_idx++] = c;
                            incoming_morse_buf[incoming_morse_idx] = '\0';
                        }
                    }
                    // --- CASE 2: End of Morse Character (Space) ---
                    else if (c == ' ') {
                        // Queue silence
                        char gap = ' ';
                        xQueueSend(xMorseRxQueue, &gap, 0);

                        // If we have accumulated dots/dashes, translate them!
                        if (incoming_morse_idx > 0) {
                            char decoded = get_char_from_morse(incoming_morse_buf);
                            
                            // Add DECODED TEXT to display buffer
                            if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                                g_rx_message.buffer[g_rx_message.length++] = decoded;
                                g_rx_message.buffer[g_rx_message.length] = '\0';
                            }
                            
                            // Reset local morse accumulator
                            incoming_morse_idx = 0;
                            memset(incoming_morse_buf, 0, sizeof(incoming_morse_buf));
                        } else {
                            // Just a normal space between words
                            if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                                g_rx_message.buffer[g_rx_message.length++] = ' ';
                                g_rx_message.buffer[g_rx_message.length] = '\0';
                            }
                        }
                    }
                    // --- CASE 3: Regular English Text (A-Z) ---
                    else {
                        const char* morse_code = get_morse_from_char(c);
                        if (morse_code != NULL) {
                            // Add MORSE to display buffer (User typed text, show morse)
                            if (g_rx_message.length + strlen(morse_code) + 1 < MSG_BUFFER_SIZE) {
                                strcat(g_rx_message.buffer, morse_code);
                                strcat(g_rx_message.buffer, " "); 
                                g_rx_message.length += strlen(morse_code) + 1;
                            }

                            // Queue audio for the translation
                            for (int j = 0; morse_code[j] != '\0'; j++) {
                                xQueueSend(xMorseRxQueue, &morse_code[j], 0);
                            }
                            char gap = ' '; 
                            xQueueSend(xMorseRxQueue, &gap, 0); 
                        }
                    }

                    // --- CASE 4: End of Message (Newline) ---
                    if (c == '\n' || c == '\r') {
                        // Check if there is leftover raw Morse to decode
                        if (incoming_morse_idx > 0) {
                            char decoded = get_char_from_morse(incoming_morse_buf);
                            if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                                g_rx_message.buffer[g_rx_message.length++] = decoded;
                                g_rx_message.buffer[g_rx_message.length] = '\0';
                            }
                            incoming_morse_idx = 0;
                            memset(incoming_morse_buf, 0, sizeof(incoming_morse_buf));
                        }

                        // Show Result on LCD
                        if (g_rx_message.length > 0) {
                            xQueueSend(xDisplayQueue, g_rx_message.buffer, 0);
                            usb_serial_print("\n[RX]: ");
                            usb_serial_print(g_rx_message.buffer);
                            usb_serial_print("\n");
                        }

                        // Trigger Audio Cleanup
                        char eom = '\0';
                        xQueueSend(xMorseRxQueue, &eom, portMAX_DELAY);

                        // Clean Global Buffer
                        memset(g_rx_message.buffer, 0, MSG_BUFFER_SIZE);
                        g_rx_message.length = 0;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void display_task(void *arg) {
    /*
    Manages display output
    Writes message from queue on the screen. Scrolls if too long.
    */
    (void)arg;                                                                  // unused FreeRTOS task argument
    char msg_buf[64];                                                           // buffer for messages from queue
    init_display();
    clear_display();
    write_text("Ready!");
    
    for (;;) {
        if (xQueueReceive(xDisplayQueue, msg_buf, portMAX_DELAY) == pdTRUE) {   // wait for message from queue and store in msg_buf (portMAX_DELAY --> wait indefinitely)
            if (strlen(msg_buf) > 10) {
                scroll_text(msg_buf, 24, 30);                                   // long message, scroll (with the function that was added to the library)
            } else {
                clear_display();
                write_text(msg_buf);                                            // short message, just write
            }
        }
    }
}

static void buzzer_task(void *arg) {
    /*
    Manages buzzer output
    - Plays tones from xBuzzerQueue (msg: freq + duration)
    - Plays morse beeps from xMorseRxQueue
    */
    (void)arg;                                                                  // unused FreeRTOS task argument
    uint32_t cmd;                                                               // packed frequency and duration for buzzer
    char morse_symbol;
    init_buzzer();
    
    for (;;) {
        if (xQueueReceive(xBuzzerQueue, &cmd, 0) == pdTRUE) {                   // check if there is a command in the buzzer queue (frequency + duration)
            uint16_t freq = (cmd >> 16) & 0xFFFF;
            uint16_t duration = cmd & 0xFFFF;                                   // extract frequency and duration into seperate variables (16 bits each)
            buzzer_play_tone(freq, duration);                                   
        }

        else if (xQueueReceive(xMorseRxQueue, &morse_symbol, 0) == pdTRUE) {    // check morse queue for incoming symbols
            if (morse_symbol == '\0') {                                         // morse message has ended
                if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (g_state == STATE_RECEIVING || g_state == STATE_DISPLAYING || g_state == STATE_IDLE) {
                        g_state = STATE_IDLE;                                   // thread safe: revert to IDLE only if not recording
                    }
                    xSemaphoreGive(xStateMutex);
                    
                    if (g_state == STATE_IDLE) {                                // update display if in STATE_IDLE
                        char msg[] = "IDLE";
                        xQueueSend(xDisplayQueue, msg, 0);
                    }
                }
            }
            
            else if (morse_symbol == '.') buzzer_play_tone(800, 150);
            else if (morse_symbol == '-') buzzer_play_tone(800, 450);
            else if (morse_symbol == ' ') vTaskDelay(pdMS_TO_TICKS(200));       // play corresponding sounds or delays
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void tinyusb_task(void *arg) {
    /*
    TinyUSB background task
    */
    (void)arg;                                                                  // unused FreeRTOS task argument
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void state_manager_task(void *arg) {
    /*
    Checks system state and manages timed transitions
    */
    (void)arg;                                                                  // unused FreeRTOS task argument
    vTaskDelay(pdMS_TO_TICKS(2000));                                            // wait 2 seconds for system to stabilize
    usb_serial_print("\n=== System Ready (Dual Mode) ===\n");
    
    for (;;) {
        if (g_state == STATE_DISPLAYING) {
            vTaskDelay(pdMS_TO_TICKS(3000));                                    // display for 3 seconds
            if (g_state == STATE_DISPLAYING) change_state(STATE_IDLE);          // back to IDLE if still displaying
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void) {
    init_hat_sdk();
    sleep_ms(300);
    init_button1();
    init_button2();
    
    xMorseTxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xMorseRxQueue = xQueueCreate(QUEUE_LEN, sizeof(char));
    xDisplayQueue = xQueueCreate(8, 64 * sizeof(char));
    xBuzzerQueue = xQueueCreate(8, sizeof(uint32_t));
    xStateMutex = xSemaphoreCreateMutex();
    xTxMsgMutex = xSemaphoreCreateMutex();
    
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    
    xTaskCreate(imu_task, "IMU", 2048, NULL, 2, NULL);
    xTaskCreate(usb_tx_task, "TX", 2048, NULL, 2, NULL);
    xTaskCreate(usb_rx_task, "RX", 2048, NULL, 2, NULL);
    xTaskCreate(display_task, "LCD", 2048, NULL, 1, NULL);
    xTaskCreate(buzzer_task, "Buz", 1024, NULL, 1, NULL);
    xTaskCreate(state_manager_task, "Mgr", 1024, NULL, 3, NULL);
    
    TaskHandle_t hUsb;
    xTaskCreate(tinyusb_task, "USB", 2048, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    
    tusb_init();
    usb_serial_init();
    
    vTaskStartScheduler(); 
    return 0;
}
