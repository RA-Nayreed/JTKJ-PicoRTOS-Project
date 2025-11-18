/*
 * JTKJ Module2 Final Project - Tier 1
 * Authors: Rezwan Ahmad Nayreed, Joni Lahtinen and Aati Samuel Strömmer
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

// --- Configuration ---
#define CDC_ITF_TX 1  // Data on ttyACM1
#define QUEUE_LEN  32
#define MSG_BUFFER_SIZE 256

// --- Timing Constants ---
#define DEBOUNCE_MS     50   // Lowered from 300ms to catch quick clicks
#define LONG_PRESS_MS   800  // Hold for 0.8s to send message
#define BTN1_DEBOUNCE   250  // 250ms lockout for Button 1 to prevent double-spaces

// --- State Machine ---
typedef enum {
    STATE_IDLE,
    STATE_RECORDING,
    STATE_SENDING,
    STATE_RECEIVING,
    STATE_DISPLAYING
} SystemState;

// --- Message Structure ---
typedef struct {
    char buffer[MSG_BUFFER_SIZE];
    uint16_t length;
    bool complete;
} MorseMessage;

// --- Global Variables ---
static volatile SystemState g_state = STATE_IDLE;
static MorseMessage g_tx_message = {0};
static MorseMessage g_rx_message = {0};

// Button Timing Variables (volatile for ISR access)
static volatile uint32_t btn2_press_time = 0;
static volatile uint32_t btn1_last_press_time = 0;

// --- Morse Translation Maps ---
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

// --- FreeRTOS Objects ---
static QueueHandle_t xMorseTxQueue;
static QueueHandle_t xMorseRxQueue;
static QueueHandle_t xDisplayQueue;
static QueueHandle_t xBuzzerQueue;
static SemaphoreHandle_t xStateMutex;
static SemaphoreHandle_t xTxMsgMutex;

// --- Function Prototypes ---
static void imu_task(void *arg);
static void usb_tx_task(void *arg);
static void usb_rx_task(void *arg);
static void display_task(void *arg);
static void buzzer_task(void *arg);
static void state_manager_task(void *arg);
static void tinyusb_task(void *arg); 
static void gpio_callback(uint gpio, uint32_t events);

// --- Helper Functions ---

static void change_state(SystemState new_state) {
    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
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
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length < MSG_BUFFER_SIZE - 1) {
            g_tx_message.buffer[g_tx_message.length++] = c;
            g_tx_message.buffer[g_tx_message.length] = '\0';
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void send_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (g_tx_message.length > 0) {
            if (g_tx_message.length < MSG_BUFFER_SIZE - 2) {
                g_tx_message.buffer[g_tx_message.length++] = '\n';
                g_tx_message.buffer[g_tx_message.length] = '\0';
                g_tx_message.complete = true;
            }
        }
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void clear_tx_message(void) {
    if (xSemaphoreTake(xTxMsgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(g_tx_message.buffer, 0, MSG_BUFFER_SIZE);
        g_tx_message.length = 0;
        g_tx_message.complete = false;
        xSemaphoreGive(xTxMsgMutex);
    }
}

static void play_feedback_tone(uint16_t freq_hz, uint16_t duration_ms) {
    uint32_t cmd = (freq_hz << 16) | duration_ms;
    xQueueSend(xBuzzerQueue, &cmd, 0);
}

static void display_text(const char* text) {
    if (text && strlen(text) > 0) {
        xQueueSend(xDisplayQueue, text, 0);
    }
}

// --- Interrupt Handler  ---

static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t hpw = pdFALSE;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (gpio == BUTTON1 && (events & GPIO_IRQ_EDGE_RISE)) {
        // DEBOUNCE: Ignore if pressed too soon after last press
        if (now - btn1_last_press_time > BTN1_DEBOUNCE) {
            btn1_last_press_time = now;
            
            if (g_state == STATE_RECORDING) {
                char space = ' ';
                xQueueSendFromISR(xMorseTxQueue, &space, &hpw);
            }
        }
    }
    else if (gpio == BUTTON2) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            btn2_press_time = now;
        }
        else if (events & GPIO_IRQ_EDGE_RISE) {
            uint32_t press_duration = now - btn2_press_time;
            
            if (press_duration >= LONG_PRESS_MS) {
                // LONG PRESS: Send message
                if (g_state == STATE_RECORDING) {
                    char end_msg = '\n';  
                    xQueueSendFromISR(xMorseTxQueue, &end_msg, &hpw);
                }
            } else if (press_duration >= DEBOUNCE_MS) {
                // SHORT PRESS: Toggle recording mode
                // Now catches clicks as fast as 50ms
                char toggle = 'T';  
                xQueueSendFromISR(xMorseTxQueue, &toggle, &hpw);
            }
            portYIELD_FROM_ISR(hpw);
        }
    }
}

// --- Tasks ---

static void imu_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;
    
    vTaskDelay(pdMS_TO_TICKS(500)); // Wait for settling

    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
        usb_serial_print("IMU OK.\n");
    } else {
        usb_serial_print("IMU FAIL.\n");
        vTaskDelete(NULL); 
        return;
    }
    
    for (;;) {
        if (g_state == STATE_RECORDING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                // DOT: Device Flat (Z down < -0.8g)
                if (az < -0.8f && fabsf(ax) < 0.3f && fabsf(ay) < 0.3f) {
                    char dot = '.';
                    xQueueSend(xMorseTxQueue, &dot, 0);
                    play_feedback_tone(1000, 100);  
                    vTaskDelay(pdMS_TO_TICKS(800)); // Wait for neutral
                }
                // DASH: Tilted on side (Y down < -0.8g)
                else if (ay < -0.8f && fabsf(ax) < 0.3f) {
                    char dash = '-';
                    xQueueSend(xMorseTxQueue, &dash, 0);
                    play_feedback_tone(1000, 300);  
                    vTaskDelay(pdMS_TO_TICKS(800)); // Wait for neutral
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
            
            // --- Command Processing ---
            if (symbol == 'T') { // Toggle
                if (g_state == STATE_IDLE) {
                    clear_tx_message();
                    change_state(STATE_RECORDING);
                    play_feedback_tone(2000, 100);
                    // Clear buffer when starting
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                } else if (g_state == STATE_RECORDING) {
                    change_state(STATE_IDLE);
                    play_feedback_tone(1500, 100);
                }
                continue;
            }
            else if (symbol == '\n') { // Send Message
                // Flush pending character if exists
                if (morse_len > 0) {
                     // Try to translate last char before sending
                     // (Copy-paste logic below or just ignore)
                }
                morse_len = 0;
                memset(current_morse, 0, sizeof(current_morse));
                
                send_tx_message();
                change_state(STATE_SENDING);
                continue;
            }
            
            // --- Morse Processing ---
            if (g_state == STATE_RECORDING) {
                
                if (symbol == '.' || symbol == '-') {
                    if (morse_len < 7) {
                        current_morse[morse_len++] = symbol;
                        current_morse[morse_len] = '\0';
                        
                        // DEBUG: Print what we are building
                        usb_serial_print(current_morse); 
                        usb_serial_print("\n");
                    }
                } 
                else if (symbol == ' ') {
                    // Space Button Pressed
                    if (morse_len == 0) {
                        // Empty buffer? Just ignore or add a literal space
                        // append_to_tx_message(' '); 
                    } 
                    else {
                        // Translate
                        char translated_char = '?'; 
                        for (int i = 0; i < 36; i++) {
                            if (strcmp(current_morse, morse_map[i]) == 0) {
                                translated_char = alphabet_map[i];
                                break;
                            }
                        }
                        append_to_tx_message(translated_char);
                        
                        // Debug Output
                        char debug_buf[10];
                        snprintf(debug_buf, 10, " -> %c\n", translated_char);
                        usb_serial_print(debug_buf);
                        
                        // Show on LCD
                        char lcd_buf[2] = {translated_char, '\0'};
                        display_text(lcd_buf);
                    }
                    // Reset buffer
                    morse_len = 0;
                    memset(current_morse, 0, sizeof(current_morse));
                }
            }
        }
        
        // --- Sending Logic ---
        if (g_state == STATE_SENDING && g_tx_message.complete) {
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                tud_cdc_n_write(CDC_ITF_TX, g_tx_message.buffer, g_tx_message.length);
                tud_cdc_n_write_flush(CDC_ITF_TX);
                play_feedback_tone(2500, 200);
                usb_serial_print("\n[MSG SENT]\n");
            }
            clear_tx_message();
            change_state(STATE_IDLE);
        }
    }
}

static void usb_rx_task(void *arg) {
    (void)arg;
    char rx_buf[64];
    for (;;) {
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX)) {
            uint32_t count = tud_cdc_n_read(CDC_ITF_TX, rx_buf, sizeof(rx_buf) - 1);
            if (count > 0) {
                rx_buf[count] = '\0';
                change_state(STATE_RECEIVING);
                for (uint32_t i = 0; i < count; i++) {
                    char c = rx_buf[i];
                    xQueueSend(xMorseRxQueue, &c, 0); // Sound feedback
                    
                    if (g_rx_message.length < MSG_BUFFER_SIZE - 1) {
                        g_rx_message.buffer[g_rx_message.length++] = c;
                        g_rx_message.buffer[g_rx_message.length] = '\0';
                    }
                    if (c == '\n') {
                        change_state(STATE_DISPLAYING);
                        display_text(g_rx_message.buffer);
                        usb_serial_print("\n[RX]: ");
                        usb_serial_print(g_rx_message.buffer);
                        memset(g_rx_message.buffer, 0, MSG_BUFFER_SIZE);
                        g_rx_message.length = 0;
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        change_state(STATE_IDLE);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

static void display_task(void *arg) {
    (void)arg;
    char msg_buf[64];
    init_display();
    clear_display();
    write_text("Ready!");
    for (;;) {
        if (xQueueReceive(xDisplayQueue, msg_buf, portMAX_DELAY) == pdTRUE) {
            clear_display();
            write_text(msg_buf);
        }
    }
}

static void buzzer_task(void *arg) {
    (void)arg;
    uint32_t cmd;
    char morse_symbol;
    init_buzzer();
    for (;;) {
        if (xQueueReceive(xBuzzerQueue, &cmd, 0) == pdTRUE) {
            uint16_t freq = (cmd >> 16) & 0xFFFF;
            uint16_t duration = cmd & 0xFFFF;
            buzzer_play_tone(freq, duration);
        }
        else if (xQueueReceive(xMorseRxQueue, &morse_symbol, 0) == pdTRUE) {
            if (morse_symbol == '.') buzzer_play_tone(800, 150);
            else if (morse_symbol == '-') buzzer_play_tone(800, 450);
            else if (morse_symbol == ' ') vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void tinyusb_task(void *arg) {
    (void)arg;
    for (;;) {
        tud_task();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void state_manager_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));
    usb_serial_print("\n=== System Ready ===\n");
    for (;;) {
        if (g_state == STATE_DISPLAYING) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            if (g_state == STATE_DISPLAYING) change_state(STATE_IDLE);
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