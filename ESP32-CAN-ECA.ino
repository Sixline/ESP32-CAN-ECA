#include <Arduino.h>
#include "driver/twai.h"
#include "driver/gpio.h"
#include <Adafruit_NeoPixel.h>

//=====================================================
// Configuration
//=====================================================

#define ARGB_LED_PIN 48 // NeoPixel/WS2812 Addressable LED

#define ECS_GPIO_IN 7 // Ethanol Content Sensor GPIO Input Pin

#define CANBUS_TX_PIN 1 //*NOT CAN HIGH OR LOW!* Goes to CANBUS transceiver!
#define CANBUS_RX_PIN 2 //*NOT CAN HIGH OR LOW!* Goes to CANBUS transceiver!
#define CANBUS_BITRATE_KBPS 500  // 1000, 500, 250, or 125

#define UPDATE_INTERVAL_MS 250

#define ENABLE_CAN_ID 0x0A5 // Common BMW RPM CAN Bus ID
#define ENABLE_TIMEOUT_MS 1000 // Receive timeout - If we don't see our specific ID within this time then we disable transmitting.

#define TX_CAN_ID 0x0EC // Zeitronix CAN Bus ID

#define ECS_AVG_SAMPLES 8 // Number of samples from ECS to average

#define ETHANOL_CHANGE_THRESHOLD 5   // Percent change to ignore once - basic spike filter

//=====================================================
// Globals
//=====================================================

Adafruit_NeoPixel neopixel(1, ARGB_LED_PIN, NEO_GRB + NEO_KHZ800);
enum class LedState : uint8_t {
  WAIT_FOR_CAN,
  ACTIVE,
};
volatile LedState led_state;

volatile uint32_t falling_edge_time = 0;
volatile uint32_t last_rising_edge_time = 0;
volatile uint32_t average_period_us = 0;
volatile uint32_t average_low_time_us = 0;

volatile uint32_t period_buffer[ECS_AVG_SAMPLES] = {0};
volatile uint32_t low_time_buffer[ECS_AVG_SAMPLES] = {0};
volatile uint32_t period_sum = 0;
volatile uint32_t low_time_sum = 0;
volatile uint8_t average_sample_index = 0;
volatile uint8_t average_sample_count = 0;

volatile bool timing_valid = false;
volatile uint8_t sensor_fault = 1;

volatile bool transmit_enabled = true;
volatile uint32_t last_enable_message_ms = 0;

volatile uint8_t last_ethanol_percent = 50;
volatile uint8_t pending_ethanol_percent = 50;
volatile bool pending_ethanol_change = false;

portMUX_TYPE timing_mux = portMUX_INITIALIZER_UNLOCKED;

//=====================================================
// NeoPixel Task
//=====================================================

void neoPixelTask(void *param) {
  bool blink_state = false;
  uint32_t last_update = millis();

  while (true) {
    neopixel.setBrightness(25);  // Set BRIGHTNESS (max = 255)

    if (sensor_fault == 1) {
      // Fast red blink
      if (millis() - last_update >= 100) {
        blink_state = !blink_state;
        last_update = millis();

        if (blink_state)
          neopixel.setPixelColor(0, neopixel.Color(255, 0, 0));
        else
          neopixel.clear();

        neopixel.show();
      }
    } else {
      switch (led_state) {
        case LedState::WAIT_FOR_CAN:
          {
            // Slow purple blink
            if (millis() - last_update >= 500) {
              blink_state = !blink_state;
              last_update = millis();

              if (blink_state)
                neopixel.setPixelColor(0, neopixel.Color(150, 0, 255));
              else
                neopixel.clear();

              neopixel.show();
            }
            break;
          }

        case LedState::ACTIVE:
          {
            // Slow green blink
            if (millis() - last_update >= 500) {
              blink_state = !blink_state;
              last_update = millis();

              if (blink_state)
                neopixel.setPixelColor(0, neopixel.Color(0, 255, 0));
              else
                neopixel.clear();

              neopixel.show();
            }
            break;
          }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//=====================================================
// Ethanol Content Sensor ISR
//=====================================================

void IRAM_ATTR ecsISR() {
  uint32_t now = micros();

  portENTER_CRITICAL_ISR(&timing_mux);

  if (gpio_get_level((gpio_num_t)ECS_GPIO_IN)) {
    // Rising Edge
    if (last_rising_edge_time != 0) { // Ignore the very first rising edge since we don't yet have a previous rising edge to calculate a period.
      uint32_t period = now - last_rising_edge_time;
      uint32_t low_time = now - falling_edge_time;

      period_sum -= period_buffer[average_sample_index]; // Remove oldest samples from running sums
      low_time_sum -= low_time_buffer[average_sample_index];

      period_buffer[average_sample_index] = period; // Store newest samples
      low_time_buffer[average_sample_index] = low_time;

      period_sum += period; // Add newest samples to running sums
      low_time_sum += low_time;

      if (average_sample_count < ECS_AVG_SAMPLES) { // Increase sample count until buffer is full
        average_sample_count++;
      }

      average_period_us = period_sum / average_sample_count; // Calculate moving averages
      average_low_time_us = low_time_sum / average_sample_count;

      average_sample_index++; // Advance circular buffer

      if (average_sample_index >= ECS_AVG_SAMPLES) {
        average_sample_index = 0;
      }

      timing_valid = true;
    }

    last_rising_edge_time = now; // Save timestamp for next period measurement

  } else {
    // Falling Edge
    falling_edge_time = now;
  }

  portEXIT_CRITICAL_ISR(&timing_mux);
}

//=====================================================
// CAN Setup
//=====================================================

bool setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CANBUS_TX_PIN, (gpio_num_t)CANBUS_RX_PIN, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config;

  switch (CANBUS_BITRATE_KBPS) {
    case 1000:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;
    case 500:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
    case 250:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;
    case 125:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;
    default:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;
  }

  twai_filter_config_t f_config = { .acceptance_code = (ENABLE_CAN_ID << 21), .acceptance_mask = ~(0x7FF << 21), .single_filter = true };

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }

  if (twai_start() != ESP_OK) {
    return false;
  }

  Serial.println("CAN Started");

  return true;
}

//=====================================================
// CAN TX
//=====================================================

void canTxMessage(uint8_t ethanol_percent, uint8_t temp_c, uint8_t sensor_status) {
  twai_message_t tx_msg = {};

  tx_msg.identifier = TX_CAN_ID;
  tx_msg.extd = 0;
  tx_msg.rtr = 0;

  tx_msg.data_length_code = 8;

  tx_msg.data[0] = ethanol_percent;
  tx_msg.data[1] = temp_c;
  tx_msg.data[2] = 0;
  tx_msg.data[3] = 0;
  tx_msg.data[4] = 0;
  tx_msg.data[5] = 0;
  tx_msg.data[6] = 0;
  tx_msg.data[7] = sensor_status;

  twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
}

//=====================================================
// CAN RX Task
//=====================================================

void canRxTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(15000)); // Delay on boot to force 15 seconds of transmit

  twai_message_t rx_msg;

  while (true) {
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
      last_enable_message_ms = millis();
      transmit_enabled = true;
      led_state = LedState::ACTIVE;
    }

    if ((millis() - last_enable_message_ms) > ENABLE_TIMEOUT_MS) {
      transmit_enabled = false;
      led_state = LedState::WAIT_FOR_CAN;
    }
  }
}

//=====================================================
// Sensor Task
//=====================================================

void sensorTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS)); // delay one interval on startup - getting startup noise from sensor?
  while (true) {
    uint32_t local_average_period_us;
    uint32_t local_average_low_time_us;
    bool local_timing_valid;

    portENTER_CRITICAL(&timing_mux);
    if ((micros() - last_rising_edge_time) > 100000) {
    timing_valid = false;
    }
    local_average_period_us = average_period_us;
    local_average_low_time_us = average_low_time_us;
    local_timing_valid = timing_valid;
    portEXIT_CRITICAL(&timing_mux);

    uint8_t ethanol_percent = 50;
    uint8_t temp_c_int = 165;

    float frequency = 0;
    float temp_c = 0;
    float temp_f = 0;

    if (local_timing_valid && local_average_period_us != 0) {
      //-------------------------------------------------
      // Sensor calculations
      //-------------------------------------------------

      frequency = 1000000.0f / local_average_period_us;

      temp_c = (41.25f * (local_average_low_time_us / 1000.0f)) - 81.25f;
      temp_f = (temp_c * 1.8) + 32;
      ethanol_percent = (uint8_t)(roundf(frequency) - 50.0f); // Frequency will be between 50 and 150 hz, subtract 50 to get Ethanol percentage.
      temp_c_int = (uint8_t)(roundf(temp_c) + 40.0f); // 8-bit integer is only 0 to 255 and can't be negative. Add 40 to support down to -40 C and offset on other end.

      sensor_fault = 0;

      //-------------------------------------------------
      // Sensor validation
      //-------------------------------------------------

      if ((frequency > 152.0f) || (frequency < 48.0f)) { // If the frequency is greater than 152 Hz or lower than 48 Hz, we transmit 50% Ethanol, max temp, and 1(failure) for sensor state.
        ethanol_percent = 50;
        temp_c_int = 165;
        sensor_fault = 1;
        Serial.println("Frequency out of range! Sensor error or failed!");
      }
    } else {
      sensor_fault = 1;
      Serial.println("No valid sensor timing! Sensor failed or disconnected!");
    }

    //-----------------------------------------------------
    // Confirm large ethanol changes
    //-----------------------------------------------------

    if (!sensor_fault) {
      int diff = abs((int)ethanol_percent - (int)last_ethanol_percent);

      if (diff > ETHANOL_CHANGE_THRESHOLD) {
        Serial.println("Ethanol spike! Confirming change!");
        if (pending_ethanol_change && (abs((int)ethanol_percent - (int)pending_ethanol_percent) <= 3)) {
            // Same large change twice in a row.
            // Accept it.

            last_ethanol_percent = ethanol_percent;
            pending_ethanol_change = false;
            pending_ethanol_percent = ethanol_percent;
        } else {
            // First occurrence.
            // Hold previous value.

            pending_ethanol_percent = ethanol_percent;
            pending_ethanol_change = true;

            ethanol_percent = last_ethanol_percent;
        }
      } else {
        // Normal change.

        last_ethanol_percent = ethanol_percent;
        pending_ethanol_change = false;
      }
    }

    //-----------------------------------------------------
    // CAN TX
    //-----------------------------------------------------

    if (transmit_enabled) {
      canTxMessage(ethanol_percent, temp_c_int, sensor_fault);
      led_state = LedState::ACTIVE;
    }

    //-----------------------------------------------------
    // Serial Debug Output
    //-----------------------------------------------------

    Serial.printf("Freq: %.2f Hz, Temp C: %.2f (CAN Int: %u), Temp F: %.2f, Ethanol: %u%%, Sensor Status: %s, CAN TX: %s\n", frequency, temp_c, temp_c_int, temp_f, ethanol_percent, sensor_fault ? "FAULT" : "OK", transmit_enabled ? "ON" : "OFF");

    vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
  }
}

//=====================================================
// Setup
//=====================================================

void setup() {
  Serial.begin(115200);

  neopixel.begin();  // INITIALIZE NeoPixel object (REQUIRED)
  neopixel.show();   // Turn OFF pixel ASAP
  xTaskCreate(
    neoPixelTask,
    "neoPixelTask",
    4096,
    NULL,
    1,
    NULL);

  pinMode(ECS_GPIO_IN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ECS_GPIO_IN), ecsISR, CHANGE);

  if (!setupCAN()) {
    Serial.println("CAN initialization failed! Restarting...");
    delay(1000);
    ESP.restart();
  }

  xTaskCreate(
    canRxTask,
    "canRxTask",
    8192,
    NULL,
    1,
    NULL);

  xTaskCreate(
    sensorTask,
    "sensorTask",
    8192,
    NULL,
    2,
    NULL);
}

void loop() {
  delay(2);
}