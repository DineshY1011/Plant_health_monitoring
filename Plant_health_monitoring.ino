/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <DineshY-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <FirebaseESP32.h>
#include "esp_camera.h"
#include <DHT.h>
#include <WiFi.h>

// Replace with your network credentials
#define WIFI_SSID "dineshnp2"
#define WIFI_PASSWORD "dinesh21"

// Replace with your Firebase project details
#define DATABASE_URL "https://agronexus-1164b-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define API_KEY "AIzaSyCa7NF1okVy8ZlLplJ1dpRkh5b31885UGM"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

#define DHTPIN 15        // Pin where the DHT sensor is connected
#define DHTTYPE DHT11    // Type of DHT sensor (use DHT22 if needed)

DHT dht(DHTPIN, DHTTYPE);

#define SOIL_MOISTURE_PIN 14
#define TRIG_PIN 12  // Ultrasonic Sensor Trigger pin
#define ECHO_PIN 13
#define PUMP_PIN 2

const char *ssid = "dineshnp2";   
const char *password = "dinesh21";   
int status = WL_IDLE_STATUS; 

const char* apiKey = "n9tsbwMwSZgj";       
const char* templateID = "101";           
const char* mobileNumber = "919500134159"; 
const char* var1 = "Water tank level";         
String var2 = "350ml";                  

void printWifiStatus() { 
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
 
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
 
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Container dimensions (adjust according to your container)
const float totalHeightCm = 7.0;          // Total container height in cm
const float widthCm = 11.5;                // Width of container in cm
const float lengthCm = 17.0;               // Length of container in cm
const float baseAreaCm2 = widthCm * lengthCm; // Base area in square cm

bool alertSent = false;  // To track if SMS is already sent

float calculateWaterVolume() {
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo time
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance to water surface in cm
  float distanceToWater = (duration * 0.034) / 2;

  // Make sure distance is not bigger than container height
  if (distanceToWater > totalHeightCm) {
    distanceToWater = totalHeightCm;
  }

  // Calculate height of water
  float waterHeightCm = totalHeightCm - distanceToWater;
  if (waterHeightCm < 0) {
    waterHeightCm = 0;
  }

  // Volume in milliliters (1 cm³ = 1 ml)
  float volumeMl = waterHeightCm * baseAreaCm2;

  return volumeMl;
}

void checkWaterLevel() {
  float waterLevelMl = calculateWaterVolume();
  int waterLevelInt = (int)waterLevelMl;  // Convert float to int
  
  Serial.print("Water Volume: ");
  Serial.print(waterLevelMl);
  Serial.println(" ml");
  
  if (Firebase.ready() && Firebase.setInt(fbdo, "/sensor/waterlevel", waterLevelInt)) {
    Serial.println("Data sent to Firebase!");
  } else {
    Serial.print("Firebase error: ");
    Serial.println(fbdo.errorReason());
  }
  
  if (waterLevelMl <= 350.0 && !alertSent) {
    // Send SMS only once when below 350ml
    Serial.println("Water level critical! Sending alert...");
    var2 = String(waterLevelMl, 0) + "ml";  // Update the var2 with current level
    sendSMS();
  }
  
  // Reset alert flag if water level rises above threshold
  if (waterLevelMl > 400.0 && alertSent) {
    Serial.println("Water level restored. Alert system reset.");
    alertSent = false;  // Ready to send again if needed
  }
}
void listenForPumpCommands() {
  if (Firebase.ready()) {
    // Listen for manual pump control from the web interface
    if (Firebase.getString(fbdo, "/sensor/pumpControl")) {
      String pumpCommand = fbdo.stringData();
      
      if (pumpCommand == "ON") {
        // Turn the pump ON
        digitalWrite(PUMP_PIN, LOW);  // LOW turns pump ON in your setup
        Serial.println("Pump turned ON manually via Firebase");
        
        // Update the pump status in Firebase
        Firebase.setString(fbdo, "/sensor/pumpstatus", "ON");
        
        // Calculate water amount for 5 second pump run
        int pumpRuntime = 5000; // 5 seconds in milliseconds
        int waterAmount = (pumpRuntime / 1000) * 60; // Assuming 50ml/second
        
        // Record manual watering with details
        unsigned long currentTime = millis();
        FirebaseJson wateringRecord;
        wateringRecord.set("timestamp", currentTime);
        wateringRecord.set("amount", waterAmount);
        wateringRecord.set("type", "manual");
        
        // Get current soil moisture and temperature for the record
        String currentMoisture = digitalRead(SOIL_MOISTURE_PIN) == LOW ? "wet" : "dry";
        float currentTemp = dht.readTemperature();
        
        wateringRecord.set("moisture", currentMoisture);
        wateringRecord.set("temperature", currentTemp);
        
        // Add to watering history
        String path = "/sensor/wateringHistory/" + String(currentTime);
        Firebase.setJSON(fbdo, path, wateringRecord);
        
        // Update last watered timestamp
        Firebase.setInt(fbdo, "/sensor/lastWatered", currentTime);
        
        // Auto-off after 5 seconds to prevent overwatering
        delay(pumpRuntime);
        digitalWrite(PUMP_PIN, HIGH);  // HIGH turns pump OFF
        Firebase.setString(fbdo, "/sensor/pumpstatus", "OFF");
        
        // Reset the command in Firebase
        Firebase.setString(fbdo, "/sensor/pumpControl", "OFF");
        Serial.println("Pump turned OFF after 5 seconds");
      }
    }
  }
}

void sendSMS() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    const char* server = "www.circuitdigest.cloud";
    String apiUrl = "/send_sms";  // Simplified URL, we'll add params in the request
    
    Serial.println("Connecting to server...");
    if (client.connect(server, 80)) {
      Serial.println("connected!");
      
      // Build the proper URL with parameters
      String url = apiUrl + "?ID=" + String(templateID);
      
      // Create proper JSON payload
      String payload = "{\"mobiles\":\"" + String(mobileNumber) + 
                      "\",\"var1\":\"" + String(var1) + 
                      "\",\"var2\":\"" + var2 + "\"}";
      
      // HTTP Headers
      client.println("POST " + url + " HTTP/1.1");
      client.println("Host: " + String(server));
      client.println("Authorization: " + String(apiKey));
      client.println("Content-Type: application/json");
      client.println("Content-Length: " + String(payload.length()));
      client.println("Connection: close");  // Important for proper closure
      client.println();
      client.println(payload);
      
      Serial.println("Request sent:");
      Serial.println("POST " + url + " HTTP/1.1");
      Serial.println("Host: " + String(server));
      Serial.println("Authorization: " + String(apiKey));
      Serial.println("Content-Type: application/json");
      Serial.println("Content-Length: " + String(payload.length()));
      Serial.println("Payload: " + payload);
      
      // Wait for response with timeout
      unsigned long timeout = millis();
      int responseCode = -1;
      bool headerEnded = false;
      String responseBody = "";
      
      while (client.connected() && (millis() - timeout < 10000)) {
        if (client.available()) {
          String line = client.readStringUntil('\n');
          
          if (line.startsWith("HTTP/")) {
            responseCode = line.substring(9, 12).toInt();
            Serial.print("HTTP Response Code: ");
            Serial.println(responseCode);
          }
          
          // Detect end of headers
          if (line == "\r") {
            headerEnded = true;
            continue;
          }
          
          // Collect response body
          if (headerEnded) {
            responseBody += line;
          } else {
            Serial.println("Header: " + line);
          }
        }
      }
      
      Serial.println("Response body: " + responseBody);
      
      if (responseCode == 200) {
        Serial.println("SMS sent successfully!");
        alertSent = true;  // Mark alert as sent
      } else {
        Serial.println("Failed to send SMS. Error code: " + String(responseCode));
        Serial.println("Response: " + responseBody);
        // Try again later by keeping alertSent as false
      }
      
      client.stop();
    } else {
      Serial.println("Connection to server failed!");
    }
  } else {
    Serial.println("WiFi not connected!");
  }
}


// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi.");
    printWifiStatus();
    
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.host = DATABASE_URL;
  config.signer.tokens.legacy_token = API_KEY;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Wait for token to be ready
  unsigned long startMillis = millis();
  while (!Firebase.ready() && millis() - startMillis < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (Firebase.ready()) {
    Serial.println("\nFirebase is ready!");
  } else {
    Serial.println("\nFirebase initialization failed!");
  }
    // Initialize sensors
    dht.begin();
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, HIGH);  // Initial state: pump OFF
    
    //comment out the below line to start inference immediately after upload
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{    
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

    free(snapshot_buf);
    delay(2000);
    
    // Get soil moisture status
    int soilStatus = digitalRead(SOIL_MOISTURE_PIN);
    String moistureStatus;

    if (soilStatus == LOW) {
        moistureStatus = "Wet";
        Serial.println("Soil Moisture: Wet");
    } else {
        moistureStatus = "Dry";
        Serial.println("Soil Moisture: Dry");
    }
      if (Firebase.ready() && Firebase.setString(fbdo, "/sensor/soilmoisture", moistureStatus)) {
    Serial.println("Data sent to Firebase!");
  } else {
    Serial.print("Firebase error: ");
    Serial.println(fbdo.errorReason());
  }

    // Check water level and send to Firebase
    checkWaterLevel();
    
    // Read and send temperature/humidity data
    float temperature = dht.readTemperature(); // Read temperature in Celsius
    float humidity = dht.readHumidity();       // Read humidity in %

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

    }
        if (Firebase.ready() && Firebase.setFloat(fbdo, "/sensor/temperature", temperature)) {
    Serial.println("Data sent to Firebase!");
  } else {
    Serial.print("Firebase error: ");
    Serial.println(fbdo.errorReason());
  }
          if (Firebase.ready() && Firebase.setFloat(fbdo, "/sensor/humidity", humidity)) {
    Serial.println("Data sent to Firebase!");
  } else {
    Serial.print("Firebase error: ");
    Serial.println(fbdo.errorReason());
  }

        // Check if automatic mode is enabled
    bool autoModeEnabled = false;
    if (Firebase.ready() && Firebase.getBool(fbdo, "/sensor/autoMode")) {
        autoModeEnabled = fbdo.boolData();
        Serial.print("Automatic Mode: ");
        Serial.println(autoModeEnabled ? "ON" : "OFF");
    }
    
    // Control the water pump based on soil moisture
    String pumpstatus = "OFF";
    
    // Inside the watering section of your code
if (autoModeEnabled && soilStatus == HIGH) { // Soil is dry and auto mode is on
    Serial.println("Soil is dry! Automatic watering initiated...");
    digitalWrite(PUMP_PIN, LOW);   // Turn pump ON
    pumpstatus = "ON";
    
    // Calculate actual water amount based on pump runtime
    // Assuming pump flow rate is approximately 50ml per second
    int pumpRuntime = 2000; // 2 seconds in milliseconds
    int waterAmount = (pumpRuntime / 1000) * 60; // Convert to ml
    
    // Record watering event with more details
    if (Firebase.ready()) {
        // Get current timestamp
        unsigned long currentTime = millis();
        
        // Create detailed watering record
        FirebaseJson wateringRecord;
        wateringRecord.set("timestamp", currentTime);
        wateringRecord.set("amount", waterAmount);
        wateringRecord.set("type", "automatic");
        wateringRecord.set("moisture", "dry");
        wateringRecord.set("temperature", temperature);
        
        // Add to watering history
        String path = "/sensor/wateringHistory/" + String(currentTime);
        if (Firebase.setJSON(fbdo, path, wateringRecord)) {
            Serial.println("Watering record saved to history");
        } else {
            Serial.print("Firebase error: ");
            Serial.println(fbdo.errorReason());
        }
        
        // Update last watered timestamp
        Firebase.setInt(fbdo, "/sensor/lastWatered", currentTime);
    }
    
    // Keep pump ON for 2 seconds
    delay(pumpRuntime);
    digitalWrite(PUMP_PIN, HIGH);  // Turn pump OFF
    pumpstatus = "OFF";
}
    
    // Update pump status in Firebase
    if (Firebase.ready() && Firebase.setString(fbdo, "/sensor/pumpstatus", pumpstatus)) {
        Serial.println("Pump status sent to Firebase!");
    } else {
        Serial.print("Firebase error: ");
        Serial.println(fbdo.errorReason());
    }
   listenForPumpCommands();
    // Wait before next readings
    Serial.println("Next Readings in 3 seconds...");
    delay(3000);
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif