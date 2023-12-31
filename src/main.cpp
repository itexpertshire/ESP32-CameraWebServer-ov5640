
/*
Here's an example code for the ESP32-CAM board with an OV5640 camera module that streams video feed on a web server. Make sure you have installed the "esp32" board support in the Arduino IDE before proceeding.
 
Replace your_SSID and your_PASSWORD with your WiFi network credentials. Upload the code to your ESP32-CAM board using an FTDI programmer or similar method.
 
After uploading, open the Serial Monitor and press the reset button on the ESP32-CAM board. Note down the IP address displayed in the Serial Monitor. Open a web browser and enter the IP address. You should see the video feed from the OV5640 camera module.
 
Please note that this code assumes you are using the AI-THINKER ESP32-CAM model with an OV5640 camera module. If you're using a different model or configuration, you might need to modify the camera pins according to your specific board.
 
Also, make sure you have selected the "ESP32 Wrover Module" board in the Arduino IDE with the "Huge APP" partition scheme before uploading the code.
*/


#include "esp_camera.h"
#include <WiFi.h>
#include "app_httpd.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
 
// Replace with your network credentials
char camModel[10];
int light=0;
int server_port = 867;

#define WIFI_TIMEOUT_S  60
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_IR
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

#if !( defined(ESP32) )
  #error This code is designed for (ESP32 + W5500) to run on ESP32 platform! Please check your Tools->Board setting.
#endif

#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial

// Debug Level from 0 to 4
#define _ETHERNET_WEBSERVER_LOGLEVEL_       4

//////////////////////////////////////////////////////////

// Optional values to override default settings
// Don't change unless you know what you're doing
#define ETH_SPI_HOST        SPI2_HOST
#define SPI_CLOCK_MHZ       25

//AI-Thinker Board
#define INT_GPIO            2   //0 if 2 doesn't work

#define MISO_GPIO           12
#define MOSI_GPIO           13
#define SCK_GPIO            14
#define CS_GPIO             15
//////////////////////////////////////////////////////////

#include <WebServer_ESP32_W5500.h>
// Enter a MAC address and IP address for your controller below.
#define NUMBER_OF_MAC      20

byte mac[][NUMBER_OF_MAC] =
{
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x04 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x06 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x08 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x09 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0A },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0B },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0C },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0D },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0E },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0F },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x11 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x12 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x13 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x14 },
};

// Select the IP address according to your local network
IPAddress myIP(192, 168, 2, 233);
IPAddress myGW(192, 168, 2, 1);
IPAddress mySN(255, 255, 255, 0);

// Google DNS Server IP
IPAddress myDNS(8, 8, 8, 8);


#if defined(CAMERA_MODEL_AI_THINKER)
 #define CONFIG_LED_ILLUMINATOR_ENABLED 1
#endif

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
 
void setup() {

  //esp_log_level_set("*", ESP_LOG_NONE); To enable debugging update platformio.ini with  -DCORE_DEBUG_LEVEL=5

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  #if defined(CAMERA_MODEL_WROVER_KIT)
// initialize the Blue pin as an output to display WiFi connection status
 
  pinMode(BLUE_LED_GPIO_NUM, OUTPUT);
  digitalWrite(BLUE_LED_GPIO_NUM, HIGH);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,1); //1 = High, 0 = Low
#endif

  #if defined(CAMERA_MODEL_AI_THINKER)
// initialize the Red led as an output to display WiFi connection status
 
  pinMode(RED_LED_GPIO_NUM, OUTPUT);
  digitalWrite(RED_LED_GPIO_NUM, LOW);
  ///Enable Led Flash
  //setupLedFlash(FLASH_LAMP);
#endif


  ET_LOGWARN(F("Default SPI pinout:"));
  ET_LOGWARN1(F("SPI_HOST:"), ETH_SPI_HOST);
  ET_LOGWARN1(F("MOSI:"), MOSI_GPIO);
  ET_LOGWARN1(F("MISO:"), MISO_GPIO);
  ET_LOGWARN1(F("SCK:"),  SCK_GPIO);
  ET_LOGWARN1(F("CS:"),   CS_GPIO);
  ET_LOGWARN1(F("INT:"),  INT_GPIO);
  ET_LOGWARN1(F("SPI Clock (MHz):"), SPI_CLOCK_MHZ);
  ET_LOGWARN(F("========================="));

  ///////////////////////////////////

  // To be called before ETH.begin()
  ESP32_W5500_onEvent();

  // start the ethernet connection and the server:
  // Use DHCP dynamic IP and random mac
  //bool begin(int MISO_GPIO, int MOSI_GPIO, int SCLK_GPIO, int CS_GPIO, int INT_GPIO, int SPI_CLOCK_MHZ,
  //           int SPI_HOST, uint8_t *W6100_Mac = W6100_Default_Mac);
  ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST );
  //ETH.begin( MISO_GPIO, MOSI_GPIO, SCK_GPIO, CS_GPIO, INT_GPIO, SPI_CLOCK_MHZ, ETH_SPI_HOST, mac[millis() % NUMBER_OF_MAC] );

  // Static IP, leave without this line to get IP via DHCP
  //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  //ETH.config(myIP, myGW, mySN, myDNS);

  ESP32_W5500_waitForConnect();
  delay(1000);
  String local_ip = ETH.localIP().toString();
  Serial.print(F("HTTP EthernetWebServer is @ IP : "));
  Serial.println(local_ip);
  ///////////////////////////////////
#if defined(CAMERA_MODEL_AI_THINKER)   
      if (local_ip == "0.0.0.0") {
        digitalWrite(RED_LED_GPIO_NUM, LOW); //Switch On
      } else {
       digitalWrite(RED_LED_GPIO_NUM, HIGH); //Switch Off
      }
#endif

 // Start streaming web server
  startCameraServer(server_port);
  server_port +=1;
  Serial.println("Camera Ready! Use below link,");
  Serial.println("Stream Link: http://"+WiFi.localIP().toString()+":"+String(server_port)+"/stream");
  Serial.println("' to connect");

    // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 8000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_count = 1;

  #if defined(CAMERA_MODEL_WROVER_KIT) || defined(CAMERA_MODEL_AI_THINKER)
    //config.frame_size = FRAMESIZE_UXGA;
    config.fb_location    = CAMERA_FB_IN_PSRAM; //*!< The location where the frame buffer will be allocated */
    config.grab_mode      = CAMERA_GRAB_LATEST;// CAMERA_GRAB_LATEST;  //*!< When buffers should be filled */
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
  #endif
 
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  delay(1200);

  sensor_t * s = esp_camera_sensor_get();
      switch (s->id.PID) {
        case (OV2640_PID):
          strcpy(camModel, "OV2640");
        break;
        case (OV3660_PID):
          strcpy(camModel, "OV3660");
        break;
        case (OV5640_PID):
          strcpy(camModel, "OV5640");

          s->set_vflip(s, 1);          // 0 = disable , 1 = enable

          s->set_brightness(s,0);
          s->set_contrast(s,0);
          s->set_saturation(s,4);
          s->set_sharpness(s,3);
          s->set_denoise(s,0);
          s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
          s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
          s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
          s->set_exposure_ctrl(s, 0);  // 0 = disable , 1 = enable
          s->set_aec2(s, 1);           // 0 = disable , 1 = enable
          s->set_ae_level(s, 0);       // -2 to 2
          //s->set_aec_value(s, 0);    // 0 to 1200
          s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
          s->set_agc_gain(s, 2);       // 0 to 30
          s->set_gainceiling(s, (gainceiling_t)6);  // 0 to 6
          s->set_bpc(s, 1);            // 0 = disable , 1 = enable
          s->set_wpc(s, 1);            // 0 = disable , 1 = enable
          s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
          s->set_lenc(s, 1);           // 0 = disable , 1 = enable
          s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
          s->set_dcw(s, 1);            // 0 = disable , 1 = enable
          s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
          
        break;
        default:
          strcpy(camModel, "Other");
        break;
      }
      Serial.printf("Camera init OK for model %s ", camModel);
      delay(1000);

      s->set_reg(s,0xff,0xff,0x01);//banksel    

      light=s->get_reg(s,0x2f,0xff);
      Serial.print("First light is ");
      Serial.println(light);
 

 
 
}
 
void loop() {
  // Nothing to do here
}
