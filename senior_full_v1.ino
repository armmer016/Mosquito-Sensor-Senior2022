// Upload
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <MD5.h>
#include "Timer.h"

// I2S
#include <driver/i2s.h>
#include <SPIFFS.h>
#include "soc/i2s_reg.h"

// Prepare the pins
#define I2S_SCK 14
#define I2S_WS 15
#define I2S_SD 32

//AIS
#include "AIS_SIM7020E_API.h"
AIS_SIM7020E_API nb;
#define serverIP "159.65.12.98"
#define serverPort "21000"


#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE (16000)
#define I2S_SAMPLE_BITS (16)
#define I2S_READ_LEN (15 * 640)
#define RECORD_TIME (0.3)  //seconds
#define I2S_CHANNEL_NUM (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)
#define trigger_pin 22
#define onboard_led 2

int isMosDetected = 0;
int allowCount = 1;
int prevInput = 1;
int count = 0;


int fileId = 0;



Timer timer(MICROS);


// Define the file to be recorded
File file;
char filename[10][16] = { "/recording0.wav", "/recording1.wav", "/recording2.wav", "/recording3.wav", "/recording4.wav", "/recording5.wav", "/recording6.wav", "/recording7.wav", "/recording8.wav", "/recording9.wav" };
const int headerSize = 44;

// File system setup
#define FILESYSTEM SPIFFS
// You only need to format the filesystem once
#define FORMAT_FILESYSTEM false
#define DBG_OUTPUT_PORT Serial

#if FILESYSTEM == FFat
#include <FFat.h>
#endif
#if FILESYSTEM == SPIFFS
#include <SPIFFS.h>
#endif

// Wi-Fi login, setup server
const char* ssid = "Arm";
const char* password = "16222621";

// const char* host = "esp32fs";
// WebServer server(80);

//holds the current upload
// File fsUploadFile;

#if !(USING_DEFAULT_ARDUINO_LOOP_STACK_SIZE)
uint16_t USER_CONFIG_ARDUINO_LOOP_STACK_SIZE = 16384;
//uint16_t USER_CONFIG_ARDUINO_LOOP_STACK_SIZE = 8192;
#endif

// ------------------------------------------------------------------------------------------------------- //

//Function to verify if a file exists
bool exists(String path) {
  bool yes = false;
  File file = FILESYSTEM.open(path, "r");
  if (!file.isDirectory()) {
    yes = true;
  }
  file.close();
  return yes;
}

// byte to hex

/* Returns a string representation of _length_ bytes beginning at
   _array_.  The caller has responsibility to release the string,
   using _free()_.  If the string could not be allocated, returns a
   null pointer instead. */
char* to_hex_string(const unsigned char* array, size_t length, int fid, int chunkid) {
  char* outstr = (char*)calloc(2 * length + 1, sizeof(char));
  if (!outstr) return outstr;

  char* p = outstr;
  //  p = itoa(fid,p,10);
  p += sprintf(p, "%d%03d", fid, chunkid);
  for (size_t i = 0; i < length - 2; ++i) {
    p += sprintf(p, "%02hhx", array[i]);
  }

  return outstr;
}


//Setup function
void setup(void) {
  DBG_OUTPUT_PORT.begin(115200);
  DBG_OUTPUT_PORT.print("setup\n");
  DBG_OUTPUT_PORT.setDebugOutput(true);
  //start udp connection
  nb.begin(serverIP, serverPort);
  if (nb.pingIP(serverIP).status) {
    DBG_OUTPUT_PORT.printf("ping %s success!\n", serverIP);
  } else {
    DBG_OUTPUT_PORT.printf("ping %s fail!\n", serverIP);
  }


  if (FORMAT_FILESYSTEM) FILESYSTEM.format();
  FILESYSTEM.begin();
  {
    File root = FILESYSTEM.open("/");
    File file = root.openNextFile();
    while (file) {
      String fileName = file.name();
      size_t fileSize = file.size();
      DBG_OUTPUT_PORT.printf("FS File: %s\n", fileName.c_str());
      file = root.openNextFile();
    }
    DBG_OUTPUT_PORT.printf("\n");
  }
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed!");
    while (1) yield();
  }


  for (int i = 0; i < 10; i++) {
    SPIFFS.remove(filename[i]);
  }
  // The I2S PART

  i2sInit();
  pinMode(trigger_pin, INPUT_PULLUP);
  //blink when the trigger send signal
  pinMode(onboard_led, OUTPUT);

  Serial.println("Setup finished! Trigger is ready!");
}

//blink function
void blink_led() {
  digitalWrite(onboard_led, HIGH);
  delay(100);
  digitalWrite(onboard_led, LOW);
}

int totalLoop = 0;
//Loop function
void loop(void) {
  totalLoop += 1;
  isMosDetected = digitalRead(trigger_pin);
  Serial.println(totalLoop);
  delay(10000);
  // server.handleClient();
  delay(2);  //allow the cpu to switch to other tasks
  if (totalLoop <= 1) {
    // if (!isMosDetected) {         //isMosDetected is changed to low(0) state >> there exists input signal obtained from trigger pulse
    prevInput = isMosDetected;  //set prevInput to low(0) >> to note the previous state of an input pin
    if (true) {
      // if (allowCount) {

      timer.start();
      int fIdNow = fileId;
      SPIFFSInit(fIdNow);

      xTaskCreate(i2s_adc, "i2s_adc", 1024 * 3, NULL, 1, NULL);  //recording
      delay((RECORD_TIME * 1000) + 300);                         // wait for recording and finish file writing

      xTaskCreate(udp_upload, "udp_upload", 1024 * 20, (void*)&fIdNow, 1, NULL);  //uploading
      delay(10);                                                                  // wait for uploading task to assign localId
      fileId++;
      count++;
      allowCount = 0;  //set to not allow counting if it still read low(0)
      Serial.print("\nMoqsuito count: ");
      Serial.print(count);
      Serial.print("\n");
      blink_led();
    }

  } else if (isMosDetected && !prevInput) {  //once receiving the first high(1) state after receiving several low(0) state
    prevInput = isMosDetected;
    allowCount = 1;
  }
}

//Initialize SPIFFS (Space Efficient File Manager System)
void SPIFFSInit(int localId) {

  SPIFFS.remove(filename[localId]);
  file = SPIFFS.open(filename[localId], FILE_WRITE);
  if (!file) {
    Serial.println("File is not available!");
  }
}

//Initialize I2S - default for ESP32
void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,                             // 16000 Hz
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),  // 32Bit
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,               // Only Right
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 15,
    .dma_buf_len = 640,  //640 * 15 = 9600 bytes
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,  // gpio 14 BCLK
    .ws_io_num = I2S_WS,    // gpio 15 LRCL
    .data_out_num = -1,     //
    .data_in_num = I2S_SD   // gpio 32 DOUT
  };
  // FIXES for SPHO645
  // REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
  // REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
  i2s_set_pin(I2S_PORT, &pin_config);
}

//Data Processor
void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2) {
    //Serial.print("dac: ");
    //Serial.println(dac_value);
    dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}


void i2s_adc(void* arg) {

  int i2s_read_len = I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read;

  char* i2s_read_buff = (char*)calloc(i2s_read_len, sizeof(char));
  uint8_t* flash_write_buff = (uint8_t*)calloc(i2s_read_len, sizeof(char));


  i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

  Serial.println(" *** Recording Start *** ");
  timer.stop();
  Serial.print(timer.read());
  Serial.print("microsec\n");
  while (flash_wr_size < FLASH_RECORD_SIZE) {
    //read data from I2S bus, in this case, from ADC.
    i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    // example_disp_buf((uint8_t*) i2s_read_buff, 64);
    //save original data from I2S(ADC) into flash.
    i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
    file.write((const byte*)flash_write_buff, i2s_read_len);

    flash_wr_size += i2s_read_len;

    Serial.print("\nrecording run from core: ");
    Serial.print(xPortGetCoreID());
    Serial.print("\n");
    ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    // ets_printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
  }


  file.close();

  free(i2s_read_buff);
  i2s_read_buff = NULL;
  // free(buf);
  free(flash_write_buff);
  flash_write_buff = NULL;

  listSPIFFS();


  // Serial.print("send_chunk = true");
  // send_chunk = true;

  vTaskDelete(NULL);
}

void udp_upload(void* arg) {

  int localId = (int)*((int*)arg);


  fs::File fileUp = SPIFFS.open(filename[localId], "rb");
  size_t read_len = 510;

  Serial.println(fileUp.size());
  Serial.println(filename[localId]);

  byte buffer[read_len + 2];
  // int totalChunk = int();
  // char buf[read_len * 2 + 2];
  for (int k = 0; k < (fileUp.size() / read_len) + 0.5; k++) {


    fileUp.read(buffer, read_len);

    Serial.print("\n *** Converting to hex *** : #chunk");
    Serial.print(k);
    Serial.println("");
    Serial.print("\nudp_upload run from core: ");
    Serial.print(xPortGetCoreID());
    Serial.print("\n");
    char* outstr = to_hex_string(buffer, sizeof(buffer), localId, k);
    if (!outstr) {
      fprintf(stderr, "Failed to allocate memory\n");
    }
    Serial.println(String("Total length: ").concat(String(read_len + 2).concat(" bytes")));
    unsigned char* hash = MD5::make_hash(outstr);
    //generate the digest (hex encoding) of our hash
    char* md5str = MD5::make_digest(hash, 16);
    //print it on our serial monitor
    Serial.println(md5str);
    //Give the Memory back to the System if you run the md5 Hash generation in a loop
    free(md5str);
    //free dynamically allocated 16 byte hash from make_hash()
    free(hash);
    nb.sendMsgHEX(serverIP, serverPort, outstr);
    delay(900);
    free(outstr);
    Serial.println("");
  }
  String end = "end";
  end.concat(localId);

  nb.sendMsgSTR(serverIP, serverPort, end);
  delay(900);
  // String retdata = "";
  // nb.waitResponse(retdata,serverIP);
  // if(retdata!="")Serial.println(retdata);
  fileId--;  // finish the task and decrease fileId by 1
  fileUp.close();

  // free(buffer);
  // free(buf);
  vTaskDelete(NULL);
}

//Upload manager to show the progress
void example_disp_buf(uint8_t* buf, int length) {
  printf("======\n");
  for (int i = 0; i < length; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 8 == 0) {
      printf("\n");
    }
  }
  printf("======\n");
}


//SPIFFS File List
void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM = "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name                              Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length();  // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String)file.size();
      spaces = 10 - fileSize.length();  // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  Serial.println(FPSTR(line));
  Serial.println();
  delay(1000);
}