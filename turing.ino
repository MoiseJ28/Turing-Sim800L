
#define MODEM_RST      5 //define the pins used by SIM800L
#define MODEM_PWKEY    4
#define MODEM_POWER_ON       23
#define MODEM_TX       27
#define MODEM_RX       26
#define I2C_SDA  21
#define I2C_SCL  22

#define LORA_SS 18    //define the pins used by LoRa
#define LORA_RST 0
#define LORA_DIO0 2

#define LED_BLUE  15
#define WDT_TIMEOUT 180


#define SerialMon Serial // Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialAT  Serial1 // Set serial for AT commands (to the module)

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <Wire.h>
#include <TinyGsmClient.h>
#include <Update.h>
#include "utilities.h"
#include <ArduinoHttpClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <esp_task_wdt.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define DEBUG_PRINT(...) { SerialMon.print(millis()); SerialMon.print(" - "); SerialMon.println(__VA_ARGS__); }
#define DEBUG_FATAL(...) { SerialMon.print(millis()); SerialMon.print(" - FATAL: "); SerialMon.println(__VA_ARGS__); delay(10000); ESP.restart(); }

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "internet"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = "5643"; // SIM card PIN code, if any


String firmwareOverTheAirURL = "http://13.244.86.214/firmware/fetch.php"; // URL to download the firmware from
String configOverTheAir = "http://13.244.86.214/config/fetch.php";

Client* client = NULL;


String confString = "";   // TODO: Fetch config from

String GVM_msg;
String id = "H10";
String LoRaMessage;
float gvm;

char testInput[200];
float myArray[3][14] = {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }};
      
int counter = 0;
int row = 0;

  float ax11, ax12, ld1, gl1,gt1, td1, d1, wm1, sn1;
  float ax21, ax22, ld2, td2, gl2, gt2, d2, wm2, sn2;
  float ax31, ax32, ld3, td3, gl3, gt3, d3, wm3, sn3;

float Amber_on = 20.0;
float Amber_max = 40.0;
float Green_on_min = 45.0;
float Green_on_max = 47.2;
float Green_f = 49.0;

int bin_1, bin_2;

long prevMillis = 0;
int interval = 60000;
boolean ledState = false;

TaskHandle_t httpThread;



String fetchConfig(const String& config_url)
{
  SerialMon.print("fetching config from : ");
  SerialMon.println(config_url);

  String protocol, host, url;
  int port;

  if (!parseURL(config_url, protocol, host, port, url)) {
    DEBUG_FATAL(F("Cannot parse URL"));
  }

  const char server[] = "13.244.86.214";  // Server details
  const char resource[] = "/config/fetch.php";

  client = new TinyGsmClient(modem);
  if (!client->connect(host.c_str(), port)) {
    DEBUG_FATAL(F("Client not connected"));
  }
  
  HttpClient http(*client, server, port);

  int err = http.get(resource);
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(10000);
    DEBUG_FATAL(F("Connection timed out in fetchConfig"));
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(10000);
    DEBUG_FATAL(F("Request timed out in fetchConfig"));
  }

  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
  String headerName = http.readHeaderName();
  String headerValue = http.readHeaderValue();
  SerialMon.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
  SerialMon.print(F("Content length is: "));
  SerialMon.println(length);
  }
  if (http.isResponseChunked()) {
  SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  SerialMon.print(F("Body length is: "));
  SerialMon.println(body.length());
  // Shutdown
  http.stop();
  SerialMon.println(F("Server disconnected"));
  return body;
}

void startOtaUpdate(const String& ota_url)
{
  String protocol, host, url;
  int port;

  if (!parseURL(ota_url, protocol, host, port, url)) {
    DEBUG_FATAL(F("Cannot parse URL"));
  }

  DEBUG_PRINT(String("Connecting to ") + host + ":" + port);

  if (protocol == "http") {
    client = new TinyGsmClient(modem);
    if (!client->connect(host.c_str(), port)) {
      DEBUG_FATAL(F("Client not connected"));
    }
  } else if (protocol == "https") {
    client = new TinyGsmClientSecure(modem);
    if (!client->connect(host.c_str(), port)) {
      DEBUG_FATAL(F("Client not connected"));
    }
  } else {
    DEBUG_FATAL(String("Unsupported protocol: ") + protocol);
  }

  DEBUG_PRINT(String("Requesting ") + url);

  client->print(String("GET ") + url + " HTTP/1.0\r\n"
          + "Host: " + host + "\r\n"
          + "Connection: keep-alive\r\n"
          + "\r\n");

  long timeout = millis();
  while (client->connected() && !client->available()) {
    if (millis() - timeout > 50000L) {
      DEBUG_FATAL("Response timeout");
    }
  }

  // Collect headers
  String md5;
  int contentLength = 0;

  while (client->available()) {
    String line = client->readStringUntil('\r\n');

    line.trim();
    //SerialMon.println(line);    // Uncomment this to show response headers
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
    } else if (line.startsWith("x-md5:")) {
      md5 = line.substring(line.lastIndexOf(':') + 1);
    } else if (line.length() == 0) {
      break;
    }
  }

  if (contentLength <= 0) {
    DEBUG_FATAL("Content-Length not defined");
  }

  
  // The server will respond with "NO UPDATE" when there's no pending update
  if (client->connected() && client->available()) {
    uint8_t tmp_buff[9];
    client->read(tmp_buff, sizeof(tmp_buff));
    if (!strncmp((char *)tmp_buff,"NO UPDATE",sizeof(tmp_buff))) {
      SerialMon.println("No pending software update");
      return;
    }
  }

  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    Update.printError(SerialMon);
    DEBUG_FATAL("OTA begin failed");
  }

  if (md5.length()) {
    DEBUG_PRINT(String("Expected MD5: ") + md5);
    if(!Update.setMD5(md5.c_str())) {
      DEBUG_FATAL("Cannot set MD5");
    }
  }

  DEBUG_PRINT("Flashing...");

  // The next loop does approx. the same thing as Update.writeStream(http) or Update.write(http)
  int written = 0;
  int progress = 0;
  uint8_t buff[256];

  while (client->connected() && written < contentLength) {
    timeout = millis();
    while (client->connected() && !client->available()) {
      delay(1);
      if (millis() - timeout > 50000L) {
        DEBUG_FATAL("Timeout");
      }
    }

    int len = client->read(buff, sizeof(buff));
    
    if (len <= 0) 
      continue;

    Update.write(buff, len);
    written += len;

    int newProgress = (written*100)/contentLength;
    if (newProgress - progress >= 5 || newProgress == 100) {
      progress = newProgress;
      SerialMon.print(String("\r ") + progress + "%");
    }
  }
  SerialMon.println();

  if (written != contentLength) {
    Update.printError(SerialMon);
    DEBUG_FATAL(String("Write failed. Written ") + written + " / " + contentLength + " bytes");
  }

  if (!Update.end()) {
    Update.printError(SerialMon);
    DEBUG_FATAL(F("Update not ended"));
  }

  if (!Update.isFinished()) {
    DEBUG_FATAL(F("Update not finished"));
  }

  DEBUG_PRINT(" Update successfully completed. Rebooting.");
  ESP.restart();
}

void setup() {
  SerialMon.begin(9600); // Set console baud rate

  pinMode(LED_BLUE, OUTPUT);
  
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  //SerialMon.setDebugOutput(true);
  printDeviceInfo();
  
 //Battery
 // Keep power when running from battery
 // Wire.begin(I2C_SDA, I2C_SCL);
 // bool isOk = setPowerBoostKeepOn(1);
 // SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  SerialMon.println(" Firmware A is running");
  SerialMon.println("--------------------------");
  DEBUG_PRINT(F("Starting OTA update in 5 seconds..."));
  delay(5000);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DEBUG_PRINT(F("Initializing modem..."));
  modem.restart();
  // Or, use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  DEBUG_PRINT(String("Modem: ") + modemInfo);

  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {  // Unlock your SIM card with a PIN if needed
    modem.simUnlock(simPIN);
  }

  DEBUG_PRINT(F("Waiting for network..."));
  if (!modem.waitForNetwork(240000L)) {
  DEBUG_FATAL(F("Network failed to connect"));
  }

  DEBUG_PRINT(F("Connecting to GPRS"));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
  DEBUG_FATAL(F("APN failed to connect"));
  }

  //create a separate thread on CORE1 to handle HTTP tasks
  xTaskCreatePinnedToCore(
    HTTPThread,       /* Task function. */
    "httpThread",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &httpThread,    /* Task handle to keep track of created task */
    0);        /* pin task to core 1 */

  // LoRa Setup
  SPI.begin(/*CLK*/ 14, /*MISO*/ 12, /*MOSI*/ 13, /*SS*/ 18);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0); //setup LoRa transceiver module
  while (!LoRa.begin(868E6)) {//replace the LoRa.begin()with your location's frequency
  SerialMon.println(".");
  }
  LoRa.setSyncWord(0xf3); // The sync word assures you don't get LoRa messages from other LoRa transceivers, (0xF3) to match the receiver
  SerialMon.println("LoRa Initializing OK!");
    //Watchdog Timer
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
}

void HTTPThread( void * pvParameters ) {
  
  SerialMon.println("HTTP thread running");
  
  for(;;){

  digitalWrite(LED_BLUE, HIGH);
  delay(200);
  digitalWrite(LED_BLUE, LOW);
  delay(200);
  
    if (millis() - prevMillis > interval) {
      
     String response = fetchConfig(configOverTheAir);  // Check for any available config
     SerialMon.println("The file content is: " + response);

     startOtaUpdate(firmwareOverTheAirURL); // Check for any pending updates
      
      prevMillis = millis();
    }
  }
}



void loop() {
  if (SerialMon.available() > 0) {
    SerialMon.readBytesUntil('\n', testInput, 200);
    //SerialMon.println(testInput);
    readData();
    Logic_1();
    Logic_2();
    sendData();
  }
}

void readData(){

  for (int i = 0; i < 4; i++) {
    SerialMon.readBytesUntil('\n', testInput, 200);
    char* pch = strtok(testInput,"US;\r\n");

    while(pch != NULL) {
      myArray[i][counter] = atof(pch);
      pch = strtok(NULL,"S;\r\n");
      counter += 1;
    }

    esp_task_wdt_reset();
    counter = 0;


    ax11 = myArray[0][0];
    ax12 = myArray[0][1];
    ld1 = myArray[0][4];
    td1 = myArray[0][5];
    gl1 = myArray[0][6];
    gt1 = myArray[0][7];
    d1 = myArray[0][8];
    wm1 = myArray[0][9];
    sn1 = myArray[0][10];

    ax21 = myArray[1][0];
    ax22 = myArray[1][1];
    ld2 = myArray[1][4];
    td2 = myArray[1][5];
    gl2 = myArray[1][6];
    gt2 = myArray[1][7];
    d2 = myArray[1][8];
    wm2 = myArray[1][9];
    sn2 = myArray[1][10];

    ax31 = myArray[2][0];
    ax32 = myArray[2][1];
    ld3 = myArray[2][4];
    td3 = myArray[2][5];
    gl3 = myArray[2][6];
    gt3 = myArray[2][7];
    d3 = myArray[2][8];
    wm3 = myArray[2][9];
    sn3 = myArray[2][10];

  }



}

void Logic_1(){
   
  if(gt1 < Amber_on) {
    bin_1 = 0;
  }

  else if(Amber_on < gt1 && gt1 <= Amber_max) {
    bin_1 = 1;
  }

  else if(Amber_max < gt1 && gt1 <= Green_on_min) {
    bin_1 = 2;
  }

  else if(Green_on_min < gt1 && gt1 <= Green_on_max)  {
    bin_1 = 4;
  }

  else if (Green_on_max < gt1 && gt1 < Green_f) {
    bin_1 = 3;
  }

  else if (gt1> Green_f) {
    bin_1 = 5;
  }

  else {
    bin_1 = 0;
  }
  
}

void Logic_2(){
   
  if(gt1 < Amber_on) {
    bin_2 = 0;
  }

  else if(Amber_on < gt1 && gt1 <= Amber_max) {
    bin_2 = 1;
  }

  else if(Amber_max < gt1 && gt1 <= Green_on_min) {
    bin_2 = 2;
  }

  else if(Green_on_min < gt1 && gt1 <= Green_on_max)  {
    bin_2 = 4;
  }

  else if (Green_on_max < gt1 && gt1 < Green_f) {
    bin_2 = 4;
  }

  else if (gt1> Green_f) {
    bin_2 = 5;
  }

  else {
    bin_2 = 0;
  }
  
}

void sendData(){
  
  LoRaMessage = String(gt1) + ";" + String(id) + '@' +  String(bin_1)+ '#' +  String(bin_2);
  
  SerialMon.println(LoRaMessage);
  LoRa.beginPacket();
  LoRa.print(LoRaMessage);
  LoRa.endPacket();
}
