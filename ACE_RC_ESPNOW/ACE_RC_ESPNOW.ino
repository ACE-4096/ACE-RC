/**

  ACE RC Controller code, tranceiver mode
  Date: 11/10/2023
  Author: ACE

  Based on:
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

#define CHANNEL 1
#define PRINTRXPACKETS 0
#define PRINTTXPACKETS 0
#define DISPLAYSTATS 0

#define TESTMODE 0

#define PINGMODE 1
#define CONTROLLER 1 // 0 - Controllee, 1 = Controller

// INPUTs 
#define XPin 34
#define YPin 35

// OUTPUTs
#define connectionLed 32

#define pingLed1 27
#define pingLed2 26
#define pingLed3 33
#define pingLed4 25


// AP SSID
const char *SSID = "ACE-Controller";

// Global copy of peer
esp_now_peer_info_t peer;
const uint8_t ControllerAddress[] = {0x24, 0x0A, 0xC4, 0x58, 0x38, 0x58}; // 24:0A:C4:58:38:58
const uint8_t ControlleeAddress[] = {0x24, 0x0A, 0xC4, 0x58, 0x2D, 0xB8}; // 24:0A:C4:58:2D:B8

// Keep track of ping durations
int milliseconds = 0;
// witing on response flag
bool waiting = false;

// ping data 
uint8_t sentData[255];
int datalength = 5;
uint8_t receivedData[255];
char receivedText[255];

// statistical data variables
int pingNum = 0;
float lossRatiosCombined = 00.00;
float totalPing = 00.00;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

bool pairToPeer(){
  if (esp_now_is_peer_exist(peer.peer_addr))
  {
    return true;
  }
  // Attempt pair
  esp_err_t addStatus = esp_now_add_peer(&peer);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
    return true;
  } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
    return false;
  } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
    return false;
  } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
    Serial.println("Peer list full");
    return false;
  } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
    return false;
  } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Peer Exists");
    return true;
  } else {
    Serial.println("Not sure what happened");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(peer.peer_addr);
  Serial.print("peer Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// send data
bool sendData(const uint8_t * data, int length) {
  const uint8_t *peer_addr = peer.peer_addr;
  //Serial.print("Sending: "); 
  //Serial.println((char*)data);
  //Serial.println("testpoint 1");
  esp_err_t result = esp_now_send(peer_addr, data, sizeof(data) * length);
  //Serial.println("testpoint 2");
  //Serial.print("Send Status: ");
  if (result == ESP_OK) {
    //Serial.println("Success");
    return true;
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    //Serial.println("ESPNOW not Init.");
    return false;
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    //Serial.println("Invalid Argument");
    return false;
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    //Serial.println("Internal Error");
    return false;
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    //Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    return false;
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    //Serial.println("Peer not found.");
    return false;
  } else {
    //Serial.println("Not sure what happened");
    return false;
  }
}

// callback when data is recv from Peer
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  memcpy(&receivedData, data, data_len);

  if (PRINTRXPACKETS){
    Serial.print("Last Packet Recv from: "); Serial.println(macStr);
    Serial.print("Last Packet Recv Data: "); Serial.println(*data);
    Serial.println("");
  }
  // In pingmode send received data back for data Validation
  if (!CONTROLLER && PINGMODE)
  {
    Serial.print("Received Data Size: "); Serial.println(data_len); 
    Serial.println("Received Data:");
    int i = 0; 
    do 
    {
      Serial.println(receivedData[i++]);
    }while (i < data_len && receivedData[i] != 0x00);
    while (!sendData(receivedData, i)) ;
  }else if (CONTROLLER && waiting && PINGMODE)
  {
    if (!TESTMODE){
      char output[255];
      for (int i = 0; i < data_len; i++){
        output[i] = (char)data[i];
      }
      memcpy(&receivedText, output, data_len);
      Serial.print("Received Text: "); Serial.println((char*)receivedText);
    }
    totalPing += milliseconds;
    int validatedDataCount = 0;
    //Serial.print("Send Data Size: "); Serial.println(datalength);
    //Serial.print("Received Data Size: "); Serial.println(data_len);
    //Serial.println("Sent byte - Received byte");
    for (int i = 0; i < data_len && i < datalength; i++)
    {
      //Serial.print(sentData[i]); Serial.print(" - "); Serial.println(receivedData[i]);
      if (sentData[i] == receivedData[i]) validatedDataCount++;
    }
    float lossRatio = validatedDataCount * (100.00 / datalength);
    lossRatiosCombined += lossRatio;
    if (DISPLAYSTATS){
      Serial.print(milliseconds); Serial.println(" milliseconds ping.");
      Serial.print("Data Validation: "); Serial.println(lossRatio);
      Serial.print("AVG Success Ratio: "); Serial.println(lossRatiosCombined/pingNum);
      Serial.print("AVG ping: "); Serial.print(totalPing/pingNum); Serial.println("");
      Serial.print("Number of pings: "); Serial.println(pingNum);
    }
    waiting = false;
  }
}

void displayPing(){
  if (milliseconds < 100){
    digitalWrite(pingLed1, HIGH);
    digitalWrite(pingLed2, HIGH);
    digitalWrite(pingLed3, HIGH);
    digitalWrite(pingLed4, HIGH);
  }else if (milliseconds < 200){
    digitalWrite(pingLed1, LOW);
    digitalWrite(pingLed2, HIGH);
    digitalWrite(pingLed3, HIGH);
    digitalWrite(pingLed4, HIGH);
  }else if (milliseconds < 300){
    digitalWrite(pingLed1, LOW);
    digitalWrite(pingLed2, LOW);
    digitalWrite(pingLed3, HIGH);
    digitalWrite(pingLed4, HIGH);
  }else if (milliseconds < 400){
    digitalWrite(pingLed1, LOW);
    digitalWrite(pingLed2, LOW);
    digitalWrite(pingLed3, LOW);
    digitalWrite(pingLed4, HIGH);
  }else{
    digitalWrite(pingLed1, LOW);
    digitalWrite(pingLed2, LOW);
    digitalWrite(pingLed3, LOW);
    digitalWrite(pingLed4, LOW);
  }
}

// callback when data is sent to peer
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (PRINTTXPACKETS){
    Serial.print("Last Packet Sent to: "); Serial.println(macStr);
    Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  }
}

uint8_t * stringToBytes(char* data, int length){
  uint8_t output[255];
  for (int i = 0; i < length; i++){
    output[i] = (uint8_t)data[i];
  }
  return output;
}

char * bytesToString(uint8_t* data, int length){
  char output[255];
  for (int i = 0; i < length; i++){
    output[i] = (char)data[i];
  }
  return output;
}

void generateRandomData(int length){
  uint8_t output[255];
  //Serial.print("Generated Data: ");
  for (int i = 0; i < length; i++){
    int x = random(0, 255);
    output[i] = (uint8_t)x;
    //Serial.println(x);
  }
  memcpy(&sentData, output, length);
}


void setup() {
  Serial.begin(19200);

  Serial.println("ESP Setup Started.");

  if (CONTROLLER)
  {
    //define inputs
    pinMode(XPin, INPUT);
    pinMode(YPin, INPUT);

    // Define outputs
    pinMode(connectionLed, OUTPUT);
    pinMode(pingLed1, OUTPUT);
    pinMode(pingLed2, OUTPUT);
    pinMode(pingLed3, OUTPUT);
    pinMode(pingLed4, OUTPUT);

    // set all leds to off
    digitalWrite(connectionLed, LOW);
    digitalWrite(pingLed1, LOW);
    digitalWrite(pingLed2, LOW);
    digitalWrite(pingLed3, LOW);
    digitalWrite(pingLed4, LOW);
  }

  //Set device in STA mode
  WiFi.mode(WIFI_STA);
  // Set Channel
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  // This is the mac address of the Controller in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  
  // Init ESPNow with a fallback logic
  InitESPNow();

  // Switch between addressing depending on role
  if (CONTROLLER) {
    memcpy(&peer.peer_addr, ControlleeAddress, 6);
    Serial.println("ESPNow - Controller Mode");
  }else{
    memcpy(&peer.peer_addr, ControllerAddress, 6);
    Serial.println("ESPNow - Controllee Mode");
  }

  // Set RX & TX callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP Setup Finished.");
}

void loop() {
  if (milliseconds > 2500) waiting = false;
  // Check pairing status
  bool isPaired = pairToPeer(); 
  if (isPaired) {

    if (CONTROLLER) digitalWrite(connectionLed, HIGH);
    // pair success or already paired
    // Send data to device
    if (CONTROLLER && PINGMODE && !waiting) {
      // get random data
      if (TESTMODE){
        generateRandomData(datalength);
      }else{

        // Convert inputs to string
        int xval = analogRead(XPin) / (4096 / 10);
        int yval = analogRead(YPin) / (4096 / 10);
        char data[10];
        sprintf(data, "%d:%d", xval,yval);

        datalength = sizeof(data) / sizeof(data[0]);
        //Serial.print("Sent Data Length: "); Serial.println(datalength);
        uint8_t output[255];
        for (int i = 0; i < datalength; i++){
          output[i] = (uint8_t)data[i];
        }
        memcpy(&sentData, output, datalength);
      }
      // try send data
      if (sendData(sentData, datalength)){
        pingNum++;
        waiting = true;
        // reset millisecond counter
        milliseconds = 0;
      }
      
    }
  } else {
    if (CONTROLLER) digitalWrite(connectionLed, LOW);
    // peer pair failed
    Serial.println("peer pair failed!");
  }

  // wait for 1 milliseconds to run the logic again
  milliseconds++;
  displayPing();
  delay(1);
}
