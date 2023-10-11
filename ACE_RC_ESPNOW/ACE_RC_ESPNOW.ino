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
#define PRINTRXPACKETS 1
#define PRINTTXPACKETS 1

#define PINGMODE 1
#define CONTROLLER 0 // 0 - Controllee, 1 = Controller

// AP SSID
const char *SSID = "ACE-Controller";

// Global copy of peer
esp_now_peer_info_t peer;
const uint8_t ControllerAddress[] = {0x24, 0x0A, 0xC4, 0x58, 0x38, 0x58}; // 24:0A:C4:58:38:58
const uint8_t ControlleeAddress[] = {0x24, 0x0A, 0xC4, 0x58, 0x2D, 0xB8}; // 24:0A:C4:58:2D:B8
// Keep track of ping durations
uint8_t milliseconds = 0;
// ping data 
uint8_t * sentData;
// witing on response flag
bool waiting = false;

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
void sendData(const uint8_t * data) {
  const uint8_t *peer_addr = peer.peer_addr;
  Serial.print("Sending: "); Serial.println(*data);
  esp_err_t result = esp_now_send(peer_addr, data, sizeof(data));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is recv from Peer
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (PRINTRXPACKETS){
    Serial.print("Last Packet Recv from: "); Serial.println(macStr);
    Serial.print("Last Packet Recv Data: "); Serial.println(*data);
    Serial.println("");
  }
  // In pingmode send received data back for data Validation
  if (!CONTROLLER && PINGMODE)
  {
    sendData(data);
  }else if (CONTROLLER && waiting && PINGMODE)
  {
    Serial.print(milliseconds); Serial.println(" milliseconds ping.");
    waiting = false;
    Serial.print(*sentData); Serial.println(" - Data Sent.");
    Serial.print(*data); Serial.println(" - Data Received.");
    if (sentData == data){
      Serial.println("Data Successfully Validated");
    }else{
      Serial.println("Data Corrupt");
    }
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

uint8_t* stringToBytes(char* data, int length){
  uint8_t * output = 0x00;
  for (int i = 0; i < length; i++){
    output[i] = (uint8_t)data[i];
  }
  return output;
}
char* bytesToString(uint8_t* data, int length){
  char * output = "";
  for (int i = 0; i < length; i++){
    output[i] = (char)data[i];
  }
  return output;
}
uint8_t* generateRandomData(int length){
  uint8_t * output = 0x00;
  for (int i = 0; i < length; i++){
    output[i] = (uint8_t)random(0, 255);
  }
  return output;
}
void setup() {
  Serial.begin(19200);

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
}

void loop() {
  if (milliseconds > 2500) waiting = false;
  // Check pairing status
  bool isPaired = pairToPeer(); 
  if (isPaired) {

    // pair success or already paired
    // Send data to device
    if (CONTROLLER && PINGMODE && !waiting) {
      // get random data
      //sentData = generateRandomData(5);

      // send data
      //sendData(sentData);
      waiting = true;
      // reset millisecond counter
      milliseconds = 0;
    }
  } else {
    // peer pair failed
    Serial.println("peer pair failed!");
  }

  // wait for 1 milliseconds to run the logic again
  milliseconds++;
  delay(1);
}
