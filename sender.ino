#include <esp_now.h>
#include <WiFi.h>
#define REED_SWITCH           4
#define NODE_RUNING           0
#define NODE_WARNING          1
#define NODE_IDLE             2
//#define NODE_RUNING         3
#define RETRY_NUMBER          5
#define NODE_ESP_NOW_ERROR    6
#define LED_BUILT_IN          33
#define DEBUG

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0xB8, 0xF0, 0x09, 0xA7, 0xFE, 0xF0};
uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0x18, 0x7F, 0xB8};

typedef struct struct_message {
  int id;
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;
struct_message incomingReadings;

volatile int nodeState;

esp_now_peer_info_t peerInfo;
esp_now_send_status_t sendingStatus;

void IRAM_ATTR detectOpenDoor() {
  detachInterrupt(REED_SWITCH);
  #ifdef DEBUG
  Serial.println("DOOR OPEN DETECTED!!!");
  #endif
  nodeState = NODE_WARNING;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
  sendingStatus = status;
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  #ifdef DEBUG
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println(incomingReadings.a);
  Serial.print("Int: ");
  Serial.println(incomingReadings.b);
  #endif
  if(incomingReadings.b == NODE_RUNING){
    nodeState = NODE_RUNING;
    attachInterrupt(digitalPinToInterrupt(REED_SWITCH), detectOpenDoor, RISING);
    //checkingSensor = true;
  }
}

TaskHandle_t checkSensorTask;
TaskHandle_t sendWarningTask;
TaskHandle_t ledNodeStateTask;

void setup() {
  
  nodeState = NODE_IDLE;

  pinMode(REED_SWITCH, INPUT_PULLUP);
  pinMode(LED_BUILT_IN, OUTPUT);
    
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #ifdef DEBUG
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  else{
    Serial.println("Success to add peer");
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
     
   xTaskCreatePinnedToCore(
       sendWarningTaskFunction,
       "sendWarningTaskFunction",
       10000,
       NULL,
       1,
       &sendWarningTask,
   1);

   xTaskCreatePinnedToCore(
       ledNodeStateTaskFunction,
       "ledNodeStateTaskFunction",
       10000,
       NULL,
       1,
       &ledNodeStateTask,
   1);

   //attachInterrupt(digitalPinToInterrupt(REED_SWITCH), detectOpenDoor, RISING);
   
}

int sendMessage(){
  esp_err_t result;
  
  result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
  delay(2000);

  if (result == ESP_OK) {
       #ifdef DEBUG
       Serial.println("Sent with success");
       #endif
       for(int i=0; i < RETRY_NUMBER; i++){
           if(sendingStatus == ESP_NOW_SEND_SUCCESS){
               nodeState = NODE_IDLE;
               return 1;
           }
           else{
               #ifdef DEBUG
               Serial.println("ESP NOW SENDING ERROR");
               #endif
               //nodeState = NODE_ESP_NOW_ERROR;
               result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
               delay(2000);
               nodeState = NODE_ESP_NOW_ERROR;
           }
       }          
  }
  else {
       #ifdef DEBUG
       Serial.println("Error sending the data");
       #endif
       nodeState = NODE_ESP_NOW_ERROR;
  }   
  return 1;
}

 void sendWarningTaskFunction( void * pvParameters ){
  esp_err_t result;
  
  while(1){
    myData.id = 10;
    strcpy(myData.a, "THIS IS A CHAR");
    myData.b = NODE_IDLE;
    myData.c = 1.2;
    myData.d = false;

    if(nodeState == NODE_WARNING){         
         //myData.b = NODE_WARNING;
         strcpy(myData.a, "NODE WARNING");
         myData.b = NODE_WARNING;
         sendMessage();
         //delay(2000);
         //nodeState = NODE_IDLE;
    }

  }
}

 void ledNodeStateTaskFunction( void * pvParameters ){
  
  while(1){
    switch(nodeState){
      case NODE_RUNING:
           digitalWrite(LED_BUILT_IN, HIGH); 
           break;

      case NODE_WARNING:
           delay(1500);
           digitalWrite(LED_BUILT_IN, HIGH);   
           delay(500);
           digitalWrite(LED_BUILT_IN, LOW);   
           delay(1500);
           break;
           
      case NODE_IDLE:
           delay(5000);
           digitalWrite(LED_BUILT_IN, HIGH);   
           delay(500);
           digitalWrite(LED_BUILT_IN, LOW);   
           delay(5000);
           break; 
           
      case NODE_ESP_NOW_ERROR:
           delay(200);
           digitalWrite(LED_BUILT_IN, HIGH);   
           delay(100);
           digitalWrite(LED_BUILT_IN, LOW);   
           delay(200);
           break;
           
      default:
           break;    
    }
  }
}

void loop() {

}
