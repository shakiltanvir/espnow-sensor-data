#include <ESP8266WiFi.h>
#include <espnow.h>
#include <DHT.h>
#define DHTTYPE DHT22
#define DHTPIN 12 
DHT dht(DHTPIN, DHTTYPE);
// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xDC, 0x4F, 0x22, 0x3B, 0x17, 0xFA};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message { 
  int flame;
  int mq2;
  float hum;  //Stores humidity value
  float temp; //Stores temperature value
  // char a[32];
  // int b;
  // float c;
  // String d;
  // bool e; 
} struct_message;

// Create a struct_message called myData
struct_message data;

unsigned long lastTime = 0;  
unsigned long timerDelay = 2000;  // send readings timer

float readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(t);
    return t;
  }
}

float readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(h);
    return h;
  }
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  dht.begin();
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("I am reading sensor. Ready to send");

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
 
    // Initialize the flame sensor
  pinMode(4, INPUT);

  // Initialize the MQ-2 sensor
  //Assuming the MQ-2 sensor is attached to A0
  pinMode(A0, INPUT);
  
  // Initialize the DH22 sensor
  //Assuming the DH22 sensor is connected to pin 2

}
 
void loop() {
  if ((millis() - lastTime) > timerDelay) {

  data.flame = digitalRead(4);
  Serial.print("Fire: ");
  Serial.println(data.flame);

  // Read data from MQ-2 sensor
  data.mq2 = analogRead(A0);
  Serial.print("Smoke: ");
  Serial.println(data.mq2);

  // Read data from DH22 sensor
    data.hum = dht.readHumidity();
    data.temp= dht.readTemperature();
    Serial.print("Temperature: ");
    Serial.println(data.temp);
    Serial.print("Humidity: ");
    Serial.println(data.hum);

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));

    lastTime = millis();
  }
}
