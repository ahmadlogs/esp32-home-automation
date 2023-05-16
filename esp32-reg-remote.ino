/*************************************************************************************************
 *  Created By: Tauseef Ahmad
 *  Created On: 10 May, 2023
 *  
 *  YouTube Video: 
 *  My Channel: https://www.youtube.com/channel/AhmadLogs
 *  
 *  *********************************************************************************************
 *  Preferences--> Aditional boards Manager URLs : 
 *  For ESP32 (2.0.4):
 *  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *  *********************************************************************************************
 *  Install the following libraries :
 *  1. Blynk - version 1.2.0
 *  2. AceButton - https://github.com/bxparks/AceButton version 1.9.2
 *  3. IRremote - https://github.com/Arduino-IRremote/Arduino-IRremote version 4.1.2
 ***********************************************************************************************/
 
 //-----------------------------------------------------------
// TemplateID, DeviceName and AuthToken are provided by Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "ENTER_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "ENTER_BLYNK_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "ENTER_BLYNK_AUTH_TOKEN"
//-----------------------------------------------------------
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer;
//-----------------------------------------------------------
// Your WiFi credentials.
char ssid[] = "ENTER_YOUR_WIFI_SSID";
char pass[] = "ENTER_YOUR_WIFI_PASS";
//-----------------------------------------------------------
#include <EEPROM.h>
#include <AceButton.h>
using namespace ace_button;
//-----------------------------------------------------------
#include <IRremote.h>
#define IR_RECEIVE_PIN 23
// Define the EEPROM address where the array will be stored
const uint8_t IRCODE_ADDRESS = 4;
const uint8_t NUM_HEX_CODES = 6;
uint32_t      hex_codes[NUM_HEX_CODES];
//-----------------------------------------------------------
#include <DHT.h>
#define DHT11_PIN 22
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
//-----------------------------------------------------------
boolean config_mode = false;
#define EEPROM_SIZE 4096
//-----------------------------------------------------------
#define WIFI_LED 2
#define BOOT_BUTTON 0
//-----------------------------------------------------------
#define RELAY_1 25
#define RELAY_2 26
#define RELAY_3 32
#define RELAY_4 33

boolean state_relay1 = false;
boolean state_relay2 = false;
boolean state_relay3 = false;
boolean state_relay4 = false;

//-----------------------------------------------------------
//define the virtual pins for your relays
#define VPIN_BUTTON1    V1
#define VPIN_BUTTON2    V2
#define VPIN_BUTTON3    V3
#define VPIN_BUTTON4    V4

#define BUTTON_PIN_1 5
#define BUTTON_PIN_2 18
#define BUTTON_PIN_3 19
#define BUTTON_PIN_4 21

ButtonConfig config1;
ButtonConfig config2;
ButtonConfig config3;
ButtonConfig config4;

AceButton button1(&config1);
AceButton button2(&config2);
AceButton button3(&config3);
AceButton button4(&config4);

void handleEvent1(AceButton*, uint8_t, uint8_t);
void handleEvent2(AceButton*, uint8_t, uint8_t);
void handleEvent3(AceButton*, uint8_t, uint8_t);
void handleEvent4(AceButton*, uint8_t, uint8_t);


//--------------------------------------------------------------------------
// This function is called every time the device is connected to Blynk.Cloud
// Request the latest state from the server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_BUTTON1);
  Blynk.syncVirtual(VPIN_BUTTON2);
  Blynk.syncVirtual(VPIN_BUTTON3);
  Blynk.syncVirtual(VPIN_BUTTON4);
}
//--------------------------------------------------------------------------
// This function is called every time the Virtual Pin state change
//i.e when you push switch from Blynk App or Web Dashboard
BLYNK_WRITE(VPIN_BUTTON1) {
  state_relay1 = param.asInt();
  digitalWrite(RELAY_1, state_relay1);
  Serial.println("BLYNK_WRITE: Relay1 State = "+String(state_relay1));
}
//--------------------------------------------------------------------------
BLYNK_WRITE(VPIN_BUTTON2) {
  state_relay2 = param.asInt();
  digitalWrite(RELAY_2, state_relay2);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(VPIN_BUTTON3) {
  state_relay3 = param.asInt();
  digitalWrite(RELAY_3, state_relay3);
}
//--------------------------------------------------------------------------
BLYNK_WRITE(VPIN_BUTTON4) {
  state_relay4 = param.asInt();
  digitalWrite(RELAY_4, state_relay4);
}
//--------------------------------------------------------------------------
//handled by the timeer in the setup function
void sendDhtData()
{
  //-----------------------------------------------------------------------
  //Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  //Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  //-----------------------------------------------------------------------
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  //-----------------------------------------------------------------------
  //Compute heat index in Celsius (isFahreheit = false)
  //float hic = dht.computeHeatIndex(t, h, false);
  //-----------------------------------------------------------------------
  Serial.print("Temperature: "); Serial.println(t);
  Serial.print("Humidity: "); Serial.println(h);
  //-----------------------------------------------------------------------
  // Please don't send more than 1 value per second
  Blynk.virtualWrite(V5, t);
  Blynk.virtualWrite(V6, h);
  //-----------------------------------------------------------------------
}
//--------------------------------------------------------------------------

/****************************************************************************************************
 * void setup()
*****************************************************************************************************/
void setup() {
  Serial.begin(115200);
  //-----------------------------------------------------------
  EEPROM.begin(EEPROM_SIZE);
  delay(1000);
  //-----------------------------------------------------------
  IrReceiver.begin(IR_RECEIVE_PIN); // Start the IR receiver
  //-----------------------------------------------------------
  dht.begin();
  //-----------------------------------------------------------
  // initialize Relays
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);

  //During Starting all Relays should TURN OFF
  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);
  digitalWrite(RELAY_3, LOW);
  digitalWrite(RELAY_4, LOW);
  //-----------------------------------------------------------
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  //-----------------------------------------------------------
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);
  pinMode(BUTTON_PIN_4, INPUT_PULLUP);

  config1.setEventHandler(handleEvent1);
  config2.setEventHandler(handleEvent2);
  config3.setEventHandler(handleEvent3);
  config4.setEventHandler(handleEvent4);

  button1.init(BUTTON_PIN_1);
  button2.init(BUTTON_PIN_2);
  button3.init(BUTTON_PIN_3);
  button4.init(BUTTON_PIN_4);
  //-----------------------------------------------------------
  connectToWIFI();
  Blynk.config(auth);
  //the below code is blocking, so I have commented it out.
  //Blynk.begin(auth, ssid, pass); //Just hangs waiting for WiFi if the SSID is not present.
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  
  //-----------------------------------------------------------
  Blynk.virtualWrite(VPIN_BUTTON1, state_relay1);
  Blynk.virtualWrite(VPIN_BUTTON2, state_relay2);
  Blynk.virtualWrite(VPIN_BUTTON3, state_relay3);
  Blynk.virtualWrite(VPIN_BUTTON4, state_relay4);
  //-----------------------------------------------------------
  // Setup a function to be called every second
  timer.setInterval(60000L, sendDhtData); //1 minutes
  timer.setInterval(5000L, checkBlynkStatus); //5 seconds
  //-----------------------------------------------------------
  readIRCodes();
  //-----------------------------------------------------------
}


/****************************************************************************************************
 * void loop()
*****************************************************************************************************/
void loop() {
  //--------------------------------------------------------------------
  checkWIFI();
  //--------------------------------------------------------------------
  Blynk.run();
  timer.run();
  //--------------------------------------------------------------------
  checkConfigMode();
  //--------------------------------------------------------------------
  button1.check();
  button2.check();
  button3.check();
  button4.check();
  //--------------------------------------------------------------------
  remote_control();
  //--------------------------------------------------------------------
}


/****************************************************************************************************
 * void readIRCodes() - reads and display hex codes from EEPROM
 * void saveIRCodes() - store hex codes in the EEPROM
*****************************************************************************************************/
void readIRCodes() {
  EEPROM.get(IRCODE_ADDRESS, hex_codes);
  
  for(uint8_t i = 0; i < NUM_HEX_CODES; i++)
    {Serial.print(hex_codes[i]); Serial.println();}
}

void saveIRCodes() {
  EEPROM.put(IRCODE_ADDRESS, hex_codes);
  EEPROM.commit();
}

/****************************************************************************************************
 * void checkConfigMode()
*****************************************************************************************************/
void checkConfigMode() {
  // check if boot button is pressed to enter config mode
  //____________________________________________________________________________________
  if(digitalRead(BOOT_BUTTON) == LOW) {
    Serial.println("Boot Button Pressed!");
    delay(100); // Key debounce handling
    int startTime = millis();
    while(digitalRead(BOOT_BUTTON) == LOW) delay(50);
    int endTime = millis();
    //-----------------------------------------------------------
    if ((endTime - startTime) > 2000) {
      // If boot button pressed for more than 2secs, Enter config mode
      Serial.println("Enter config mode");
      config_mode = true;
      enterConfigMode();
    }
  }
  //____________________________________________________________________________________
}


/****************************************************************************************************
 * void enterConfigMode()
*****************************************************************************************************/
void enterConfigMode() {
  // enter config mode to register new hex codes
  Serial.println("Config mode");
  //digitalWrite(WIFI_LED, LOW);
  //____________________________________________________________________________________
  uint8_t i = 0;
  uint32_t last_ir_code = 0;
  
  while(i < NUM_HEX_CODES) {
    //-----------------------------------------------------------------------------
    if (IrReceiver.decode()) {
      uint32_t ir_code = IrReceiver.decodedIRData.command;
      //-------------------------------------------------------------------
      if(ir_code == 0) {IrReceiver.resume(); continue; }
      // if the current IR code is the same as the last one received, skip it
      if (ir_code == last_ir_code) { IrReceiver.resume(); continue; }
      last_ir_code = ir_code;
      //-------------------------------------------------------------------
      hex_codes[i] = ir_code;
      IrReceiver.resume(); // receive the next value
      //-------------------------------------------------------------------
      Serial.println(ir_code);
      //-------------------------------------------------------------------
      digitalWrite(WIFI_LED, LOW); delay(3000);
      i++;
      //-------------------------------------------------------------------
    } else {
      digitalWrite(WIFI_LED, HIGH);
      delay(100);
      digitalWrite(WIFI_LED, LOW);
      delay(100);
    }
    //-----------------------------------------------------------------------------
  }
  //____________________________________________________________________________________
  saveIRCodes();
  
  config_mode = false;  // exit config mode
  Serial.println("Normal mode");
  digitalWrite(WIFI_LED, HIGH);
  
}



/****************************************************************************************************
 * void remote_control
*****************************************************************************************************/
void remote_control()
{
  static uint32_t last_ir_code_time = 0;
  
  if(IrReceiver.decode()) 
  {
    uint32_t ir_code = IrReceiver.decodedIRData.command;
    //----------------------------------------------------------------------
    if(ir_code == 0) {IrReceiver.resume(); return; }
    // ignore duplicate IR codes received within 1000 ms
    if (millis() - last_ir_code_time < 1000) { IrReceiver.resume(); return;}
    last_ir_code_time = millis();
    //----------------------------------------------------------------------
    Serial.println(ir_code);
    //----------------------------------------------------------------------
    //IR KEY 1
    if(ir_code == hex_codes[0]){ 
        state_relay1 = !state_relay1;
        control_relay(1, RELAY_1, state_relay1);
        Blynk.virtualWrite(VPIN_BUTTON1, state_relay1);
    }
    //IR KEY 2
    else if(ir_code == hex_codes[1]){
      state_relay2 = !state_relay2;
      control_relay(2, RELAY_2, state_relay2);
      Blynk.virtualWrite(VPIN_BUTTON2, state_relay2);
    }
    //IR KEY 3
    else if(ir_code == hex_codes[2]){
      state_relay3 = !state_relay3;
      control_relay(3, RELAY_3, state_relay3);
      Blynk.virtualWrite(VPIN_BUTTON3, state_relay3);
    }
    //IR KEY 4
    else if(ir_code == hex_codes[3]){
      state_relay4 = !state_relay4;
      control_relay(4, RELAY_4, state_relay4);
      Blynk.virtualWrite(VPIN_BUTTON4, state_relay4);
    }
    //IR KEY 5
    else if(ir_code == hex_codes[4]){
      allON();
    }
    //IR KEY 6
    else if(ir_code == hex_codes[5]){
      allOFF();
    }
    //IR KEY 7
    //else if(ir_code == hex_codes[6]){
      //write code control your device
    //}
    //IR KEY 8
    //else if(ir_code == hex_codes[7]){
      //write code to control your 8th device.
    //}
    //----------------------------------------------------------------------
    IrReceiver.resume();
  }
}


/****************************************************************************************************
 * control_relay function
*****************************************************************************************************/
void control_relay(const int &relay_no, const int &relay_pin, boolean &status){
  //status = !status;
  //status = !digitalRead(relay_no)
  digitalWrite(relay_pin, status);
  String text = (status)? "ON" : "OFF";
  Serial.println("Relay"+String(relay_no)+" is "+text);
  delay(500);
}
void allON(){
  state_relay1 = 1; control_relay(1, RELAY_1, state_relay1); Blynk.virtualWrite(VPIN_BUTTON1, state_relay1); delay(100);
  state_relay2 = 1; control_relay(2, RELAY_2, state_relay2); Blynk.virtualWrite(VPIN_BUTTON2, state_relay2); delay(100);
  state_relay3 = 1; control_relay(3, RELAY_3, state_relay3); Blynk.virtualWrite(VPIN_BUTTON3, state_relay3); delay(100);
  state_relay4 = 1; control_relay(4, RELAY_4, state_relay4); Blynk.virtualWrite(VPIN_BUTTON4, state_relay4); delay(100);
}
void allOFF(){
  state_relay1 = 0; control_relay(1, RELAY_1, state_relay1); Blynk.virtualWrite(VPIN_BUTTON1, state_relay1); delay(100);
  state_relay2 = 0; control_relay(2, RELAY_2, state_relay2); Blynk.virtualWrite(VPIN_BUTTON2, state_relay2); delay(100);
  state_relay3 = 0; control_relay(3, RELAY_3, state_relay3); Blynk.virtualWrite(VPIN_BUTTON3, state_relay3); delay(100);
  state_relay4 = 0; control_relay(4, RELAY_4, state_relay4); Blynk.virtualWrite(VPIN_BUTTON4, state_relay4); delay(100);
}
/****************************************************************************************************
 * event handling functions for buttons
*****************************************************************************************************/
void handleEvent1(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      state_relay1 = 1;
      control_relay(1, RELAY_1, state_relay1);
      Blynk.virtualWrite(VPIN_BUTTON1, state_relay1);
      break;
    case AceButton::kEventReleased:
      state_relay1 = 0;
      control_relay(1, RELAY_1, state_relay1);
      Blynk.virtualWrite(VPIN_BUTTON1, state_relay1);
      break;
  }
}
void handleEvent2(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      state_relay2 = 1;
      control_relay(2, RELAY_2, state_relay2);
      Blynk.virtualWrite(VPIN_BUTTON2, state_relay2);
      break;
    case AceButton::kEventReleased:
      state_relay2 = 0;
      control_relay(2, RELAY_2, state_relay2);
      Blynk.virtualWrite(VPIN_BUTTON2, state_relay2);
      break;
  }
}
void handleEvent3(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      state_relay3 = 1;
      control_relay(3, RELAY_3, state_relay3);
      Blynk.virtualWrite(VPIN_BUTTON3, state_relay3);
      break;
    case AceButton::kEventReleased:
      state_relay3 = 0;
      control_relay(3, RELAY_3, state_relay3);
      Blynk.virtualWrite(VPIN_BUTTON3, state_relay3);
      break;
  }
}
void handleEvent4(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      state_relay4 = 1;
      control_relay(4, RELAY_4, state_relay4);
      Blynk.virtualWrite(VPIN_BUTTON4, state_relay4);
      break;
    case AceButton::kEventReleased:
      state_relay4 = 0;
      control_relay(4, RELAY_4, state_relay4);
      Blynk.virtualWrite(VPIN_BUTTON4, state_relay4);
      break;
  }
}

//handle by timer in the setup function.
void checkBlynkStatus() {
  boolean connected = Blynk.connected();
  if (connected) { digitalWrite(WIFI_LED, HIGH); }
  else { digitalWrite(WIFI_LED, LOW); }
}

//connect to WIFI
void connectToWIFI()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  
  // Wait for Wi-Fi connection for up to 5 seconds
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    //delay(1000);
    Serial.print(".");
    attempts++;
  }
  Serial.println("");
  
  // Print Wi-Fi credentials
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected");
    Serial.println("SSID: " + WiFi.SSID());
    Serial.println("IP address: " + WiFi.localIP().toString());
  }
  else {
    Serial.println("Wi-Fi connection failed");
  }
}

void checkWIFI() {
  if (WiFi.status() == WL_CONNECTED){return;}
  static unsigned long previousMillis = 0;
  // Run the code every 10 seconds (10000 milliseconds)
  if (millis() - previousMillis <= 10000) { return; }
  
  previousMillis = millis();
  connectToWIFI();
}
