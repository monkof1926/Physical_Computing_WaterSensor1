/*This code is made out from RUC's fablab plantmonitoring project https://fablab.ruc.dk/plantmonitoring/ and the windcalculator is made out from G6EJD from https://github.com/G6EJD/ESP32-Miniature-Weather-Station/blob/master/ESP32_Miniature_OLED_Weather_Station_SH1106_v01.ino */
#include "DHTesp.h"
#include "Ticker.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Preferences.h>
#include "credentials.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <NTPClient.h>
#include <Arduino_JSON.h>
#include "time.h"

//Defining the unit name and what interval that sends data to google sheet
#define UNITNAME "Water%20Sensor%201"
#define UPDATEINTERVAL 60000 
#define SERVERNAME "Hallo" // Don't have a server need to change name if there is one 

//Min Hours of sun need for the plant to grow (General) 4 hours 
#define MinSolarHours 14400000

// wind speeds and the thresholds 
#define LightBreeze 11
#define ModerateBreeze 29
#define StrongBreeze 50
#define Gale 75
#define Strom 102

//Allows the pump to run for 3 minuter max and downtime of 3 mins
#define PumpInterval 180000
#define PumpDownInterval 180000
#define PumpCheckInterval 60000

#define pressure_offset 3.5       // Air pressure calibration, adjust for your altitude
#define WindSpeed_Calibration  1.1       // Wind Speed calibration factor
#define WindSpeed_Samples      10        // Number of Wind Speed samples for an average

#define TemperatureAlarmTimerInterval 14400000 // 4 hours with high or low temperatures

//Soil humidity threshold and makes then gobal
int AlarmDryThreshold;
int AlarmWetThreshold; 
int VeryMoistThreshold;
int MoistThreshold;
int SlightMoisture;

//Pins
const int LEDPIN = 16;
const int SOILPIN = 32;
const int SOLARPIN = 35;
const int dhtPin = 22;
const int GPSTXPin = 16;
const int WindPin = 14;
const int PumpRelayPin = 4;

// Setting up gobal variables
int waterlevel = 0; // soil humidity
unsigned long Timer = 0; // the over all timer
float temp = 0;  // temperature
float rh = 0;    // Relative humidity
unsigned int alarmstatus = 0;
unsigned long updateTimer = 0;
unsigned long pumpUpdateTimer = 0;
unsigned long tempAlarmTimer = 0;
long last_display_update_millis = 0;
long last_wind_update_millis = 0;
boolean toggle = 0;
int disconnected_seconds = 0;
int solarvalue = 0;
float dewPoint = 0;
int tempAlarmStartTimer = 0;
int downState = 0;
static unsigned int LastEventWind;
String UNIT = "Plant%20Butterfly";
String formatSoil = "G";
String formatTemp = "G";
String formatUnit;
String payload;
String payloadDMI = "{}"; 
String payloadAPP; 
String tempCheck;
String windCheck;

//Gobal veriables for the windsensor 
float windsensor;
float windspeed;


// Sets the wind gobal variables
float WindSpeed_Average       = 0; 
float WindSpeed_Total         = 0;
int   WindSpeed_Index = 0;
float WindReadings[WindSpeed_Samples];

// general temperaturs for plants in Denmark
int HighTemperature = 27;
int ExtremeHighTemperature = 35;
int LowTemperature = 12;
int ExtremeLowTemperature = 2;

int InterruptCounter;
float WindSpeed;
String gpsdata;A
byte gpss;

// // character buffer for storing URL, text and network status in.
char dataText[1024];
char text[1024];
char netStatus[512];

// Making the HTTpClient and DHTesp
HTTPClient http;
DHTesp dht;
WiFiMulti wifiMulti;
Preferences preferences;
TinyGPSPlus gps;
Ticker tinkerTemp;
SoftwareSerial ss(GPSTXPin);
WiFiClient client;

// getting the newvalues for the temperature and humidity from the dht sensor
TempAndHumidity newValues = dht.getTempAndHumidity();

// Gobal veriables for the gps and if no responce from gps then set default latitude and longitude
float latitude = (gps.location.lat(), 55.719128855994164);
float longitude = (gps.location.lng(), 12.530645435542642);

//Url's and the keys that is need
String url = "https://script.google.com/macros/s/AKfycbxEk1tg1zDBP7uHv4JSTBwpMIElXJsH0jhpeh3Brj98mphcSVXfC9R_ObAZLGp2iRGW/exec?";
String api_key = "06cf1988-ec55-4476-8d33-f7115a166c99"; 
String DMI_tUrl = "https://dmigw.govcloud.dk/v1/forecastedr/collections/harmonie_nea_sf/position?coords=POINT%"; // temp url for dmi api else it don't work
String DMI_url = DMI_tUrl + latitude + "%" + longitude + "%29&crs=crs84&parameter-name=temperature-0m&relative-humidity&api-key=" + api_key;
String APP_url = url + "read";

//Making the final url that is send up all in it can change 
String finalurl; 
// Making the temptask, getTemperature and triggerGetTemp global 
void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

//Starting temperature before reading as null
TaskHandle_t tempTaskHande = NULL;

//Starts the task as false
bool taskEnabled = false;

// Calculate the windspeed over periode of time is made with the use of 
void IRAM_ATTR MeasureWindSpeed() {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  LastEventWind = millis();    // Record current time for next event calculations
  portEXIT_CRITICAL_ISR(&timerMux);
}
//initilize the tempratur sensor
bool initTemp() {
  byte resultValue = 0;
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT is running"); 

  xTaskCreatePinnedToCore(
    tempTask,
    "tempTask",
    4000,
    NULL,
    5,
    &tempTaskHande,
    1);
  // If the temperatur sensor fails or it can't finde the sensor
  if (tempTaskHande == NULL) {
    Serial.println("Failed to start temperatur sensor ");
    return false;
  } else {
    // Set the update interval for the temperatur sensor
    tinkerTemp.attach(10, triggerGetTemp);
  }
  return true;
}
//If taskhander is on then get temperature
void triggerGetTemp() {
  if (tempTaskHande != NULL) {
    xTaskResumeFromISR(tempTaskHande);
  }
}
//Getting the temeperature if the sensor is on
void getTemp() {
  if (tempTaskHande != NULL) {
    xTaskResumeFromISR(tempTaskHande);
  }
}
//Looks after temptask id on if yes then get temperature if not set temperature to null
void tempTask(void *pvParameters) {
  while (1) {
    if (taskEnabled) {
      getTemperatur();
    }
    vTaskSuspend(NULL);
  }
}
//prints and gets temperatur, humidity, dewpoint, soil humidity, solarvalue and windspped and addes latitude and longitude to the print out 
bool getTemperatur() {
  TempAndHumidity newValues = dht.getTempAndHumidity();
  Calculate_WindSpeed();  

  if (dht.getStatus() != 0) {
    Serial.println("DHT11 Error :" + String(dht.getStatusString()));
    return false;
  }

  temp = newValues.temperature; // read a new value of the sensor
  rh = newValues.humidity;// read a new value of the sensor
  dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity); // calculate the dewpoint
  waterlevel = analogRead(SOILPIN);  // reads a new value of the sensor
  solarvalue = analogRead(SOLARPIN);// reads a new value of the sensor

  // prints all the values from the higrow and solarpanel 
  Serial.println("Soil humidty: "+ String(waterlevel) +" Temperatur is " + String(temp ) + " Humidity is " + String(rh ) + " DewPoint is " + String(dewPoint ) + " SolarValue is " + String(solarvalue ) + " WindSpeed " + String(WindSpeed ) + " latitude " + String(latitude) + " longitude " + String(longitude));
  
  //prints the temperature, humidity, dewpoint and solarvalue it did not work
  sprintf(dataText, "temp%%20%.1f%%20Humidity%%20RH%%20%.1f%%dewPoint%%20%.1f.1f%%20solarvalue%%20%.1f",
          temp,
          rh,
          dewPoint,
          solarvalue);
  return true;
}
// calcutate the windspeed and takes a avege under 1 sek but over 1000 millisec 
float Calculate_WindSpeed() {
  // the interrupt for the windsensor else it only return 1 or 0 
  attachInterrupt(digitalPinToInterrupt(WindPin), &MeasureWindSpeed, RISING);

  if ((millis() - LastEventWind) > 2) { 
    WindSpeed = (1.00 / (((millis() - LastEventWind) / 1000) * 2)) * WindSpeed_Calibration; // Calculate wind speed
  }

  // Calculate average wind speed over the time set above time 
  WindSpeed_Total = WindSpeed_Total - WindReadings[WindSpeed_Index]; // Subtract the last reading
  WindReadings[WindSpeed_Index] = WindSpeed;
  WindSpeed_Total = WindSpeed_Total + WindReadings[WindSpeed_Index]; // Plus the last reading
  WindSpeed_Index = WindSpeed_Index + 1; 

  if (WindSpeed_Index >= WindSpeed_Samples) {                                           
    WindSpeed_Index = 0;
  }
  // calculate the average wind speed and Convert to kph Convert to kph 
  WindSpeed = WindSpeed_Total / WindSpeed_Samples;          
  WindSpeed = WindSpeed * 1.60934; 
                             
  return WindSpeed;
}
// Is used to get the network status and allows for a macAddress so it can interact with mac computers.
void displayUpdate() {
  netStatus[0] = '\0';
  sprintf(netStatus, "%s %s %s %ddBm %i %s",
          WIFISSID,
          WiFi.macAddress().c_str(),
          WiFi.localIP().toString().c_str(),
          WiFi.RSSI(),
          WiFi.status(),
          UNITNAME);

  Serial.println(netStatus);
}
//Get alot of data for this project the only importen is gps, time and altitude that are imported for this project
byte Gpsinfo(){

  while (ss.available() > 0){
    // get the byte data from the GPS
    byte gpsData = ss.read();
    byte gpss =  Serial.write(gpsData);
  } return gpss;
}
//Check the windspeed and see if it goes over a windspeed threshold if the wind speed is over then print it out and set an alarmstatus 
String WindSensor(){
  Calculate_WindSpeed();
  waterlevel = analogRead(SOILPIN);
 
  if(WindSpeed != 0){
    if(WindSpeed >= LightBreeze && waterlevel >= MoistThreshold ){
     Pump();
     Serial.print(WindSpeed);
     Serial.println("kmph");
     windspeed;
     windCheck = "Light%20breeze";
     alarmstatus = 1;
     return windCheck;
  }

    if (WindSpeed >= ModerateBreeze && waterlevel >= MoistThreshold ){
      Pump();
      Serial.print(WindSpeed);
      Serial.println("kmph");
      windCheck = "Moderate%20breeze";
      alarmstatus = 1;
      return windCheck;
    }

    if(WindSpeed >= StrongBreeze && waterlevel >= MoistThreshold ){
      Pump();
      Serial.print(WindSpeed);
      Serial.println("kmph");
      windCheck = "Strong%20breeze";
      alarmstatus = 2;
      return windCheck;
    }

    if(WindSpeed >= Gale && waterlevel >= AlarmWetThreshold){
      Pump();
      Serial.print(WindSpeed);
      Serial.println("kmph");
      windCheck = "Gale";
      alarmstatus = 2;
      return windCheck;
    }
  
    if (WindSpeed >= Strom && waterlevel >= AlarmWetThreshold){
      Pump();
      Serial.print(WindSpeed);
      Serial.println("kmph");
      windCheck = "Storm";
      alarmstatus = 3;
      return windCheck;
    }else{
    Serial.println("All good");
    Serial.print(WindSpeed);
    Serial.println("kmph");
    windCheck = "The%20wind%20is%20good";
    alarmstatus = 0;
    return windCheck;
    }
  }
}

//check what the threshold for the soil humidity and allows for change by the user du inputting B,T,G  more can be added later 
void check()
  checkUrl();
  if(formatSoil == "B"){
    //Change the threshold to butterfly plants limits 
    AlarmDryThreshold  <= 1288;
    AlarmWetThreshold >= 2756;
    VeryMoistThreshold >= 2531;
    MoistThreshold  >= 2162;
    SlightMoisture  <= 1556;
    Serial.println("The soil humidity is now set for butterfly plants");

  }else if(formatSoil == "T"){
    //Change the threshold to tomato plants limits 
    AlarmDryThreshold  <= 1888;
    AlarmWetThreshold >= 3058;
    VeryMoistThreshold <= 2831;
    MoistThreshold  >= 2562;
    SlightMoisture  <= 2056;
    Serial.println("The soil humidity is now set for tomatoes");

  }else if(formatSoil = "G"){
    //Change the threshold to general plants limits 
     AlarmDryThreshold  >= 4050;
    AlarmWetThreshold <= 2000;
    VeryMoistThreshold <= 2500;
    MoistThreshold  <= 3800;
    SlightMoisture  >= 4000;
    Serial.println("The soil humidity is now set for general");
  }else {
     //Change the threshold to general plants limits 
     AlarmDryThreshold  >= 4050;
    AlarmWetThreshold <= 2000;
    VeryMoistThreshold <= 2500;
    MoistThreshold  <= 3800;
    SlightMoisture  >= 4000;
    Serial.println("The soil humidity is now set for general");
  }
}
//check what the Temperature for the plants and allows for change by the user du inputting B,T,G  more can be added later 
void checkTemp(){
  if(formatTemp == "B"){ // Change temperature limits to butterfly plant perfence 
    HighTemperature >= 27;
    ExtremeHighTemperature >= 35;
    LowTemperature >= 12;  
    ExtremeLowTemperature >= 2;
    Serial.println("The temperature is now set for butterfly plants");
  }else if(formatTemp == "T"){// Change temperature limits to tomato plant perfence 
    HighTemperature >= 27;
    ExtremeHighTemperature >= 35;
    LowTemperature >= 12;
    ExtremeLowTemperature >= 2;
    Serial.println("The temperature is now set for tomato's");
  }else if(formatTemp == "G"){// Change temperature limits to general plant perfence 
    HighTemperature >= 27;
    ExtremeHighTemperature >= 35;
    LowTemperature >= 12;
    ExtremeLowTemperature >= 2;
    Serial.println("The temperature is now set for general");
  }else{
    HighTemperature = 27;
    ExtremeHighTemperature = 35;
    LowTemperature = 12;
    ExtremeLowTemperature = 2;
    Serial.println("The temperature is now set for general");
  }
}
//Check after user input for new unit name
void checkUnit(){
  if(UNIT !=  "Plant%20Butterfly"){
    Serial.println(UNIT);
  }else if(UNIT.isEmpty() || UNIT == "{}"){
    UNIT =  "Plant%20Butterfly";
  }else{
    UNIT =  "Plant%20Butterfly";
  }
}
//check the temperature and if it is higher or lower then the values from checkTemp if yes then start timer and if timer goes over send message and chenge alarmstatus
String TemperatureCheck(){ 
  if(temp >= HighTemperature){
    tempAlarmStartTimer = millis(); // start the cloack on 
    Serial.println("Check the plant");
  }else if(temp >= ExtremeHighTemperature){
    tempAlarmStartTimer = millis();
    Serial.println("Check the plant");
  }else if(temp <= LowTemperature){
    tempAlarmStartTimer = millis();
    Serial.println("Check the plant");
  }else if(temp <= ExtremeLowTemperature){
    tempAlarmStartTimer = millis();
    Serial.println("Check the plant");
  }else{
    Serial.println("The temperature is good for the plants");
    alarmstatus = 0;
  }

  if(millis() - tempAlarmStartTimer >= TemperatureAlarmTimerInterval && temp == HighTemperature ){
    Serial.println("The Temperature is high please put plant in shade");
    alarmstatus = 1;
    tempCheck = "Temperature%20high";
    return tempCheck;
  }if(millis() - tempAlarmStartTimer >= TemperatureAlarmTimerInterval && temp == ExtremeHighTemperature ){
    Serial.println("The temperature is extremely high put the plant in the shade ");
    alarmstatus = 2;
    tempCheck = "Temperature%20extremely%20high";
    return tempCheck;
  }if(millis() - tempAlarmStartTimer >= TemperatureAlarmTimerInterval && temp == LowTemperature ){
    Serial.println("The temperature is low please put the plant in the sun");
    alarmstatus = 1;
    tempCheck = "Temperature%20low";
    return tempCheck;
  }if(millis() - tempAlarmStartTimer >= TemperatureAlarmTimerInterval && temp == ExtremeLowTemperature ){
    Serial.println("The temperature is extremely low plese put the plant in the sun or take inside ");
    alarmstatus = 2;
    tempCheck = "Temperature%20extremely%20low";
    return tempCheck;
  }else{
    tempCheck = "Teperature%20ok";
    return tempCheck;
    alarmstatus = 0;
  }
}
//Displays the latitude, longitude and altitude and set a starting point in copenhagen 
void displayGPSInfo(){
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 55.719128855994164);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 12.530645435542642);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }

}
//Setup the the device
void setup() {
  Serial.begin(115200);
  Serial.println("Boot");
  initTemp(); // begins the temperature taking 
  pinMode(PumpRelayPin, OUTPUT);
  pinMode(WindPin, INPUT);
  wifiMulti.addAP(WIFISSID, WIFIPASS); // connects with the wifi ssid and password
  preferences.begin("WaterSensor1", false); 
  ss.begin(9600); // begins the gps it takes up to 5 min+ to start 
  if (wifiMulti.run() == WL_CONNECTED) { // check wifi connection

    Serial.println("Wifi connneted");

  }
   while (ss.available() > 0) // check that gps is on/there
    if (gps.encode(ss.read()))
      displayGPSInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10) // if no gps is foundt then check ones a min and prints No GPS detected
  {
    Serial.println("No GPS detected");
    while(true);
  }
  // Signal end of setup() to tasks
  taskEnabled = true;
}
//the loop
void loop() {
  if (!taskEnabled) {
    delay(2000);
    taskEnabled = true;
    if (tempTaskHande != NULL) {
      vTaskResume(tempTaskHande);
    }
  }
  yield(); 
  // Check if there is wifi conmection very 1000 milli sec 
  if (millis() > last_display_update_millis > 1000) {

    last_display_update_millis = millis();

    displayUpdate();

    if (WiFi.status() == WL_CONNECTED) {
      toggle = !toggle;
      disconnected_seconds = 0;
    } else {
      toggle = 0;
      disconnected_seconds++;
    }
  }
  // if the device is disconnect for more then 60 sec then restart the device 
  if (disconnected_seconds > 60) {
    ESP.restart();
  }

  //Runs the programes and input the soil humidity, relative humidity, temperature, windspeed and solarvalue then runs the pump if need
  if (millis() - updateTimer > UPDATEINTERVAL || updateTimer == 0) {
     //If alarmstatus is 2 all wronge either need more water or less water if alarmstatus is 1 need to check up on plant
    check();
    checkTemp();
    checkUnit();
  if (waterlevel > AlarmDryThreshold) {
    sprintf(text, "%s%s",
            "Warning%20soil%20dry%20",
            dataText);
    getTemperatur(); // check temperature, humidity and solarvalue
    WindSensor(); // check windspeed
    TemperatureCheck(); // check high or low level of the temperature 
    displayGPSInfo(); // shouds location
    alarmstatus = 2;
    Pump(); // runs pump if need
    checkTempDMI(); // check the data from dmi 
    Serial.println("Water Level is "+ waterlevel);
  } else if (waterlevel < AlarmWetThreshold) {
    sprintf(text, "%s%s",
            "Warning%20soil%20too%20wet%20",
            dataText);
    getTemperatur();
    WindSensor();
    TemperatureCheck();
    displayGPSInfo();
    alarmstatus = 2;
    Pump();
    checkTempDMI();
    Serial.println("Water Level is " + waterlevel);
  } else if(waterlevel < SlightMoisture && waterlevel > AlarmDryThreshold){
    sprintf(text, "%s%s",
            "Warning%20soil%20slight%20dry%20",
            dataText);
    getTemperatur();
    WindSensor();
    TemperatureCheck(); 
    displayGPSInfo();
    alarmstatus = 1;
    Pump();
    checkTempDMI();
    Serial.println("Water Level is " + waterlevel);
  }else if(waterlevel < VeryMoistThreshold && waterlevel > MoistThreshold){
    sprintf(text, "%s%s",
            "Warning%20soil%20Very%moist%20",
            dataText);
    getTemperatur();
    WindSensor();
    TemperatureCheck();
    displayGPSInfo();
    alarmstatus = 1;
    Pump();
    checkTempDMI();
    Serial.println("Water Level is " + waterlevel);
  }else if(waterlevel > VeryMoistThreshold && waterlevel < AlarmWetThreshold){
    sprintf(text, "%s%s",
            "Warning%20soil%20Very%20moist%20",
            dataText);
    getTemperatur();
    WindSensor();
    TemperatureCheck();
    displayGPSInfo();
    alarmstatus = 1;
    Pump();
    checkTempDMI();
    Serial.println("Water Level is " + waterlevel);
  } else {
    sprintf(text, "%s%s",
            "Soil%20probably%20ok%20",
            dataText);
    getTemperatur();
    WindSensor();
    TemperatureCheck();
    displayGPSInfo();
    alarmstatus = 0;
    Pump();
    checkTempDMI();
    Serial.println("Water Level is " + waterlevel);
  }
    statusUpdate(); // sends data to google sheet
    httpGETRequestDMI(); // request data from dmi 
    httpGETRequestAPP(); // request data from google sheet

    updateTimer = millis(); // reset timer 
    
  }
}
//sends update to the preperdt google sheet
String statusUpdate() {
  Serial.print("[HTTP] begin...\n");
  
 finalurl = url +"unit="+ UNIT + "&soil=" +  waterlevel  + "&temp=" + temp + "&rh=" + rh + "&dewPoint=" + dewPoint + "&solarvalue=" + solarvalue + "&windspeed="+ WindSpeed + "&latitude="+ latitude + "&longitude="+ longitude + "&tempCheck="+ tempCheck + "&windCheck="+ windCheck + "&alarmstatus="+ alarmstatus;

  http.begin(finalurl);

  Serial.print("[HTTP] GET...\n");
  Serial.println(finalurl);
  // start connection and send HTTP header
  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);

    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
      Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  // No longer looks for response
  http.end(); //Free's the resources

  return payload;
}
//Gets temperature and humidity forcast from DMI 
String httpGETRequestDMI() {
  http.begin(DMI_url);
  
  // Send HTTP GET request
  int httpResponseCodeDMI = http.GET();
  
  //String for getting data from the dmi  
  
  if (httpResponseCodeDMI > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCodeDMI);
    payloadDMI = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCodeDMI);
  }
  // No longer looks for response
  http.end(); //Free's the resources

  return payloadDMI;
}
//Gets a respornce from the google sheet
void httpGETRequestAPP() {
  // Send HTTP GET request
  http.begin(APP_url.c_str()); //String for getting data from the APP/google sheet   
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  int httpResponseCodeAPP = http.GET();
  //Looks for the response from the APP and what code to use
  if (httpResponseCodeAPP > 0) {
    payloadAPP = http.getString();
    Serial.println(httpResponseCodeAPP);
    Serial.println(payloadAPP);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCodeAPP);
  }
  // No longer looks for response
  http.end(); //Free's the resources
  //Return the payload with data from the APP
}
//Checks if there is hole through to dmi api 
String checkTempDMI(){
  if(payloadDMI != NULL){
    String hallo = "It Works";
    return hallo;
  }else{
    String NOOO = "It don't work";
    return NOOO;
  }
}

void checkUrl(){
  if(payloadAPP == "B"){
    formatTemp = payloadAPP;
    formatSoil = payloadAPP;
    Serial.println("changes to butterfly plant specifications ");
  }else if(payloadAPP == "T"){
    formatTemp = payloadAPP;
    formatSoil = payloadAPP; 
    Serial.println("changes to tomatoes specifications ");
  }else if(payloadAPP == "G"){
    formatTemp = payloadAPP;
    formatSoil = payloadAPP;
  }if(payloadAPP.length() == 1){
    formatTemp = payloadAPP;
    formatSoil = payloadAPP;
  }
  if(payloadAPP.length() > 1){
    UNIT = payloadAPP;
  }else{
    UNIT = "Plant%20Butterfly";
  }
}
//Activate the pump and stops the pump
void Pump(){
  //makes a new waterlevel to read the soil humidity
  waterlevel = analogRead(SOILPIN);
  //check if the soil humidity id over the moisThreshold if yes then the pump stops or don't starts.
  if(waterlevel >= SlightMoisture){
    digitalWrite(PumpRelayPin, HIGH);
  }else if(waterlevel <= MoistThreshold){
    digitalWrite(PumpRelayPin, LOW);
    }
}

