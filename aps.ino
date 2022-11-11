#include <Time.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h> //non compatible with samd

#define PI 3.1415926535897932384626433832795
#define MULTICURRENT 73.3
#define SUBTRACTCURRENT 36.7

#define testMode 1

#define echoCleanPin 4 // pin D2 Arduino to pin Echo of HC-SR04
#define trigCleanPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoDirtyPin 6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigDirtyPin 5 //attach pin D3 Arduino to pin Trig of HC-SR04

#define rele1Pin 12
#define rele2Pin 13

#define flowInPin 8
#define flowOutPin 9

#define SolarCurPin A5
#define PumpsCurPin A3
#define PumpInCurPin A2

#define TdsSensorPin A1
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

//Da aggiungere pin per relè pompette, sensore torbidità, sensore conduttività, sensore flusso

//VBAT -> solar input

// Tempo trascorso (per timeout di sicurezza)
time_t seconds = 0;
// Tempo trascorso (per gestione pompette)
//time_t pumpSeconds = 0;

//LiquidCrystal_I2C lcd(0x27, 16, 2); // TO DO: check i2c address via code in the aps_resources doc

// Flag sistema
bool systemPowered=false;
bool activeSystem=true; 
bool rele1=false;
bool rele2=false;


//bool relePumps=false; //Falso -> pompe default di caffe REMOVED

// Costanti dei limiti
// Volume acqua 
const long warningClearWater = 574.32; // warning litri
const long maxClearWater = 662.68; // max litri
const long warningDirtyWater = 132.53; //warning litri
const long maxDirtyWater = 662.68; // max litri
const long waterThreshold = 5; // limiti litri

// Valori torbidità (NTU)
const float warningTorbidity = 0.5;
const int limitTorbidity = 1;

// Valori Conduttività (µmhos/cm)
const int maxConductivity = 3;
const float minConductivity = 0.5;

// Variabili sensori US
long cleanDuration; // Variabile per la durata del 'rimbalzo'
long dirtyDuration;
int cleanDistance; // Variabile per la distanza del 'rimbalzo'
int dirtyDistance;

// Valori container cm
const float radiusContainer = 37.5;
const long areaContainerBase = PI*pow(radiusContainer,2);
const long heightContainer = 150;

// Valori random per solare e liquidi
long volumeCleanLiquid;
long volumeDirtyLiquid;
long solarVoltageR;
long solarPowerR;
long waterTorbidityR;
long waterConductivityR;


// Variabili temporanee di distanza
//long cleanDuration; // variable for the duration of sound wave travel
//long dirtyDuration;
//int cleanDistance; // variable for the distance measurement
//int dirtyDistance;

#define DEBUG true

int PWR_KEY = 9;
int RST_KEY = 6;
int LOW_PWR_KEY = 5;

#define VBAT A1
#define VSYS A2
#define PS 3.3
#define RES 4096

float valBat = 0.0;
float valSys = 0.0;

#define CHECK_ENV     1
#define READ_INPUT    2
#define INIT_MODEM    3
#define GPS_LOCATION  4
#define MQTT_CONN     5
#define MQTT_PUB      6
#define MQTT_SUB      7
#define MQTT_WAIT_MSG 8
#define MQTT_DISCONN  9
#define SLEEP         10


// Your GPRS credentials, if any
const char apn[]      = "myinternet.wind";
const char gprsUser[] = "";
const char gprsPass[] = "";


// MQTT details
const char* mqtt_broker    = "maqiatto.com";
const char* mqtt_topicPub  = "roberto@w2wsolutions.it/maduino";
const char* mqtt_topicSub  = "roberto@w2wsolutions.it/maduino/cmd";
const char* mqtt_user      = "roberto@w2wsolutions.it";
const char* mqtt_psw       = "maduino";

int len;
int st = READ_INPUT;
String loc, location, stringToSend;

bool ModuleState=false;

// Setup di sistema con pin
void setup() {
  
    pinMode(trigCleanPin, OUTPUT); // Sets both the trigPin as OUTPUT
    pinMode(trigDirtyPin, OUTPUT);
    pinMode(echoCleanPin, INPUT); // Sets both the echoPin as INPUT
    pinMode(echoDirtyPin, INPUT);

    pinMode(rele1Pin, OUTPUT); 
    pinMode(rele2Pin, OUTPUT);
    
    pinMode(A5, INPUT);
    pinMode(A3, INPUT);
    pinMode(A2, INPUT);
    pinMode(A1, INPUT);

    pinMode(TdsSensorPin,INPUT);

//    lcd.begin();
//
//
//    lcd.backlight();
//    lcd.clear();
//    lcd.setCursor(4,0);
//    lcd.print("Test LCD");
    
    pinMode(flowInPin, INPUT);
    pinMode(flowOutPin,INPUT);
    
    pinMode(PWR_KEY, OUTPUT);
    pinMode(RST_KEY, OUTPUT);
    pinMode(LOW_PWR_KEY, OUTPUT);
    
    //setTime(seconds);
    //setTime(pumpSeconds);

    if(testMode==1){
    digitalWrite(rele1Pin, LOW);
    digitalWrite(rele2Pin, LOW);
    }
    Serial1.begin(115200);
    SerialUSB.begin(115200);
    

    analogReadResolution(12);

    //while (!SerialUSB)
    {
        ; // wait for serial port to connect
    }
 
 /*   ModuleState=moduleStateCheck();
    if(ModuleState==false)//if it's off, turn on it.
    {
      digitalWrite(PWR_KEY, LOW);
      delay(3000);
      digitalWrite(PWR_KEY, HIGH);
      delay(10000);
      SerialUSB.println("Now turnning the A9/A9G on.");
    }*/
 
  SerialUSB.println("Starting APS test on Maduino Zero A9G board");
}



void loop() {

toggleRelay1();
toggleRelay2();

//Struttura base del codice da bypassare in fase di relé testing
if(testMode==0){

  //Futura implementazione di timeout di sicurezza (con calcoli futuri si conteranno i giorni senza cambiamento di flag -> al quarto giorno arriva un allarme ulteriore)
  //seconds = now();
  //SerialUSB.println(seconds);


  String cmd = checkData();
  if(cmd != "")
   parseCommand(cmd);


  
  // Di default il sistema è attivo, poi facendo i check necessari si ferma o continua a funzionare a seconda dei risultati della valutazione
  activeSystem=true;


  
  switch (st){

   case CHECK_ENV:
 // Generazione valori pannello solare
  
  solarVoltageR=getSolarVoltage();
  SerialUSB.print("Active Solar Voltage: ");
  SerialUSB.println(solarVoltageR);
  
  solarPowerR=getSolarPower();
  SerialUSB.print("Active Solar Power: ");
  SerialUSB.println(solarPowerR);
  delay(1000);
  // Controllo input energia solare
  systemPowered=checkSolar();

  // Se sistema ha abbastanza energia per funzionare
  if(systemPowered){
    st=READ_INPUT;
    SerialUSB.println("Enough Power to turn the System on, proceding");
  }
  else{
    SerialUSB.println("Not Enough Power");
    st=CHECK_ENV;
  }
   break;
   
   case READ_INPUT:
   
    SerialUSB.println("State READ_INPUT");
    
    calculateCleanDistance();
  
    delay(2000);
    calculateDirtyDistance();
    delay(1000);
    
    // Calcolo e generazione dei valori random
    waterTorbidityR=getWaterTorbidity();
    waterConductivityR=getWaterConductivity();
    
    // Calcolo volumi liquido in cisterne (ipotizzandone lo stesso volume max)
    //volumeCleanLiquid=calcLiquidVolume();
    //volumeDirtyLiquid=calcLiquidVolume();

    // Controllo per valutare se fermare sistema o meno 
    if(evaluateWaterLvlStatus()==false){activeSystem=false;}
    if(evaluateWaterTorbidity()==false){activeSystem=false;}
    //if(evaluateCleanWaterConductivity()==false){activeSystem=false;}
    //if(evaluateWaterFlow()==false){activeSystem=false;}

    SerialUSB.print("Solar Voltage: ");
    SerialUSB.println(solarVoltageR);
    SerialUSB.print("Solar Power: ");
    SerialUSB.println(solarPowerR);
    
//    SerialUSB.println("Dirty Water Volume: ");
//    SerialUSB.print(volumeDirtyLiquid);
//    SerialUSB.println("Clean Water Volume: ");
//    SerialUSB.print(volumeCleanLiquid);
    delay(1000);

    SerialUSB.print("CleanDistance: ");
    SerialUSB.println(cleanDistance);
    SerialUSB.print("DirtyDistance: ");
    SerialUSB.println(dirtyDistance);


    if(activeSystem){
          SerialUSB.println("OUT TRUE");
          st = INIT_MODEM;
          //Dopo aver effettuato controlli, se tutto va bene -> da qua si potranno attivare / switchare le pompette caffe, ecc
      }
    else{
          SerialUSB.println("OUT FALSE");
          st = CHECK_ENV;
        }
    
    
   break;
   case INIT_MODEM:
    SerialUSB.println("State INIT_MODEM");
    digitalWrite(RST_KEY, LOW);
    digitalWrite(LOW_PWR_KEY, HIGH);
    digitalWrite(PWR_KEY, HIGH);
    digitalWrite(PWR_KEY, LOW);
    delay(3000);
    digitalWrite(PWR_KEY, HIGH);
    delay(10000);
    
    ModuleState = moduleStateCheck();
    if(!ModuleState) 
     st = INIT_MODEM;
    else{
     sendData("AT+CCID", 3000, DEBUG);
     sendData("AT+EGMR=2,7", 3000, DEBUG);
     sendData("AT+CPIN?", 3000, DEBUG);
     sendData("AT+CREG?", 3000, DEBUG);
     sendData("AT+CSQ", 3000, DEBUG);
     sendData("AT+COPS?", 3000, DEBUG);
     sendData("AT+CGACT=0", 2000, DEBUG);
     sendData("AT+CGATT=0", 1000, DEBUG);
     sendData("AT+CGDCONT=1,\"IP\",\"myinternet.wind\"", 3000, DEBUG);
     sendData("AT+CGATT=1", 1000, DEBUG);
     sendData("AT+CGACT=1,1", 2000, DEBUG);
     sendData("AT+CIFSR",2000, DEBUG);
     sendData("AT+GPS=1",5000, DEBUG);
     
     SerialUSB.println("Maduino A9/A9G Cicle Begin!");
     //st = GPS_LOCATION;
      st = MQTT_CONN;
     
     valBat = (PS*2*analogRead(VBAT)/RES);
     
     valSys = (PS*2*analogRead(VSYS)/RES);
     
     SerialUSB.print("Batteria V.: ");
     SerialUSB.println(valBat);
     SerialUSB.print("Sistema V.: ");
     SerialUSB.println(valSys);
    }
   break;

   case GPS_LOCATION:
     SerialUSB.println("GPS_LOCATION");
     location = gspLocation(5000);
     loc = location.substring(2,20);
     
     SerialUSB.println(location);
     SerialUSB.println(loc);
     
     if (location != "")
      st = MQTT_CONN;
     else{
      location = "NO FIX";
      st = INIT_MODEM;
     }
   break;
   
   case MQTT_CONN:
    SerialUSB.println("MQTT CONNECTION");
    sendData("AT+MQTTCONN=\"maqiatto.com\",1883,\"maduino\",120,0,\"roberto@w2wsolutions.it\",\"maduino\"", 2000, DEBUG);
    st = MQTT_PUB;
   break;
   
   case MQTT_PUB:
    SerialUSB.println("MQTT PUBLISH");
     // Build string to send
    stringToSend = "AT+MQTTPUB=\"roberto@w2wsolutions.it/maduino\",\"{'l':";
    //stringToSend += loc;
    stringToSend +=";'B':";
    stringToSend += valBat;
    stringToSend +=";'P':";
    stringToSend += valSys;
    stringToSend +=";'CD':";
    stringToSend += cleanDistance;
    stringToSend +=";'DD':";
    stringToSend += dirtyDistance;
    stringToSend += "}\",0,0,0";
    
    SerialUSB.println(stringToSend);
    sendData(stringToSend, 1000, DEBUG);
    st = MQTT_SUB;
   break;

   case MQTT_SUB:
    SerialUSB.println("MQTT SUBSCRIBE");
    sendData("AT+MQTTSUB=\"roberto@w2wsolutions.it/maduino/cmd\",1,0", 1000, DEBUG);
    st = MQTT_WAIT_MSG;
   break;
  
   case MQTT_WAIT_MSG:
    SerialUSB.println("MQTT WAIT MESSAGE");
    SerialUSB.println(waitData(10000));
     st = MQTT_DISCONN;
   break;

   case MQTT_DISCONN:
    SerialUSB.println("MQTT DISCONN");
    sendData("AT+MQTTDISCONN", 2000, DEBUG);
    st = SLEEP;
   break;
   
   case SLEEP:
    SerialUSB.println("DEEP SLEEP");
    sendData("AT+CGACT=0", 2000, DEBUG);
    sendData("AT+CGATT=0", 2000, DEBUG);
    digitalWrite(LOW_PWR_KEY, LOW);
    sendData("AT+GPS=0",2000, DEBUG);
    //LowPower.deepSleep(30000);
    digitalWrite(LOW_PWR_KEY, HIGH);
    sendData("AT+GPS=1",2000, DEBUG);
    delay(6000);
    st = CHECK_ENV;
   break;
   
   default:
    st = CHECK_ENV;
    break;
  
  }
  
}


  
 
  

  // Refresh di 3 secondi
  delay(3000);
}

//Calcolo volume liquido in container (prendendo default 0.75m x 1.5m)
long calcLiquidVolume(){
   long randomDistance=random(0,heightContainer);
   long height;
   long volume;
   height=heightContainer-randomDistance;
   volume=(height*areaContainerBase)/1000; //1000 cm cubi = 1 litro
   return volume;
}

// Ottieni un valore pseudo randomico per voltaggio 
long getSolarVoltage(){
  return 8;
  //solarVoltageR= random(0,8.2);
}

// Ottieni un valore pseudo randomico per potenza 
long getSolarPower(){
  return 1;
  //solarPowerR= random(0,0.5);
}

//Ottieni un valore pseudo randomico per torbidità acqua
long getWaterTorbidity(){
  return 0.6;
  //waterTorbidityR= random(0,2);
}

//Ottieni un valore pseudo randomico per conducibilità acqua
long getWaterConductivity(){
  return 2;
  //waterConductivityR= random(0,4);
}


// Calcola distanza sensori-acqua nei diversi container
void getUSDistance(){
  digitalWrite(trigCleanPin, LOW);
  digitalWrite(trigDirtyPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigCleanPin, HIGH);
  digitalWrite(trigDirtyPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigCleanPin, LOW);
  digitalWrite(trigDirtyPin, LOW);
  
// Legge il pin echo dai container
  cleanDuration = pulseIn(echoCleanPin, HIGH);
  dirtyDuration = pulseIn(echoDirtyPin, HIGH);
  
// Calcolo distanze
  cleanDistance = cleanDuration * 0.034 / 2; // Velocità del suono divisa per due (andata e ritorno dopo rimbalzo)
  dirtyDistance = dirtyDuration * 0.034 / 2;
  SerialUSB.println("Clean Sensor Distance: ");
  SerialUSB.print(cleanDistance);
  SerialUSB.print(" cm");
  SerialUSB.println("Dirty Sensor Distance: ");
  SerialUSB.print(dirtyDistance);
  SerialUSB.print(" cm");
  }


// Controllo voltaggio e potenza pannello solare
bool checkSolar(){
  if(solarVoltageR=0)
  {
    SerialUSB.println("Insufficient Voltage");
    return false;
    }
  else
  {
    if(solarPowerR=0)
    {
      SerialUSB.println("Insufficient Power");
      return false;
      }
    else
    {
      SerialUSB.println("Power and Voltage are sufficient. Starting the system");
      return true;
      }
    }
}

bool calcSolarCurrent(){
  float instantCurrent;
  float curSensorValue;
  curSensorValue= analogRead(SolarCurPin);
  instantCurrent = MULTICURRENT * (curSensorValue/3.3) - SUBTRACTCURRENT;
  
}
bool calcPumpsCurrent(){
  float instantCurrent;
  float curPumpsSensorValue;
  curPumpsSensorValue= analogRead(PumpsCurPin);
  instantCurrent = MULTICURRENT * (curPumpsSensorValue/3.3) - SUBTRACTCURRENT;
  
}
bool calcPumpInCurrent(){
  float instantCurrent;
  float curPumpInSensorValue;
  curPumpInSensorValue= analogRead(PumpInCurPin);
  instantCurrent = MULTICURRENT * (curPumpInSensorValue/3.3) - SUBTRACTCURRENT;
  
}

// Controllo livelli di acqua dei container
bool evaluateWaterLvlStatus(){
  
  long cleanLimit = maxClearWater-waterThreshold;

  //Ipotizzando che siano della stessa capienza
  long dirtyLimit = maxDirtyWater-waterThreshold;
  
  SerialUSB.println("Evaluating water levels");
  delay(1000);


  volumeCleanLiquid=500;
  
  // Clean Water Container
  if(volumeCleanLiquid<warningClearWater){
    SerialUSB.println("Clean Water Volume is in range");
    //Essendo un valore normale, il sistema deve continuare a funzionare
  }
  else if(volumeCleanLiquid>=warningClearWater && volumeCleanLiquid<cleanLimit){
    SerialUSB.println("Warning, clean water container almost full");
    //Manda messaggio di warning su capienza quasi riempita della cisterna 

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }
  else if(volumeCleanLiquid>cleanLimit && volumeCleanLiquid<=maxClearWater)
  {
     SerialUSB.println("Alert, clean water container filled");
    //Manda messaggio di alert su capienza riempita della cisterna 

    //Essendo un valore di alert, il sistema deve fermarsi
    return false;
    }
  else{
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }



  
  delay(1000);

  volumeDirtyLiquid=500;
  
  // Dirty Water Container
  if(volumeDirtyLiquid<waterThreshold){
     SerialUSB.println("Alert, dirty water container empty");
    //Manda messaggio di alert su capienza riempita della cisterna 

    //Essendo un valore di alert, il sistema deve fermarsi
    return false;
  }
  else if(volumeDirtyLiquid<=warningDirtyWater && volumeDirtyLiquid>waterThreshold){
    SerialUSB.println("Warning, dirty water container almost empty");
    //Manda messaggio di warning su volume quasi terminato dell'acqua sporca 

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }

  else if(volumeDirtyLiquid>warningDirtyWater && volumeDirtyLiquid<=maxDirtyWater)
  {
    
    SerialUSB.println("Dirty Water Volume is in range");
     //Essendo un valore normale, il sistema deve continuare a funzionare
    }
  else{
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }
}


bool evaluateWaterTorbidity()
{
  
  SerialUSB.println("Evaluating Dirty Water Source's Torbidity levels");
  delay(1000);
  
  // Dirty Water Source
  
  if(waterTorbidityR<warningTorbidity){
     SerialUSB.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare 
  }
  else if(waterTorbidityR>=warningTorbidity && waterTorbidityR<limitTorbidity){
    SerialUSB.println("Warning, Water Source Torbidity values are near the limit");
    //Manda messaggio di warning sulla torbidità dell'acqua sporca della fonte

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }
  else if(waterTorbidityR>=limitTorbidity)
  {
    SerialUSB.println("Alert, Water Source Torbidity values are over the limit");
    return false;
  }
  else
  {
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }
}


bool evaluateCleanWaterConductivity()
{

  SerialUSB.println("Evaluating Clean Water's Conductivity levels");
  delay(1000);
  // Clean Water Container
  
  if(waterConductivityR<minConductivity){
    SerialUSB.println("Alert, Clean Water Conductivity values are under the limit");
    return false;
  }
  else if(waterConductivityR>maxConductivity){
    SerialUSB.println("Alert, Clean Water Conductivity values are over the limit");
    return false;
  }
  else if(waterConductivityR>=minConductivity && waterConductivityR<=maxConductivity){
    SerialUSB.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare 
  }
  else
  {
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }
}

bool evaluateWaterFlow(){


}


 
String checkData()
{
  String response = "";
  if(SerialUSB.available()> 0) {
   while (SerialUSB.available()){
    char c = SerialUSB.read();
    response += c;
   }
  return response;
  }
}


String gspLocation(int timeout)
{
  Serial1.println("AT+LOCATION=2");
   String response;
   long int time = millis();
   while ((time + timeout) > millis())
    {
        while (Serial1.available())
        {
            char c = Serial1.read();
            response += c;
        }
    }
    return response;
}

void parseCommand(String cmd)
{
   String response;
   
   while (SerialUSB.available())
   {
      char c = SerialUSB.read();
      response += c;
   }
  SerialUSB.println(cmd);
}

String sendData(String command, const int timeout, boolean debug)
{
    String response = "";
    Serial1.println(command);
    long int time = millis();
    while ((time + timeout) > millis())
    {
        while (Serial1.available())
        {
            char c = Serial1.read();
            response += c;
        }
    }
    if (debug)
    {
        SerialUSB.print(response);
    }
    return response;
}

String waitData(const int timeout)
{
    String response = "";
    long int time = millis();
    while ((time + timeout) > millis())
    {
        while (Serial1.available())
        {
            char c = Serial1.read();
            response += c;
        }
    }
    return response;
}

bool moduleStateCheck()
{
    int i = 0;
    bool moduleState=false;
    for (i = 0; i < 5; i++)
    {
        String msg = String("");
        msg = sendData("AT", 1000, DEBUG);
        if (msg.indexOf("OK") >= 0)
        {
            SerialUSB.println("A9/A9G Module had turned on.");
                moduleState=true;
            return moduleState;
        }
        delay(1000);
    }
    return moduleState;
}

void calculateCleanDistance(){
 
  digitalWrite(trigCleanPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigCleanPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigCleanPin, LOW);
  cleanDuration = pulseIn(echoCleanPin, HIGH);
  cleanDistance = cleanDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

}

void calculateDirtyDistance(){

  digitalWrite(trigDirtyPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigDirtyPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigDirtyPin, LOW);
  // Reads the echoPin from the containers
  dirtyDuration = pulseIn(echoDirtyPin, HIGH);  
  dirtyDistance = dirtyDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

}



void getTorbidityValues(){
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
   }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}


void toggleRelay1(){
if(rele1){
  digitalWrite(rele1Pin, LOW);
  rele1=false;
}
  else{
    digitalWrite(rele1Pin, HIGH);
    rele1=true;
  }
}

void toggleRelay2(){
if(rele2){
  digitalWrite(rele2Pin, LOW);
  rele2=false;
}
  else{
    digitalWrite(rele2Pin, HIGH);
    rele2=true;
  }
  
}
