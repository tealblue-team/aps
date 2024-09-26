#include <Time.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h> //non compatible with samd

#define PI 3.1415926535897932384626433832795
#define MULTICURRENT 73.3
#define SUBTRACTCURRENT 36.7

#define testMode 0
#define skipmqtt 1

#define echoCleanPin 3  //4 // pin D2 Arduino to pin Echo of HC-SR04
#define trigCleanPin 2  //3 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoDirtyPin 5  //6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigDirtyPin 4  //5 //attach pin D3 Arduino to pin Trig of HC-SR04

#define rele1Pin 12
#define rele2Pin 13

#define flowInPin 8
#define flowOutPin 9

#define Current2 A5
#define Current1 A4
#define TensionMicro A0
#define PumpsCurPin A3
#define PumpInCurPin A2

#define TdsSensorPin A1
#define VREF 3.3           // analog reference voltage(Volt) of the ADC
#define SCOUNT 30          // sum of sample point
int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

//Da aggiungere pin per relè pompette, sensore torbidità, sensore conduttività, sensore flusso

//VBAT -> solar input


float voltFinale = 0;
float correntePompa1 = 0;
float correntePompa2 = 0;
float valACorrUno = 0;
float valACorrDue = 0;

// Tempo trascorso (per timeout di sicurezza)
time_t seconds = 0;
// Tempo trascorso (per gestione pompette)
//time_t pumpSeconds = 0;

//LiquidCrystal_I2C lcd(0x27, 16, 2); // TO DO: check i2c address via code in the aps_resources doc

// Flag sistema
bool systemPowered = false;
bool activeSystem = true;
bool rele1 = false;
bool rele2 = false;

bool pump1active = false;
bool pump2active = false;

// Valori container cm
const float radiusContainer = 37.5;
const long areaContainerBase = PI * pow(radiusContainer, 2);
long heightCleanContainer = 23;  //150; //real height, based on max water level (95% of container?)
long heightDirtyContainer = 23;

//bool relePumps=false; //Falso -> pompe default di caffe REMOVED

// Costanti dei limiti
// Volume acqua
const long warningClearWater = 574.32;  // warning litri
const long maxClearWater = 662.68;      // max litri
const long warningDirtyWater = 132.53;  //warning litri
const long maxDirtyWater = 662.68;      // max litri
const long waterThreshold = 5;          // limiti litri

// Distanza
long warningClearDistance = heightCleanContainer * 0.85;  // warning cm
long warningDirtyDistance = heightDirtyContainer * 0.15;  // warning cm
const long distanceThreshold = 6;                         // limiti cm, in considerazione imprecisione sensore e scarto extra

// Valori torbidità (NTU)
const float warningTorbidity = 0.5;
const int limitTorbidity = 1;

// Valori Conduttività (µmhos/cm)
const int maxConductivity = 3;
const float minConductivity = 0.5;

// Variabili sensori US
long cleanDuration;  // Variabile per la durata del 'rimbalzo'
long dirtyDuration;
int cleanDistance;  // Variabile per la distanza del 'rimbalzo'
int dirtyDistance;

//Variabili sensori corrente pompa
float currentDirtyPump;
float currentCleanPump;


// Valori random per solare e liquidi
long volumeCleanLiquid;
long volumeDirtyLiquid;
long solarVoltageR;
long solarPowerR;
long waterTorbidityR;
long waterConductivityR;



#define DEBUG true

int PWR_KEY = 16;      //9;
int RST_KEY = 15;      //6;
int LOW_PWR_KEY = 14;  //5;

#define VBAT A1
#define VSYS A2
#define PS 3.3
#define RES 4096

float valBat = 0.0;
float valSys = 0.0;

#define CHECK_ENV 1
#define READ_INPUT 2
#define INIT_MODEM 3
#define GPS_LOCATION 4
#define MQTT_CONN 5
#define MQTT_PUB 6
#define MQTT_SUB 7
#define MQTT_WAIT_MSG 8
#define MQTT_DISCONN 9
#define SLEEP 10
#define EXECUTION 11


// Your GPRS credentials, if any
const char apn[] = "TM";  //myinternet.wind";
const char gprsUser[] = "";
const char gprsPass[] = "";


// MQTT details (OLD)
const char* mqtt_broker = "maqiatto.com";
const char* mqtt_topicPub = "iot1@coiote.io/maduino";
const char* mqtt_topicSub = "iot1@coiote.io/maduino/cmd";
const char* mqtt_user = "iot1@coiote.io";
const char* mqtt_psw = "coiote123";

String ip, iptemp;
String timestamp;
String loc, location, longitude, latitude, stringToSend;
String rssi, imei;
String rssitemp, imeitemp;
char* locptr = NULL;
char* locstrings[3];
char locarray[3];
byte locindex = 0;


String coiotebroker;

int len;
int st = CHECK_ENV;

bool ModuleState = false;


volatile int buttonState = 0;


// Setup di sistema con pin
void setup() {

  pinMode(trigCleanPin, OUTPUT);  // Sets both the trigPin as OUTPUT
  pinMode(trigDirtyPin, OUTPUT);
  pinMode(echoCleanPin, INPUT);  // Sets both the echoPin as INPUT
  pinMode(echoDirtyPin, INPUT);

  pinMode(rele1Pin, OUTPUT);
  pinMode(rele2Pin, OUTPUT);

  pinMode(TdsSensorPin, INPUT);

  //    lcd.begin();
  //
  //
  //    lcd.backlight();
  //    lcd.clear();
  //    lcd.setCursor(4,0);
  //    lcd.print("Test LCD");

  pinMode(flowInPin, INPUT);
  pinMode(flowOutPin, INPUT);

  pinMode(PWR_KEY, OUTPUT);
  pinMode(RST_KEY, OUTPUT);
  pinMode(LOW_PWR_KEY, OUTPUT);

  //setTime(seconds);
  //setTime(pumpSeconds);

  digitalWrite(rele1Pin, LOW);
  digitalWrite(rele2Pin, LOW);

  //attachInterrupt(0,pin_ISR, CHANGE);

  Serial1.begin(115200);
  SerialUSB.begin(115200);

  analogReadResolution(12);

  //Calculate dynamically the height of the containers (THIS on empty containers)
  //calcDirtyHeight();
  //calcCleanHeight();

  SerialUSB.println("Starting APS test on Maduino Zero A9G board");
}



void loop() {



  /////////////codice di test
  if (testMode == 1) {
    getUSDistance();
    turnOnRelay1();
    turnOffRelay2();


    //    if(cleanDistance<15){
    //      //testing manual
    //      toggleRelay2();
    //      toggleRelay1();
    //    }
  }
  //////////////


  //Struttura base del codice da bypassare in fase di relé testing
  if (testMode == 0) {

    String cmd = checkData();
    if (cmd != "")
      parseCommand(cmd);

    // Di default il sistema è attivo, poi facendo i check necessari si ferma o continua a funzionare a seconda dei risultati della valutazione
    activeSystem = true;

    switch (st) {

      case CHECK_ENV:

        //FORZANDO VOLTAGGIO POSITIVO

        // Controllo input energia solare
        systemPowered = true;  //checkSolar(10);//20);
        //test = checkSolar(20);
        //String outSolar = "Voltage on 20V is " + test; 
        //SerialUSB.println(outSolar);
        // Se sistema ha abbastanza energia per funzionare
        if (systemPowered) {
          activeSystem = true;
          st = READ_INPUT;
          SerialUSB.println("Enough Power to turn the System on, proceding");
        } else {
          activeSystem = false;
          turnOffRelay1();
          turnOffRelay2();
          SerialUSB.println("Not Enough Power");
          st = INIT_MODEM;  //CHECK_ENV;
        }
        break;

      case READ_INPUT:

        SerialUSB.println("State READ_INPUT");

        // Misura correnti su pompe
        checkValUnoCurrent();
        checkValDueCurrent();

        // Calcolo e generazione dei valori random
        //waterTorbidityR=getWaterTorbidity();
        //waterConductivityR=getWaterConductivity();

        // Calcolo volumi liquido in cisterne (ipotizzandone lo stesso volume max)
        //volumeCleanLiquid=calcLiquidVolume();
        //volumeDirtyLiquid=calcLiquidVolume();

        // Controllo per valutare se fermare sistema o meno
        //if (evaluateWaterLvlStatus() == false) { activeSystem = false; }
        // Controllo aggiornato per verificare indipendentemente pompe
        evaluateWaterLvlStatus();

        //ON HOLD FINO A INTRODUZIONE NUOVI SENSORI
        //if(evaluateWaterTorbidity()==false){activeSystem=false;}
        //if(evaluateCleanWaterConductivity()==false){activeSystem=false;}
        //if(evaluateWaterFlow()==false){activeSystem=false;}



        if (activeSystem) {
          SerialUSB.println("OUT TRUE");
          st = EXECUTION;
        } else {
          SerialUSB.println("OUT FALSE");
          st = CHECK_ENV;
          turnOffRelay1();
          turnOffRelay2();
        }



        //FORZATO DISATTIVATO
        //turnOffRelay1();
        //turnOffRelay2();
        //st = INIT_MODEM;

        break;

      case EXECUTION:
        SerialUSB.println("State EXECUTION");
        //Dopo aver effettuato controlli, se tutto va bene -> da qua si potranno attivare / switchare le pompette caffe, ecc

        if (pump2active) {

          turnOnRelay2();
          SerialUSB.println("Pump 2 is active");
        } else {
          turnOffRelay2();
          SerialUSB.println("Pump 2 is inactive");
        }
        delay(500);
        checkValDueCurrent();


        if (pump1active) {
          //if(rele2=false){
          SerialUSB.println("Pump 1 is active");
          turnOnRelay1();
        } else {
           SerialUSB.println("Pump 1 is inactive");
          turnOffRelay1();
        }
        delay(500);
        checkValUnoCurrent();
        //}
        // Check da sistemare (fallisce)
        //      if(!checkSolar(10)){
        //           activeSystem=false;
        //           turnOffRelay1();
        //           turnOffRelay2();
        //        }

        delay(500);
        st = INIT_MODEM;
        if (skipmqtt) { st = CHECK_ENV; }
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
        if (!ModuleState)
          st = INIT_MODEM;
        else {
          sendData("AT+CCID", 3000, DEBUG);

          //sendData("AT+ATQ2", 1000, DEBUG);  //added

          imeitemp = sendData("AT+EGMR=2,7", 3000, DEBUG);  //imei
          sendData("AT+CPIN?", 3000, DEBUG);                //sim pin
          sendData("AT+CREG?", 3000, DEBUG);                //net registration
                                                            //      sendData("AT+QIFGCNT=0", 3000, DEBUG);
                                                            //      sendData("AT+QICSGP=1,"bluevia.movistar.es", 3000, DEBUG);
                                                            //      timestamp=sendData("AT+QGSMLOC=2", 3000, DEBUG);
                                                            //      SerialUSB.println("TIMESTAMPPPPPPPPPPPPPPPPPPP");
                                                            //      SerialUSB.println(timestamp);

          rssitemp = sendData("AT+CSQ", 3000, DEBUG);  // signal quality
          sendData("AT+COPS?", 3000, DEBUG);           //select vender
          //sendData("AT+CGACT=0", 2000, DEBUG);         //pdpcontext
          //sendData("AT+CGATT=0", 1000, DEBUG);         //attach mt to packet domain

          //sendData("AT+CGDCONT=1,\"IP\",\"CMNET\"", 3000, DEBUG);  //choose pdpcontext (INSTRUCTABLES PROJECT)
          sendData("AT+CGDCONT=1,\"IP\",\"TM\"", 3000, DEBUG);  //choose pdpcontext


          sendData("AT+CGATT=1", 1000, DEBUG);
          sendData("AT+CGACT=1,1", 2000, DEBUG);
          iptemp = sendData("AT+CIFSR", 1000, DEBUG);  //2000,DEBUG); //ip

          //AT+HTTPACTION=0    // Connect the HTTP. (0-get, 1-post, 2-head)
          //AT+HTTPHEAD        // Read the response's header.
          //AT+HTTPREAD=0,3    // Read the content (“3” means the number of the reading data)

          //sendData("AT+HTTPINIT",2000, DEBUG);
          //sendData("AT+HTTPPARA=\"CID\",\"1\"",2000, DEBUG);
          //sendData("AT+HTTPPARA=\"URL\",\"timestamp.coiote.it\"",2000, DEBUG);
          //sendData("AT+HTTPPARA=\"URL\",\"http://timestamp.globalsign.com\"",2000, DEBUG);
          //sendData("AT+HTTPACTION=0",2000,DEBUG);

          //timestamp=sendData("AT+HTTPHEAD",2000,DEBUG);
          //timestamp=sendData("AT+HTTPREAD=0,3",2000,DEBUG);
          //timestamp=sendData("AT+HTTPGET",2000, DEBUG);




          //  SerialUSB.println("TIMESTAMPPPPPPPPPPPPPPPP");
          //  SerialUSB.println(timestamp);
          if (imeitemp != "" && imei == "") {
            imei = imeitemp;
            imei.remove(0, 8);                   //remove initial newline chars
            imei.remove(imei.length() - 8, 10);  //remove final newline chars
            //coiotebroker="data.coiote.it:1883/coiote/kva/aps/"+imei+"/";
            coiotebroker = "data.coiote.it:1883/coiote/kva/fotovoltaicofabio/" + imei + "/";
            //tcp://data.coiote.it:1883/coiote/val/[ apy-key ]/(Unique ID)/
            //SerialUSB.println("PRINTING IMEI===============================");
            //SerialUSB.println(imei);
          }
          if (iptemp != "" && ip == "") {
            ip = iptemp;
            ip.remove(0, 2);                 //remove initial newline chars
            ip.remove(ip.length() - 8, 10);  //remove final newline chars
          }
          if (rssitemp != "" && rssi == "") {
            rssi = rssitemp;
            rssi.remove(0, 8);                   //remove initial newline chars
            rssi.remove(rssi.length() - 8, 10);  //remove final newline chars
            rssi.replace(",", ".");
          }



          //      sendData("AT+HTTPINIT",1000, DEBUG);
          //      sendData("AT+HTTPPARA=\"CID\",\"1\"",1000, DEBUG);
          //      sendData("AT+HTTPPARA=\"URL\",\"timestamp.coiote.io\"",1000, DEBUG);
          //      timestamp=sendData("AT+HTTPGET",2000, DEBUG);


          sendData("AT+GPS=1", 5000, DEBUG);

          SerialUSB.println("Maduino A9/A9G Cicle Begin!");
          //st = GPS_LOCATION;
          st = MQTT_CONN;

          valBat = (PS * 2 * analogRead(VBAT) / RES);

          valSys = (PS * 2 * analogRead(VSYS) / RES);

          SerialUSB.print("Batteria V.: ");
          SerialUSB.println(valBat);
          SerialUSB.print("Sistema V.: ");
          SerialUSB.println(valSys);
        }
        break;

      case GPS_LOCATION:
        SerialUSB.println("GPS_LOCATION");
        location = gspLocation(5000);
        loc = location.substring(2, 20);
        loc.toCharArray(locarray, loc.length() + 1);
        if (loc != "") {
          locptr = strtok(locarray, ",");  // delimiter
          while (locptr != NULL) {
            locstrings[locindex] = locptr;
            locindex++;
            locptr = strtok(NULL, ",");
          }
          //   latitude = atoi(locstrings[0]);
          //  longitude = atoi(locstrings[1]);
        }
        SerialUSB.println(location);
        SerialUSB.println(loc);

        if (location != "")
          st = MQTT_CONN;
        else {
          location = "NO FIX";
          st = MQTT_CONN;  //INIT_MODEM;
        }
        break;

      case MQTT_CONN:


        //mosquitto_pub -d -q 1 -h thingsboard.coiote.it -p 1883 -t v1/devices/me/telemetry -u "9c6vzew9p14o84tjbg07" -m "{temperature:25}"

        //mosquitto_pub -d -q 1 -h thingsboard.coiote.it -p 1883 -t v1/devices/me/telemetry -i "n4hrybf265obf2fi63l9" -u "tenant@thingsboard.org" -P "z8f8xr1cjn050rr8mvh7" -m "{temperature:25}"

        SerialUSB.println("MQTT CONNECTION");
        //sendData("AT+MQTTCONN=\""+coiotebroker+"\",1883,\"aps\",120,0,\"aps\",\"aps\"", 2000, DEBUG);
        //sendData("AT+MQTTCONN=\""+coiotebroker+"\",1883,\"fotovoltaicofabio\",120,0,\"riva.fabio@gmail.com\",\"asd\"", 2000, DEBUG);
        sendData("AT+MQTTCONN=\"thingsboard.coiote.it\",1883,\"n4hrybf265obf2fi63l9\",120,0,\"tenant@thingsboard.org\",\"#arM1pF6t2N&E&R\"", 2000, DEBUG);
        //{clientId:"n4hrybf265obf2fi63l9",userName:"tenant@thingsboard.org",password:"#arM1pF6t2N&E&R"}
        //sendData("AT+MQTTCONN=\"maqiatto.com\",1883,\"maduino\",120,0,\"roberto@w2wsolutions.it\",\"maduino\"", 2000, DEBUG);
        st = MQTT_PUB;
        break;

      case MQTT_PUB:
        SerialUSB.println("MQTT PUBLISH");

        //String timestamp= String(day())+String(month())+String(year())+String(hour())+String(minute())+String(second());
        //sendData("AT+CCLK",2000, DEBUG);//String(now());

        // Build string to send


        //FORCING DATA

        //timestamp= "1234567";
        //ip= "128.121.12.1";
        latitude = "45.6130597";
        longitude = "9.5019483";

        //    stringToSend = "AT+MQTTPUB=\"roberto@w2wsolutions.it/maduino\",\"{\"_t\":\"";
        //    stringToSend += timestamp;
        //    stringToSend += "\",\"_y\":\"l\",\"_s\":\"\"";
        //    stringToSend +=",\"_i\":\"";
        //    stringToSend += ip;
        //    stringToSend +="\",\"poms\":[{\"_p\":\"0";
        //    stringToSend +="\",\"R\":";
        //    stringToSend += rssi;
        //    stringToSend +=",\"b\":";
        //    stringToSend += valBat;
        //    stringToSend +="},{\"_p\":\"1\"";
        //    stringToSend +=",\"ON\":";
        //    stringToSend += activeSystem;
        //    stringToSend +="\",\"L\":\"";
        //    stringToSend += loc;
        //    stringToSend +="\",\"B\":";
        //    stringToSend += valBat;
        //    stringToSend +=",\"P\":";
        //    stringToSend += valSys;
        //    stringToSend +=",\"CD\":";
        //    stringToSend += cleanDistance;
        //    stringToSend +=",\"DD\":";
        //    stringToSend += dirtyDistance;
        //    stringToSend += "}]}\",0,0,0";

        stringToSend = "AT+MQTTPUB=\"v2/devices/me/telemetry\",\"{\"temperature\":\"77\"";

        //stringToSend = "AT+MQTTPUB=\"thingsboard.coiote.it/v1/devices/me/telemetry\",\"{\"temperature\":\"77\"";
        //_t=";
        //stringToSend = "AT+MQTTPUB=\"fotovoltaicofabio/"+imei+"\",\"_t=";
        //stringToSend = "AT+MQTTPUB=\"aps/"+imei+"\",\"_t=";
        //stringToSend = "AT+MQTTPUB=\"roberto@w2wsolutions.it/maduino\",\"_t=";
        // stringToSend += timestamp;
        // stringToSend += ";_y=l;_s=0;";
        // stringToSend +="_i=";
        // stringToSend += ip;
        // stringToSend +="_p=0;";
        // stringToSend +="R=";
        // stringToSend += rssi;
        // stringToSend +=";b=";
        // stringToSend += valBat;
        // stringToSend +="_p=1";
        // stringToSend +=";ON=";
        // stringToSend += activeSystem;
        // stringToSend +=";LAT=";
        // stringToSend += latitude;
        // stringToSend +=";LON=";
        // stringToSend += longitude;
        // stringToSend +=";L=";
        // stringToSend += loc;
        // stringToSend +=";B=";
        // stringToSend += valBat;
        // stringToSend +=";P=";
        // stringToSend += valSys;
        // stringToSend +=";CD=";
        // stringToSend += cleanDistance;
        // stringToSend +=";DD=";
        // stringToSend += dirtyDistance;
        // stringToSend +=";CP=";
        // stringToSend += currentCleanPump;
        // stringToSend +=";DP=";
        // stringToSend += currentDirtyPump;
        stringToSend += "}\",0,0,0";

        SerialUSB.println(stringToSend);
        sendData(stringToSend, 1000, DEBUG);
        st = MQTT_DISCONN;  //MQTT_SUB;
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
        sendData("AT+GPS=0", 2000, DEBUG);
        //LowPower.deepSleep(30000);
        digitalWrite(LOW_PWR_KEY, HIGH);
        sendData("AT+GPS=1", 2000, DEBUG);
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
long calcLiquidVolume() {
  long randomDistance = random(0, heightCleanContainer);
  long height;
  long volume;
  height = heightCleanContainer - randomDistance;
  volume = (height * areaContainerBase) / 1000;  //1000 cm cubi = 1 litro
  return volume;
}

// Ottieni un valore pseudo randomico per voltaggio
long getSolarVoltage() {
  float valATen = 0;
  valATen = analogRead(TensionMicro);  // lettura tensione partitore per Vmicro
  float voltMicro = (valATen / 4096) * 3.3;
  float voltFinale = voltMicro * (106.8 / 6.8);
  //SerialUSB.println(valATen);
  SerialUSB.print("Tensione Partitore: ");
  SerialUSB.println(voltFinale);
  return voltFinale;
}

// Ottieni un valore pseudo randomico per potenza
long getSolarPower() {

  return 1;
  //solarPowerR= random(0,0.5);
}

//Ottieni un valore pseudo randomico per torbidità acqua
long getWaterTorbidity() {
  return 0.6;
  //waterTorbidityR= random(0,2);
}

//Ottieni un valore pseudo randomico per conducibilità acqua
long getWaterConductivity() {
  return 2;
  //waterConductivityR= random(0,4);
}


// Calcola distanza sensori-acqua nei diversi container
void getUSDistance() {

  digitalWrite(trigCleanPin, LOW);

  delayMicroseconds(2);
  digitalWrite(trigCleanPin, HIGH);

  delayMicroseconds(10);
  digitalWrite(trigCleanPin, LOW);

  // Legge il pin echo dai container
  cleanDuration = pulseIn(echoCleanPin, HIGH);

  //delay(20);

  digitalWrite(trigDirtyPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigDirtyPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigDirtyPin, LOW);

  dirtyDuration = pulseIn(echoDirtyPin, HIGH);

  // Calcolo distanze
  cleanDistance = cleanDuration * 0.034 / 2;  // Velocità del suono divisa per due (andata e ritorno dopo rimbalzo)
  dirtyDistance = dirtyDuration * 0.034 / 2;
  SerialUSB.println("Clean Sensor Distance: ");
  SerialUSB.print(cleanDistance);
  SerialUSB.println(" cm");
  SerialUSB.println("Dirty Sensor Distance: ");
  SerialUSB.print(dirtyDistance);
  SerialUSB.println(" cm");
}


// Controllo voltaggio e potenza pannello solare
bool checkSolar(int limit) {

  solarVoltageR = getSolarVoltage();
  SerialUSB.print("Active Solar Voltage: ");
  SerialUSB.println(solarVoltageR);

  solarPowerR = getSolarPower();
  SerialUSB.print("Active Solar Power: ");
  SerialUSB.println(solarPowerR);

  if (solarVoltageR < limit) {
    SerialUSB.println("Insufficient Voltage");
    return false;
  } else {
    if (solarPowerR = 0) {
      SerialUSB.println("Insufficient Power");
      return false;
    } else {
      SerialUSB.println("Power and Voltage are sufficient. Starting the system");
      return true;
    }
  }
}


// Controllo livelli di acqua dei container
bool evaluateWaterLvlStatus() {

  calculateCleanDistance();
  calculateDirtyDistance();

  //testing with faulty US sensor
  //return true;

  int limitClean = heightCleanContainer - distanceThreshold;
  int limitDirty = heightDirtyContainer - distanceThreshold;
  //distanceClean=20; //FORCING INPUT

  // Clean Water Container
  if (cleanDistance <= distanceThreshold) {
    SerialUSB.println("Alert, clean water container full");
    //Manda messaggio di alert su capienza riempita della cisterna

    //Essendo un valore di alert, il sistema deve fermarsi
    //return false;

    //Alert -> Pompa indipendente si ferma
    pump2active = false;
  } else if (cleanDistance < warningClearDistance && cleanDistance > distanceThreshold) {
    SerialUSB.println("Clean Water Volume is in range");
    //Essendo un valore normale, il sistema deve continuare a funzionare
    pump2active = true;
  } else if (cleanDistance >= warningClearDistance && cleanDistance < limitClean) {
    SerialUSB.println("Warning, clean water container almost empty");
    //Manda messaggio di warning su capienza quasi riempita della cisterna

    //Essendo un valore di warning, il sistema deve continuare a funzionare
    pump2active = true;
  } else if (cleanDistance > limitClean && cleanDistance <= heightCleanContainer) {
    SerialUSB.println("Empty clean water container");
    //Manda messaggio di alert su capienza riempita della cisterna

    //Non essendo vincolante, non c'è bisogno che si fermi
    //return false;
    pump2active = true;
  } else {
    SerialUSB.println("Unknown data, system stopped");
    pump2active = true;
    //return false;
  }


  //distanceDirty=40; //FORCING INPUT

  // Dirty Water Container
  if (dirtyDistance <= distanceThreshold) {
    SerialUSB.println("Alert, dirty water container full");
    //Manda messaggio di alert su capienza riempita della cisterna

    //Essendo un valore di alert, il sistema deve fermarsi
    //return false;
    pump1active = false;
  } else if (dirtyDistance <= warningDirtyDistance && dirtyDistance > distanceThreshold) {
    SerialUSB.println("Warning, dirty water container almost full");
    //Manda messaggio di warning su volume quasi terminato dell'acqua sporca

    //Essendo un valore di warning, il sistema deve continuare a funzionare
    pump1active = true;
  }

  else if (dirtyDistance > warningDirtyDistance && dirtyDistance <= limitDirty) {

    SerialUSB.println("Dirty Water Volume is in range");
    //Essendo un valore normale, il sistema deve continuare a funzionare
    pump1active = true;
  } else if (dirtyDistance > limitDirty && dirtyDistance <= heightDirtyContainer) {
    SerialUSB.println("Alert, dirty water container empty");
    //Manda messaggio di alert su capienza riempita della cisterna

    //Essendo un valore di alert, il sistema deve fermarsi
    //return false;
    pump1active = true;
    pump2active = false;
  } else {
    SerialUSB.println("Unknown data, system stopped");
    pump1active = false;
    //return false;
  }

  return true;


  // Test e calcoli volumi
  //  long cleanLimit = maxClearWater-waterThreshold;
  //
  //  //Ipotizzando che siano della stessa capienza
  //  long dirtyLimit = maxDirtyWater-waterThreshold;
  //
  //  SerialUSB.println("Evaluating water levels");
  //  delay(1000);
  //
  //
  //  volumeCleanLiquid=500;
  //
  //  // Clean Water Container
  //  if(volumeCleanLiquid<warningClearWater){
  //    SerialUSB.println("Clean Water Volume is in range");
  //    //Essendo un valore normale, il sistema deve continuare a funzionare
  //  }
  //  else if(volumeCleanLiquid>=warningClearWater && volumeCleanLiquid<cleanLimit){
  //    SerialUSB.println("Warning, clean water container almost full");
  //    //Manda messaggio di warning su capienza quasi riempita della cisterna
  //
  //    //Essendo un valore di warning, il sistema deve continuare a funzionare
  //  }
  //  else if(volumeCleanLiquid>cleanLimit && volumeCleanLiquid<=maxClearWater)
  //  {
  //     SerialUSB.println("Alert, clean water container filled");
  //    //Manda messaggio di alert su capienza riempita della cisterna
  //
  //    //Essendo un valore di alert, il sistema deve fermarsi
  //    return false;
  //    }
  //  else{
  //    SerialUSB.println("Unknown data, system stopped");
  //    return false;
  //  }
  //
  //
  //
  //
  //  delay(1000);
  //
  //  volumeDirtyLiquid=500;
  //
  //  // Dirty Water Container
  //  if(volumeDirtyLiquid<waterThreshold){
  //     SerialUSB.println("Alert, dirty water container empty");
  //    //Manda messaggio di alert su capienza riempita della cisterna
  //
  //    //Essendo un valore di alert, il sistema deve fermarsi
  //    return false;
  //  }
  //  else if(volumeDirtyLiquid<=warningDirtyWater && volumeDirtyLiquid>waterThreshold){
  //    SerialUSB.println("Warning, dirty water container almost empty");
  //    //Manda messaggio di warning su volume quasi terminato dell'acqua sporca
  //
  //    //Essendo un valore di warning, il sistema deve continuare a funzionare
  //  }
  //
  //  else if(volumeDirtyLiquid>warningDirtyWater && volumeDirtyLiquid<=maxDirtyWater)
  //  {
  //
  //    SerialUSB.println("Dirty Water Volume is in range");
  //     //Essendo un valore normale, il sistema deve continuare a funzionare
  //    }
  //  else{
  //    SerialUSB.println("Unknown data, system stopped");
  //    return false;
  //  }
}


bool evaluateWaterTorbidity() {

  SerialUSB.println("Evaluating Dirty Water Source's Torbidity levels");
  //delay(1000);

  // Dirty Water Source

  if (waterTorbidityR < warningTorbidity) {
    SerialUSB.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare
  } else if (waterTorbidityR >= warningTorbidity && waterTorbidityR < limitTorbidity) {
    SerialUSB.println("Warning, Water Source Torbidity values are near the limit");
    //Manda messaggio di warning sulla torbidità dell'acqua sporca della fonte

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  } else if (waterTorbidityR >= limitTorbidity) {
    SerialUSB.println("Alert, Water Source Torbidity values are over the limit");
    return false;
  } else {
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }
}


bool evaluateCleanWaterConductivity() {

  SerialUSB.println("Evaluating Clean Water's Conductivity levels");
  delay(1000);
  // Clean Water Container

  if (waterConductivityR < minConductivity) {
    SerialUSB.println("Alert, Clean Water Conductivity values are under the limit");
    return false;
  } else if (waterConductivityR > maxConductivity) {
    SerialUSB.println("Alert, Clean Water Conductivity values are over the limit");
    return false;
  } else if (waterConductivityR >= minConductivity && waterConductivityR <= maxConductivity) {
    SerialUSB.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare
  } else {
    SerialUSB.println("Unknown data, system stopped");
    return false;
  }
}

bool evaluateWaterFlow() {
}



String checkData() {
  String response = "";
  if (SerialUSB.available() > 0) {
    while (SerialUSB.available()) {
      char c = SerialUSB.read();
      response += c;
    }
    return response;
  }
}


String gspLocation(int timeout) {
  Serial1.println("AT+LOCATION=2");
  String response;
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }
  return response;
}

void parseCommand(String cmd) {
  String response;

  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    response += c;
  }
  SerialUSB.println(cmd);
}

String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  Serial1.println(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }
  if (debug) {
    SerialUSB.print(response);
  }
  return response;
}

String waitData(const int timeout) {
  String response = "";
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
    }
  }
  return response;
}

bool moduleStateCheck() {
  int i = 0;
  bool moduleState = false;
  for (i = 0; i < 5; i++) {
    String msg = String("");
    msg = sendData("AT", 1000, DEBUG);
    if (msg.indexOf("OK") >= 0) {
      SerialUSB.println("A9/A9G Module had turned on.");
      moduleState = true;
      return moduleState;
    }
    delay(1000);
  }
  return moduleState;
}

void calculateCleanDistance() {
  digitalWrite(trigCleanPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigCleanPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigCleanPin, LOW);
  cleanDuration = pulseIn(echoCleanPin, HIGH);
  cleanDistance = cleanDuration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  SerialUSB.print("CleanDistance: ");
  SerialUSB.println(cleanDistance);
}

void calculateDirtyDistance() {
  digitalWrite(trigDirtyPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigDirtyPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigDirtyPin, LOW);
  // Reads the echoPin from the containers
  dirtyDuration = pulseIn(echoDirtyPin, HIGH);
  dirtyDistance = dirtyDuration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  SerialUSB.print("DirtyDistance: ");
  SerialUSB.println(dirtyDistance);
}

void getTorbidityValues() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
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


void toggleRelay1() {
  if (rele1) {
    digitalWrite(rele1Pin, LOW);
    rele1 = false;
  } else {
    digitalWrite(rele1Pin, HIGH);
    rele1 = true;
  }
}

void toggleRelay2() {
  if (rele2) {
    digitalWrite(rele2Pin, LOW);
    rele2 = false;
  } else {
    digitalWrite(rele2Pin, HIGH);
    rele2 = true;
  }
}

void turnOnRelay1() {
  digitalWrite(rele1Pin, HIGH);
  rele1 = true;
}
void turnOnRelay2() {
  digitalWrite(rele2Pin, HIGH);
  rele2 = true;
}

void turnOffRelay1() {
  digitalWrite(rele1Pin, LOW);
  rele1 = false;
}
void turnOffRelay2() {
  digitalWrite(rele2Pin, LOW);
  rele2 = false;
}

//V = 0 + (I/Ipn) * 0.625
//V = 0 + (I/valA) * 0.625
//  instantCurrent = MULTICURRENT * (curPumpsSensorValue/3.3) - SUBTRACTCURRENT;
void checkValUnoCurrent() {
  valACorrUno = analogRead(Current1) - 13;              // lettura corrente 1
  currentDirtyPump = (valACorrUno / 4096 * 3.3) * 3.2;  //calcolo in base ai 10A e ai 5 giri
  if (currentDirtyPump < 25) {
    currentDirtyPump = 0;
  }
  //SerialUSB.println(valACorrUno);
  SerialUSB.println("Corrente 1: ");
  SerialUSB.println(currentDirtyPump);
}

void checkValDueCurrent() {
  valACorrDue = analogRead(Current2);                   // lettura corrente 2
  currentCleanPump = (valACorrDue / 4096 * 3.3) * 3.2;  //calcolo in base ai 10A e ai 5 giri
  if (currentCleanPump < 25) {
    currentCleanPump = 0;
  }

  SerialUSB.println("Corrente 2: ");
  SerialUSB.println(currentCleanPump);
}

void calcCleanHeight() {

  digitalWrite(trigCleanPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigCleanPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigCleanPin, LOW);
  cleanDuration = pulseIn(echoCleanPin, HIGH);
  heightCleanContainer = cleanDuration * 0.034 / 2;
  warningClearDistance = heightCleanContainer * 0.85;  // warning cm
}

void calcDirtyHeight() {

  digitalWrite(trigDirtyPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigDirtyPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigDirtyPin, LOW);
  dirtyDuration = pulseIn(echoDirtyPin, HIGH);
  heightDirtyContainer = dirtyDuration * 0.034 / 2;
  warningDirtyDistance = heightDirtyContainer * 0.15;  // warning cm
}

void getStaticInfo() {
}
//void processSyncMessage() {
// // if time sync available from serial port, update time and return true
// while (Serial.available() >= TIME_MSG_LEN ) { // time message consists of header & 10 ASCII digits
// char c = Serial.read() ;
// Serial.print(c);
// if ( c == TIME_HEADER ) {
// time_t pctime = 0;
// for (int i = 0; i < TIME_MSG_LEN - 1; i++) {
// c = Serial.read();
// if ( c >= '0' && c <= '9') {
// pctime = (10 * pctime) + (c - '0') ; // convert digits to a number
// }
// }
// setTime(pctime); // Sync Arduino clock to the time received on the serial port
// }
// }
//}
