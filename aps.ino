#include <Time.h>

#define PI 3.1415926535897932384626433832795

#define echoCleanPin 4 // pin D4 Arduino pin Echo HC-SR04
#define trigCleanPin 3 // pin D3 Arduino  pin Trig HC-SR04
#define echoDirtyPin 6 //  pin D6 Arduino pin Echo HC-SR04
#define trigDirtyPin 5 // pin D5 Arduino pin Trig HC-SR04
//Da aggiungere pin per relè pompette, sensore torbidità, sensore conduttività, sensore flusso

//VBAT -> solar input

// Tempo trascorso (per timeout di sicurezza)
time_t seconds = 0;
// Tempo trascorso (per gestione pompette)
//time_t pumpSeconds = 0;

// Flag sistema
bool systemPowered=false;
bool activeSystem=true; 
bool relePumps=false; //Falso -> pompe default di caffe

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
long cleanDistance; // Variabile per la distanza del 'rimbalzo'
long dirtyDistance;

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



// Setup di sistema con pin
void setup() {
  pinMode(trigCleanPin, OUTPUT); // Settato trigPin come OUTPUT
  pinMode(echoCleanPin, INPUT); // Settato echoPin come INPUT
  
  pinMode(trigDirtyPin, OUTPUT);
  pinMode(echoDirtyPin, INPUT);
  //setTime(seconds);
  //setTime(pumpSeconds);
  Serial.begin(9600);
  Serial.println("Starting APS test on Maduino Zero A9G board");
}



void loop() {

  //Futura implementazione di timeout di sicurezza (con calcoli futuri si conteranno i giorni senza cambiamento di flag -> al quarto giorno arriva un allarme ulteriore)
  //seconds = now();
  //Serial.println(seconds);
  
  // Di default il sistema è attivo, poi facendo i check necessari si ferma o continua a funzionare a seconda dei risultati della valutazione
  activeSystem=true;

  // Generazione valori pannello solare
  
  solarVoltageR=getSolarVoltage();
  Serial.println("Active Solar Voltage: ");
  Serial.print(solarVoltageR);
  
  solarPowerR=getSolarPower();
  Serial.println("Active Solar Power: ");
  Serial.print(solarPowerR);

  // Controllo input energia solare
  systemPowered=checkSolar();

  // Se sistema ha abbastanza energia per funzionare
  if(systemPowered){

    
    // Calcolo e generazione dei valori random
    waterTorbidityR=getWaterTorbidity();
    waterConductivityR=getWaterConductivity();

    // Ottenimento valori US
    getUSDistance();

    // Ai fini di testing, calcolo volume in base a distanza US
    //volumeCleanLiquid=calcLiquidVolumeUS(cleanDistance);
    //volumeDirtyLiquid=calcLiquidVolumeUS(dirtyDistance);
    
    // Alternativamente, calcolo volumi liquido in cisterne con valori random (ipotizzandone lo stesso volume max)
    volumeCleanLiquid=calcLiquidVolume();
    volumeDirtyLiquid=calcLiquidVolume();

    // Controllo per valutare se fermare sistema o meno 
    if(evaluateWaterLvlStatus()==false){activeSystem=false;}
    if(evaluateWaterTorbidity()==false){activeSystem=false;}
    if(evaluateCleanWaterConductivity()==false){activeSystem=false;}
    if(evaluateWaterFlow()==false){activeSystem=false;}

    Serial.println("Solar Voltage: ");
    Serial.print(solarVoltageR);
    Serial.println("Solar Power: ");
    Serial.print(solarPowerR);
    Serial.println("Dirty Water Volume: ");
    Serial.print(volumeDirtyLiquid);
    Serial.println("Clean Water Volume: ");
    Serial.print(volumeCleanLiquid);
    

    if(activeSystem){
          Serial.println("OUT TRUE");
          //Dopo aver effettuato controlli, se tutto va bene -> da qua si potranno attivare / switchare le pompette caffe, ecc
      }
    else{
      Serial.println("OUT FALSE");
        }
  }

  // Refresh di 5 secondi
  delay(5000);
}

long calcLiquidVolumeUS(long USDistance){
   long height;
   long volume;
   height=heightContainer-USDistance;
   volume=(height*areaContainerBase)/1000; //1000 cm cubi = 1 litro
   return volume;
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
  solarVoltageR= random(0,8.2);
}

// Ottieni un valore pseudo randomico per potenza 
long getSolarPower(){
  solarPowerR= random(0,0.5);
}

//Ottieni un valore pseudo randomico per torbidità acqua
long getWaterTorbidity(){
  waterTorbidityR= random(0,2);
}

//Ottieni un valore pseudo randomico per conducibilità acqua
long getWaterConductivity(){
  waterConductivityR= random(0,4);
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
  Serial.println("Clean Sensor Distance: ");
  Serial.print(cleanDistance);
  Serial.print(" cm");
  Serial.println("Dirty Sensor Distance: ");
  Serial.print(dirtyDistance);
  Serial.print(" cm");
  }


// Controllo voltaggio e potenza pannello solare
bool checkSolar(){
  if(solarVoltageR=0)
  {
    Serial.println("Insufficient Voltage");
    return false;
    }
  else
  {
    if(solarPowerR=0)
    {
      Serial.println("Insufficient Power");
      return false;
      }
    else
    {
      Serial.println("Power and Voltage are sufficient. Starting the system");
      return true;
      }
    }
}


// Controllo livelli di acqua dei container
bool evaluateWaterLvlStatus(){
  
  long cleanLimit = maxClearWater-waterThreshold;

  //Ipotizzando che siano della stessa capienza
  long dirtyLimit = maxDirtyWater-waterThreshold;
  
  Serial.println("Evaluating water levels");

  // Clean Water Container
  if(volumeCleanLiquid<warningClearWater){
    Serial.println("Clean Water Volume is in range");
    //Essendo un valore normale, il sistema deve continuare a funzionare
  }
  else if(volumeCleanLiquid>=warningClearWater && volumeCleanLiquid<cleanLimit){
    Serial.println("Warning, clean water container almost full");
    //Manda messaggio di warning su capienza quasi riempita della cisterna 

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }
  else if(volumeCleanLiquid>cleanLimit && volumeCleanLiquid<=maxClearWater)
  {
     Serial.println("Alert, clean water container filled");
    //Manda messaggio di alert su capienza riempita della cisterna 

    //Essendo un valore di alert, il sistema deve fermarsi
    return false;
    }
  else{
    Serial.println("Unknown data, system stopped");
    return false;
  }

  // Dirty Water Container
  if(volumeDirtyLiquid<waterThreshold){
     Serial.println("Alert, dirty water container empty");
    //Manda messaggio di alert su capienza riempita della cisterna 

    //Essendo un valore di alert, il sistema deve fermarsi
    return false;
  }
  else if(volumeDirtyLiquid<=warningDirtyWater && volumeDirtyLiquid>waterThreshold){
    Serial.println("Warning, dirty water container almost empty");
    //Manda messaggio di warning su volume quasi terminato dell'acqua sporca 

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }
  else if(volumeDirtyLiquid>dirtyLimit && volumeCleanLiquid<=maxDirtyWater)
  {
    
    Serial.println("Dirty Water Volume is in range");
     //Essendo un valore normale, il sistema deve continuare a funzionare
    }
  else{
    Serial.println("Unknown data, system stopped");
    return false;
  }
}


bool evaluateWaterTorbidity()
{
  
  Serial.println("Evaluating Dirty Water Source's Torbidity levels");
  // Dirty Water Source
  
  if(waterTorbidityR<warningTorbidity){
     Serial.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare 
  }
  else if(waterTorbidityR>=warningTorbidity && waterTorbidityR<limitTorbidity){
    Serial.println("Warning, Water Source Torbidity values are near the limit");
    //Manda messaggio di warning sulla torbidità dell'acqua sporca della fonte

    //Essendo un valore di warning, il sistema deve continuare a funzionare
  }
  else if(waterTorbidityR>=limitTorbidity)
  {
    Serial.println("Alert, Water Source Torbidity values are over the limit");
    return false;
  }
  else
  {
    Serial.println("Unknown data, system stopped");
    return false;
  }
}


bool evaluateCleanWaterConductivity()
{

  Serial.println("Evaluating Clean Water's Conductivity levels");
  
  // Clean Water Container
  
  if(waterConductivityR<minConductivity){
    Serial.println("Alert, Clean Water Conductivity values are under the limit");
    return false;
  }
  else if(waterConductivityR>maxConductivity){
    Serial.println("Alert, Clean Water Conductivity values are over the limit");
    return false;
  }
  else if(waterConductivityR>=minConductivity && waterConductivityR<=maxConductivity){
    Serial.println("Water Source Torbidity values are regular");
    //Essendo un valore normale, il sistema deve continuare a funzionare 
  }
  else
  {
    Serial.println("Unknown data, system stopped");
    return false;
  }
}

bool evaluateWaterFlow(){


}
