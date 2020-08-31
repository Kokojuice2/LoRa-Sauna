/*
 *  DHT temp en vochtigheidsmeter
 *  3pin Pinout :
 *                Signaal(D3) - 3.3V - GND
 *  
 *  LDR Pinout : 
 *               5V - Signaal(A0)  
 *                  - Resistor 1000 Ohm naar GND
 *  
 *  Relais Pinout : 
 *               Signaal(D4) - 5V - GND
 *  
 *  RGB Led Pinout : 
 *               Blauw(D10)
 *               Groen(D11)
 *               Rood(D12)
 *               - => GND
 *  
 *  Als het even is als "threshold = 28", dan groen licht aan.
 *  Als het warmer is als "threshold = 28", dan rood licht aan.
 *  Als het kouder is als "threshold = 28", dan blauw licht aan 
 *          en relais aan voor ventilator.
 *              Ventilator heeft hoger Voltage nodig die Arduino niet kan voorzien. 
 *              Ventilator aangesloten op 12V batterij als volgt:
 *                  venti + --> + batterij
 *                  venti - --> COM Relais
 *                NO Relais --> - batterij want willen pas AAN als signaal erdoor  
 *                              NO = Normaal open, als signaal erdoor --> volle elektriciteitskring
 *                              NC = Normaal gesloten, als signaal erdoor --> geen contact, verbroken.
 * 
 *  Geel LED licht voor BINNEN licht : 
 *                Signaal(D5) - 1000 Ohm naar GND
 *                
 *  Groen LED licht voor BUITEN licht :
 *                Signaal(D6) - 1000 Ohm naar GND 
 *  
 *  
 *  Created by Joachim Pham
 *  23/08/2020
 *  
 */

#include "DHT.h"
#define DHTTYPE DHT11          //type DHT sensor
#define DHTPIN 3            //input van DHT signal
DHT dht(DHTPIN, DHTTYPE);

const int thresholdTemp = 27;   //temperatuurswaarde limiet
float temp, vocht;

//*******************************************************
        // Relay als 
#define RELAYPIN 4


//*******************************************************
        // LDR licht sensor
#define LDRPIN A0
float licht;
const int thresholdLicht = 400;  // lichtwaarde limiet

//*******************************************************
        //RGB LED  als thermostaat
#define ROODPIN  13
#define GROENPIN 11
#define BLAUWPIN 10
  
//*******************************************************
        // Binnen en buitenlicht
#define BINNEN 5
#define BUITEN 6

//******************_Licht schakelaar_*******************

    #define KNOPPIN 12

    int knopStatus;         // Status van de knop: niet ingedrukt = 0, ingedrukt = 1
    int lichtStatus = HIGH;      // huidige status lichtpin
    int laatsteknopstatus = LOW; // vorige lezing knop 

    unsigned long lastDebounceTime = 0;     //laaste keer dat outpin werd gebruikt
    unsigned long debounceDelay = 20;     //de debounce tijd, als led flikkert, verhogen.


//*******************************************************


void setup() {

    // LDR licht sensor
    pinMode(LDRPIN, INPUT);

    // Relay
    pinMode(RELAYPIN, OUTPUT);

    // RGB led
    pinMode(ROODPIN, OUTPUT);
    pinMode(GROENPIN, OUTPUT);
    pinMode(BLAUWPIN, OUTPUT);

    // Binnen Buiten licht
    pinMode(BINNEN, OUTPUT);
    pinMode(BUITEN, OUTPUT);

    digitalWrite(BINNEN, lichtStatus);    //lichtstatus gaat veranderen naar HIGH/LOW
    digitalWrite(BUITEN, lichtStatus);    //waardoor deze direct de lichten aan of uit zet


    // Knop 
    pinMode(KNOPPIN, INPUT);

    // DHT
    pinMode(DHTPIN, INPUT);
    dht.begin();

Serial.begin(9600);

Serial.print("DHT Temperatuur en Vochtigheid Start");
Serial.println("");
Serial.print("Temperature threshold is gezet op: ");
Serial.print(thresholdTemp);
Serial.print("°C");
Serial.println("");
Serial.print("Licht threshold is gezet op: ");
Serial.print(thresholdLicht);
Serial.print(" lux");
Serial.print("");

}

void loop() {



  float licht = analogRead(LDRPIN);
  float temp = dht.readTemperature();
  float vocht = dht.readHumidity();
  
            // Als er geen data is van DHT meter
if (isnan(temp) ||  isnan(vocht))  {
  Serial.println(F("Kijk even na of het juist aangesloten is!"));
  return;
}


Serial.print("");
Serial.println("------------------------");
Serial.print("De temperatuur is: ");
Serial.print(temp);
Serial.print(" °C");

Serial.println(" ");
Serial.print("De vochtigheid is: ");
Serial.print(vocht);
Serial.println(" %\t");

//************************************************************

if (temp < thresholdTemp)  {             //Als de gemeten temperatuur LAGER/KLEINER is dan vaste waarde, verwarming gaat aan, ROOD.
  digitalWrite(ROODPIN, HIGH);
  digitalWrite(GROENPIN, LOW);
  digitalWrite(BLAUWPIN, LOW);
  digitalWrite(RELAYPIN, LOW);
  Serial.println("Verwarming AAN,");
  Serial.println("Ventilatie UIT");
}
  
if (temp > thresholdTemp)  {             //Als de gemeten temperatuur HOGER/GROTER is dan vaste waarde, ventilator gaat aan, BLAUW.
  digitalWrite(ROODPIN, LOW);
  digitalWrite(GROENPIN, LOW);
  digitalWrite(BLAUWPIN, HIGH);
  digitalWrite(RELAYPIN, HIGH);
  Serial.println("Verwarming UIT,");
  Serial.println("Ventilatie AAN");
}

if (temp == thresholdTemp)  {             //Als de gemeten temperatuur hetzelfde is als vaste waarde, niets gaat aan.
  digitalWrite(ROODPIN, LOW);
  digitalWrite(GROENPIN, HIGH);
  digitalWrite(BLAUWPIN, LOW);
  digitalWrite(RELAYPIN, LOW);
  Serial.println("Alles UIT");
}

//**************************************************

Serial.print("Het licht is: ");
Serial.print(licht);
Serial.println(" lux");

//**************************************************

if (licht < thresholdLicht) {             // Als het donkerder is dan thresholdLight, dan gaat licht aan.
  digitalWrite(BINNEN, HIGH);
  digitalWrite(BUITEN, HIGH);
  Serial.println("Lichten AAN");
}

if (licht > thresholdLicht) {             // Anders lichten uit
  digitalWrite(BINNEN, LOW);
  digitalWrite(BUITEN, LOW);
  Serial.println("Lichten UIT");
}

      //***********_Manuele knop besturing licht_******************

 int leesin = digitalRead(KNOPPIN);

if (leesin != laatsteknopstatus) {
  lastDebounceTime = millis();
}

if ((millis() - lastDebounceTime) > debounceDelay) {

  if (leesin != knopStatus) {
    knopStatus = leesin;

    if (knopStatus == HIGH) {
      lichtStatus = !lichtStatus;
    }
  }
}

digitalWrite(BINNEN, lichtStatus);
digitalWrite(BUITEN, lichtStatus);

laatsteknopstatus = leesin;


/*
if (knopstatus == HIGH) {
  digitalWrite(BINNEN, HIGH);
  digitalWrite(BUITEN, HIGH);
  Serial.println("Lichten AAN met knop");
} 
else 
{
  digitalWrite(BINNEN, LOW);
  digitalWrite(BUITEN, LOW);
  Serial.println("Lichten UIT met knop");
}*/

//**************************************************

}
