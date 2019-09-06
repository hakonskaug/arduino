#include <Servo.h>
#include <PS3BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

// Pinner og variabler for motor og servo{
//Deklareringer
const int motor1 = 5; //ENA - Styrer farten til Motor1
const int motor2 = 6; //ENB - Styrer farten til Motor2
const int in1 = 22; //Signal in1 til Motor1
const int in2 = 24; //Signal in2 til Motor1
const int in3 = 26; //Signal in3 til Motor2
const int in4 = 28; //Signal in4 til Motor2
const int servoPWM = 3; //PWM-signalet til servoen
int grader;
int servoX;
Servo myServo;
//}

//Pinne og variabler for vår "Rødt lys sensor"{
const int redLimitValue = 600;
const int redLightPin = A0;
int redLightSensor;
//}



//Pinner og variabler for sensorene forran på bilen
//Disse vil gjøre det mulig og sette bilen i autopilot {
int sensor1 = 0; 
int sensor2 = 0; 
int sensor3 = 0; 
int sensor4 = 0;
int sensor5 = 0;
const int IRsensor1 = A15;
const int IRsensor2 = A14;
const int IRsensor3 = A13; 
const int IRsensor4 = A12;
const int IRsensor5 = A11;

//}

int kjor; 
int rygg;
bool autoTrue = false; //Usikker på om denne trengs - failsafe

// Pinner og variabler for ryggesensor{
const int trigPin = 10;
const int echoPin = 11;
const int buzzer = 12;
long duration;
int avstandVegg;
int minsteAvstand;

//}


//Setter i setup hva de forskjellige pinnene skal være
void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  pinMode(motor1, OUTPUT);      //  Pin ~5: Styrer fart på motor1 
  pinMode(motor2, OUTPUT);      //  Pin ~6: Styrer fart på motor2 
  pinMode(in1, OUTPUT);         //  Pin 22: Setter polaritet på motor1 
  pinMode(in2, OUTPUT);         //  Pin 24: Setter polaritet på motor1
  pinMode(in3, OUTPUT);         //  Pin 26: Setter polaritet på motor2
  pinMode(in4, OUTPUT);         //  Pin 28: Setter polaritet på motor2
  pinMode(IRsensor1, INPUT);    //  Pin A15:
  pinMode(IRsensor2, INPUT);    //  Pin A14:
  pinMode(IRsensor3, INPUT);    //  Pin A13:
  pinMode(IRsensor4, INPUT);    //  Pin A12:
  pinMode(IRsensor5, INPUT);    //  Pin A11:
  pinMode(redLightPin, INPUT);  //  Pin A0:
  myServo.attach(servoPWM);
  pinMode(trigPin, OUTPUT);     //  Setter trigPin til å være en UTGANG
  pinMode(echoPin, INPUT);      //  Setter echoPin til å være en INNGANG 
  pinMode(buzzer, OUTPUT);      //  Setter Buzzeren til utgang
  
  
  Serial.print(F("\r\nBluetooth klar for tilkobling."));
}
void loop() {
  Usb.Task();// Holder usb tilgangen "åpen"
  servoX = PS3.getAnalogHat(LeftHatX); // Setter servomotoren styring til ps3 kontroller
  redLightSensor = analogRead(A0); // Starter avlesning av "rødlys" sensoren
  kjor = PS3.getAnalogButton(R2); // Knappen til og kjøre fremover
  rygg = PS3.getAnalogButton(L2); // Knappen til og kjøre bakover
  
  if (PS3.PS3Connected) // Sjekker om PS3 kontrolleren er koblet til
  {    
    //SERVO styring
    if(servoX >= 130 || servoX <= 100)
    {
      grader = map(servoX, 0, 255, 60, 119);
      Serial.print("\nGraden er:  ");
      Serial.print(grader);
      myServo.write(grader);
    }
    
    //AKSELERASJON 
    if(kjor >= 70 && rygg < 70)
    {
      Serial.println("Kjører.");
      analogWrite(motor1, kjor);
      analogWrite(motor2, kjor);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }/*
    else
    {
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
    }*/
    
    
    //REVERSERING
    if(rygg >= 70 && kjor < 70)
    {
      // Setter polariteten riktig sånn at bilen rygger
      Serial.println("Rygger.");
      analogWrite(motor1, rygg);
      analogWrite(motor2, rygg);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      
      // Gjør klar "trigpin", her starter ryggesensoren og scanne
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
    
      // Setter trigpin høy i 10 microsekunder
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      // Settes lav igjen
      digitalWrite(trigPin, LOW);
    
      // Leser inn lydbølgene som blir sendt ut av sensoren, også måler hvor lang tid det tar før de kommer tilbake
      duration = pulseIn(echoPin, HIGH); 
      // Kalkulerer avstanden til veggen
      avstandVegg= duration*0.034/2+1;
    
      // Setter en grense for hvor nærme bilen kan komme veggen
      minsteAvstand = avstandVegg;
      // Hvis du er for nærme veggen vil det bli spilt en lyd til advarsel
      if (minsteAvstand <= 5){
        digitalWrite(buzzer, HIGH);
        tone(12,400);
      }
      else{
      // Spiller ikke lyd når du er utenfor grenseområdet for avstanden
      digitalWrite(buzzer, LOW);
      noTone(12); 
      }
    // Printer ut avstanden til veggen
    Serial.print("Distance: ");
    Serial.println(avstandVegg);
    }
    
    // Bilen vil stoppe
    else if(kjor < 70 && rygg < 70)
    {
      // Setter motorene til 0
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
    }
    //BREMS
    if(kjor >= 100 && rygg >= 100)
    {
      // Setter motorene lave, dette bremser bilen
      Serial.println("Bremser.");
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
    //RØDT LYS
    if(redLightSensor > redLimitValue){
      Serial.println("Rødt lys oppdaget.");
      redLight();
    }
    //Koble fra kontroller, har egen knapp
    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS\nKobler fra kontrolleren."));
      PS3.disconnect();
    }
    else {
      //AUTOPILOT slås på ved trykk på opp pilen
      if (PS3.getButtonClick(UP)) {
        Serial.println(F("\r\nAutopilot initialiseres."));
        autopilot();
      }
    }
  }
}
void autopilot(){
  Serial.println(F("Autopilot aktivert."));// Autopiloten blir aktivert
  autoTrue = true;
  while(autoTrue == true){// Hvis den er aktivert vil den kjøre av seg selv
    Usb.Task();
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(motor1, 140);
    analogWrite(motor2, 140);
    sensor1 = analogRead(IRsensor1);// Bruker 5 sensorer forran bilen, for og følge en svart linje på bakken
    sensor2 = analogRead(IRsensor2);
    sensor3 = analogRead(IRsensor3);
    sensor4 = analogRead(IRsensor4);
    sensor5 = analogRead(IRsensor5);
    
    //Sensorkode     
    //RETT FRAM
    if(sensor1 < 800 && sensor2 < 800 && sensor3 > 800 && sensor4 < 800 && sensor5 < 800)
    {
      myServo.write(90);
      //Setter en hastighet rett frem, uten noe servo styring.
      Serial.println("Steg 1");
      analogWrite(motor1, 200);
      analogWrite(motor2, 200);
    }
    //SVAKT HØYRE
    if(sensor2 > 800)
    {
      // Svinger svakt mot høyre, for og følge streken
      Serial.println("Svinger svakt mot hoyre.");
      myServo.write(105);
      analogWrite(motor1, 220);
      analogWrite(motor2, 220);
    }
    //MYE HØYRE
    if(sensor1 > 800 || (sensor1 > 800 && sensor2 > 800))
    {
      // Svinger brått mot høyre, for og følge streken
      Serial.println("Svinger hardt mot hoyre.");
      myServo.write(120);   
      analogWrite(motor1, 220);
      analogWrite(motor2, 220);
    }
    //SVAKT VENSTRE
    if(sensor4 > 800)
    {
      // Svinger svakt mot venstre, for og følge streken
      Serial.println("Svinger svakt mot venstre.");
      myServo.write(75);
      analogWrite(motor1, 220);
      analogWrite(motor2, 220);   
    }
    //MYE VENSTRE
    if(sensor5 > 800 || (sensor4 > 800 && sensor5 > 800))
    {
      // Svinger brått mot venstre, for og følge streken
      Serial.println("Svinger hardt mot venstre.");
      myServo.write(60);
      analogWrite(motor1, 220);
      analogWrite(motor2, 220);  
    }
        //Skrur av autopilot med ned pilen
    if(PS3.getButtonClick(DOWN)){
      Serial.println(F("UP.\nAutopilot deaktiveres."));
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      autoTrue = false;
      break;
    }
    else{
      // Vil stoppe på rødt lys selv i autopilot modus
      if(redLightSensor > redLimitValue){
      Serial.println("Rødt lys oppdaget.");
      redLight();
      }
    }
  }
}
// Rødt lys, bilen stopper på rødt lys
void redLight(){
  bool redTrue = true;
  // Skriver ut om "redlight" er aktivert
  Serial.println("RedLight(): aktivert.");
  // Hvis sensoren plukker opp signalet om rødt lys
  // vil bilen stoppe
  while(redTrue == true){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    redLightSensor = analogRead(redLightPin); //Leser inn sensorverdi på nytt.
    Serial.println(redLightSensor);
    //Hvis det ikke er rødt lys lenger:
    // Ikke rødt lys, gå tilbake til manuel styring
    if(redLightSensor<redLimitValue && autoTrue == false)
    {
      Serial.println("Rødt lys borte. Redlight() off. Returnerer til loop().");
      redTrue = false;
      break;
    }
    // Ikke rødt lys lengre, gå til autopilot igjen
    if(redLightSensor<redLimitValue && autoTrue == true)
    {
      Serial.println("Rødt lys borte. Redlight() off. Returnerer til autopilot().");
      autopilot();
    }
    delay(500);
  }
}
