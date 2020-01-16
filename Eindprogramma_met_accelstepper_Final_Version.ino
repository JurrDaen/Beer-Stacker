/*
    Programma voor de Beer Stacker
*/

#include <FastLED.h>                                   //Bibliotheek voor de WS2812B
#include <Wire.h>                                      //I2C bibliotheek
#include <LCD.h>                                       //LCD bibliotheek
#include <LiquidCrystal_I2C.h>                         //LCD I2C bibliotheek
#include <SoftwareSerial.h>                            //bluetooth module bibliotheek
#include <AccelStepper.h>                              //Stepper motor library


#define NUM_LEDS 4                                     //4 LEDs worden gebruikt
#define LED_PIN 4                                      //LED data pin

CRGB led[NUM_LEDS];                                    //Initialiseren LEDs
LiquidCrystal_I2C  lcd(0x27, 2, 1, 0, 4, 5, 6, 7);     //0x27 is het standaard I2C adres van de LCD backpack
SoftwareSerial BT(3, 5);                               //BT module TX D3, BT module RX D5
const int upperLimitSwitch = 6;                        //Limit switch bovenaan de Stacker
const int lowerLimitSwitch = 7;                        //Limit switch onderaan de Stacker
const int trigPin1 = 11;                               //Trigger pin voor de afstandssensor
const int echoPin1 = 12;                               //Echo pin voor de afstandssensor
int buttonPushSelector = 1;                            //Zorgt er voor dat er in ieder geval altijd een kratje geselecteerd is

AccelStepper stepper(1, 2, 10);                        //pin2 = step, pin 10 = direction

long duration, cm;                                     //intitialiseren van de tijd en afstand
long initial_homing = -1;                              //Startpunt voor de steppermotor

void bierKrat();                                       //initialiseren bierkrat functie
void euroKrat();                                       //initialiseren eurokrat functie
void starting();                                       //initialiseren starting functie
void readingSensorAfstand();                           //initialiseren readingSensorAfstand functie
void GreenLight();                                     //initialiseren GreenLight functie
void YellowLight();                                    //initialiseren YellowLight functie
void RedLight();                                       //initialiseren RedLight functie
void BlueLight();                                      //initialiseren BlueLight functie
void homing();                                         //initialiseren homing functie

char a;                                                //Character voor de seriële communicatie
char b;                                                //Character voor de seriële communicatie
char laatsteCharacter;

void setup()
{
  stepper.setMaxSpeed(600);                            //Maximale snelheid voor de steppermotor. 600Hz
  stepper.setAcceleration(500);                        //Maximale acceleratie voor de steppermotor.
  BT.begin(9600);                                      //Bluetoothsneldheid initialiseren
  Serial.begin(9600);                                  //Seriële snelheid initialiseren
  FastLED.clear();                                     //LEDs uitzetten
  FastLED.addLeds<NEOPIXEL, LED_PIN>(led, NUM_LEDS);   //Aantal LEDS initialiseren

  lcd.begin (16, 2); // for 16 x 2 LCD module          //Grote van het LCD scherm wordt ingesteld
  lcd.setBacklightPin(3, POSITIVE);                    //Backlight van het LCD instellen
  lcd.setBacklight(HIGH);                              //Zet de backlight aan van het LCD

  pinMode(trigPin1, OUTPUT);                           //Trigpin
  pinMode(echoPin1, INPUT);                            //Vangt het hoog frequente signaal op voor de afstandssensor
  pinMode(upperLimitSwitch, INPUT);

  starting();                                          //Startanimatie weergeven
  homing();                                            //Homing van de stepper motor
}


void loop()
{
  if (BT.available()>0)                                    //Leest of er een bluetooth signaal binnenkomt
  {
    laatsteCharacter = (BT.read());                                      //Leest wat het bluetooth signaal inhoud
    if (laatsteCharacter == 'A')                                         //A is een waarde die vanaf de app wordt verstuurd
    {
      buttonPushSelector = 1;
    }
    if (laatsteCharacter == 'B')                                         //B is een waarde die vanaf de app wordt verstuurd
    {
      buttonPushSelector = 2;
    }
  }
  switch (buttonPushSelector)                           //Een switch om de keuze te maken tussen de verschillende kratjes op basis van een variabele.
  {
    case 1:
      bierKrat();
      break;
    case 2:
      euroKrat();
      break;
  }
}


void homing()
{
  lcd.setCursor (0, 0);                                //LCD cursor links bovenaan neerzetten
  lcd.print("Homing...       ");
  while (digitalRead(upperLimitSwitch) == LOW)         //Zolang de upperlimit switch niet wordt ingedrukt beweegt de motor omhoog
  {
    stepper.moveTo(initial_homing);                    //Zorgt er voor dat de motor omhoog beweegt
    initial_homing--;                                  //Zet de stap weer op 1 lager waardoor de motor weer omhoog zal bewegen
    stepper.run();                                     //Laat de motor een stap zetten
    delay(3);                                          //Zorgt er voor dat de motor op 300Hz draait
  }

  stepper.setCurrentPosition(0);                       //Zet de positie van de stepper motor op 0
  delay(1000);
  stepper.moveTo(1000);                                //Zet 1000 stappen omlaag
  while (stepper.distanceToGo() != 0)                  //Zolang de stapper motor nog geen stap heeft gezet blijft hij in de loop
  {
    stepper.run();                                     //Laat de motor een stap zetten
  }

  delay(5);
  stepper.setCurrentPosition(0);                       //Zet de positie van de stpper motor op 0
  stepper.setMaxSpeed(700);                            //Maximale snelheid voor de steppermotor. 700Hz
  stepper.setAcceleration(400);                        //Maximale acceleratie voor de steppermotor.
}


void starting()
{
  lcd.setCursor (0, 0);
  lcd.print("Lifter starting  ");
  for (int x = 0; x < 4; x++)                          //Loop voor het herhalen van het effect
  {
    for (int j = 0; j < 256; j++)                      //Loop voor het feller worden van de LED's
    {
      for (int i = 0; i < NUM_LEDS; i++)               //Loop voor het tellen van de LED's
      {
        led[i] = CRGB(j, 0, j);
      }
      FastLED.show();
      delay(2);
    }
    for (int j = 255; j >= 0; j--)                     //Loop voor het donkerder worden van de LED's
    {
      for (int i = 0; i < NUM_LEDS; i++)               //Loop voor het tellen van de LED's
      {
        led[i] = CRGB(j, 0, j);                        //Geeft Rood en Blauw de waardes van j
      }
      FastLED.show();
      delay(2);                                        //Debug delay
    }
  }

}


void Tillen(int Hoogte)                                //Deze functie krijgt de hoogte mee afhankelijk van het kratje en zorgt dat het kratje wordt opgetilt
{
  Serial.println("Klik op de knop");
    if (laatsteCharacter == 'C')
    {
      Serial.println("Tillen..");
      stepper.moveTo(Hoogte);                            //Zet het kratje weer neer op de grond
      while (stepper.distanceToGo() != 0)                //Kijkt of het aantal stappen al zijn gedaan
      {
        stepper.run();
      }
          delay(1000);                                       //Wacht 2 seconden
    stepper.moveTo(14000);                             //Ga naar een hoogte van 14000
    while (stepper.distanceToGo() != 0)                //Zolang hij het aantal stappen nog niet heeft gedaan komt hij hier in
    {
      stepper.run();
    }
    }
    laatsteCharacter = 0;
}


void Neerzetten(int Hoogte)                            //Deze functie zorgt dat hij het desbetreffende kratje ook weer neer kan zetten
{                            //Leest wat er binnen komt via bluetooth
    if (laatsteCharacter == 'D')
    {
      Serial.println("Neerzetten");
      stepper.moveTo(Hoogte);                            //Zet het kratje weer neer op de grond
      while (stepper.distanceToGo() != 0)                //Kijkt of het aantal stappen al zijn gedaan
      {
        stepper.run();
      }
          delay(1000);                                       //Wacht 2 seconden
    stepper.moveTo(14000);                             //Ga naar een hoogte van 14000
    while (stepper.distanceToGo() != 0)                //Zolang hij het aantal stappen nog niet heeft gedaan komt hij hier in
    {
      stepper.run();
    }
    }
}

void bierKrat()                                        //Deze functie wordt aangreoepen als het bierkrat gekozen is
{
  lcd.setCursor (0, 1);                                //Zet de cursor van het LCD op de 2e regel
  lcd.print("Bierkrat      ");                         //Geeft aan op het LCD welk kratje getilt wordt
  readingSensorAfstand();                              //Leest de afstand
  Neerzetten(24900);                                   //Geeft de hoogte mee die nodig is voor het neerzetten
  if (cm >= 10 && cm <= 15)                            //Als de waarde cm tussen 10 en 15 is, zal de LED groen worden
  {
    GreenLight();
    //delay(1);
    Tillen(24900);
    laatsteCharacter = 'A';
   // Serial.println("Wachten op tillen");
  }
  else if (cm >= 16 && cm <= 30)                      //Als de waarde cm tussen de 16 en 30 is, zal de LED geel worden
  {
    YellowLight();
  }
  else if (cm >= 31)                                   //Als de waarde cm boven de 31 is, zal de LED blauw worden
  {
    BlueLight();
  }
  else if (cm <= 9)                                    //Als de waarde cm kleiner is dan 9, zal de LED rood worden
  {
    RedLight();
  }
  lcd.home (); // set cursor to 0,0
  lcd.print(F("Afstand: "));
  lcd.print(cm);
  lcd.print(F(" cm    "));
}


void euroKrat()                                        //Deze functie wordt aangeroepen als het eurokrat gekozen is
{
  lcd.setCursor (0, 1);                                //Zet de cursor van het LCD op de 2e regel
  lcd.print("Eurokrat      ");                         //Geeft aan op het LCD welk kratje getilt wordt
  readingSensorAfstand();                              //Leest de sensorafstand
  Neerzetten(21150);                                   //Geeft de hoogte mee die nodig is voor het neerzetten
  if (cm >= 8 && cm <= 12)                             //Als de waarde cm tussen 10 en 25 is, zal de LED groen worden
  {
    GreenLight();
    delay(1);
    Tillen(21150);
    laatsteCharacter = 'B';
  }
  else if (cm >= 13 && cm <= 30)                       //Als de waarde cm tussen de 26 en 79 is, zal de LED geel worden
  {
    YellowLight();
  }
  else if (cm >= 31)                                   //Als de waarde cm boven de 80 is, zal de LED blauw worden
  {
    BlueLight();
  }
  else if (cm <= 7)                                    //Als de waarde cm kleiner is dan 9, zal de LED rood worden
  {
    RedLight();
  }
  lcd.home ();                                         //Set de cursor op 0,0
  lcd.print(F("Afstand: "));
  lcd.print(cm);                                       //Print de afstand op het LCD in cm
  lcd.print(F(" cm    "));
}

void readingSensorAfstand()                            //Leest de sensor voor de afstand uit
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);                        //Verstuurd een geluidssignaal
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);                         //Stopt met het sturen van een geluidssignaal
  duration = pulseIn(echoPin1, HIGH);
  cm = (duration / 2) / 29.1;                          //Zet de tijd van het signaal om naar een afstand
}

void RedLight()                                        //Zorgt er voor dat de LED rood licht kan geven
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    led[i] = CRGB(153, 0, 0);                          //Rood
  }
  FastLED.show();
}


void YellowLight()                                     //Zorgt er voor dat de LED geel licht kan geven
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    led[i] = CRGB(153, 153, 0);                        //Geel
  }
  FastLED.show();
}


void GreenLight()                                      //Zorgt er voor dat de LED groen licht kan geven
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    led[i] = CRGB(0, 153, 0);                          //Groen
  }
  FastLED.show();
}


void BlueLight()                                       //Zorgt er voor dat de LED blauw licht kan geven
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    led[i] = CRGB(0, 0, 153);                          //Blauw
  }
  FastLED.show();
}
