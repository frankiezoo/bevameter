#include "HX711.h" // Load Load-Cell Library
#include <SD.h> // Load SD Library
#include <SPI.h> //For SD card
#include <TinyGPS++.h> //For gps
#include <SoftwareSerial.h> //For gps
#include <SevSeg.h>
#define CLK 30
#define DT 31
#define SW 32

//For motor controller:
// Yellow - OUT1
// Green - OUT2

//For rotary encoder set up
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;

//Number display pins
int pinA = 38;int pinB = 40;int pinC = 39;int pinD = 37;
int pinE = 35;int pinF = 36;int pinG = 34;
int pinDP = 41;
int displayCounter;

//For GPS setup
//(RX, TX)   -   Middle pin is RX
static const int RXPin = 7, TXPin = 8;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
volatile float minutes, seconds;
volatile int degree, secs, mins;
bool timeValid = false;
bool locValid = false;
bool altValid = false;
String timeStr;
float latValue;
float lngValue;
float altValue;

//All pin slots for different electronics
int chipSelect = 53; // Pin for MicroSD Card Adapter
File myFile; //file object for read and write data
int fileCount = 0; // Track number of files
String fileName = "data" + String(fileCount) + ".txt";
int joystickVal = 0;
bool joystickTime = true;
#define VRX_PIN A0
const int InPin = 35;
const int OutPin = 37;

#define ENA_PIN  27
/* BYE BYE POT

#define Potentiometer_pin A1
int initDROPos = 0;
*/ // Hello DRO
int droClock = 2;
int droData = 3;
int bit_array[25];
unsigned long time_now;
float initDROPos = 0.0;
bool initPos = false;
float tempPos = 0.0;

//Load cell data readings
const int LOADCELL_DOUT_PIN = 42;
const int LOADCELL_SCK_PIN = 43;
HX711 scale;
int avgreading;
float weightLimit = 280;
//Pin slot for start/stop button
const int buttonPin = 7;
int buttonState = 0;
//Pin slots for LED signal lights
const int yellowLight = 25;
const int greenLight = 24;

void displayInfo(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    locValid = true;
    altValid = true;
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
    locValid = false;
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
    timeValid = false;
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    timeValid = true;
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    timeStr = String(String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year()) + "  " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
  }
  else
  {
    Serial.print(F("INVALID"));
    timeValid = false;
  }

  Serial.println();
}


void setup(){
  // put your setup code here, to run once:
  Serial.begin(19200);
  ss.begin(9600);
  pinMode(InPin, OUTPUT);
  pinMode(OutPin, OUTPUT);
  pinMode (ENA_PIN, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(yellowLight, OUTPUT);
  pinMode(greenLight, OUTPUT);
  digitalWrite(InPin, LOW);
  digitalWrite(OutPin, LOW);
  digitalWrite(yellowLight, LOW);
  digitalWrite(greenLight, LOW);
  digitalWrite(ENA_PIN, HIGH);
  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinDP, OUTPUT);      
	
  // Set encoder pins as inputs
	pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(SW, INPUT_PULLUP);
  delay(1000);
  
  // DRO setup
  pinMode(droClock, INPUT);
  pinMode(droData, INPUT);

  //Load cell setup up
  Serial.println("Load cell setup...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-75.7004);
  

  // SD card setup
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial) {
    ;
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  if(SD.exists("data0.txt")){
    while(SD.exists(fileName)){
      Serial.println(fileName + " already exists.");
      fileCount += 1;
      fileName = "data" + String(fileCount) + ".txt";
    }
  }
  Serial.println(fileName + " does not exist!");
  // open a new file and immediately close it:
  Serial.println("Creating new data.txt...");\
  myFile = SD.open(fileName, FILE_WRITE);
  fileCount += 1;
  buttonState = 0;
  Serial.println("New data file created!");
  
  //Adjust plate size
  int encbuttonState = 0;
  while (encbuttonState == 0){
    // Read the current state of CLK
    currentStateCLK = digitalRead(CLK);
    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(DT) != currentStateCLK) {
        counter --;
        currentDir ="CCW";
        displayCounter --;
      } else {
        // Encoder is rotating CW so increment
        counter ++;
        currentDir ="CW";
        displayCounter ++;
      }
      Serial.print("Direction: ");
      Serial.print(currentDir);
      Serial.print(" | Counter: ");
      Serial.println(counter);
    }
    // Remember last CLK state
    lastStateCLK = currentStateCLK;
    // Read the button state
    int btnState = digitalRead(SW);
    //If we detect LOW signal, button is pressed
    if (btnState == LOW) {
      //if 50ms have passed since last LOW pulse, it means that the
      //button has been pressed, released and pressed again
      if (millis() - lastButtonPress > 50) {
        Serial.println("Button pressed!");
        //blink();
        encbuttonState = 1;
      }
      // Remember last button press event
      lastButtonPress = millis();
    }

    //Capping the values of the number so that it does not exceed what the number display can display
    if(displayCounter > 19){
      displayCounter = 19;
      counter = 19;
    }
    else if(displayCounter < 0){
      displayCounter = 0;
      counter = 0;
    }

    switch (displayCounter){
      case 0:
        zero();
        break;
      case 1:
        zero(); dot();
        break;
      case 2:
        one();
        break;
      case 3:
        one(); dot();
        break;
      case 4:
        two();
        break;
      case 5:
        two(); dot();
        break;
      case 6:
        three(); 
        break;
      case 7:
        three(); dot();
        break;
      case 8:
        four(); 
        break;
      case 9:
        four(); dot();
        break;
      case 10:
        five(); 
        break;
      case 11:
        five(); dot();
        break;
      case 12:
        six();
        break;
      case 13:
        six(); dot();
        break;
      case 14:
        seven();
        break;
      case 15:
        seven(); dot();
        break;
      case 16:
        eight();
        break;
      case 17:
        eight(); dot();
        break;
      case 18:
        nine();
        break;
      case 19:
        nine(); dot();
        break;
    }

    // Put in a slight delay to help debounce the reading
    delay(1);

    
  }
  

  delay(250);
  //Setting up the GPS
  digitalWrite(yellowLight, HIGH);
  Serial.println("Setting up gps stuff...");
  //Allow calibration of joystick
  bool endLoop = false;
  while(joystickTime == true){
    //Serial.println("Finding GPS connection...");
    
    while (endLoop != true)
    //Testing for valid connection to satellite for data
      while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
          if (gps.location.isUpdated()) {
            if (gps.satellites.isValid() && gps.satellites.value() > 0) {
              Serial.println("GPS fix acquired. Connected to satellite(s).");
              //Serial.println();
              Serial.println("GPS connection found!");
              displayInfo();
              int x = 0;
              while (x < 10){
                digitalWrite(yellowLight, HIGH);
                delay(25);
                digitalWrite(yellowLight, LOW);
                delay(25);
                x++;
              }
              if(myFile){
                
                if (timeValid == true){
                  
                  myFile.println(timeStr); 
                }
                else{
                  myFile.println("Time : Unknown");
                }
                if(locValid == false){
                  myFile.print("Latitude : Unknown | ");
                  myFile.println("Longitude : Unknown");
                }
                else{
                  latValue = gps.location.lat();
                  lngValue = gps.location.lng();
                  myFile.print("Latitude : " + String(latValue, 6) + "  | ");
                  myFile.println("Longitude : " + String(lngValue, 6));
                }
                if(altValid == false){
                  myFile.println("Altitude : Unknown");
                }
                else{
                  //            ADD CODE TO GET ALTITUDE VALUE
                  altValue = gps.altitude.meters();
                  myFile.println("Altitude : " + String(altValue, 6));
                }
                myFile.println("Plate Diameter: " + String(displayCounter*0.5, 2) + "cm");
                myFile.println("Weight (N), Potentiometer (mm)");
                myFile.flush();
              }
              endLoop = true;
              Serial.println("Calibrate the linear actuator with the joystick.");
              digitalWrite(greenLight, HIGH);
            } else {
              Serial.println("No GPS fix. Not connected to satellites.");
            }
          }
        }
      }
      //Wait stops code until it finds it
      //Serial.println("Waiting...");
    //Printing time, date, gps coords into new txt file
    
    
    if (buttonState == LOW){
      
      joystickVal = analogRead(VRX_PIN);

      if (joystickVal >= 900) {
        digitalWrite(InPin, LOW);
        digitalWrite(OutPin, HIGH);
      } else if (joystickVal == 0) {
        digitalWrite(OutPin, LOW);
        digitalWrite(InPin, HIGH);
      } else {
        digitalWrite(InPin, LOW);
        digitalWrite(OutPin, LOW);
      }
      buttonState = digitalRead(buttonPin);
    }
    else{
      joystickTime = false;
      //Load cell Taring
      Serial.println("Taring...");
      delay(1000);
      scale.tare();
      Serial.println("Tare Complete!");
      delay(1000);
      // From Pot: initDROPos = analogRead(Potentiometer_pin);
      // delay(1000);
      Serial.println("Beginning test...");
    }
  }
  myFile.close();
  myFile = SD.open(fileName, FILE_WRITE);
}

void loop(){

  if (initPos == false) {
    while (digitalRead(droClock) == LOW) {}  // If clock is LOW wait until it turns to HIGH
    time_now = micros();
    while (digitalRead(droClock) == HIGH) {} // Wait for the end of the HIGH pulse
    if ((micros() - time_now) > 500) {        // If the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
      initDROPos = decode(); //decode the bit sequence
    }
  }

  //Moving linear actuator down continuously
  digitalWrite(InPin, HIGH);
  digitalWrite(OutPin, LOW);

  /* BYE BYE POT
  //Joystick movement code was previously here
  int pot_pos = analogRead(Potentiometer_pin);
  Serial.print("Potentiometer position = ");
  Serial.println(pot_pos - initDROPos);
  */

  //DRO data
  while (digitalRead(droClock) == LOW) {}  // If clock is LOW wait until it turns to HIGH
  time_now = micros();
  while (digitalRead(droClock) == HIGH) {} // Wait for the end of the HIGH pulse
  if ((micros() - time_now) > 500) {        // If the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
    float tempPos = decode(); //decode the bit sequence
  }
  //float tempPos = decode();
  Serial.print("DRO position = ");
  Serial.println(tempPos - initDROPos);

  avgreading = scale.get_units(1);
  Serial.print("Weight reading: ");
  Serial.println(avgreading);

  //save in data
  buttonState = digitalRead(buttonPin);
  
  //myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    //        Weight, Pot. readings
    String dataString = String(String(avgreading*9.81/1000) + "," + String(abs(tempPos-initDROPos)));
    myFile.println(dataString);
    myFile.flush();
  } else { 
    Serial.print("Error");
    myFile.close();
    digitalWrite(InPin, LOW);
    digitalWrite(OutPin, LOW);
    //exit(0);
  }
  
  /*Methods for stopping the program
      1. Weight limit
      2. Button press             */
  if(avgreading/1000*9.81 >= weightLimit){
    digitalWrite(InPin, LOW);
    digitalWrite(OutPin, LOW);
    myFile.print("End (:");
    myFile.close();
    returnToInit();
    
  }
  if(buttonState == HIGH){
    digitalWrite(InPin, LOW);
    digitalWrite(OutPin, LOW);
    myFile.print("End (:");
    myFile.close();
    exit(0);
  }

}

void returnToInit() {
  while (digitalRead(droClock) == LOW) {}  // If clock is LOW wait until it turns to HIGH
  time_now = micros();
  while (digitalRead(droClock) == HIGH) {} // Wait for the end of the HIGH pulse
  if ((micros() - time_now) > 500) {        // If the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
    float tempPos = decode(); //decode the bit sequence
  }
  while(initDROPos > tempPos){
    //Moving linear actuator back up continuously
    digitalWrite(InPin, LOW);
    digitalWrite(OutPin, HIGH);
    //Serial.println(initDROPos + " vs. " + pot_pos);
    
    while (digitalRead(droClock) == LOW) {}  // If clock is LOW wait until it turns to HIGH
    time_now = micros();
    while (digitalRead(droClock) == HIGH) {} // Wait for the end of the HIGH pulse
    if ((micros() - time_now) > 500) {        // If the HIGH pulse was longer than 500 micros we are at the start of a new bit sequence
      float tempPos = decode(); //decode the bit sequence
    }
  }
  digitalWrite(InPin, LOW);
  digitalWrite(OutPin, LOW);
  
  exit(100);
  
}


//Functions for all the number labeling
void zero(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, LOW);
  digitalWrite(pinDP, LOW);
}
void one(){
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
  digitalWrite(pinDP, LOW);
}
void two(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
}
void three(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
}   
void four(){
  digitalWrite(pinA, LOW);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
}
void five(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);  
}   
void six(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
}  
void seven(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
  digitalWrite(pinDP, LOW);
}  
void eight(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, HIGH);
  digitalWrite(pinE, HIGH);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
} 
void nine(){
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
  digitalWrite(pinC, HIGH);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, HIGH);
  digitalWrite(pinG, HIGH);
  digitalWrite(pinDP, LOW);
}
void dot(){
  digitalWrite(pinDP, HIGH);
}

float decode() {
  int sign = 1;
  int i = 0;
  float value = 0.0;
  float result = 0.0;
 
  bit_array[i] = digitalRead(droData);       // Store the 1st bit (start bit) which is always 1.
  while (digitalRead(droClock) == HIGH) {};
 
  for (i = 1; i <= 24; i++) {
    while (digitalRead(droClock) == LOW) { } // Wait until clock returns to HIGH
    bit_array[i] = digitalRead(droData);  
    while (digitalRead(droClock) == HIGH) {} // Wait until clock returns to LOW
  }

  for (i = 1; i <= 20; i++) {                 // Turning the value in the bit array from binary to decimal.
      value = value + (pow(2, i-1) * bit_array[i]);
  }
 
  if (bit_array[21] == 1) sign = -1;          // Bit 21 is the sign bit. 0 -> +, 1 => -
 
  /*
  if (bit_array[24] == 1) {                   // Bit 24 tells the measuring unit (1 -> in, 0 -> mm)
     result = (value*sign) / 2000.00;
     return result;
  } else { */
     result = (value*sign) / 100.00;
     return result;
  //}
}