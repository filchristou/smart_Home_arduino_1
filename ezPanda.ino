//declaring variables
int ledsState[7];
/*
 * 0 --> Led Group Up
 * 1 --> Led Group Down
 * 2 --> Led Red
 * 3 --> Led Green
 * 4 --> RGB Led RED 
 * 5 --> RGB Led GREEN 
 * 6 --> RGB Led BLUE 
 */
float measurements[4];
/*
 * 0 --> temperature
 * 1 --> moisture
 * 2 --> Luminocity
 * 3 --> Distance
 */
boolean doorState;

boolean streamWatts = 0;
boolean streamDistance = 0;
//---------------------------temperature sensor---------------------------//
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 7 on the Arduino
#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature tempSensors(&oneWire);

float temperature()
{
   // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  tempSensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.print("DONE. ");

  measurements[0] = tempSensors.getTempCByIndex(0);
  //Serial.print("Temperature :");
  //Serial.print(measurements[0]); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  //Serial.println("C");

  return measurements[0];
}

//---------------------------moisture sensor---------------------------//
#include <DHT.h>

#define DHTPIN A5 //analog pin for the moisture sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

float moisture()
{
  measurements[1] = dht.readHumidity(); 

  //Serial.print("Current humdity = ");
  //Serial.print(measurements[1]); 
  //Serial.println('%');

  return measurements[1];
}

//---------------------------luminocity sensor---------------------------//
const int luminPin = A4;

float luminocity() {
  int value = analogRead(luminPin); 
  measurements[2] = (1-(float)(value/1023.0)) * 100;
  //Serial.print("luminocity : ");
  //Serial.print(measurements[2], 2); // light intensity
                // high values for bright environment
                // low values for dark environment
  //Serial.println("%");

  return measurements[2];
}

//---------------------------magnetic sensor---------------------------//
const int magnitoPin = 8; // Button

boolean magnito()
{ 
  int count = 0;
  for (int i=0 ; i<10; i++)
  {
    int magnSwitch = digitalRead (magnitoPin);
    
    if(magnSwitch==1)
    {
        count++;
    }
  }
  if (count > 1)
  {
    doorState = true;
    return doorState;
  }
  else
  {
    doorState = false;
    return doorState;
  }
}

//---------------------------distance sensor---------------------------//
const int echoPin = 12;
const int trigPin = 13;

float distance1()  //returns distance in cm
{
  for (int i=0 ; i<25; i++)
  {
    //make a positive pulse
    digitalWrite(trigPin, LOW); //give a short LOW pulse beforehand to ensure a clean HIGH pulse
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    //read the echo signal from the sensor : a HIGH pulse whose
    // duration is the time (in microseconds) from the sending of the trigger
    // to the reception of it's echo off of an object
    long duration = pulseIn(echoPin, HIGH);
  
    //convert time to distance (in cm)
    // 2*range(m) = duration(sec) * 343(m/s) wave sound speed
    // => range(cm) = (duration/2) * 0.0343
    measurements[3] += (float) duration / 2 * 0.0343;
  }
  measurements[3] = measurements[3]/25;
  
  return measurements[3];
}


//---------------------------relay---------------------------//
const int relayPinU = 3; //up circuit
const int relayPinD = 4; //down circuit
//---------------------------simple leds---------------------------//
const int redLedPin = 5;
const int blueLedPin = 6;

void write2Leds(int device, int state)
{
  ledsState[device] = state;
  switch (device)
  {
    case 0:
      digitalWrite(relayPinU, 1-state); //relay is working in the opposite way
      break;
    case 1:
      digitalWrite(relayPinD, 1-state);
      break;
    case 2:
      digitalWrite(redLedPin, state);
      break;
    case 3:
      digitalWrite(blueLedPin, state);
      break;
  }
}


//---------------------------RGB led---------------------------//
const int RGBPin_Red = 11; // select the pin for the red LED
const int RGBPin_Green = 10; // select the pin for the green LED
const int RGBPin_Blue = 9; // select the pin for the  blue LED


void rgbLed(int r, int g, int b) {
  analogWrite(RGBPin_Red, r);
  ledsState[4] = r;
  analogWrite(RGBPin_Blue, g);
  ledsState[5] = g;
  analogWrite(RGBPin_Green, b);
  ledsState[6] = b;
}

//---------------------------Wattometer---------------------------//
const int voltagePinU = A1;  //analog
const int currentPinU = A2;

const int voltagePinD = A0;  //analog
const int currentPinD = A3;

const float R1 = 30000.0;
const float R2 = 7500.0;

void sendWatts()
{
  float currentInU = 0;
  int curReadU = 0;
  float currentInD = 0;
  int curReadD = 0;
  float voltageInU = 0; 
  float voltageInD = 0; 
  //basically a voltage divider :
  // *sensorValue = analogRead(voltagePin);
  // *Vout = (sensorValue * 5.0) / 1024.0 ;
  // *voltageIn = Vout / (R2 / R1 + R2); 
  //voltage
  voltageInU = ((analogRead(voltagePinU) * 5.0)/ 1024.0) / (R2/(R1+R2));
  voltageInD = ((analogRead(voltagePinD) * 5.0)/ 1024.0) / (R2/(R1+R2));
  //current from Up circuit
  curReadU = analogRead(currentPinU);
  currentInU = (curReadU * 5.0 )/ 1024.0; // scale the ADC  
  //current from Down circuit
  curReadD = analogRead(currentPinD);
  currentInD = (curReadD * 5.0 )/ 1024.0; // scale the ADC  

  
  String wattReport = "W:" + String(voltageInU*currentInU) + ',' + String(voltageInD*currentInD) + '\n' + "$!\n"; //Dollar means stream
  Serial.print(wattReport);
  
}

String getTotalReport()
{
  String send_report = "";
  send_report = send_report + '0' + ':' + String(ledsState[0]) + '\n';
  send_report = send_report + '1' + ':' + String(ledsState[1]) + '\n';
  send_report = send_report + '2' + ':' + String(ledsState[2]) + '\n';
  send_report = send_report + '3' + ':' + String(ledsState[3]) + '\n';
  send_report = send_report + '4' + ':' + String(ledsState[4]) + ',' + String(ledsState[5]) + ',' + String(ledsState[6]) + '\n';
  send_report = send_report + '5' + ':' + String(temperature()) + '\n';
  send_report = send_report + '6' + ':' + String(moisture()) + '\n';
  send_report = send_report + '7' + ':' + String(luminocity()) + '\n';
  send_report = send_report + '8' + ':' + String(distance1()) + '\n';

  return send_report;
}

unsigned long current_time_dist;
unsigned long current_time_watt;
unsigned long current_time_report;
void setup() {
  Serial.begin(115200);
  pinMode(relayPinU, OUTPUT);
  pinMode(relayPinD, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  digitalWrite(redLedPin, 0);
  digitalWrite(blueLedPin, 0);
  digitalWrite(relayPinU, 1);
  digitalWrite(relayPinD, 1);

  pinMode(voltagePinU, INPUT);
  pinMode(voltagePinD, INPUT);
  pinMode(currentPinD, INPUT);
  pinMode(currentPinU, INPUT);
  pinMode(luminPin, INPUT);
  
  pinMode(magnitoPin, INPUT); //set magnetic sensor pin as input

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(echoPin, HIGH); //use a pull-up resistor

  pinMode(RGBPin_Red, OUTPUT);
  pinMode(RGBPin_Blue, OUTPUT);
  pinMode(RGBPin_Green, OUTPUT);
  digitalWrite(RGBPin_Red, 0);
  digitalWrite(RGBPin_Blue, 0);
  digitalWrite(RGBPin_Green, 0);

  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  digitalWrite(redLedPin, 0);
  digitalWrite(blueLedPin, 0);

  //distance sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  int ledsState[7];
  for (int i=0 ; i<7 ; i++)
    ledsState[i] = 0;
    
  for (int i=0 ; i<4 ; i++)
    measurements[i] = 0;
   
  doorState = false;

  current_time_dist = millis();
  current_time_watt = millis();
  current_time_report = millis();
  
  tempSensors.begin();
}

byte inChar;
String request_buf = "";
String send_buf = "";

void loop() {
   // check if data has been sent from the computer:
  while (Serial.available())
  {
    // read the most recent byte
    inChar = Serial.read();
    //Serial.print("diabasa_");
    //Serial.print((char)inChar);
    //Serial.print("_");
    if (inChar == '#') //hurry up. real time connection is on
    {
      while(true)
      {
        if (Serial.available()) //end of packet
        {
          inChar = Serial.read();
          if (inChar == '\n')
            break;
        }
      }
      //zero receive buffer
      request_buf = "";
      //finalize response
      //Serial.print("before:"+send_buf);
      send_buf = send_buf + "#!\n";
      //begin transimitting
      Serial.print(send_buf);
      //clear the output buffer
      send_buf ="";
    }else if (inChar == '!') //end of message
    {
      while(true)
      {
        if (Serial.available()) //end of packet
        {
          inChar = Serial.read();
          if (inChar == '\n')
            break;
        }
      }
      //zero receive buffer
      request_buf = "";
      //finalize response
      send_buf = send_buf + "!\n";
      //begin transimitting
      Serial.print(send_buf);
      //clear the output buffer
      send_buf ="";
    }else if (inChar == '\n') //end of sentence
    {
      //Serial.print(".n.");
      /*
      Serial.println("-----------------");
      Serial.print("requested : \n");
      Serial.println(request_buf);
      Serial.println("-----------------");
      */
      //handle the request
      int device = (request_buf.charAt(0)- '0');
      switch (request_buf.charAt(0))
      {
        case '0':
        case '1':
        case '2':
        case '3':
          if (request_buf.charAt(2) != '?')
          {
            write2Leds(device, request_buf.charAt(2) - '0');
          }
          send_buf = send_buf + device + ':' + String(ledsState[device]) + '\n';
          break;
        case '4':
           if (request_buf.charAt(2) != '?')
          {   //e.g. 4:250,155,205
            String rgb = request_buf.substring(2);
            int first_comma = rgb.indexOf(',');
            int second_comma = rgb.lastIndexOf(',');

            rgbLed( rgb.substring(0, first_comma).toInt(),
              rgb.substring(first_comma+1, second_comma).toInt(),
              rgb.substring(second_comma+1, rgb.length()).toInt() );
          }
          send_buf = send_buf + device + ':' + String(ledsState[4]) + ',' + String(ledsState[5]) + ',' + String(ledsState[6]) + '\n';
          break;
        case '5':
          send_buf = send_buf + device + ':' + String(temperature()) + '\n';
          break;
        case '6':
          send_buf = send_buf + device + ':' + String(moisture()) + '\n';
          break;
        case '7':
          send_buf = send_buf + device + ':' + String(luminocity()) + '\n';
          break;
        case '8':
          if (request_buf.charAt(2) == 'E')
          {
            streamDistance = true;
            send_buf = send_buf + "8:E" + '\n';
          }
          else if (request_buf.charAt(2) == 'D')
          {
            //Serial.print("###|send_buf:"+send_buf+"\n...ended...\n");
            streamDistance = false;
            send_buf = send_buf + "8:D" + '\n';
          }
          else
            send_buf = send_buf + device + ':' + String(distance1()) + '\n';
          break;
        case '9':
          send_buf = send_buf + device + ':' + String(magnito()) + '\n';
          break;
        case 'W':
          if (request_buf.charAt(2) == 'E')
          {
            streamWatts = true;
            send_buf = send_buf + "W:E" + '\n';
          }
          else if (request_buf.charAt(2) == 'D')
          {
            streamWatts = false;
            send_buf = send_buf + "W:D" + '\n';
          }
        //default:
          
      }
      //clear the request, in order to handle the next one
      request_buf = "";
    }else if ((inChar == '*')) //send full report
    {
      send_buf = send_buf + getTotalReport();
    }else
    {
      request_buf = request_buf + char(inChar);
    }
  }

  
  //check if 30 minute passed,so that I give my report.
  
  
  if (millis() - current_time_report > 1800000)
  {
    String send_report = getTotalReport();
    
    //finalize response
    send_report = send_report + "!\n";
    //begin transimitting
    Serial.print(send_report);
    
    current_time_report = millis();
  }
  

  //check if stream Distance is on and act appropriately
  if (streamDistance && (millis()-current_time_dist > 1000) )
  {
    String distStream = "";
    distStream = distStream + '8' + ':' + String(distance1()) + '\n' + "$!\n"; //Dollar means stream
    Serial.print(distStream);
    current_time_dist = millis();
  }

  //check if stream Watts is on and act appropriately
  if (streamWatts && (millis()-current_time_watt > 1000))
  {
    sendWatts();
    current_time_watt = millis();
  }

  
  //------------------------TESTING------------------------//
  /*
  digitalWrite(relayPinU, LOW);
  digitalWrite(relayPinD, HIGH);
  measureWatt(5000);
  
  digitalWrite(relayPinU, HIGH);
  digitalWrite(relayPinD, LOW);
  measureWatt(5000);

  moisture();
  temperature();
  luminocity();

  magnito(); //doorState gets value inside the function
  Serial.print("Door state : ");
  Serial.println(doorState);
    
  
  distance1(); //measurements[3] gets value inside the function
  Serial.print("Distance to Car : ");
  Serial.println(measurements[3]);

  rgbLed(200,75,150);
  */
}

