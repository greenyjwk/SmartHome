/* A program to vary the brightness of an LED */
#include <rgb_lcd.h>
#include <Grove_LED_Bar.h>
#include <Servo.h>
#include "Ultrasonic.h"

// const int SOUND = A0; // define the sound sensor pin
#define TEMP_SENSOR (A0)  // Grove - Temperature Sensor connect to A0
#define SOUND (A1)        // define the sound sensor to A1
#define LIGHT (A3)


rgb_lcd lcd;
Ultrasonic ultrasonic(2);
Grove_LED_Bar bar(7, 6, 0, LED_BAR_10);  // Clock pin, Data pin, Orientation
float temp_C;                            // Variable used to store temperature
float temp_F;                            // Variable used to store temperature

// Lab6
// passive mode : 0
// passive mode : 1
bool currentMode = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bar.setLevel(1);
  pinMode(7, INPUT_PULLUP);     // Touch sensor is attached to D7
  pinMode(TEMP_SENSOR, INPUT);  // Temperature: Configure pin A0 as an INPUT
  pinMode(LIGHT, INPUT);        // Light: Configure pin A3 as an INPUT
  Serial.begin(9600);
}

long RangeInInches;
long RangeInCentimeters;  // two measurements should keep an interval
long temperatureArray[10];
long soundArray[10];
long lightArray[10];
long distanceArray[10];
int initStep = 1;
float initStartTime = millis();
int intialCheck = 0;

void loop() {

  if (intialCheck == 0) {
    Serial.println("This is initial step");
    float initDatCollection[4];
    // initial_collect(initStartTime);
    float a, b, c, d;
    float distanceArray[10], temperatureArray[10], lightArray[10], soundArray[10];
    initStartTime = millis();
    int count = 0;
    while(millis() - initStartTime < 5000){
      
      temperatureArray[count] = read_Temperature(TEMP_SENSOR);
      distanceArray[count] = read_Ultrasonic();
      lightArray[count] = read_Light(LIGHT);
      soundArray[count] = read_sound(SOUND);
      
      Serial.print("count is : ");
      Serial.println(count);
  
      Serial.println(distanceArray[count]);
      Serial.println(temperatureArray[count]);
      Serial.println(lightArray[count]);
      Serial.println(soundArray[count]);
  
      delay(500);
      count++;
    }

    float DistanceAvg = average(distanceArray);
    float TemperatureAvg = average(temperatureArray);
    float LightAvg = average(lightArray);
    float SoundAvg = average(soundArray);

    Serial.println("Final Average: ");
    Serial.println(DistanceAvg);
    Serial.println(TemperatureAvg);
    Serial.println(LightAvg);
    Serial.println(SoundAvg);


    // bool rangeBoundCheck = average();
    bool check[4];
    check[0] = distanceArrangeCheck(DistanceAvg);
    check[1] = temperatureArrangeCheck(TemperatureAvg);
    check[2] = lightArrangeCheck(LightAvg);
    check[3] = SoundArrangeCheck(SoundAvg);

  
    if(check[0] & check[1] & check[2] & check[3]){
      Serial.println("Calibration Successful");
      //print in the LED bar to say 'calibration successful'
      
      int colorR = 255;
      int colorG = 255;
      int colorB = 255;

      lcd.begin(16, 2);
      lcd.setRGB(colorR, colorG, colorB);
      // Print a message to the LCD.
      lcd.print("Distance: ");
      lcd.print(DistanceAvg);
      delay(1000);
      lcd.clear();
      

      lcd.print("Temperature: ");
      lcd.print(TemperatureAvg);
      delay(1000);
      lcd.clear();


      lcd.print("Light: ");
      lcd.print(LightAvg);
      delay(1000);
      lcd.clear();


      lcd.print("Sound: ");
      lcd.print(SoundAvg);
      delay(1000);
      lcd.clear();


      lcd.print("Calibration Successful");
      delay(1000);


      intialCheck = 1;
    }else{
      Serial.println("Calibration Failure");
      //print in the LED bar to say 'calibration failure'
    }

  }


  // if( millis() - initStartTime < 5000 ){
  //   read_ultrasonic(2);
  //   delay(1000);
  //   read_Temperature(TEMP_SENSOR);
  //   read_sound(SOUND);
  //   read_Light(LIGHT);
  //   initStep = 0;
  // }


  // Detecting Light
  // int analog_value = analogRead(LIGHT);
  // int mapped_value = map(analog_value, 0, 800, 0, 10);
  // Serial.println("sending command read light");
  // Serial.print("Ambient light: ");
  // Serial.println(mapped_value);
  // delay(2000);





  // /////////////////////////////////////////////////
  //   int startTime = millis();
  //   if(currentMode == 0){ //passive Mode
  //     int startTime = 0;

  //     int touchSensorDetection = digitalRead(7);
  //     while(touchSensorDetection == 1){
  //       Serial.println("Sensor is being pressed");
  //       int currentTime = millis();
  //       int interval = currentTime - startTime;
  //       if(interval > 3000){
  //         Serial.println("It's time to change the mode"); // prints time since program started
  //         // Vibration Motor
  //         digitalWrite(motorPin, HIGH); //vibrate
  //         delay(1500);  // delay one second
  //         digitalWrite(motorPin, LOW);  //stop vibrating
  //         delay(1000); //wait 50 seconds.
  //         currentMode = 1; // Changing to active mode
  //         break;
  //       }
  //     }
  //     currentMode = 1;  // Changing the mode

  //   }else if(currentMode == 1){
  //     int touchSensorDetection = digitalRead(7);
  //     Serial.println(touchSensorDetection);

  //     while(touchSensorDetection == 1){
  //       Serial.println("Sensor is being pressed");
  //       int currentTime = millis();
  //       int interval = currentTime - startTime;
  //       if(interval > 2000){
  //         Serial.println("It's time to change the mode"); // prints time since program started

  //         digitalWrite(motorPin, HIGH); //vibrate
  //         delay(500);  // delay one second

  //         digitalWrite(motorPin, LOW);  //stop vibrating
  //         delay(500); //wait 50 seconds.

  //         digitalWrite(motorPin, HIGH); //vibrate
  //         delay(500);  // delay one second

  //         digitalWrite(motorPin, LOW);  //stop vibrating
  //         delay(500); //wait 50 seconds.

  //         currentMode = 0; // Changing to passive mode
  //         break;
  //       }
  //     }
  //   }
  // /////////////////////////////////////////////////
}


// float initial_collect(int initStartTime){
//   float a, b, c, d;
//   initStartTime = millis();
//   while( millis() - initStartTime < 5000){

//     a = read_Ultrasonic();
//     b = read_Light(LIGHT);
//     c = read_Temperature(TEMP_SENSOR);
//     d = read_sound(SOUND);
//     Serial.println(a);
//     Serial.println(b);
//     Serial.println(c);
//     Serial.println(d);
//     delay(500);
//   }
//   return a;
// }


float average(float dataStream[]){
  float sum = 0 ;
  for(int i = 0; i < sizeof(dataStream); i++) {sum += dataStream[i];}
  float average = sum/(sizeof(dataStream));
  return average;
}


bool distanceArrangeCheck(float DistanceAvg){
  bool arrangeCheck = true;
  if(DistanceAvg >=232 ){
    arrangeCheck = false;
  }
  return arrangeCheck;
}

bool temperatureArrangeCheck(float TemperatureAvg){
  bool arrangeCheck = true;
  if( TemperatureAvg <= -20 || TemperatureAvg >= 60 ){
    arrangeCheck = false;
  }
  return arrangeCheck;
}

bool lightArrangeCheck(float LightAvg){
  bool arrangeCheck = true;
  if(LightAvg <= 1 || LightAvg >=1000 ){
    arrangeCheck = false;
  }
  return arrangeCheck;
}

bool SoundArrangeCheck(float SoundAvg){
  bool arrangeCheck = true;
  if( SoundAvg <= 10 || SoundAvg >=7000 ){
    arrangeCheck = false;
  }
  return arrangeCheck;
}


float read_Ultrasonic(){
  // RangeInInches = ultrasonic.MeasureInInches();
  RangeInCentimeters = ultrasonic.MeasureInCentimeters();  // two measurements should keep an interval
  // Serial.println("Ultrasonic is working: ");
  // Serial.println(RangeInCentimeters);
  return RangeInCentimeters;
}



/*
 * @brief:  Reads an Analog input. 
 *          Converts the analog voltage value into a Temperature value in degrees Celsius. 
 * @param:  pin - Analog Input pin number
 * @ret:    mapped light
 */
float read_Light(int pin) {
  int analog_value = analogRead(pin);
  // Serial.print("Analog Value = ");
  // Serial.print(analog_value);
  int mapped_value = map(analog_value, 0, 1023, 0, 10);
  // Serial.print("  |  Scaled Value = ");
  // Serial.print(mapped_value);
  return analog_value;
}


// read_sound
float read_sound(int pin) {
  int soundValue = 0;                 //create variable to store many different readings
  for (int i = 0; i < 32; i++)        //create a for loop to read
  { soundValue += analogRead(pin); }  //read the sound sensor
  // soundValue >>= 5;                   //bitshift operation
  return soundValue;
}

/*
 * @brief:  Reads an Analog input. 
 *          Converts the analog voltage value into a Temperature value in degrees Celsius. 
 * @param:  pin - Analog Input pin number
 * @ret:    temperature - Temperature value in degrees Celsius (float).
 */
float read_Temperature(int pin) {
  const int B = 4275;       // B value of the thermistor
  const int R0 = 100000;    // R0 = 100k
  int a = analogRead(pin);  // Integer: 0-1023
  float R = 1023.0 / a - 1.0;
  R = R0 * R;
  float temperature = 1.0 / (log(R / R0) / B + 1 / 298.15) - 273.15;  // convert to temperature via datasheet
  return temperature;
}


float read_Temperature_farenheit(int temp_c) {
  float temp_f = 1.8 * temp_c + 32;
  return temp_f;
}
// Jiwoong Kim