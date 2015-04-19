#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <math.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float start_alt = 0;
float target_alt = 1650;
float predicted_alt = 0;

int motor_delay = 5000;  // Delat in ms
int state = 0;
int solenoid = 13;

// State variables
float current_alt = 0;
float last_alt = 0;
float velocity = 0;
unsigned long time = 0;
unsigned long last_time = 0;

// Kalman values, all that we care about is altitude, on state variable is velocity
float X;
float u = 0;
bool brake = false;

void conn_sensors() {
  if(!accel.begin()) {
    Serial.println(F("No accelerometer found"));
    while(1);
  }
  if(!mag.begin()) {
    Serial.println(F("No magnetometer found"));
    while(1);
  }
  if(!bmp.begin()) {
    Serial.println(F("No pressure sensor found"));
    while(1);
  }
}

void setup() {
  pinMode(solenoid, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Connecting sensors"));
  conn_sensors();
}

void loop() {
  if(state == 0){
    // Get the starting altitude
    sensors_event_t bmp_event;
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      /* Get ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude    */
      start_alt = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature); 
      /* Display the temperature */
      Serial.print(F("Temp: "));
      Serial.print(temperature);
      Serial.println(F(" C"));
    }
    
    // When we have a start altitude signal ready
    if(start_alt != 0) {
      // Signal prepared for launch
      digitalWrite(solenoid, HIGH);
      delay(1000);
      digitalWrite(solenoid, LOW);
      delay(1000);
      digitalWrite(solenoid, HIGH);
      delay(1000);
      digitalWrite(solenoid, LOW);
      state = 1;
    }
  } else if(state == 1){
    // Wait for acceleration > 1.5G before advancing to next state
    // The sensor returns this value in m/s so 15 is about 1.5*(9.8m/s)
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_event_t bmp_event;
    sensors_vec_t   orientation;
    
    accel.getEvent(&accel_event);
    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
    {
      if(accel_event.acceleration.y > 15.0) {
        state = 2;
        Serial.println("Entering flight mode");
        Serial.print("Start altitude set ");
        Serial.println(start_alt);
        delay(motor_delay);
        Serial.println("Entering control mode");
      }
    }
  } else if(state == 2){  // Flight control goes here
    sensors_event_t bmp_event;
    bmp.getEvent(&bmp_event);
    
    if(bmp_event.pressure) {
      float temperature;
      bmp.getTemperature(&temperature);
      current_alt = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
      time = millis();
      
      // Kalman filter
      if(last_alt != 0 && last_time != 0) {
        velocity = (current_alt - last_alt) * ( (float)(time - last_time) / 1000 );
        Serial.print("Velocity: ");
        Serial.println(velocity);
        
        predicted_alt = current_alt + (pow(velocity, 2.0) / 4.9) - (pow(velocity, 4.0) / 4.9);
        Serial.print(predicted_alt);
        
        if(predicted_alt > (target_alt + start_alt)){
          brake = true;
        } else {
          brake = false;
        }
      }
      last_alt = current_alt;
      last_time = time;
    
      // Check if we have passed the target altitude
      if(current_alt > target_alt) {
        // We have reached the target altitude, brake for 2s then transition state
        Serial.print("fligh completed at ");
        Serial.println(current_alt);
        digitalWrite(solenoid, HIGH);
        delay(2000);
        digitalWrite(solenoid, LOW);
        state = 3;
      }
    }
    
    if (brake) {
      digitalWrite(solenoid, HIGH);
    } else {
      digitalWrite(solenoid, LOW);
    }
    
    
    
    delay(50);
  } else if(state == 3){
    // Close the flaps for landing
    digitalWrite(solenoid, LOW);
    while(1);
  }
}
