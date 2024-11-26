//conectivity
#include <esp_now.h>//including libraries
#include <WiFi.h> 


// gyro
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Madgwick filter;

unsigned long lastUpdate = 0; // for timing
float deltaTime = 0.0;

float roll = 0.0, pitch = 0.0, yaw = 0.0; // varibals for rotation 


// servos
#include <ESP32Servo.h> 
Servo LeftAileron;  // all the varibals for the servos to be initilized
int LeftPos = 0; 
int LeftPin = 18;
Servo RightAileron;
int RightPos = 0; 
int RightPin = 19;
Servo Elevator;
int ElevatorPos = 0; 
int ElevatorPin = 5;

// for timing and making sure we have know if we have recived a message
bool HaveWeRecivedMeasge = false;
int lastDoTime = 0;


float scaleValue(float); // declaing a function

struct JoystickData { // a structure so we can store varibals in one place and is how we get data in one message from the controler
  int rightX;
  int rightY;
  int leftX;
};

struct AileronAngles { // another structure just makes it easier to store data and clamp values later 
    float leftAileron;
    float rightAileron;
};


int centerThresholdX = 1850; // these are the offsets since the joystick data might not be perfect 
int centerThresholdY = 1850; 
int deadZone = 50; // gets rid of small movments


float mapJoystickToServo(int value, int centerThreshold) { // this function will make the value we get from the joysticks 0-4095 be mapped to 0-180 instead so we can send it to the servos to move
    if (value > centerThreshold + deadZone) {
        return map(value, centerThreshold + deadZone, 4095, 90, 180); // checking the zone if it above the center value that we declared  
    } else if (value < centerThreshold - deadZone) { 
        return map(value, 0, centerThreshold - deadZone, 0, 90); // and if its smaller 
    } else {
        return 90; // and if we are in between 
    }
}


void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int dataLen) { // this is the function that is called when we recive a message from the other esp32
    HaveWeRecivedMeasge = true; // since we want to take over only when we get a message we will set this flag true so we only use the joystick data 
    lastDoTime = millis(); // part of the timing so that the timeing resets on a new message, this shows up later in loop

    JoystickData receivedData; // initilizing the structure 
    memcpy(&receivedData, data, sizeof(receivedData)); // copying the recived data into the structure to be used

    float elevatorAngle = mapJoystickToServo(receivedData.rightY, centerThresholdY); // this gets the angle needed for the elevator with the offset

    AileronAngles ailerons = {
        mapJoystickToServo(receivedData.rightX, centerThresholdX),       // Left aileron
        mapJoystickToServo(receivedData.rightX, centerThresholdX)         // this is extra since the way the sero is positioned on the wing it needs to go the same direction but originaly it needed to be inverted 
    };

    Elevator.write(elevatorAngle); // these all move the servos to the new angle 
    LeftAileron.write(ailerons.leftAileron);
    RightAileron.write(ailerons.rightAileron);
}


void setup() { // all the setup
  Serial.begin(115200); 
  
  WiFi.mode(WIFI_STA); // making sure both esp32's are on the same wifi type
  if (esp_now_init() != ESP_OK) { // checking if the esp-now is running right 
    Serial.println("Error with ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onReceive); // leting esp-now know what function to call if we recive a message

  // gyro
  if (!mpu.begin()) { // confirms that we have the chip connected 
    Serial.println("No MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // setting up the gyro and all the settings to make it work best
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  filter.begin(50); // this is going to help with jitter 

  LeftAileron.attach(LeftPin, 1000, 2000); // setting up all the servos 
  RightAileron.attach(RightPin, 1000, 2000);
  Elevator.attach(ElevatorPin, 1000, 2000);
}


void loop() {
  if(millis() - lastDoTime >= 100){ // this is to check how long it has been since we recived a message 
    HaveWeRecivedMeasge = false; // if it has been longer then 100ms then we can set the flag back to false so we can have the gyro take over insted of the joysticks 
    lastDoTime  = millis();
  }

  if (!HaveWeRecivedMeasge){ // if we havent recived a message we want the servos to keep the plane level 
    sensors_event_t accel, gyro, temp; // varibals for the data from the gyro to go into
    mpu.getEvent(&accel, &gyro, &temp); // actully get that data 

    unsigned long currentTime = millis(); // this is for filtering and to get rid of jitteryness
    deltaTime = (currentTime - lastUpdate) / 1000.0; 
    lastUpdate = currentTime;

    float gx = gyro.gyro.x; // Convert gyroscope data to radians/sec
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    float ax = accel.acceleration.x;// Convert accelerometer data to g
    float ay = accel.acceleration.y;
    float az = accel.acceleration.z;

    filter.updateIMU(gx, gy, gz, ax, ay, az); //this is where we update the filter will all the information it needs

    pitch = filter.getPitch();  // Get filtered orientation in degrees
    roll = filter.getRoll();

    Elevator.write(scaleValue(pitch));    // Control servos based on pitch and roll
    AileronAngles ailerons = scaleAilerons(roll);
    LeftAileron.write(ailerons.leftAileron);
    RightAileron.write(ailerons.rightAileron);
  }
}

float scaleValue(float x) { // this changes the angle of the plane to 0-180 so that we can put it into the servos, we only are using the angle -25 to 25 if its more it will keep the servo at 0 or 180
    if (x >= 25) {
        return 180; // Clamp to 180
    } else if (x <= -25) {
        return 0; // Clamp to 0
    } else if (x >= 0 && x < 25) {
        // Scale 0 to 25 -> 90 to 180
        return 90 + ((x - 0) * (180 - 90)) / (25 - 0);
    } else { // x < 0 && x > -25
        // Scale -0 to -25 -> 90 to 0
        return 90 + ((x - 0) * (0 - 90)) / (-25 - 0);
    }
}

AileronAngles scaleAilerons(float roll) { // this is the structure where we set the angles to the variables 
    AileronAngles angles;
    if (roll > 25) {
        angles.leftAileron = scaleValue(25);  // Clamp left to max down (25)
        angles.rightAileron = scaleValue(25); // Clamp right to max up (25)
    } else if (roll < -25) {
        angles.leftAileron = scaleValue(-25);  // Clamp left to max up (-25)
        angles.rightAileron = scaleValue(-25); // Clamp right to max down (-25)
    } else {
        // Scale roll values to servo angles
        angles.leftAileron = scaleValue(roll); 
        angles.rightAileron = scaleValue(roll);
    }
    return angles;
}




