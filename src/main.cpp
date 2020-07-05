#include <Arduino.h>
#include <../lib/MPU6050_handle/MPU5060_handle.h>

// Create an object for the MPU6050
MPU5060_handle mpu;

// Define the motor pins on the Arduino
#define pinM1 5
#define pinM2 9
#define pinDir1M1 6
#define pinDir2M1 10
#define pinDir1M2 7
#define pinDir2M2 8

// Define LED pins
#define pinLED 2
#define pinStatusLED 13

unsigned long looptime = 10000;//2500;  // Looptime in microseconds
float dt = 0.000001*looptime;   // loop time in seconds
unsigned long loopTimer;

float roll_acc  = 0;
float pitch_acc = 0;
float tot_roll  = 0;
float tot_pitch = 0;

// Setup sensor variables
float gForceY, gForceZ, gForceX;     // Accelerometer sensor values
float rotX;                          // Gyroscope sensor values
float calibX;                        // Gyroscope calibration values

// PID-calibration
unsigned long Kp = 2;

/****************** NEEDED METHODS ******************/

void my_digitalWrite(uint8_t pin, uint8_t val){
  uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  out = portOutputRegister(port);

    uint8_t oldSREG = SREG;
    cli();

    if (val == LOW) {
      *out &= ~bit;
    } else {
      *out |= bit;
    }

    SREG = oldSREG;

}

boolean set_motor_speeds(signed long speed){
  /*
  * Method for writing speed to the motors. Sets the same amount of speed to the both motors.
  * Param: speed - Signed integer that corresponds to the wanted speed of the motor. Range of -100 - 100.
  * Return: Boolean - True if the method executed successfully. False if something failed.
  */
  
  // Check if the speed is whithin the allowed range.
  if (speed < -100 || speed > 100){
    Serial.print("Speed out of range: "); Serial.println(speed);
    return false;
  }

  // Set the direction of the motors
  if (speed >= 0){
      // Forward
      my_digitalWrite(pinDir1M1, HIGH);
      my_digitalWrite(pinDir2M1, LOW);
      my_digitalWrite(pinDir1M2, LOW);
      my_digitalWrite(pinDir2M2, HIGH);
  }
  else{
      // Backward 
      my_digitalWrite(pinDir1M1, LOW);
      my_digitalWrite(pinDir2M1, HIGH);
      my_digitalWrite(pinDir1M2, HIGH);
      my_digitalWrite(pinDir2M2, LOW);
  }

  // Map a value between 0 and 100 to a range accepted by the motors.
  long speed_map = map(abs(speed), 0, 100, 89, 255);

  // Write the speed to the motors
  analogWrite(pinM1, speed_map);
  analogWrite(pinM2, speed_map);

  return true;
}

void complementaryFilter(){
  // Calculate the angles according to the accelerometer
  // 180/pi ≈ 57.29

  gForceY = mpu.gForceY;
  gForceZ = mpu.gForceZ;
  rotX = mpu.rotX;
  calibX = mpu.calibX;

  roll_acc = atan(gForceY/gForceZ)*(57.29);

  // Complementary filter
  tot_roll = 0.97*(tot_roll + ((rotX - calibX)*dt)) + 0.03*(roll_acc);
} 

void PIDController(int ref){

  signed long error = tot_roll - ref;

  signed long motorSpeed = error*Kp;

  set_motor_speeds(motorSpeed);
}

/******************* MAIN METHODS *******************/

void setup() {
  // Start serial communication
  Serial.begin(9600);
  
  // Direction pins
  pinMode(pinDir1M1, OUTPUT);
  pinMode(pinDir2M1, OUTPUT);
  pinMode(pinDir1M2, OUTPUT);
  pinMode(pinDir2M2, OUTPUT);

  // Motor ctrl signal pins
  pinMode(pinM1, OUTPUT);
  pinMode(pinM2, OUTPUT);

  // Set up MPU6050
  if (mpu.SetupMPU()){
    mpu.calibrateGyro();
  }
  else{
    Serial.println("MPU6050 failed. Shutting down program.");
    while(true){
      delay(1000);
    }
  }
}

void loop() {
  // Todo:  * Kolla vad som tar lång tid genom micros. Kolla vilken metod det är och vad man kan göra för att fixa det.
  //        * Bryta ut comp-filter till eget lib?
  //        * Implementera PID
  //        * Testa om jag kan byta ut vissa variabeltyper till andra.
  loopTimer = micros();

  // Get new sensor data
  mpu.MPUReadGyro();
  mpu.MPUReadAccel();
  
  // Run complementary filter
  complementaryFilter();
  
  PIDController(0);

  if(micros() - loopTimer > looptime){
    Serial.print("WARNING!! TOO LONG MAIN LOOP!: "); Serial.println(micros() - loopTimer);
  }

  while(micros() - loopTimer < looptime);
  
}