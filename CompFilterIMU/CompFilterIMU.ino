#include <Wire.h>
#include <math.h>
#define MPU_I2C 0b1101000

//Offsets for the IMU values
const float X_ACCEL_OFFSET = 1236.10;
const float Y_ACCEL_OFFSET = -47.72;
const float Z_ACCEL_OFFSET = -156.68;
const float X_GYRO_OFFSET = -923.22;
const float Y_GYRO_OFFSET = -274.23;
const float Z_GYRO_OFFSET = -234.62;



//Constants which depend on the selected sensitivity of the IMU
const float ACCEL_SENSITIVITY_CONVERSION = 16384.0;
const float GYRO_SENSITIVITY_CONVERSION = 131.0;

//The Complementary filter needs a weight for the gyro data and the accel data to melt them
const float COMPL_FILTER_GYRO_WEIGHT = 0.99;
struct AxisValues {
  float X;
  float Y;
  float Z;
}; 
//The raw data from the IMU
AxisValues accel_Raw;
AxisValues gyro_Raw;

//The values adapted to the current sensivity of the IMU
AxisValues accel;
AxisValues gyro;

//The estimated roll and pitch angles estimated by the accel data
float roll_Angle_Accel;
float pitch_Angle_Accel;
//The estimated roll and pitch angles estimated by the gyro data
float roll_Angle_Gyro;
float pitch_Angle_Gyro;

//To integrate the gyro-data you have to store the last timestempt to form a deltaTime
long oldTime;

//The estimated roll and pitch angel by the Complementary filter
float roll_Angle;
float pitch_Angle;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}

void setupMPU() {
  Wire.beginTransmission(MPU_I2C); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  applyOffsets();
  convertFromRawData();
  estimateAnglesAccel();
  complementaryFilter();
  Serial.print(accel.X);
  Serial.print(",");
  Serial.print(accel.Y);
  Serial.print(",");
  Serial.print(accel.Z);
  Serial.print(",");
  Serial.print(gyro.X);
  Serial.print(",");
  Serial.print(gyro.Y);
  Serial.print(",");
  Serial.print(gyro.Z);
  Serial.print(",");
  Serial.print(roll_Angle_Accel);
  Serial.print(",");
  Serial.print(pitch_Angle_Accel);
  Serial.print(",");
  Serial.print(roll_Angle_Gyro);
  Serial.print(",");
  Serial.print(pitch_Angle_Gyro);
  Serial.print(",");
  Serial.print(roll_Angle);
  Serial.print(",");
  Serial.println(pitch_Angle);

}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accel_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accel_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyro_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyro_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void applyOffsets() {
  //Apply the accelerometer offsets
  accel_Raw.X -= X_ACCEL_OFFSET;
  accel_Raw.Y -= Y_ACCEL_OFFSET;
  accel_Raw.Z -= Z_ACCEL_OFFSET;
  
  //Apply the gyroscope offsets
  gyro_Raw.X -= X_GYRO_OFFSET;
  gyro_Raw.Y -= Y_GYRO_OFFSET;
  gyro_Raw.Z -= Z_GYRO_OFFSET;
}

void convertFromRawData() {
  //Convert raw accel data
  accel.X = accel_Raw.X / ACCEL_SENSITIVITY_CONVERSION ;
  accel.Y = accel_Raw.Y / ACCEL_SENSITIVITY_CONVERSION ; 
  accel.Z = accel_Raw.Z / ACCEL_SENSITIVITY_CONVERSION ;

  //Convert raw gyro data
  gyro.X = gyro_Raw.X / GYRO_SENSITIVITY_CONVERSION;
  gyro.Y = gyro_Raw.Y / GYRO_SENSITIVITY_CONVERSION; 
  gyro.Z = gyro_Raw.Z / GYRO_SENSITIVITY_CONVERSION;
}

void estimateAnglesAccel() {
  roll_Angle_Accel = atan2(accel_Raw.Y , accel_Raw.Z)/(2*M_PI)*360;
  pitch_Angle_Accel = atan2(-accel_Raw.X , sqrt((accel_Raw.Y*accel_Raw.Y)+(accel_Raw.Z*accel_Raw.Z)))/(2*M_PI)*360;
}

void complementaryFilter() {
  long currentTime = millis();
  double dt = (currentTime - oldTime)/1000.0;
  oldTime = currentTime;
  //Calculate the estimated gyro angles
  roll_Angle_Gyro += gyro.X * dt;
  pitch_Angle_Gyro += gyro.Y * dt;

  //Estimated final compl. filter angles
  roll_Angle = COMPL_FILTER_GYRO_WEIGHT * (roll_Angle + gyro.X * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * roll_Angle_Accel;
  pitch_Angle = COMPL_FILTER_GYRO_WEIGHT * (pitch_Angle + gyro.Y * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * pitch_Angle_Accel;

}
