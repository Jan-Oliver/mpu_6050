# MPU 6050 - Arduino

## Usage
- Download the code and open the .ino file
- Connect the Arduino board you are using to the MPU6050
- Upload the code

## What it does
- Sets up the MPU6050 
- Reads raw gyro and accelerometer data 
- Converts the data by taking into account the selected sensitivity
- Calculates the actual angles from gyro-/accel-data seperately 
- Uses complementary filter to fuse both angles

### Note:
The MPU6050 is a 6DOF IMU. Therefore only the Pitch and Roll angles can be calculated. 
