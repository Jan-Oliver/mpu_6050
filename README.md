# MPU 6050 - Arduino

## Usage
- Download the code
- Connect the Arduino board you are using to the MPU6050
![IMU_Circuit_Diagram_Breadboard](https://user-images.githubusercontent.com/49548379/122672021-b463c080-d1c9-11eb-8a18-a1f5d8c134a5.png)

- Open the OffsetGenerator .ino file and upload the code
- Open the Serial Monitor and wait for the calculations to finish
- Copy the output and paste it into the ComplFilter .ino file where the offsets are defined
- Upload the code
- Open the Serial Monitor to see the calculated angles

## What it does
- Sets up the MPU6050 
- Reads raw gyro and accelerometer data 
- Converts the data by taking into account the selected sensitivity
- Calculates the actual angles from gyro-/accel-data seperately 
- Uses complementary filter to fuse both angles

### Note:
The MPU6050 is a 6DOF IMU. Therefore only the Pitch and Roll angles can be calculated. 
