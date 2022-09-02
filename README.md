# MPU 6050 - Arduino

## How to use the scripts
- Clone the repository
- Connect the Arduino board you are using to the `MPU6050` as illustrated below
![IMU_Circuit_Diagram_Breadboard](https://user-images.githubusercontent.com/49548379/122672021-b463c080-d1c9-11eb-8a18-a1f5d8c134a5.png)

- Open the `OffsetGenerator.ino` file, put the board in the neutral-zero position and upload the code to your Arduino Board
- Open the Serial Monitor and wait for the calculations to finish. This calculates the Offsets to account for slight tilts of the MPU in the zero position lateron.
- Next copy the output in the Serial Monitor and paste it into the `ComplFilter.ino` file where the offsets are defined
```
//Offsets for the IMU values
const float X_ACCEL_OFFSET = 1236.10;
const float Y_ACCEL_OFFSET = -47.72;
const float Z_ACCEL_OFFSET = -156.68;
const float X_GYRO_OFFSET = -923.22;
const float Y_GYRO_OFFSET = -274.23;
const float Z_GYRO_OFFSET = -234.62;
```
- Upload the code
- Open the Serial Monitor to see the calculated angles. Tilt the board to see them change.

## Steps to calculate the angles
- Sets up the MPU6050 
- Reads raw gyro and accelerometer data 
- Converts the data by taking into account the selected sensitivity
- Calculates the actual angles from gyro-/accel-data seperately 
- Uses complementary filter to fuse both angles

### Note:
The MPU6050 is a 6DOF IMU. Therefore only the Pitch and Roll angles can be calculated. 
