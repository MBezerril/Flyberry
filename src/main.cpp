///////////////////////////////////////////////////////////////////////////////////////
/*Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


///////////////////////////////////////////////////////////////////////////////////////
//Support
///////////////////////////////////////////////////////////////////////////////////////
Website: http://www.brokking.net/imu.html
Youtube: https://youtu.be/4BoIE8YQwM8
Version: 1.0 (May 5, 2016)

///////////////////////////////////////////////////////////////////////////////////////
//Connections
///////////////////////////////////////////////////////////////////////////////////////
Power (5V) is provided to the Arduino pro mini by the FTDI programmer

Gyro - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5

LCD  - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5
*/
/////////////////////////////////////////////////////////////////////////////////////

//Include LCD and I2C
#include "Madgwick.h"
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

//Declaring some global variables
s16 gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mx, my, mz;
long acc_total_vector;
s16 temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
s16 angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
Madgwick mFilter;
Servo ServoDireita, ServoEsquerda;

float A, B, C, current_state_estimate, current_prob_estimate, Q, R;

void setup_mpu_6050_registers();
void read_mpu_6050_data();
void setup_magnetometer_registers();
void read_Magnetometer_data();

void setup()
{
    Wire.begin(); //Start I2C as master
    Serial.begin(115200); //Use only for debugging
    pinMode(13, OUTPUT); //Set output 13 (LED) as output
    ServoDireita.attach(D3); //servo direta
    ServoEsquerda.attach(D5); //servo esquerda
    ServoDireita.write(80);
    ServoEsquerda.write(80);

    setup_mpu_6050_registers(); //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
    setup_magnetometer_registers();

    digitalWrite(13, HIGH); //Set digital output 13 high to indicate startup

    for (int cal_int = 0; cal_int < 2000; cal_int++) { //Run this code 2000 times
        if (cal_int % 125 == 0)
            Serial.print("."); //Print a dot on the LCD every 125 readings
        read_mpu_6050_data(); //Read the raw acc and gyro data from the MPU-6050
        gyro_x_cal += gyro_x; //Add the gyro x-axis offset to the gyro_x_cal variable
        gyro_y_cal += gyro_y; //Add the gyro y-axis offset to the gyro_y_cal variable
        gyro_z_cal += gyro_z; //Add the gyro z-axis offset to the gyro_z_cal variable
        delay(4); //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= 2000; //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 2000; //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 2000; //Divide the gyro_z_cal variable by 2000 to get the avarage offset

    digitalWrite(13, LOW); //All done, turn the LED off
    mFilter.begin(250.0f);
    loop_timer = micros(); //Reset the loop timer
}

void loop()
{
    read_mpu_6050_data(); //Read the raw acc and gyro data from the MPU-6050
    read_Magnetometer_data();
    mFilter.update(gyro_x/65.5, gyro_y/65.5, gyro_z/65.5, acc_x, acc_y, acc_z, mx, my, mz);
    Serial.print(mFilter.getPitch());
    Serial.print(" | ");
    Serial.print(mFilter.getRoll());
    Serial.print(" | ");
    Serial.println(mFilter.getYaw());
    while (micros() - loop_timer < 4000)
        ; //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    loop_timer = micros(); //Reset the loop timer
}

void read_mpu_6050_data()
{ //Subroutine for reading the raw gyro and accelerometer data
    Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
    Wire.write(0x3B); //Send the requested starting register
    Wire.endTransmission(); //End the transmission
    Wire.requestFrom(0x68, 14); //Request 14 bytes from the MPU-6050
    while (Wire.available() < 14)
        ; //Wait until all the bytes are received
    acc_x = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_x variable
    acc_y = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_y variable
    acc_z = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the acc_z variable
    temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
    gyro_x = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_x variable
    gyro_y = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_y variable
    gyro_z = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers()
{
    //Activate the MPU-6050
    Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
    Wire.write(0x6B); //Send the requested starting register
    Wire.write(0x00); //Set the requested starting register
    Wire.endTransmission(); //End the transmission
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
    Wire.write(0x1C); //Send the requested starting register
    Wire.write(0x10); //Set the requested starting register
    Wire.endTransmission(); //End the transmission
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
    Wire.write(0x1B); //Send the requested starting register
    Wire.write(0x08); //Set the requested starting register
    Wire.endTransmission(); //End the transmission
}

void read_Magnetometer_data()
{
    Wire.beginTransmission(0x1E);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(0x1E, 6);
    if (6 <= Wire.available()) {
        mx = Wire.read() << 8;
        mx |= Wire.read();
        mz = Wire.read() << 8;
        mz |= Wire.read();
        my = Wire.read() << 8;
        my |= Wire.read();
    }
}

void setup_magnetometer_registers()
{
    Wire.beginTransmission(0x1E); //start talking
    Wire.write(0x02); // Set the Register
    Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
    Wire.endTransmission();
}
