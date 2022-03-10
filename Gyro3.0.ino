#include "Wire.h" //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions
// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h" //to install
#include "MPU6050.h" //to install
#include <SD.h>
#include <SPI.h>
File myFile;

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

const int MPU=0x68; //I2C address of the MPU-6050
int pinCS = 53; // Pin 10 on Arduino 
double t,tx,tf,pitch,roll;
char tmp_str[7]; // temporary variable used in convert function

MPU6050 accelgyro(0x68);
int16_t ax, ay, az,gx, gy, gz, tcal, Tmp;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0, tacc, x, y, z;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,time;

// MPU Control/Status
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;           // Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer


char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup()
{
    Wire.begin(); //initiate wire library and I2C
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.
    Serial.begin(115200);
    // initialize device
    accelgyro.initialize();
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(9600); //serial communication at 9600 bauds
    // wait for ready
     while (Serial.available() && Serial.read()); // empty buffer
     while (!Serial.available()){
      Serial.println(F("Send any character to start sketch.\n"));
      delay(150);
    }
    while (Serial.available() && Serial.read()); // empty buffer again
    Serial.println("\nMPU6050 Calibration Sketch");
    delay(200);
    Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
    delay(300);
    // verify connection
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    delay(100);
    // reset offsets
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    pinMode(pinCS, OUTPUT);
    // SD Card Initialization
    if (SD.begin())
    {
      Serial.println("SD card is ready to use.");
    } else
    {
      Serial.println("SD card initialization failed");
      return;
    }
    tacc=200;
}
void loop()
{
  if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(100);
  }
  if (state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(100);
  }
  if (state==2) {
    meansensors();
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU,14,true); //request 14 registers in total  

    //Temperature correction
    tcal = -1600;
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 
    //temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //for deg fahrenheit
    time = millis(); //get time
    //get pitch/roll
    getAngle(mean_ax,mean_ay,mean_az);
    //calculate the positions
    x=(mean_ax * time * time)/2; //get the position x
    y=(mean_ay * time * time)/2; //get the position y
    z=(mean_az * time * time)/2; //get the position z
    //printing values to serial port
    Serial.print("Time = ");  Serial.print(convert_int16_to_str(time)); 
    //prints time since program started
    Serial.print("Position: ");
    Serial.print("X = "); Serial.print(x);
    Serial.print(" Y = "); Serial.print(y);
    Serial.print(" Z = "); Serial.println(z);
    Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" Roll = "); Serial.println(roll);
    Serial.print("Accelerometer: ");
    Serial.print("aX = "); Serial.print(mean_ax);
    Serial.print(" aY = "); Serial.print(mean_ay);
    Serial.print(" aZ = "); Serial.println(mean_az); 
    // doesn't count beacause the positions are already calculated
    Serial.print("Temperature in celsius = "); Serial.print(t);  
    Serial.print(" fahrenheit = "); Serial.println(tf);  
    Serial.print("Gyroscope: ");
    // doesn't count beacause the pitch and the roll are already calculated
    Serial.print("gX = "); Serial.print(mean_gx); //
    Serial.print(" gY = "); Serial.print(mean_gy); //
    Serial.print(" gZ = "); Serial.println(mean_gz); //
    Serial.println();

    //print data on SD card
    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
    Serial.print("Time = ");  Serial.print(convert_int16_to_str(time)); //prints time since program started
    myFile.print(",");  
    Serial.print("Pitch = "); Serial.print(convert_int16_to_str(pitch));
    myFile.print(",");  
    Serial.print(" Roll = "); Serial.print(convert_int16_to_str(roll));
    myFile.print(",");  
    Serial.print(" tmp = "); Serial.print(t);
    myFile.print(",");  
    Serial.print(" X = "); Serial.print(convert_int16_to_str(x));
    myFile.print(",");  
    Serial.print(" Y = "); Serial.print(convert_int16_to_str(y));
    myFile.print(",");    
    Serial.print(" Z = "); Serial.print(convert_int16_to_str(z));
    myFile.close(); // close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
    
    delay(tacc);
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone){
      ready++;
    }
    else {
      ax_offset=ax_offset-mean_ax/acel_deadzone;
    }

    if (abs(mean_ay)<=acel_deadzone) {
      ready++;
    }
    else {
      ay_offset=ay_offset-mean_ay/acel_deadzone;
    }

    if (abs(16384-mean_az)<=acel_deadzone) {
      ready++;
    }
    else {
      az_offset=az_offset+(16384-mean_az)/acel_deadzone;
    }

    if (abs(mean_gx)<=giro_deadzone) {
      ready++;
    }
    else {
      gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
    }

    if (abs(mean_gy)<=giro_deadzone) {
      ready++;
    }
    else {
      gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
    }

    if (abs(mean_gz)<=giro_deadzone) {
      ready++;
    }
    else {
      gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
    }

    if (ready==6) {
      break;
    }
  }
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

//function to convert accelerometer values into pitch and roll
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}
