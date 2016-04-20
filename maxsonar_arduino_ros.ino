/*                         maxsonar_arduino_ros                         */
//////////////////////////////////////////////////////////////////////////
//  This script is a modification of the Arduino I2C for a MaxSonar by  //
//  Carl Myhre.                                                         //
//                                                                      //
//  Arduino I2C for a MaxSonar by Carl Myhre is licensed under a        //
//  Creative Commons Attribution-ShareAlike 4.0 International License.  //
//  Original Author:  Carl Myhre, 10-02-2014, Revision: 1.0             //
//  Modifications by:                                                   //
//                                                                      //
//  Revision History: 1.0 -- 10-02-2014 -- Created initial code build   //
//                                                                      //
//  The original I2C libraries were created by Peter Fleury             //
//    http://homepage.hispeed.ch/peterfleury/avr-software.html          //
//                                                                      //
//  These libraries were adapted by Bernhard Nebel for use on Arduino   //
//    https://github.com/felias-fogg/SoftI2CMaster                      //
//                                                                      //
//  Special Thanks to MaxBotix Inc. for sponsoring this project!        //
//    http://www.maxbotix.com -- High Performance Ultrasonic Sensors    //
//                                                                      //
//  For more information on installing the I2C libraries for Arduino    //
//    visit http://playground.arduino.cc/Main/SoftwareI2CLibrary        //
//////////////////////////////////////////////////////////////////////////

//Hints on installing this code:
// 1. You will need to install the <SoftI2CMaster.h> library before using this code.
//      On Windows, the files are placed in C:\Program Files (x86)\Arduino\libraries\SoftI2CMaster\
// 2. As of 10-02-14 the Arduino library page (reference above) has the wrong name for the include file
//      it lists <SoftI2C.h> instead of <SoftI2CMaster.h> -- use the one that matches your installation.
// 3. Make sure to load the library into the Arduino compiler. 
//      To do this go to: SKETCH >> IMPORT LIBRARY... >> ADD LIBRARY...
//      Then navigate to C:\Program Files (x86)\Arduino\libraries\SoftI2CMaster\SoftI2CMaster.h
// 4. Be sure to set the SCL and SDA pins so that they match the pins you are using.

/*
Below, I define the SCL and SDA pins by their ATMEGA pins I have included links to common mappings below.
    UNO:  http://arduino.cc/en/Hacking/PinMapping168
    NANO: (matches UNO but has fewer pins)
    MEGA 2560: http://arduino.cc/en/Hacking/PinMapping2560
The current data matches the setup for the Arduino Uno -- they may need to be changed if the hardware changes.
You can also switch the I2C interface
to any of the tristate pins that you want (not just the SDA or SCL pins).
*/
#define SCL_PIN 5              //Default SCL is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC 
#define SDA_PIN 4              //Default SDS is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100        //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time
#define I2C_FASTMODE 1         //Run in fast mode (400 kHz)

/*
I have included a couple of extra useful settings for easy reference.
//#define I2C_CPUFREQ (F_CPU/8)//Useful if you plan on doing any clock switching
#define I2C_FASTMODE 1         //Run in fast mode (400 kHz)
#define I2C_SLOWMODE 1         //If you do not define the mode it will run at 100kHz with this define set to 1 it will run at 25kHz
*/
#include <SoftI2CMaster.h>     //You will need to install this library
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <ros/time.h>

sensor_msgs::Range sonar_msg;
ros::Publisher pub_sonar("height", &sonar_msg);
ros::NodeHandle nh;

char frameid[] = "/maxsonar";

void setup()
{
  // Initialize both the I2C bus

  i2c_init();
  
  nh.initNode();
  nh.advertise(pub_sonar);
  
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.header.frame_id =  frameid;
  sonar_msg.field_of_view = 1.8; //based on datasheet for MB1222
  sonar_msg.min_range = 0.2;
  sonar_msg.max_range = 7.65;
}

void loop()
{
  read_the_sensor();
}

///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//	had an error, 0 = the function was successful
boolean start_sensor(byte bit8address)
{
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}

///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address)
{
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if(errorlevel)
  {
    return 0;
  }
  else
  {
    return range;
  }
}

//////////////////////////////////////////////////////////
// Code Example: Read the sensor at the default address //
//////////////////////////////////////////////////////////
void read_the_sensor()
{
  boolean error = 0;  //Create a bit to check for catch errors as needed.
  float range;
  
  //Take a range reading at the default address of 224
  error = start_sensor(224);    //Start the sensor and collect any error codes.
  if (!error)
  {                  //If you had an error starting the sensor there is little point in reading it as you will get old data.
    delay(100);
    range = read_sensor(224)/100.00;   //reading the sensor will return an integer value -- if this value is 0 there was an error
    //Serial.print("R:");Serial.println(range);
    sonar_msg.header.stamp = nh.now();
    sonar_msg.range = range; //Dividing by 100 to convert cm to m
    pub_sonar.publish(&sonar_msg);
  }
  nh.spinOnce();
}
