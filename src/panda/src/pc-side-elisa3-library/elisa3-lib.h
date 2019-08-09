#include <stdio.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>
#include "usb-comm.h"

#ifndef ELISA3_LIB_H_
#define ELISA3_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief To be called once at the beginning, it init the USB communication with the RF module that is responsible to send data to the robots and initialize the list of robots to be controlled; max number of simultaneous robots is 100.
 * \param robotAddr array list of robot addresses to be handled.
 * \param numRobots the array size (number of robots to handle).
 * \return none
 */
void startCommunication(int *robotAddr, int numRobots);

/**
 * \brief To be called once at the end, it closes the USB communication.
 * \return none
 */
void stopCommunication();

/**
 * \brief Set the left speed of the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \param value The speed, range is between -128 to 127.
 * \return none
 */
void setLeftSpeed(int robotAddr, char value);

/**
 * \brief Set the right speed of the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \param value The speed, range is between -128 to 127.
 * \return none
 */
void setRightSpeed(int robotAddr, char value);

/**
 * \brief Set the left speed of all the robots specified in the list.
 * \param value The speed array, range is between -128 to 127.
 * \return none
 */
void setLeftSpeedForAll(char *value);

/**
 * \brief Set the right speed of all the robots specified in the list.
 * \param value The speed array, range is between -128 to 127.
 * \return none
 */
void setRightSpeedForAll(char *value);

/**
 * \brief Set the red intensity of the RGB led on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \param value The intensity, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setRed(int robotAddr, unsigned char value);

/**
 * \brief Set the green intensity of the RGB led on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \param value The intensity, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setGreen(int robotAddr, unsigned char value);

/**
 * \brief Set the blue intensity of the RGB led on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \param value The intensity, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setBlue(int robotAddr, unsigned char value);

/**
 * \brief Set the red intensity of the RGB led of all the robots specified in the list.
 * \param value The intensity array, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setRedForAll(unsigned char *value);

/**
 * \brief Set the green intensity of the RGB led of all the robots specified in the list.
 * \param value The intensity array, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setGreenForAll(unsigned char *value);

/**
 * \brief Set the blue intensity of the RGB led of all all the robots specified in the list.
 * \param value The intensity array, range is between 0 (led off) to 100 (max power).
 * \return none
 */
void setBlueForAll(unsigned char *value);

/**
 * \brief Turn on both the front IRs transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOnFrontIRs(int robotAddr);

/**
 * \brief Turn off both the front IRs transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOffFrontIRs(int robotAddr);

/**
 * \brief Turn on the back IR transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOnBackIR(int robotAddr);

/**
 * \brief Turn off the back IR transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOffBackIR(int robotAddr);

/**
 * \brief Turn on all the 3 IRs transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOnAllIRs(int robotAddr);

/**
 * \brief Turn off all the 3 IRs transmitter on the robot.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void turnOffAllIRs(int robotAddr);

/**
 * \brief Enable the reception of commands from a TV remote.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void enableTVRemote(int robotAddr);

/**
 * \brief Disable the reception of commands from a TV remote.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void disableTVRemote(int robotAddr);

/**
 * \brief Put the robot in sleep mode (energy saving mode).
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void enableSleep(int robotAddr);

/**
 * \brief Put the robot in normal mode (out of sleep).
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void disableSleep(int robotAddr);

/**
 * \brief Calibrate the sensors of the robot (proximity, accelerometer).
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void calibrateSensors(int robotAddr);

/**
 * \brief Calibrate the sensors (proximity, accelerometer) of all the robots specified in the list.
 * \return none
 */
void calibrateSensorsForAll();

/**
 * \brief Enable the onboard obstacle avoidance.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void enableObstacleAvoidance(int robotAddr);

/**
 * \brief Disable the onboard obstacle avoidance.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void disableObstacleAvoidance(int robotAddr);

/**
 * \brief Enable the onboard cliff avoidance.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void enableCliffAvoidance(int robotAddr);

/**
 * \brief Disable the onboard cliff avoidance.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none
 */
void disableCliffAvoidance(int robotAddr);

/**
 * \brief Set the address of a robot related to a particular index, the index must be in the range of the robot list size.
 * \param robotIndex from 0 to number of robots - 1 (set with "setRobotAddresses" function)
 * \param robotAddr the address of the robot
 * \return none
 */
void setRobotAddress(int robotIndex, int robotAddr);

/**
 * \brief Set the addresses of the robots that need to be controlled; max number of simultaneous robots is 100.
 * \param robotAddr array list of robot addresses to be handled.
 * \param numRobots the array size (number of robots to handle).
 * \return none
 */
void setRobotAddresses(int *robotAddr, int numRobots);

/**
 * \brief Request one of the proximity sensors value of the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param proxId from 0 to 7 (0 is in front, then increases clockwise).
 * \return current proximity value from 0 to 1023, the greater the value the nearer the objects to the sensor
 */
unsigned int getProximity(int robotAddr, int proxId);

/**
 * \brief Request one of the ambient value (from proximity sensors) of the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param proxId from 0 to 7 (0 is in front, then increases clockwise).
 * \return current ambient value from 0 to 1023, the smaller the value the brighter the ambient light
 */
unsigned int getProximityAmbient(int robotAddr, int proxId);

/**
 * \brief Request one of the ground sensors value of the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param groundId from 0 to 3 (0 is on the left side, then increases clockwise).
 * \return current ground value from 512 to 1023, the smaller the value the darker the surface
 */
unsigned int getGround(int robotAddr, int groundId);

/**
 * \brief Request one of the ambient value (from ground sensors) of the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param groundId from 0 to 3 (0 is on the left side, then increases clockwise).
 * \return current ambient value from 0 to 1023, the smaller the value the brighter the ambient light
 */
unsigned int getGroundAmbient(int robotAddr, int groundId);

/**
 * \brief Request all the proximity sensors values at once.
 * \param robotAddr the address of the robot from which receive data.
 * \param proxArr destination array for the proximity values (size must be 8).
 * \return none
 */
void getAllProximity(int robotAddr, unsigned int* proxArr);

/**
 * \brief Request all the ambient values (from proximity sensors) at once.
 * \param robotAddr the address of the robot from which receive data.
 * \param proxArr destination array for the ambient values (size must be 8).
 * \return none
 */
void getAllProximityAmbient(int robotAddr, unsigned int* proxArr);

/**
 * \brief Request all the ground sensors values at once.
 * \param robotAddr the address of the robot from which receive data.
 * \param groundArr destination array for the ground values (size must be 4).
 * \return none
 */
void getAllGround(int robotAddr, unsigned int* groundArr);

/**
 * \brief Request all the ambient values (from ground sensors) at once.
 * \param robotAddr the address of the robot from which receive data.
 * \param groundArr destination array for the ambient values (size must be 4).
 * \return none
 */
void getAllGroundAmbient(int robotAddr, unsigned int* groundArr);

/**
 * \brief Request all the proximity sensors values of all the robots specified in the list.
 * \param proxArr destination matrix for the proximity values (size must be list_size x 8).
 * \return none
 */
void getAllProximityFromAll(unsigned int proxArr[][8]);

/**
 * \brief Request all the ambient values (from proximity sensors) of all the robots specified in the list.
 * \param proxArr destination matrix for the ambient values (size must be list_size x 8).
 * \return none
 */
void getAllProximityAmbientFromAll(unsigned int proxArr[][8]);

/**
 * \brief Request all the ground sensors values of all the robots specified in the list.
 * \param groundArr destination matrix for the ground values (size must be list_size x 4).
 * \return none
 */
void getAllGroundFromAll(unsigned int groundArr[][4]);

/**
 * \brief Request all the ambient values (from ground sensors) of all the robots specified in the list.
 * \param groundArr destination matrix for the ambient values (size must be list_size x 4).
 * \return none
 */
void getAllGroundAmbientFromAll(unsigned int groundArr[][4]);

/**
 * \brief Request the raw battery value, it contains the sampled value of the battery.
 * \param robotAddr the address of the robot from which receive data.
 * \return current battery value, the values range is between 780 (battery discharged) and 930 (battery charged).
 */
unsigned int getBatteryAdc(int robotAddr);

/**
 * \brief Request the charge percentage of the battery.
 * \param robotAddr the address of the robot from which receive data.
 * \return current charge percentage, the values range is between 0 and 100.
 */
unsigned int getBatteryPercent(int robotAddr);

/**
 * \brief Request the raw value of the accelerometer x axis.
 * \param robotAddr the address of the robot from which receive data.
 * \return current x value, the range is between -64 to 64.
 */
signed int getAccX(int robotAddr);

/**
 * \brief Request the raw value of the accelerometer y axis.
 * \param robotAddr the address of the robot from which receive data.
 * \return current y value, the range is between -64 to 64.
 */
signed int getAccY(int robotAddr);

/**
 * \brief Request the raw value of the accelerometer z axis.
 * \param robotAddr the address of the robot from which receive data.
 * \return current z value, the range is between -64 to 64.
 */
signed int getAccZ(int robotAddr);

/**
 * \brief Request the selector position.
 * \param robotAddr the address of the robot from which receive data.
 * \return current selector position, range is between 0 and 15.
 */
unsigned char getSelector(int robotAddr);

/**
 * \brief Request last command received from the TV remote.
 * \param robotAddr the address of the robot from which receive data.
 * \return current TV remote value.
 */
unsigned char getTVRemoteCommand(int robotAddr);

/**
 * \brief Set a new stato (on or off) for the small green leds around the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param ledId the led to change, from 0 to 7 (0 is in front, then increases clockwise).
 * \param state 0 to turn off, 1 to turn on.
 * \return none.
 */
void setSmallLed(int robotAddr, int ledId, int state);

/**
 * \brief Turn off all the small green leds around the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \return none.
 */
void turnOffSmallLeds(int robotAddr);

/**
 * \brief Turn on all the small green leds around the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \return none.
 */
void turnOnSmallLeds(int robotAddr);

/**
 * \brief Request the current orientation of the robot computed from the measured speed of the motors.
 * \param robotAddr the address of the robot from which receive data.
 * \return current orientation of the robot expressed in 1/10 of degree (3600 degrees for a full turn).
 */
signed int getOdomTheta(int robotAddr);

/**
 * \brief Request the current position (x component) of the robot computed from the measured speed of the motors.
 * \param robotAddr the address of the robot from which receive data.
 * \return current x position of the robot expressed in millimiters.
 */
signed int getOdomXpos(int robotAddr);

/**
 * \brief Request the current position (y component) of the robot computed from the measured speed of the motors.
 * \param robotAddr the address of the robot from which receive data.
 * \return current y position of the robot expressed in millimiters.
 */
signed int getOdomYpos(int robotAddr);

/**
 * \brief Request the current orientation of the robot given by the accelerometer. This function is usable only if the robot is moving vertically.
 * \param robotAddr the address of the robot from which receive data.
 * \return current robot orientation, range is between 0 and 360 degrees (0 pointing to the right).
 */
int getVerticalAngle(int robotAddr);

/**
 * \brief Start the odometry calibration.
 * \param robotAddr the address of the robot for which to change the packet.
 * \return none.
 */
void startOdometryCalibration(int robotAddr);

/**
 * \brief This function need to be called periodically (as fast as possible) in order to exchange data with the robots.
 * \return none.
 */
void transferData();

/**
 * \brief This function stop the transmission of the packets to the robots.
 * \return none.
 */
void stopTransferData();

/**
 * \brief This function resume the transmission of the packets to the robots.
 * \return none.
 */
void resumeTransferData();

/**
 * \brief Tell whether the robot is charging or not.
 * \param robotAddr the address of the robot from which receive data.
 * \return 0 not charging, 1 robot is charging.
 */
unsigned char robotIsCharging(int robotAddr);

/**
 * \brief Tell whether the robot is completely charged or not.
 * \param robotAddr the address of the robot from which receive data.
 * \return 0 not charged, 1 robot is charged.
 */
unsigned char robotIsCharged(int robotAddr);

/**
 * \brief Tell whether the button on the back side of the robot is pressed or not.
 * \param robotAddr the address of the robot from which receive data.
 * \return 0 not pressed, 1 button is pressed.
 */
unsigned char buttonIsPressed(int robotAddr);

/**
 * \brief Reset all the flags sent to the robot.
 * \param robotAddr the address of the robot for which to change data.
 * \return none.
 */
void resetFlagTX(int robotAddr);

/**
 * \brief Request the current flags value (raw) sent to the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \param flagInd 0 or 1
 * \return current flag value.
 */
unsigned char getFlagTX(int robotAddr, int flagInd);

/**
 * \brief Request the current flags value (raw) received from the robot.
 * \param robotAddr the address of the robot from which receive data.
 * \return current flag value.
 */
unsigned char getFlagRX(int robotAddr);

/**
 * \brief Request the current raw value of the left motor steps (sum of the measured speed).
 * \param robotAddr the address of the robot from which receive data.
 * \return current motor steps.
 */
signed long int getLeftMotSteps(int robotAddr);

/**
 * \brief Request the current raw value of the right motor steps (sum of the measured speed).
 * \param robotAddr the address of the robot from which receive data.
 * \return current motor steps.
 */
signed long int getRightMotSteps(int robotAddr);

/**
 * \brief Request the current percentage of failed trasnfers in the last 5 seconds.
 * \param robotAddr the address of the robot from which receive data.
 * \return current quality, 0 to 100.
 */
double getRFQuality(int robotAddr);

/**
 * \brief Reset the flag indicating that the last modified packet is sent to the robot; it should be called before using "messageIsSent".
 * \param robotAddr the address of the robot for which to change data.
 * \return none.
 */
void resetMessageIsSentFlag(int robotAddr);

/**
 * \brief Request if the last modified packet is sent to the robot; this function is thought to be used in polling mode (continue to check whether the message is sent untill it is actually sent). Before the polling the function "resetMessageIsSentFlag" need to be called. It can be used for both check whether the robots received the new messages and to know when new data from the robots are available.
 * \param robotAddr the address of the robot from which receive data.
 * \return 0 if message not received by the robot, 1 if message sent correctly.
 */
unsigned char messageIsSent(int robotAddr);

/**
 * \brief Wait for the message to be actually sent to the robot; if the robot isn't reachable (out of range, discharged, ...) then the function will exit after the amount of time specified as parameter even if the robot doesn't receive the data.
 * \param robotAddr the address of the robot for which to change data.
 * \param us function timeout given in microseconds
 * \return 0 if message sent, 1 otherwise.
 */
unsigned char waitForMessageTransmission(int robotAddr, unsigned long us);

/**
 * \brief Set all the values of a robot at one time.
 * \param robotAddr the address of the robot for which to change data.
 * \param red, green, red RGB channels intensity, range is between 0 (led off) to 100 (max power).
 * \param flags raw flag bytes:
 * byte0:
 * - first two bits are dedicated to the IRs:
 *   0x00 => all IRs off
 *   0x01 => back IR on
 *   0x02 => front IRs on
 *   0x03 => all IRs on
 * - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
 * - fourth bit is used for sleep (1 => go to sleep for 1 minute)
 * - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
 * - sixth bits is reserved (used by radio station)
 * - seventh bit is used for enabling/disabling onboard obstacle avoidance
 * - eight bit is used for enabling/disabling onboard cliff avoidance
 * byte1:
 * - first is used for starting odometry calibration
 * \param left, right left and right speed, range is -128..127.
 * \param leds raw byte value for small green leds (bit0=0/1 turn off/on led0, bit1=0/1 turn off/on led1, ...).
 * \return none.
 */
void setCompletePacket(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds);

/**
 * \brief Set all the values of all the robots specified in the list.
 * \param robotAddr array the addresses of the robots for which to change data.
 * \param red, green, red arrays RGB channels intensity, range is between 0 (led off) to 100 (max power).
 * \param flags array raw flag bytes:
 * byte0:
 * - first two bits are dedicated to the IRs:
 *   0x00 => all IRs off
 *   0x01 => back IR on
 *   0x02 => front IRs on
 *   0x03 => all IRs on
 * - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
 * - fourth bit is used for sleep (1 => go to sleep for 1 minute)
 * - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
 * - sixth bits is reserved (used by radio station)
 * - seventh bit is used for enabling/disabling onboard obstacle avoidance
 * - eight bit is used for enabling/disabling onboard cliff avoidance
 * byte1:
 * - first is used for starting odometry calibration
 * \param left, right arrays left and right speed, range is -128..127.
 * \param leds array raw byte value for small green leds (bit0=0/1 turn off/on led0, bit1=0/1 turn off/on led1, ...).
 * \return none.
 */
void setCompletePacketForAll(int *robotAddr, char *red, char *green, char *blue, char flags[][2], char *left, char *right, char *leds);

/**
 * \brief Wait for updated data coming from the robot; if the robot isn't reachable (out of range, discharged, ...) then the function will exit after the amount of time specified as parameter even if no data are received. this function can be used also to check whether a message is successfully sent to a robot or not.
 * \param robotAddr the address of the robot from which receive data.
 * \param us function timeout given in microseconds
 * \return 0 if updated data received, 1 otherwise.
 */
unsigned char waitForUpdate(int robotAddr, unsigned long us);

/**
 * \brief Send a message to a particular robot resetting the list of robots to a single one (all previous address set with "startCommunication" will be lost).
 * \param robotAddr the address of the robot for which to change data.
 * \param red, green, red RGB channels intensity, range is between 0 (led off) to 100 (max power).
 * \param flags raw flag bytes:
 * byte0:
 * - first two bits are dedicated to the IRs:
 *   0x00 => all IRs off
 *   0x01 => back IR on
 *   0x02 => front IRs on
 *   0x03 => all IRs on
 * - third bit is used for enabling/disablng IR remote control (0=>diabled, 1=>enabled)
 * - fourth bit is used for sleep (1 => go to sleep for 1 minute)
 * - fifth bit is used to calibrate all sensors (proximity, ground, accelerometer)
 * - sixth bits is reserved (used by radio station)
 * - seventh bit is used for enabling/disabling onboard obstacle avoidance
 * - eight bit is used for enabling/disabling onboard cliff avoidance
 * byte1:
 * - first is used for starting odometry calibration
 * \param left, right left and right speed, range is -128..127.
 * \param leds raw byte value for small green leds (bit0=0/1 turn off/on led0, bit1=0/1 turn off/on led1, ...).
 * \param us function timeout given in microseconds
 * \return 0 if message sent succesfully, 1 otherwise.
 */
unsigned char sendMessageToRobot(int robotAddr, char red, char green, char blue, char flags[2], char left, char right, char leds, unsigned long us);

#ifdef __cplusplus
}
#endif

#endif // ELISA3_LIB_H_



