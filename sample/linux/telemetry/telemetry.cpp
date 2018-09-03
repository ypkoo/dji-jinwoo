/*! @file telemetry_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "telemetry.hpp"
#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
  return buf;
}

std::chrono::steady_clock::time_point begin;
std::chrono::steady_clock::time_point end;
bool
init_subscription(Vehicle* vehicle, int responseTimeout)
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // RTK can be detected as unavailable only for Flight controllers that don't support RTK
  bool rtkAvailable = false;
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 2000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Acceleration at 50 Hz
  // 6. Quaternion at 200 Hz
  // 7. Battery at 50 Hz

  // Please make sure your drone is in simulation mode. You can fly the drone
  // with your RC to
  // get different values.

  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;
  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED};
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 2: Subscribe to RC Channel and Velocity and Battery at freq 50 Hz

  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY, TOPIC_ACCELERATION_BODY, TOPIC_BATTERY_INFO };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }


  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup before return
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    return false;
  }

  // Package 4: Subscribe to RTK at freq 5 Hz.
  pkgIndex                   = 4;
  freq                       = 5;
  TopicName topicListRTK5Hz[] = {TOPIC_RTK_POSITION, TOPIC_RTK_YAW_INFO,
                                  TOPIC_RTK_POSITION_INFO, TOPIC_RTK_VELOCITY,
                                  TOPIC_RTK_YAW};
  numTopic        = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
  enableTimestamp = false;

  pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicListRTK5Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }
  else {
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if(subscribeStatus.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      std::cout << "RTK Not Available" << "\n";
      rtkAvailable = false;
    }
    else
    {
      rtkAvailable = true;
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }
    }
  }
  return true;
}


bool
subscribeToData(Vehicle* vehicle, std::ofstream &LogFile, float act_vx, float act_vy, float act_yr, int responseTimeout)
{
  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type         latLon;
  TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
  TypeMap<TOPIC_RC>::type                rc;
  TypeMap<TOPIC_VELOCITY>::type          velocity;
  TypeMap<TOPIC_ACCELERATION_BODY>::type acceleration;
  TypeMap<TOPIC_QUATERNION>::type        quaternion;
  TypeMap<TOPIC_RTK_POSITION>::type      rtk;
  TypeMap<TOPIC_RTK_POSITION_INFO>::type rtk_pos_info;
  TypeMap<TOPIC_RTK_VELOCITY>::type      rtk_velocity;
  TypeMap<TOPIC_RTK_YAW>::type           rtk_yaw;
  TypeMap<TOPIC_RTK_YAW_INFO>::type      rtk_yaw_info;
  TypeMap<TOPIC_BATTERY_INFO>::type      battery;


  // Print in a loop
  //while (elapsedTimeInMs < timeToPrintInMs)
  int loop_count = 0;
  while (loop_count == 0)
  {
    // time stamp
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    //flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    rc           = vehicle->subscribe->getValue<TOPIC_RC>();
    velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    acceleration = vehicle->subscribe->getValue<TOPIC_ACCELERATION_BODY>();
    quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    battery      = vehicle->subscribe->getValue<TOPIC_BATTERY_INFO>();
/*
    if(rtkAvailable) {
      rtk = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
      rtk_pos_info = vehicle->subscribe->getValue<TOPIC_RTK_POSITION_INFO>();
      rtk_velocity = vehicle->subscribe->getValue<TOPIC_RTK_VELOCITY>();
      rtk_yaw = vehicle->subscribe->getValue<TOPIC_RTK_YAW>();
      rtk_yaw_info = vehicle->subscribe->getValue<TOPIC_RTK_YAW_INFO>();
    }
*/
    // quaternion to roll, pitch, yaw
    double sinr = +2.0 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3);
    double cosr = +1.0 - 2.0 * (quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2);
    double roll = atan2(sinr, cosr);
    double sinp = +2.0 * (quaternion.q0 * quaternion.q2 - quaternion.q3 * quaternion.q1);
    double pitch;
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp);
    else
      pitch = asin(sinp);
    double siny = +2.0 * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2);
    double cosy = +1.0 - 2.0 * (quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
    double yaw = atan2(siny, cosy);
    //std::cout << "Flight Status                         = " << (int)flightStatus << "\n";
    //std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", " << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
    //rc.gear;
    /*
    std::cout << "RC: " << rc.mode << ", " << rc.gear << std::endl;
    //std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0 << ", " << quaternion.q1 << ", " << quaternion.q2 << ", " << quaternion.q3 << "\n";
    //std::cout << "Time: " << currentDateTime() << std::endl;
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << "(us)" << std::endl;
    std::cout << "Location: " << latLon.latitude << ", " << latLon.longitude << ", " << altitude << std::endl;
    std::cout << "Position: " << roll << ", " << pitch << ", " << yaw << std::endl;
    std::cout << "Velocity: " << velocity.data.x << ", " << velocity.data.y << ", "<< velocity.data.z << std::endl;
    std::cout << "Acceleration: " << acceleration.x << ", " << acceleration.y << ", " << acceleration.z << std::endl;
    std::cout << "Action: " << act_vx << ", " << act_vy << ", " << act_yr << std::endl;
    std::cout << "Battery: " << battery.capacity << ", " << battery.voltage*0.001 << "V, " << -battery.current*0.001 << "A, " 
              << battery.voltage*(-battery.current)*0.00001 << "W" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    */
    // Logging to file
    // timestamp,lat,log,alt,roll,pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,act_vx,act_vy,act_yr,capacity,vol,cur,power
    LogFile << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << ",";
    LogFile << latLon.latitude << "," << latLon.longitude << "," << altitude << ",";
    LogFile << roll << "," << pitch << "," << yaw << ",";
    LogFile << velocity.data.x << "," << velocity.data.y << "," << velocity.data.z << ",";
    LogFile << acceleration.x << "," << acceleration.y << "," << acceleration.z << ",";
    LogFile << act_vx << "," << act_vy << "," << act_yr << ",";
    LogFile << battery.capacity << "," << battery.voltage*0.001 << "," << -battery.current*0.001 << "," << battery.voltage*(-battery.current)*0.00001 << std::endl;
    //usleep(100000);
    //elapsedTimeInMs += 5;
    loop_count += 1;
  }
  if (rc.mode == -10000)
    return true;
  else {
    std::cout << "Remote Controller Mode Interrupt Detected" << std::endl;
    return false;
  }
}

bool
end_subscription(Vehicle* vehicle, int responseTimeout)
{
  std::cout << "Done printing!\n";
  vehicle->subscribe->removePackage(0, responseTimeout);
  vehicle->subscribe->removePackage(1, responseTimeout);
  vehicle->subscribe->removePackage(2, responseTimeout);
  vehicle->subscribe->removePackage(3, responseTimeout);
  vehicle->subscribe->removePackage(4, responseTimeout);
  std::cout << std::endl;
  std::cout << "------------------------------------------------------------" << std::endl;
  return true;
}

