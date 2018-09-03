/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
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

#include "flight_control.hpp"
#include "telemetry.hpp"
#include <iostream>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


bool flight_control(DJI::OSDK::Vehicle* vehicle, std::ofstream &LogFile, int T, float act_vx, float act_vy, float act_yr)
{
  for (int iter = 0; iter < T; iter ++) {
    vehicle->control->velocityAndYawRateCtrl(act_vx, act_vy, 0, act_yr);
    if (subscribeToData(vehicle, LogFile, act_vx, act_vy, act_yr) == false) {
      vehicle->control->emergencyBrake();
      std::cout << "Emergency Brake!!!" << std::endl;
      return false;
    }
    usleep(100000);
  }
  return true;
}

bool flight_control_body(DJI::OSDK::Vehicle* vehicle, std::ofstream &LogFile, int T, float act_vx, float act_vy, float act_yr)
{
  for (int iter = 0; iter < T; iter ++) {
    vehicle->control->velocityAndYawRateBodyCtrl(act_vx, act_vy, 0, act_yr);
    if (subscribeToData(vehicle, LogFile, act_vx, act_vy, act_yr) == false) {
      vehicle->control->emergencyBrake();
      std::cout << "Emergency Brake!!!" << std::endl;
      return false;
    }
    usleep(100000);
  }
  return true;
}

bool fense_check(DJI::OSDK::Vehicle* vehicle, float lat_start, float lon_start, float lat_end, float lon_end)
{
  TypeMap<TOPIC_GPS_FUSED>::type latLon;
  latLon = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
  float target_lat = latLon.latitude;
  float target_lon = latLon.longitude;
  float slope = (lat_end-lat_start)/(lon_end-lon_start);
  //std::cout << target_lat << ", " << target_lon << std::endl;
  if (slope*target_lon - slope*lon_start + lat_start > target_lat)
    return false;
  return true;
  
}
