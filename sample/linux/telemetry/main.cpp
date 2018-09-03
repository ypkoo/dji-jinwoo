/*! @file telemetry/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Telemetry API usage in a Linux environment.
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
#include "flight_control.hpp"
#include <time.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <sstream>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


const std::string LoggingDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
  return buf;
}


int
main(int argc, char** argv)
{

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Display interactive prompt
  std::cout
    << "\n\n\n"
    << "=================== M600 Energy Logging Client ==================="
    << std::endl;
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [t] Takeoff before running logging commands                    |"
    << std::endl;
  std::cout
    << "| [l] Land M600                                                  |"
    << std::endl;
  std::cout
    << "| [f] Read command from file and log information                 |"
    << std::endl;
  std::cout
    << "| [r] Generate random command and log information                |"
    << std::endl;
  std::cout
    << "| [h] Help for more availabe commands                            |"
    << std::endl;
  std::cout
    << "| [q] Quit the program and exit                                  |"
    << std::endl;
  std::cout
    << "\n>> Commands: ";


  while (true) {
    char inputChar;
    std::cin >> inputChar;
    


    switch (inputChar)
    {
      case 't':
        vehicle->control->takeoff(3);
	std::cout << "Taking off..." << std::endl;
	for (int count = 0; count < 10; count++) {
	  std::cout << 10-count << std::endl;
	  sleep(1);
	}
        std::cout << "\n>> Commands: ";
        break;
      case 'l':
        vehicle->control->land(3);
        std::cout << "\n>> Commands: ";
        break;
      case 'o':
        vehicle->obtainCtrlAuthority(1);
	std::cout << "Control Authority obatined by SDK" << std::endl;
        std::cout << "\n>> Commands: ";
        break;
      case 'x':
	vehicle->releaseCtrlAuthority(1);
	std::cout << "Release Control Authority from SDK" << std::endl;
	std::cout << "\n>> Commands: ";
	break;
      case 'i':
	if (vehicle->getFwVersion() != Version::M100_31) {
	  std::cout << "Initializing Control Broadcast..." << std::endl;
	  for (int send = 0; send < 10; send++) {
	    vehicle->control->velocityAndYawRateCtrl(0, 0, 0, 0);
	    std::cout << 10-send << std::endl;
	    usleep(100000);
	  }
	}
	std::cout << "\n>> Commands: ";
	break;
      case 'u':
	if (vehicle->getFwVersion() != Version::M100_31) {
          vehicle->control->velocityAndYawRateCtrl(0, 0, 1, 0);
	}
        std::cout << "\n>> Commands: ";
        break;
      case 'd':
        vehicle->control->velocityAndYawRateCtrl(0, 0, -1, 0);
        std::cout << "\n>> Commands: ";
        break;
      case 'y':
	vehicle->control->velocityAndYawRateCtrl(0, 0, 0, -10);
	std::cout << "Flight Control Yaw Rate Test" << std::endl;
	std::cout << "\n>> Commands: ";
	break;
      case 'v':
	vehicle->control->velocityAndYawRateCtrl(1, 1, 0, 0);
	std::cout << "Flight Control Velocity Test" << std::endl;
	std::cout << "\n>> Commands: ";
	break;
      case 'b':
	if (vehicle->getFwVersion() != Version::M100_31) {
	  vehicle->control->emergencyBrake();
	  std::cout << "Emergency Brake!!!" << std::endl;
	}
        std::cout << "\n>> Commands: ";
        break;
      case 'f':
        if (vehicle->getFwVersion() != Version::M100_31){
          int T;
          std::string path, line, des;
          std::cout << "Name of Command File: ";
          std::cin >> path;
          std::cout << "\nPath to the file: /home/nvidia/energy_m600/cmd/" << path << std::endl;
          std::cout << "Duration time(0.1s) for each command: ";
          std::cin >> T; 
          std::ifstream CmdFile;
          CmdFile.open("/home/nvidia/energy_m600/cmd/"+path);
          std::cout << "Add description to Log File name: ";
          std::cin >> des;
          init_subscription(vehicle);
          std::ofstream LogFile;
          LogFile.open("/home/nvidia/energy_m600/log/"+LoggingDateTime()+des+".csv");
          LogFile << "timestamp,lat,log,alt,roll,pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,act_vx,act_vy,act_yr,capacity,vol,cur,power" << std::endl;
          while (getline(CmdFile, line)) {
            int fvx, fvy, fyr;
            std::cout << line << std::endl;
            std::vector<std::string> split;
            std::istringstream iss(line);
            for (std::string line; iss >> line; )
              split.push_back(line);
            fvx = std::stoi(split[0]);
            fvy = std::stoi(split[1]);
            fyr = std::stoi(split[2]);
	    if (flight_control(vehicle, LogFile, T, fvx, fvy, fyr) == false) {
	      vehicle->releaseCtrlAuthority(1);
	      std::cout << "Release Control Authority" << std::endl;
	      std::cout << "Exit Process" << std::endl;
	      break;
	    }
          }
          LogFile.close();
          CmdFile.close();
          end_subscription(vehicle);
        }
        std::cout << "\n>> Commands: ";
        break;
      case 'r':
        if (vehicle->getFwVersion() != Version::M100_31){
          int T, range_vx, range_vy, range_yr;
          int num, rvx, rvy, ryr;
          std::string des;
          std::cout << "Set Number of Random Commands: ";
          std::cin >> num;
          std::cout << "Set duration time(*0.1s) for each command: ";
          std::cin >> T;
          std::cout << "Set range for Vx command (0~60): ";
          std::cin >> range_vx;
          std::cout << "Set range for Vy command (0~60): ";
          std::cin >> range_vy;
          std::cout << "Set range for Yr command (0~300): ";
          std::cin >> range_yr;
          srand(time(NULL));
          std::cout << "Add description to Log File name: ";
          std::cin >> des;
          init_subscription(vehicle);
          std::ofstream LogFile;
          LogFile.open("/home/nvidia/energy_m600/log/"+LoggingDateTime()+des+".csv");
          LogFile << "timestamp,lat,log,alt,roll,pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,act_vx,act_vy,act_yr,capacity,vol,cur,power" << std::endl;      
          for (int iter = 0; iter < num; iter++) {
	    if (range_vx == 0)
	      rvx = 0;
	    else
	      rvx = rand() % range_vx - range_vx*0.5;
	    if (range_vy == 0)
	      rvy = 0;
	    else
	      rvy = rand() % range_vy - range_vy*0.5;
	    if (range_yr == 0)
	      ryr = 0;
	    else
	      ryr = rand() % range_yr - range_yr*0.5;
            if (flight_control(vehicle, LogFile, T, rvx, rvy, ryr) == false) {
	      vehicle->releaseCtrlAuthority(1);
	      std::cout << "Release Control Authority" << std::endl;
	      std::cout << "Exit Process" << std::endl;
	      break;
	    }
          }
          LogFile.close();
          end_subscription(vehicle);
        }
        std::cout << "\n>> Commands: ";
        break;
      case 'g':
	if (vehicle->getFwVersion() != Version::M100_31){
          int T, num, fense;
	  int max_vx, max_vy, max_yr, min_vx, min_vy, min_yr;
	  float lat1, lat2, lat3, lat4, lon1, lon2, lon3, lon4;
          float gvx, gvy, gyr;
	  float deg2rad = 0.01745329251;
          std::string des;
	  std::cout << "Set GPS Fense ([1]Sports Field, [2]Sports Field Maximum, [3]Manual) :";
	  std::cin >> fense;
	  if (fense == 1) {
	    lat1 = 36.369699*deg2rad;
	    lon1 = 127.368166*deg2rad;
	    lat2 = 36.369870*deg2rad;
	    lon2 = 127.368469*deg2rad;
	    lat3 = 36.369418*deg2rad;
	    lon3 = 127.368788*deg2rad;
	    lat4 = 36.369263*deg2rad;
	    lon4 = 127.368563*deg2rad;
	  }
	  if (fense == 3) {
	    lat1 = 36.369470*deg2rad;
	    lon1 = 127.368459*deg2rad;
	    lat2 = 36.369638*deg2rad;
	    lon2 = 127.368708*deg2rad;
	    lat3 = 36.369418*deg2rad;
	    lon3 = 127.368788*deg2rad;
	    lat4 = 36.369230*deg2rad;
	    lon4 = 127.368544*deg2rad;
	  }
          std::cout << "Set Number of Random Commands: ";
          std::cin >> num;
          std::cout << "Set duration time(*0.1s) for each command: ";
          std::cin >> T;
          std::cout << "Set Maximum for Vx command (0~30): ";
          std::cin >> max_vx;
	  std::cout << "Set Minimum for Vx command (0~30): ";
          std::cin >> min_vx;
          std::cout << "Set Maximum for Vy command (0~30): ";
          std::cin >> max_vy;
	  std::cout << "Set Minimum for Vy command (0~30): ";
          std::cin >> min_vy;
	  /*
          std::cout << "Set Maximum for Yr command (0~150): ";
          std::cin >> max_yr;
          std::cout << "Set Minimum for Yr command (0~150): ";
          std::cin >> min_yr;
	  */
	  // Yaw Rate Not Implemented Yet
	  max_yr = 0;
	  min_yr = 0;
          srand(time(NULL));
          std::cout << "Add description to Log File name: ";
          std::cin >> des;
          init_subscription(vehicle);
	  sleep(1);
          std::ofstream LogFile;
          LogFile.open("/home/nvidia/energy_m600/log/"+LoggingDateTime()+des+".csv");
          LogFile << "timestamp,lat,log,alt,roll,pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,act_vx,act_vy,act_yr,capacity,vol,cur,power" << std::endl;      
          for (int iter = 0; iter < num; iter++) {
	    if (max_vx == 0)
	      gvx = 0;
	    else {
	      gvx = (float)(rand() % (20*(max_vx-min_vx)+1))*0.1 - (max_vx-min_vx);
	      if (gvx > 0)
		gvx = gvx + min_vx;
	      else
		gvx = gvx - min_vx;
	    }	    
	    if (max_vy == 0)
	      gvy = 0;
	    else {
	      gvy = (float)(rand() % (20*(max_vy-min_vy)+1))*0.1 - (max_vy-min_vy);
	      if (gvy > 0)
		gvy = gvy + min_vy;
	      else
		gvy = gvy - min_vy;
	    }
	    if (max_yr == 0)
	      gyr = 0;
	    else
	      gyr = (float)(rand() % (20*(max_yr)+1))*0.1 - max_yr;
	    if (fense_check(vehicle, lat1, lon1, lat2, lon2) == true) {
	      std::cout << "GPS Fense 1 Activated" << std::endl;
	      if (gvy<0)
		gvy = gvy*(-1);
	    }
	    if (fense_check(vehicle, lat2, lon2, lat3, lon3) == true) {
	      std::cout << "GPS Fense 2 Activated" << std::endl;
	      if (gvx>0)
		gvx = gvx*(-1);
	    }
	    if (fense_check(vehicle, lat4, lon4, lat3, lon3) == false) {
	      std::cout << "GPS Fense 3 Activated" << std::endl;
	      if (gvy>0)
		gvy = gvy*(-1);
	    }
	    if (fense_check(vehicle, lat1, lon1, lat4, lon4) == false) {
	      std::cout << "GPS Fense 4 Activated" << std::endl;
	      if (gvx<0)
		gvx = gvx*(-1);
	    }
            if (flight_control_body(vehicle, LogFile, T, gvx, gvy, gyr) == false) {
	      vehicle->releaseCtrlAuthority(1);
	      std::cout << "Release Control Authority" << std::endl;
	      std::cout << "Exit Process" << std::endl;
	      break;
	    }
	    std::cout << "Number of random actions left: " << num-iter << std::endl;
          }
          LogFile.close();
          end_subscription(vehicle);
        }
        std::cout << "\n>> Commands: ";
	break;
      case 's':
	break;
      case 'h':
        std::cout << "| Available commands:                                            |" << std::endl;
        std::cout << "| [t]: Take off                                                  |" << std::endl;
        std::cout << "| [l]: Land                                                      |" << std::endl;
        std::cout << "| [o]: Obtain control authority                                  |" << std::endl;
	std::cout << "| [x]: Release control authority                                 |" << std::endl;
	std::cout << "| [i]: Initializing flight control by SDK                        |" << std::endl;
        std::cout << "| [u]: Position the drone upward                                 |" << std::endl;
        std::cout << "| [d]: Position the drone downward                               |" << std::endl;
	std::cout << "| [y]: Testing yaw rate flight control                           |" << std::endl;
	std::cout << "| [v]: Testing velocity flight control                           |" << std::endl;
        std::cout << "| [b]: Emergency break                                           |" << std::endl;
        std::cout << "| [f]: Read command from file and start logging                  |" << std::endl;
        std::cout << "| [r]: Generate random command and start logging                 |" << std::endl;
	std::cout << "| [s]: Subscription until keyboard interrupt                     |" << std::endl;
        std::cout << "| [h]: Help for available commands                               |" << std::endl;
        std::cout << "| [q]: Quit program and exit                                     |" << std::endl;
        std::cout << "\n>> Commands: ";
        break;
      case 'q':
        return 0;
      default:
        break;
    }
  }
  return 0;
}
