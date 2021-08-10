/*
 * gps.c
 *
 *  Created on: Aug 10, 2021
 *      Author: clapd
 */
#include "TinyGPS.h"
#include "usart.h"
#include "MPU9250.h"
#include "math.h"
#include "motor.h"
uint8_t byte_gps = 0;
int16_t Acc[3] = {0};
int16_t Mag[3] = {0};
int16_t Gyr[3] = {0};
int16_t compass_heading = 0;
int16_t desired_heading = 0;
int16_t heading_A = 0, heading_B = 0;
float_t waypoint_lat = 0;
float_t waypoint_lon = 0;
void getGPS(){
  while(HAL_UART_Receive(&huart2, &byte_gps, 1, 100)==HAL_OK){
    encode(byte_gps);
  }
}
void setWaypoint(){
  //may be used to set Waypoint From Raspberry pi
}
void clearWaypoint(){

}
void getCompass(){
  MPU9250_GetData(Acc, Mag, Gyr);
  float_t heading = atan2(Mag[2], Mag[1]);
  if(heading < 0)
    heading += 2 * M_PI;
  compass_heading = (int16_t)(heading * 180/M_PI);
}
void setHeading(){
  for(uint8_t i = 0; i <=5; i++){
    {
      getCompass();
    }
  }
  desired_heading = compass_heading;

}
void gpsInfo(){

}
void goWaypoint(){
  float_t cur_lat = 0, cur_lon = 0;
  uint32_t cur_age = 0;
  getCompass();
  getGPS();
  f_get_position(&cur_lat, &cur_lon, &cur_age);
  cur_lat /= 0.0001;
  cur_lat /= 10000;
  cur_lon /= 0.0001;
  cur_lon /= 10000;
  waypoint_lat /= 0.0001;
  waypoint_lon /= 0.0001;
  waypoint_lat /= 10000;
  waypoint_lon /= 10000;
  float_t Distance_To_Home = distance_between(cur_lat, cur_lon, waypoint_lat, waypoint_lon);
  if(Distance_To_Home == 0){
    Stop();
  }
  float_t GPS_Course = course_to(cur_lat, cur_lon, waypoint_lat, waypoint_lon);
  if(abs(GPS_Course - compass_heading) <= 15 ){
    mvForward();
  }
  else{
    uint16_t x = (GPS_Course - 360);
    uint16_t y = (compass_heading - (x));
    uint16_t z = (y - 360);
    if (z <= 180 && z >= 0){
      mvLeft();
    }
    else mvRight();
  }
}
void Startup(){
  uint8_t Number_of_SATS = 0;
  while (Number_of_SATS <=4){
    getGPS();
    Number_of_SATS = satellites();
  }
  setWaypoint();
}



