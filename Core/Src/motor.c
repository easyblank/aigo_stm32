#include "tim.h"
#include "gpio.h"
const uint8_t FORWARD = 1;
const uint8_t BACKWARD = 2;
const uint8_t LF = 1;
const uint8_t RF = 2;
const uint8_t LB = 3;
const uint8_t RB = 4;
uint32_t motor_speed = 4000;
uint32_t motor_slow = 2000;
int32_t motor_PWM[4] = {0, 0, 0, 0};
void set_motor(uint8_t loc, uint8_t dir, uint32_t spd){

  if (loc == LF && dir == FORWARD){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
    TIM1->CCR1 = spd;
  }
  else if (loc == LF && dir == BACKWARD){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
    TIM1->CCR1 = spd;
  }
  else if (loc == RF && dir == FORWARD){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
    TIM1->CCR2 = spd;
  }
  else if (loc == RF && dir == BACKWARD){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
    TIM1->CCR2 = spd;
  }
  else if (loc == LB && dir == FORWARD){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);
    TIM1->CCR3 = spd;
  }
  else if (loc == LB && dir == BACKWARD){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);
    TIM1->CCR3 = spd;
  }
  else if (loc == RB && dir == BACKWARD){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);
    TIM1->CCR4 = spd;
  }
  else if (loc == RB && dir == BACKWARD){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, SET);
    TIM1->CCR4 = spd;
  }
}
void motor_move(){
  TIM1->CCR1 = motor_PWM[0];
  TIM1->CCR2 = motor_PWM[1];
  TIM1->CCR3 = motor_PWM[2];
  TIM1->CCR4 = motor_PWM[3];
}
void mvForward(){
  set_motor(LF, FORWARD, motor_speed+motor_PWM[0]);
  set_motor(RF, FORWARD, motor_speed+motor_PWM[1]);
  set_motor(LB, FORWARD, motor_speed+motor_PWM[2]);
  set_motor(RB, FORWARD, motor_speed+motor_PWM[3]);
}
void mvBackward(){
  set_motor(LF, BACKWARD, motor_speed);
  set_motor(RF, BACKWARD, motor_speed);
  set_motor(LB, BACKWARD, motor_speed);
  set_motor(RB, BACKWARD, motor_speed);
}
void mvLeft(){
  set_motor(LF, FORWARD, motor_slow+motor_PWM[0]);
  set_motor(RF, FORWARD, motor_speed+motor_PWM[1]);
  set_motor(LB, FORWARD, motor_slow+motor_PWM[2]);
  set_motor(RB, FORWARD, motor_speed+motor_PWM[3]);

}
void mvRight(){
  set_motor(LF, FORWARD, motor_speed+motor_PWM[0]);
  set_motor(RF, FORWARD, motor_slow+motor_PWM[1]);
  set_motor(LB, FORWARD, motor_speed+motor_PWM[2]);
  set_motor(RB, FORWARD, motor_slow+motor_PWM[3]);
}
void Stop(){
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
}
