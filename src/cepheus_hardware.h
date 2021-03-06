#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm7820_library.h>

#define LIMIT_L1 1 // limits switch pin in port 0
#define LIMIT_L2 3 // limits switch pin in port 0

#define FIR_LENGTH 2 
#define PWM_MOTOR_FREQ 5000
#define PWM_HOBBY_SERVO_FREQ 50
#define PWM_THRUSTER_FREQ 10
#define ADC_PWM_FREQ 25000

#define PWM_MOTOR_PERIOD_COUNTS (25000000/PWM_MOTOR_FREQ)
#define PWM_MOTOR_RANGE (PWM_MOTOR_PERIOD_COUNTS-(PWM_MOTOR_PERIOD_COUNTS/5)) //PWM range must be 10%-90% for escon drivers so 80% of full range
#define PWM_MOTOR_MIN_DT (PWM_MOTOR_PERIOD_COUNTS/10)
#define PWM_MOTOR_MAX_DT (PWM_MOTOR_PERIOD_COUNTS - (PWM_MOTOR_PERIOD_COUNTS/10))

#define PWM_HOBBY_SERVO_PERIOD_COUNTS (2500000/PWM_HOBBY_SERVO_FREQ)
#define PWM_HOBBY_SERVO_RANGE  4750//PWM range must be 0.6ms-2.4ms = 1.8ms for the servos
#define PWM_HOBBY_SERVO_MIN_DT 1250 //0.6ms for period of 1/25MHz 
#define PWM_HOBBY_SERVO_MAX_DT 6000 //2.5ms for period of 1/25MHz 

#define ADC_PWM_PERIOD_COUNTS (25000000/ADC_PWM_FREQ)


class CepheusHW : public hardware_interface::RobotHW
{
public:
  void heartbeat();
  void setThrustPwm(double*, double, double);
  void writeMotors();
  bool isLimitReached(int i);
  void setHomePos(int i, float val);
  void readLimitSwitches();
  uint8_t init();
  void readEncoders(ros::Duration);
  void setParam(double*, double);
  void setCmd(int,double);
  double getVel(int);
  void safeClose(); 
  CepheusHW(); 
 
private:
/***controller manager interface***/
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  enum motors{ rw, l1, l2, r1, r2}; 

/*****Motors parameters*********/
  double max_current[8];
  double max_thrust;
/*****SERVOS parameters*********/
  double force[4];

/******motors********/
  int limit[8];
  double cmd[12];
  uint16_t width[12];
  uint16_t dir[10];
  double current[8];
  double home_pos[8];
  double pos[8], prev_pos[8], offset_pos[8];
  double vel[8];
  double eff[8];
  double servo_pos[4];
  double vel_new[8];
  double vel_fir[FIR_LENGTH][8];
  //encoder values from card
  uint16_t encoder_1_val;
  uint16_t encoder_2_val;
  uint16_t encoder_3_val;
  uint16_t encoder_4_val;
  uint16_t encoder_5_val;
  uint16_t encoder_6_val;
  uint16_t encoder_7_val;
  uint16_t encoder_8_val;
  //encoder overflow counters
  int encoder_1_ovf;
  int encoder_2_ovf;
  int encoder_3_ovf;
  int encoder_4_ovf;
  int encoder_5_ovf;
  int encoder_6_ovf;
  int encoder_7_ovf;
  int encoder_8_ovf;
  //encoder values to transmit
  int encoder_1;
  int encoder_2;
  int encoder_3;
  int encoder_4;
  int encoder_5;
  int encoder_6;
  int encoder_7;
  int encoder_8;
/******thrusters********/
  uint32_t period;

/*******IO board********/
  DM7820_Board_Descriptor *board;
  DM7820_Board_Descriptor *manipulator_board;
  DM7820_Error dm7820_status;
  uint8_t encoder_status;
  dm7820_incenc_phase_filter phase_filter;
  int status;


  int count;
  uint16_t output_value;
  bool strobe;
};
