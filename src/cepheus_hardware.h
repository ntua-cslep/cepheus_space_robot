#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dm7820_library.h>

#define FIR_LENGTH 15 
#define PWM_DIVIDER 25000
#define MAX_CURRENT 1.72//A


class CepheusHW : public hardware_interface::RobotHW
{
public:
  void writeMotors();
  void readEncoders(ros::Duration);
  CepheusHW(); 
  ~CepheusHW(); 
 
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  double cmd[4];
  double pos[4], prev_pos[4], offset_pos[4];
  double vel[4];
  double eff[4];
  
  double vel_new[4];
  double vel_fir[FIR_LENGTH][4];

  //CARD
  DM7820_Board_Descriptor *board;
  DM7820_Error dm7820_status;
  uint8_t encoder_status;
  dm7820_incenc_phase_filter phase_filter;
  int status;

  //ENCODER VALUES FROM CARD
  uint16_t encoder_1_val;
  uint16_t encoder_2_val;
  uint16_t encoder_3_val;
  uint16_t encoder_4_val;

  //ENCODER OVERFLOW COUNTERS
  int encoder_1_ovf;
  int encoder_2_ovf;
  int encoder_3_ovf;
  int encoder_4_ovf;

  //ENCODER VALUES TO TRANSMIT
  int encoder_1;
  int encoder_2;
  int encoder_3;
  int encoder_4;

};
