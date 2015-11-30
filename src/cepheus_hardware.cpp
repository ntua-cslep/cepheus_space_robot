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
#include <ros/ros.h>
#include <dm7820_library.h>
#include "cepheus_hardware.h"
#include <math.h>


void CepheusHW::writeMotors()
{
  uint16_t width[6];
  width[0]= (uint16_t)cmd[0]*250;
  width[1]= (uint16_t)cmd[1]*250;
  width[2]= (uint16_t)cmd[2]*250;
  width[3]= (uint16_t)cmd[3]*250;
  width[4]= (uint16_t)cmd[4]*250;
  width[5]= (uint16_t)cmd[5]*250;

  // Set output A width to obtain 20% duty cycle
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A,  width[0]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B,  width[1]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C,  width[2]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D,  width[3]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  eff[0] = cmd[0];
  eff[1] = cmd[1];
  // eff[2] = cmd[2];
  eff[3] = cmd[3];
  eff[4] = cmd[4];
  eff[5] = cmd[5];
}

void CepheusHW::readEncoders(ros::Duration dt)
{
// read robots joint state
  //Read encoder 0 channel A value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0, 
    DM7820_INCENC_CHANNEL_A,
    &encoder_1_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //Read encoder 0 channel B value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0, 
    DM7820_INCENC_CHANNEL_A,
    &encoder_2_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //Read encoder 1 channel A value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
    DM7820_INCENC_CHANNEL_A,
    &encoder_3_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //Read encoder 1 channel B value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
    DM7820_INCENC_CHANNEL_A,
    &encoder_4_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

// Handling of encoder value overflow
  if      ((int)encoder_1_val - (int)encoder_1_old < -62000) encoder_1_ovf++;
  else if ((int)encoder_1_val - (int)encoder_1_old >  62000) encoder_1_ovf--;
  encoder_1_old = encoder_1_val;
  encoder_1 = (int)encoder_1_val + 65535*encoder_1_ovf;

  if      ((int)encoder_2_val - (int)encoder_2_old < -62000) encoder_2_ovf++;
  else if ((int)encoder_2_val - (int)encoder_2_old >  62000) encoder_2_ovf--;
  encoder_2_old = encoder_2_val;
  encoder_2 = (int)encoder_2_val + 65535*encoder_2_ovf;

  if      ((int)encoder_3_val - (int)encoder_3_old < -62000) encoder_3_ovf++;
  else if ((int)encoder_3_val - (int)encoder_3_old >  62000) encoder_3_ovf--;
  encoder_3_old = encoder_3_val;
  encoder_3 = (int)encoder_3_val + 65535*encoder_3_ovf;

  if      ((int)encoder_4_val - (int)encoder_4_old < -62000) encoder_4_ovf++;
  else if ((int)encoder_4_val - (int)encoder_4_old >  62000) encoder_4_ovf--;
  encoder_4_old = encoder_4_val;
  encoder_4 = (int)encoder_4_val + 65535*encoder_4_ovf;

  if      ((int)encoder_5_val - (int)encoder_5_old < -62000) encoder_5_ovf++;
  else if ((int)encoder_5_val - (int)encoder_5_old >  62000) encoder_5_ovf--;
  encoder_5_old = encoder_5_val;
  encoder_5 = (int)encoder_5_val + 65535*encoder_5_ovf;

  if      ((int)encoder_6_val - (int)encoder_6_old < -62000) encoder_6_ovf++;
  else if ((int)encoder_6_val - (int)encoder_6_old >  62000) encoder_6_ovf--;
  encoder_6_old = encoder_6_val;
  encoder_6 = (int)encoder_6_val + 65535*encoder_6_ovf;

  ROS_DEBUG("1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d", encoder_1, encoder_2, encoder_3, encoder_4, encoder_5, encoder_6);

// Position Calculation
  pos[0]=  (double)encoder_1*2*M_PI/(4096*190) - offset_pos[0];
  pos[1]= -(double)encoder_2*2*M_PI/(4096*190) - offset_pos[1];
  pos[2]=  (double)encoder_3*2*M_PI/(4096*190) - offset_pos[2];//disconnected
  
  pos[3]=  (double)encoder_4*2*M_PI/(4096*190) - offset_pos[3];
  pos[4]=  (double)encoder_5*2*M_PI/(4096*190) - offset_pos[4];
  pos[5]=  (double)encoder_6*2*M_PI/ 4096      - offset_pos[5];

  ROS_DEBUG("encoder6: %d, pos (10^3): %d", encoder_6, (int)(pos[5]*1000));
  
  ROS_DEBUG("POS: 1: %f, 2: %f, 3: %f, 4: %f, 5: %f, 6: %f", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

// Speed Calculation
  for(int i=0; i<6; i++)
  {
    vel[i]=(pos[i] - prev_pos[i])/dt.toSec();
    prev_pos[i] = pos[i];
  }

}


CepheusHW::CepheusHW() 
{ 
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_right_shoulder("right_shoulder", &pos[0], &vel[0], &eff[0]);
  hardware_interface::JointStateHandle state_handle_right_elbow(   "right_elbow",    &pos[1], &vel[1], &eff[1]);

  hardware_interface::JointStateHandle state_handle_left_shoulder( "left_shoulder",  &pos[3], &vel[3], &eff[3]);
  hardware_interface::JointStateHandle state_handle_left_elbow(    "left_elbow",     &pos[4], &vel[4], &eff[4]);
  hardware_interface::JointStateHandle state_handle_reaction_wheel("reaction_wheel_joint", &pos[5], &vel[5], &eff[5]);

  jnt_state_interface.registerHandle(state_handle_left_shoulder);
  jnt_state_interface.registerHandle(state_handle_left_elbow);
  jnt_state_interface.registerHandle(state_handle_right_shoulder);
  jnt_state_interface.registerHandle(state_handle_right_elbow);
  jnt_state_interface.registerHandle(state_handle_reaction_wheel);

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interface
  hardware_interface::JointHandle effort_handle_right_shoulder(jnt_state_interface.getHandle("right_shoulder"),&cmd[0]);
  hardware_interface::JointHandle effort_handle_right_elbow(   jnt_state_interface.getHandle("right_elbow"),   &cmd[1]);

  hardware_interface::JointHandle effort_handle_left_shoulder( jnt_state_interface.getHandle("left_shoulder"), &cmd[3]);
  hardware_interface::JointHandle effort_handle_left_elbow(    jnt_state_interface.getHandle("left_elbow"),    &cmd[4]);
  hardware_interface::JointHandle effort_handle_reaction_wheel(jnt_state_interface.getHandle("reaction_wheel_joint"), &cmd[5]);

  jnt_eff_interface.registerHandle(effort_handle_left_shoulder);
  jnt_eff_interface.registerHandle(effort_handle_left_elbow);
  jnt_eff_interface.registerHandle(effort_handle_right_shoulder);
  jnt_eff_interface.registerHandle(effort_handle_right_elbow);
  jnt_eff_interface.registerHandle(effort_handle_reaction_wheel);

  registerInterface(&jnt_eff_interface);



  //ENCODER CARD


  // TRY ACCESS CARDS
  uint32_t minor_number = 0;

  dm7820_status = DM7820_General_Open_Board(minor_number, &board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");


//INCREMENTAL ENCODERS 0 & 1
  //disable encoders for safety
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0x00); 
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

  //Set each port 0 bit to input which enables incremental encoder inputs
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_INPUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  //Set each port 1 bit to input which enables incremental encoder inputs
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF, DM7820_STDIO_MODE_INPUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  //Set master clock to 25 MHz clock
  //Incremental encoder 0 initialization
  dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");
  //Incremental encoder 1 initialization
  dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

  //Disable value register hold 0
  dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

  //Disable value register hold 1
  dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

  /*
   * Configure the incremental encoder as follows: 1) disable all up
   * transitions from modifying the counter, 2) set single-ended input mode,
   * 3) disable the input filter, 4) set channels A and B to operate
   * independently of each other, and 5) disable index input
   */
  DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);

  dm7820_status = DM7820_IncEnc_Configure(board,
            DM7820_INCENC_ENCODER_0,
            phase_filter,
            DM7820_INCENC_INPUT_SINGLE_ENDED,
            0x00,
            DM7820_INCENC_CHANNEL_INDEPENDENT,
            0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_0,
                  DM7820_INCENC_CHANNEL_A,
                  0x0000);
  DM7820_Return_Status(dm7820_status,
           "DM7820_IncEnc_Set_Independent_Value()");

  //Set initial value for channel B counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_0,
                  DM7820_INCENC_CHANNEL_B,
                  0x1000);
  DM7820_Return_Status(dm7820_status,
           "DM7820_IncEnc_Set_Independent_Value()");



  /*
   * Configure the incremental encoder as follows: 1) disable all down
   * transitions from modifying the counter, 2) set single-ended input mode,
   * 3) disable the input filter, 4) set channels A and B to operate
   * independently of each other, and 5) disable index input
   */

  fprintf(stdout, "    Configuring encoder ...\n");

  DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
  DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION(phase_filter, DM7820_INCENC_PHASE_BA_01_TO_00_DOWN);
  DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION(phase_filter, DM7820_INCENC_PHASE_BA_11_TO_01_DOWN);
  DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION(phase_filter, DM7820_INCENC_PHASE_BA_10_TO_11_DOWN);
  DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION(phase_filter, DM7820_INCENC_PHASE_BA_00_TO_10_DOWN);

  dm7820_status = DM7820_IncEnc_Configure(board,
            DM7820_INCENC_ENCODER_1,
            phase_filter,
            DM7820_INCENC_INPUT_SINGLE_ENDED,
            0x00,
            DM7820_INCENC_CHANNEL_INDEPENDENT,
            0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_1,
                  DM7820_INCENC_CHANNEL_A,
                  0x8000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_1,
                  DM7820_INCENC_CHANNEL_A,
                  0x9000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");

  //Secondary incremental encoder 0 initialization
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

  //Secondary incremental encoder 1 initialization
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");


  encoder_1_ovf = 0;
  encoder_2_ovf = 0;
  encoder_3_ovf = 0;
  encoder_4_ovf = 0;
  encoder_5_ovf = 0;
  encoder_6_ovf = 0;

  uint16_t encoder_init_value = 0;

  encoder_1 = encoder_init_value;
  encoder_2 = encoder_init_value;
  encoder_3 = encoder_init_value;
  encoder_4 = encoder_init_value;
  encoder_5 = encoder_init_value;
  encoder_6 = encoder_init_value;

  encoder_1_old = encoder_init_value;
  encoder_2_old = encoder_init_value;
  encoder_3_old = encoder_init_value;
  encoder_4_old = encoder_init_value;
  encoder_5_old = encoder_init_value;
  encoder_6_old = encoder_init_value;

  for(int i=0; i<6; i++)
  {
    pos[i] = 0;
    prev_pos[i] = pos[i];
    offset_pos[i] = 0;
  }

  //device opened above

  //PWM Init
  //Disable all pulse width modulators to put them into a known state; any
  //pulse width modulator should be disabled before programming it
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

  // Set each port 2 bit to peripheral output
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

  //Set each port 2 bit to output pulse width modulator peripheral
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_PERIPH_PWM);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");
  
  //Set period master clock to 25 MHz clock
  dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_PERIOD_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

  // Set pulse width modulator period to obtain 100 kHz frequency
  dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 250);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

  //Set width master clock to 25 MHz clock
  dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_WIDTH_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

  //zero out all pwms
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, 50);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B, 100);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C, 150);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D, 200);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  //enable PWM 0
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

//PWM 1
  dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_PERIOD_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");
  
  dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_1, 250);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

  dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_WIDTH_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

  //zero out 
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, 50);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, 100);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
 
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, 150);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, 200);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  //PWM 1 enable
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");
}

// CepheusHW::~CepheusHW() 
// { 
//   dm7820_status = DM7820_General_Close_Board(board);
//   DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");
// }

