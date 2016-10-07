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


void CepheusHW::setThrustPwm(double *thrust, double min_duty, double max_duty)
{
  double duty[4];
  uint16_t dir[4];
  uint16_t port_2;

  for(int i=0; i<4; i++)
  {
    // Calculate the correct duty cycle 
    // and the pin to output the pulse
    if(thrust[i] >= 0)
    {
      if (thrust[i] < max_thrust)
        duty[i] = thrust[i]/max_thrust;
      else
        duty[i] = 1;
      //select the non-inverted pin
      dir[i] = 1;
    }
    else
    {
      //0 is  the max duty for inverted   
      if (thrust[i] > -max_thrust)
        duty[i] = 1 + (thrust[i]/max_thrust);
      else
        duty[i] = 0;
      //select the inverted pin
      dir[i] = 2;
    }

    //ensure that PWM is inside the limit 0-1 and has deadband areas removed
    if (dir[i] == 1)
    {
      if (duty[i] > max_duty) 
        duty[i]=1;
      else if (duty[i] < min_duty) 
        duty[i]=0;
    }
    else if (dir[i] == 2)
    {
      if ( (1 - duty[i]) > max_duty )
        duty[i]=0;
      else if ( (1 - duty[i]) < min_duty ) 
        duty[i]=1;
    }
    else
      ROS_INFO("PWM thrust calculation error");
  }

  //Seting which pin of the inverted and non inverted pin of a PWM will be used
  uint16_t output = dir[0]<<0 | dir[1]<<2 | dir[2]<<4 | dir[3]<<6;

  // dm7820_status = DM7820_StdIO_Get_Input(board, DM7820_STDIO_PORT_2, &port_2);
  // DM7820_Return_Status(dm7820_status, "DM7820_StdIO_get_port_state");
  // ROS_INFO("port 2: %x", port_2);
  //dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
  //DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");

  // Set port's 2 // bits to peripheral output
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, (output<<8), DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
  //Set port's 2 bits to PWM output pulse width modulator peripheral
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, (output<<8), DM7820_STDIO_PERIPH_PWM);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

  //Reset port's 2 bits to STD IO bits in order not to act like an PWM
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, ((~output)<<8), DM7820_STDIO_MODE_OUTPUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");


  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, (uint32_t)(duty[0]*period) );
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, (uint32_t)(duty[1]*period) );
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, (uint32_t)(duty[2]*period) );
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, (uint32_t)(duty[3]*period) );
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
}

void CepheusHW::writeMotors()
{
  uint16_t dir[4];
  uint16_t width[4];


  for (int i=0; i<4; i++)
  {
    //ROS_INFO("max_cuurent[%d]=%f", i, max_current[i]);
    double current = (cmd[i]/0.0538);//cmd is in Nm
    //saturate to max current
    if (current >= max_current[i]) current = max_current[i];
    if (current <=-max_current[i]) current =-max_current[i];
    //ROS_INFO("current %f", current);

    eff[i] = current*0.0538;//eff is in Nm


    if (current >= 0.0) 
    {
      dir[i] = 0;
      width[i] = (uint16_t)(current*(PWM_DIVIDER/max_current[i]));
    }
    else
    {
      dir[i] = 1;
      width[i] = (uint16_t)(-current*(PWM_DIVIDER/max_current[i]));
    }
  }

  //Seting direction (pins for Port 2)
  uint16_t directions = (dir[0]<<1) | (dir[1]<<3) | (dir[2]<<5) | (dir[3]<<7);

  dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, directions);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_motor_direction");

  // Set output PWM width
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A,  width[0]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[0]");   
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B,  width[1]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[1]");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C,  width[2]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[2]");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D,  width[3]);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_motor_Width[3]");
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
    DM7820_INCENC_CHANNEL_B,
    &encoder_2_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //Read encoder 1 channel A value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
    DM7820_INCENC_CHANNEL_A,
    &encoder_3_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //Read encoder 1 channel B value
  dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1,
    DM7820_INCENC_CHANNEL_B,
    &encoder_4_val);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");

  //handle overflow
  // Channel 0A Handling of encoder value overflow
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0A");
  if (encoder_status) 
    encoder_1_ovf++;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0A");
  if (encoder_status) 
    encoder_1_ovf--;

  encoder_1 = (int)encoder_1_val + 65535*encoder_1_ovf;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0A");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 0A positive overflow status not cleared");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0A");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 0A negative overflow status not cleared");


  // Channel 0B Handling of encoder value overflow
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read positive overflow channel 0B");
  if (encoder_status) 
    encoder_2_ovf++;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read negative overflow channel 0B");
  if (encoder_status) 
    encoder_2_ovf--;

  encoder_2 = (int)encoder_2_val + 65535*encoder_2_ovf;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 0B");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 0B positive overflow status not cleared");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "negative overflow clear channel 0B");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 0B negative overflow status not cleared");


  // Channel 1A Handling of encoder value overflow
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1A");
  if (encoder_status) 
    encoder_3_ovf++;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1A");
  if (encoder_status) 
    encoder_3_ovf--;

  encoder_3 = (int)encoder_3_val + 65535*encoder_3_ovf;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1A");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 1A positive overflow status not cleared");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1A");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 1A negative overflow status not cleared");


  // Channel 1B Handling of encoder value overflow
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read positive overflow channel 1B");
  if (encoder_status) 
    encoder_4_ovf++;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Read negative overflow channel 1B");
  if (encoder_status) 
    encoder_4_ovf--;

  encoder_4 = (int)encoder_4_val + 65535*encoder_4_ovf;

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "Positive overflow clear channel 1B");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 1B positive overflow status not cleared");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "negative overflow clear channel 1B");
  if (encoder_status)  
    error(EXIT_FAILURE, 0, "ERROR: Channel 1B negative overflow status not cleared");


  // ROS_DEBUG("1: %d, 2: %d, 3: %d, 4: %d", encoder_1, encoder_2, encoder_3, encoder_4);

  // Position Calculation radians
  pos[0]=  (double)encoder_1*2*M_PI/(4095) - offset_pos[0];
  pos[1]=  (double)encoder_2*2*M_PI/(4095) - offset_pos[1];
  pos[2]=  (double)encoder_3*2*M_PI/(4095) - offset_pos[2];
  pos[3]=  (double)encoder_4*2*M_PI/(4095) - offset_pos[3];

  //ROS_INFO("POS: 1: %f, 2: %f, 3: %f, 4: %f", pos[0], pos[1], pos[2], pos[3]);

  // Speed Calculation radians/sec
  for(int i=0; i<4; i++)
  {
    vel_new[i]= ((pos[i] - prev_pos[i])) / dt.toSec();
    prev_pos[i] = pos[i];

     // vel[0] = vel_new[0]; 
    for (int j=0; j<(FIR_LENGTH-1); j++)
      vel_fir[j][i] = vel_fir[j+1][i]; 

    vel_fir[FIR_LENGTH-1][i] = vel_new[i];

    double filtered=0;
    for (int j=0; j<FIR_LENGTH; j++)
      filtered += vel_fir[j][i];

    vel[i] = filtered/FIR_LENGTH;

  }

  // ROS_INFO("VEL: 1: %f, new: %f", vel[0], vel_new[0]);
}

void CepheusHW::setParam(double max_cur, double f_thrust) 
{
  this->max_thrust = f_thrust;
  for(int i; i<4; i++) {
    this->max_current[i] = max_cur;
    ROS_INFO("max_current[%d]=%f", i, max_current[i] );
  }
  ROS_INFO("max_currents Setted");
}

void CepheusHW::setCmd(int idx, double _cmd)
{
  cmd[idx] = _cmd;
}

double CepheusHW::getVel(int idx)
{
  return vel[idx];
}

CepheusHW::CepheusHW() 
{ 
  int16_t encoder_init_value = 0;

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_reaction_wheel("reaction_wheel_joint", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_reaction_wheel);

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interface
  hardware_interface::JointHandle effort_handle_reaction_wheel(jnt_state_interface.getHandle("reaction_wheel_joint"), &cmd[0]);

  jnt_eff_interface.registerHandle(effort_handle_reaction_wheel);

  registerInterface(&jnt_eff_interface);



  //ENCODER CARD
  // TRY ACCESS CARDS
  uint32_t minor_number = 0;

  dm7820_status = DM7820_General_Open_Board(minor_number, &board);
  DM7820_Return_Status(dm7820_status, "Opening Device 0");


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
            DM7820_INCENC_INPUT_DIFFERENTIAL,
            0x00,
            DM7820_INCENC_CHANNEL_INDEPENDENT,
            0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_0,
                  DM7820_INCENC_CHANNEL_A,
                  encoder_init_value);
  DM7820_Return_Status(dm7820_status,
           "DM7820_IncEnc_Set_Independent_Value_0_A");

  //Set initial value for channel B counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_0,
                  DM7820_INCENC_CHANNEL_B,
                  encoder_init_value);
  DM7820_Return_Status(dm7820_status,
           "DM7820_IncEnc_Set_Independent_Value_0_B");



  /*
   * Configure the incremental encoder as follows: 1) disable all down
   * transitions from modifying the counter, 2) set single-ended input mode,
   * 3) disable the input filter, 4) set channels A and B to operate
   * independently of each other, and 5) disable index input
   */
  DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
  dm7820_status = DM7820_IncEnc_Configure(board,
            DM7820_INCENC_ENCODER_1,
            phase_filter,
            DM7820_INCENC_INPUT_DIFFERENTIAL,
            0x00,
            DM7820_INCENC_CHANNEL_INDEPENDENT,
            0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_1,
                  DM7820_INCENC_CHANNEL_A,
                  encoder_init_value);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_A");

  //Set initial value for channel A counter
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
                  DM7820_INCENC_ENCODER_1,
                  DM7820_INCENC_CHANNEL_B,
                  encoder_init_value);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value_1_B");

  //incremental encoder 0 enable again
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

  //incremental encoder 1 enable again
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

  //Clear channel negative and positive rollover status flag without checking its state
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");
  dm7820_status = DM7820_IncEnc_Get_Status(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER, &encoder_status);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

  encoder_1_ovf = 0;
  encoder_2_ovf = 0;
  encoder_3_ovf = 0;
  encoder_4_ovf = 0;

  encoder_1 = encoder_init_value;
  encoder_2 = encoder_init_value;
  encoder_3 = encoder_init_value;
  encoder_4 = encoder_init_value;

  //initialize prev_pos, offset_pos and filter buffer
  for(int i=0; i<4; i++)
  {
    pos[i] = 0;
    prev_pos[i] = pos[i];
    offset_pos[i] = 0;

    //initialize FIR filter
    for(int j=0; j<FIR_LENGTH; j++)
      vel_fir[j][i]=0;
  }



  //device opened above

  /******************************************************************
                          motor PWM Init
  *******************************************************************/
  //Disable pulse width modulators 0 to put them into a known state.
  //pulse width modulator should be disabled before programming it
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_0_Disable()");
 
  // Set port's 2 (0,2,4,6) bits to peripheral output. For using as current control signal for motro controller
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_Set port2 (0,2,4,6) bits to peripheral output");

  //Set port's 2 (0,2,4,6) bits to PWM output pulse width modulator peripheral. For using as current control signal for motro controller
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0x0055, DM7820_STDIO_PERIPH_PWM);
  DM7820_Return_Status(dm7820_status, "Set port2 (0,2,4,6) bits to PWM output");
  
  //set Port's 2 (1,3,5,7) bits to standard output for using it as direction signal of motor controller
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x00AA, DM7820_STDIO_MODE_OUTPUT);
  DM7820_Return_Status(dm7820_status, "Set Port's 2 (1,3,5,7) bits to standard output");

  //Set period master clock to 25 MHz clock
  dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_PERIOD_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

  // Set pulse width modulator period to obtain frequency 25000000/PWM_DIVIDER Hz
  dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, PWM_DIVIDER);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

  //Set width master clock to 25 MHz clock
  dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_WIDTH_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

  //zero out all pwms
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_B, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_C, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_D, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

  //enable PWM 0
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

  /*************************************************************************
                          THRUSTER CLOCK/PWM SETUP
   ************************************************************************/

  /***Programmable clock 0 initialization***/

  //Disable PRGmble clock 0 
  dm7820_status = DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_DISABLED);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");
  //maybe output init
  //Set master clock to 25 MHz clock
  dm7820_status = DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");
  //Set clock start trigger to start immediately
  dm7820_status = DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_START_IMMEDIATE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Start_Trigger()");
  //Set clock stop trigger so that clock is never stopped
  dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_STOP_NONE);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");
  //Set clock period to obtain 50000 Hz frequency
  dm7820_status = DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 500);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");
  //Put clock into continuous mode and enable it
  dm7820_status = DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0, DM7820_PRGCLK_MODE_CONTINUOUS);
  DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");


  /****Thruster PWM 1 Init****/

  //Disable pulse width modulator 1 to put them into a known state
  //pulse width modulator should be disabled before programming it
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Disable()");

  // Set port's 2 (8,9,10,11,12,13,14,15) bits to output.
  //this bits will used as PWM output but will be selected dynamicaly in the loop depending on the thrust direction
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x0FF00, DM7820_STDIO_MODE_OUTPUT);
  DM7820_Return_Status(dm7820_status, "Set port2 (8,9,10,11,12,13,14,15) bits to standard output");

  //Set period master clock to programmable clock 0 with freq 500 hz
  dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Period_Master()");

  // Set pulse width modulator period to obtain frequency 500/freq Hz
  period = (uint32_t)(50000/10);
  dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_1, period);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Period()");

  //Set width master clock to programmable clock 0 with freq 500 hz
  dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_Width_Master()");

  // Set thrusters OFF initially
  dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_SET_THRUSTER_OFF()");

  //zero out all pwms
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_A, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_B, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_C, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");
  dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1, DM7820_PWM_OUTPUT_D, 0);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Set_0");

  //enable PWM 1
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_THRUSTER_PWM_Enable()");

  /*************************************************************************
                          GENERALSETUP
   ************************************************************************/
  count=1;
  output_value=0;
}

void CepheusHW::safeClose() 
{ 
  double thrust[4];

  for(int i=0;i<4;i++) 
  {
    cmd[i]=0;
    thrust[i]=0;
  }
  writeMotors();  
  setThrustPwm(thrust, 0.05, 0.95);

  dm7820_status = DM7820_General_Close_Board(board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");
  
  ROS_WARN("Hardware safely closed");
}

