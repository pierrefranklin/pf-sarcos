/*!=============================================================================
 ==============================================================================

 \file    valve_controller.cpp

 \author  righetti
 \date    Jul 27, 2012

 ==============================================================================
 \remarks


 ============================================================================*/

#include <valve_controller.h>
#include <utility_macros.h>
#include <SL.h>
#include <SL_common.h>
#include <SL_user.h>
#include <SL_motor_servo.h>
#include <SL_collect_data.h>
#include <iostream>
#include "SL_sensor_proc.h"
#include "SL_filters.h" 


ValveController::ValveController()
{
  proportional_gains_.resize(N_DOFS+1,0);
  integral_gains_.resize(N_DOFS+1,0);
  derivative_gains_.resize(N_DOFS+1,0);
  theta_proportional_gains_.resize(N_DOFS+1,0);
  theta_der_gains_.resize(N_DOFS+1,0);

  invert_valve_SL_.resize(N_DOFS+1,0);
  
  previous_error_.resize(N_DOFS+1,0);
  integral_error_.resize(N_DOFS+1,0);

  velocity_comp_gains_.resize(N_DOFS+1,0);
  
  k_torqued_d_.resize(N_DOFS+1,0);
  integral_error_time_window_.resize(N_DOFS+1, 1.0);
  integral_error_saturation_.resize(N_DOFS+1, 0.0);
  integral_control_saturation_.resize(N_DOFS+1, false);


  valve_command_saturation_.resize(N_DOFS+1, 0.0);
  valve_bias_.resize(N_DOFS+1, 0.0);

  //collect some data
  for(int i=1; i<=N_DOFS; ++i)
  {
    char var_name[20];
    sprintf(var_name, "%s_T_int_err", joint_names[i]);
    addVarToCollect((char *)&(integral_error_[i]), var_name, "-", DOUBLE, 1);
  }
   /*char var_name[20];
		sprintf(var_name, "velocity_comp");                                                        
    addVarToCollect((char*) &(tmp1), var_name, "xx", DOUBLE, 1); 
		*/
}

ValveController::~ValveController()
{
}

bool ValveController::initialize()
{

  int i, j;

  //load the parameters from config file
  double tmp_values[num_controller_params_+1];
  for (i=1; i<= n_dofs; ++i)
  {
    if(!read_parameter_pool_double_array("ValveController.cf", joint_names[i], num_controller_params_, tmp_values))
    {
      printf("Error cannot read valve control parameters for DOF %s\n", joint_names[i]);
      return false;
    }
    proportional_gains_[i] = tmp_values[1];
    derivative_gains_[i] = tmp_values[2];
    integral_gains_[i] = tmp_values[3];
    integral_error_saturation_[i] = tmp_values[4];
    integral_error_time_window_[i] = tmp_values[5];
    velocity_comp_gains_[i] = tmp_values[6];
    valve_command_saturation_[i] = tmp_values[7];
    valve_bias_[i] = tmp_values[8];
    printf("Proportional gain initialized for joint = %d and is = %f", i, tmp_values[11]);
    theta_proportional_gains_[i] = tmp_values[9];
    theta_der_gains_[i] = tmp_values[10];
		invert_valve_SL_[i] = tmp_values[11];
		k_torqued_d_[i] = tmp_values[12];

    safetyChecks(i);
  }

  return true;
}

void ValveController::setGains(int joint_num, double p_gain, double d_gain, double i_gain, double theta_p_gain, double theta_d_gain, double k_torque, int invert_valve_SL)
{
  if(joint_num<1 || joint_num >N_DOFS)
    printf("ERROR - cannot change gains of joint %d. Wrong index\n", joint_num);

  proportional_gains_[joint_num] = p_gain;
  derivative_gains_[joint_num] = d_gain;
  integral_gains_[joint_num] = i_gain;
  theta_proportional_gains_[joint_num] = theta_p_gain;
  theta_der_gains_[joint_num] = theta_d_gain;
  k_torqued_d_[joint_num] = k_torque;
  invert_valve_SL_[joint_num] = invert_valve_SL;
  printf(" p gain = %f d gain = %f %d \n", theta_proportional_gains_[joint_num], theta_der_gains_[joint_num], joint_num);
  safetyChecks(joint_num);
}

void ValveController::getGains(int joint_num, double& p_gain, double& d_gain, double& i_gain, double& theta_p_gain, double& theta_d_gain, double& k_torqued_d, int& invert_valve_SL)
{
  p_gain = proportional_gains_[joint_num];
  i_gain = integral_gains_[joint_num];
  d_gain = derivative_gains_[joint_num];
  theta_p_gain = theta_proportional_gains_[joint_num];
  theta_d_gain = theta_der_gains_[joint_num];
  k_torqued_d = k_torqued_d_[joint_num];
  invert_valve_SL = invert_valve_SL_[joint_num];
}

bool ValveController::getIntegralSaturation(int joint_num)
{
  if(joint_num<1 || joint_num >N_DOFS)
    printf("ERROR - get integral sat, wrong joint number %d. Wrong index\n", joint_num);

  return integral_control_saturation_[joint_num];
}

void ValveController::getValveBias(int joint_num, double& bias)
{
  bias = valve_bias_[joint_num];
}

void ValveController::setValveBias(int joint_num, double bias)
{
  if(joint_num<1 || joint_num>N_DOFS)
    printf("ERROR - cannot change valve bias of joint %d. Wrong index\n",joint_num);
  valve_bias_[joint_num] = bias;
}

void ValveController::getValveSaturation(int joint_num, double& valve_saturation)
{
  valve_saturation = valve_command_saturation_[joint_num];
}

void ValveController::setValveSaturation(int joint_num, double valve_saturation)
{
  if(joint_num<1 || joint_num >N_DOFS)
    printf("ERROR - cannot change valve sat. of joint %d. Wrong index\n", joint_num);

  valve_command_saturation_[joint_num] = valve_saturation;
}

void ValveController::getVelocityCompGains(int joint_num, double& vel_comp_gain)
{
  vel_comp_gain = velocity_comp_gains_[joint_num];
}

void ValveController::setVelocityCompGains(int joint_num, double vel_comp_gain)
{
  if(joint_num<1 || joint_num >N_DOFS)
    printf("ERROR - cannot change velocity compensation gains of joint %d. Wrong index\n", joint_num);

  velocity_comp_gains_[joint_num] = vel_comp_gain;
}

void ValveController::getIntegralParameters(int joint_num, double& time_window, double& saturation)
{
  time_window = integral_error_time_window_[joint_num];
  saturation = integral_error_saturation_[joint_num];
}

void ValveController::setIntegralParameters(int joint_num, double time_window, double saturation)
{
  if(joint_num<1 || joint_num >N_DOFS)
    printf("ERROR - cannot change integral params of joint %d. Wrong index\n", joint_num);

  integral_error_saturation_[joint_num] = saturation;
  integral_error_time_window_[joint_num] = time_window;
}

void ValveController::safetyChecks(int joint_num)
{
  //error checking (if time_window == 1/motor_servo_rate then forgetting is unstable)
  if(integral_error_time_window_[joint_num] < 1/double(motor_servo_rate))
  {
    printf("Warning - integral time window is too low %f - disabling integral control for joint %d\n",
           integral_error_time_window_[joint_num], joint_num);

    integral_error_time_window_[joint_num] = 1/double(motor_servo_rate);
    integral_gains_[joint_num] = 0.0;
    integral_error_saturation_[joint_num] = 0.0;
  }

  if(integral_error_saturation_[joint_num] < 0.0)
  {
    printf("Warning - integral error saturation %f must be > 0.0 - disabling integral control for joint %d\n",
           integral_error_saturation_[joint_num], joint_num);

    integral_error_time_window_[joint_num] = 1/double(motor_servo_rate);
    integral_gains_[joint_num] = 0.0;
    integral_error_saturation_[joint_num] = 0.0;
  }

  if(valve_command_saturation_[joint_num] < 0.0)
  {
    printf("Warning - valve command saturation %f must be > 0.0 - disabling valve control for joint %d\n",
           valve_command_saturation_[joint_num], joint_num);

    valve_command_saturation_[joint_num] = 0.0;
  }
}

bool ValveController::computeValveCommands(std::vector<hermes_communication_tools::GDCState>& gdc_states,   GDCSLInterface& gdc_sl_interface, SL_Jstate* j_state)
{
  for(int i=0; i<(int)gdc_states.size(); ++i)
  {

    //get the dof number
    int dof_num = gdc_states[i].dof_number_;


    //compute the error
    //double error = double(gdc_states[i].desired_torque_ - gdc_states[i].actual_torque_);
    double error = double(joint_state[dof_num].u - joint_state[dof_num].load); 

    //double d_error = (error - previous_error_[dof_num])*double(motor_servo_rate);
    double d_error = (error - previous_error_[dof_num])*double(motor_servo_rate);


    //compute the position error
    //double error_pos = double(gdc_states[i].desired_position_ - gdc_states[i].actual_position_);
    //double error_vel = (error_pos - previous_error_[dof_num])*double(motor_servo_rate);
    //double error_pos = (joint_des_state[dof_num].th - joint_state[dof_num].th);
    //double error_vel = (joint_des_state[dof_num].thd - joint_state[dof_num].thd);

    //filtering
    //double error_velf = error_vel; //filt(error_vel, &fthd[i]);


    //integral forgetting
    /*integral_error_[dof_num] -= integral_error_[dof_num]/(integral_error_time_window_[dof_num]*double(motor_servo_rate));

    //increase integral error (times the integral gains)
    integral_error_[dof_num] += integral_gains_[dof_num] * error;

    //integral error saturation
    integral_control_saturation_[dof_num] = false;
    if(integral_error_[dof_num] > integral_error_saturation_[dof_num])
    {
      integral_error_[dof_num] = integral_error_saturation_[dof_num];
      integral_control_saturation_[dof_num] = true;
    }
    else if(integral_error_[dof_num] < -integral_error_saturation_[dof_num])
    {
      integral_error_[dof_num] = -integral_error_saturation_[dof_num];
      integral_control_saturation_[dof_num] = true;
    }

    //save the previous error
    previous_error_[dof_num] = error;*/


    //compute the command
    double command = 0.0;

    //compute the PID command - without the integral
    command += proportional_gains_[dof_num] * error + derivative_gains_[dof_num] * d_error;
    //command += theta_proportional_gains_[dof_num] * error_pos + theta_der_gains_[dof_num] * error_vel;//+ integral_error_[dof_num];

	//if(dof_num == 18) printf("feedforward valve, %f \n", k_torqued_d_[dof_num]*joint_des_state[dof_num].vff);
	//command += k_torqued_d_[dof_num]*joint_des_state[dof_num].vff;

    //add the velocity compensation
    double vel;
    if(!gdc_sl_interface.computePistonVelocity(dof_num, j_state[dof_num].th, j_state[dof_num].thd, vel))
    {
      printf("Error computing velocity forward gains for joint %d\n", dof_num);
      return false;
    }
    command += velocity_comp_gains_[dof_num] * vel;

    //if(dof_num == 18) tmp1 = velocity_comp_gains_[dof_num] * vel;

    //add the valve offset
    command += valve_bias_[dof_num];

    //if(dof_num == 18) tmp2 = valve_bias_[dof_num];  
    
    //test
    //command = 1000;
    //add theta position proportional error


    //command += theta_proportional_gains_[dof_num] * error_pos + theta_der_gains_[dof_num] * error_vel; // + proportional_gains_[dof_num] * gdc_states[i].actual_torque_;
     //if(dof_num == 18) tmp3 = theta_proportional_gains_[dof_num] * error_pos + theta_der_gains_[dof_num] * error_vel;     

    command = invert_valve_SL_[dof_num]*command;


    //valve command saturation
    if(command > valve_command_saturation_[dof_num])
      command = valve_command_saturation_[dof_num];
    else if(command < -valve_command_saturation_[dof_num])
      command = -valve_command_saturation_[dof_num];

    gdc_states[i].desired_valve_command_ = int16_t(command);
  }

  return true;
}


