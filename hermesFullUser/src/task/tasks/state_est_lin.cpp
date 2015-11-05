/*
 * state_est_lin.cpp
 *
 *  Created on: Nov 4, 2015
 *      Author: perry
 */

#include <Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

// system headers
#include "SL_system_headers.h"

/* SL includes */
#include "SL.h"
#include "SL_user.h"
#include "SL_tasks.h"
#include "SL_task_servo.h"
#include "SL_kinematics.h"
#include "SL_dynamics.h"
#include "SL_collect_data.h"
#include "SL_shared_memory.h"
#include "SL_man.h"

const Eigen::Matrix<double,3,1> _gravity(0,0,-9.81);
Eigen::Matrix<double,3,1> _imu_lin_offset(0, 0, 0);
Eigen::Quaterniond _imu_quat_offset(1, 0, 0, 0);

static const int X0 = 0;
static const int V0 = 3;
// Filter Params
int zDim;
Eigen::Matrix<double,6,1> _x;
Eigen::Matrix<double,6,1> _z;
Eigen::Matrix<double,6,1> _innov;

Eigen::Matrix<double,6,6> _P;
Eigen::Matrix<double,6,6> _Q;
Eigen::Matrix<double,6,6> _R;
Eigen::Matrix<double,6,6> _K;
Eigen::Matrix<double,6,6> _S;

Eigen::Matrix<double,6,6> _A;
Eigen::Matrix<double,6,6> _C;
Eigen::Matrix<double,6,1> _dx;
// Input
Eigen::Quaternion<double> _q;
Eigen::Matrix<double,3,1> _a; // Acceleration as input from IMU ekf or raw data
Eigen::Matrix<double,3,1> _w; // Angular velocity as input IMU ekf or raw data
Eigen::Matrix<double,3,1> _vimu;
Eigen::Matrix<double,50,1> _joints;
Eigen::Matrix<double,50,1> _jointsd;
//Flag
int _toe_flag; // Toe off flag from controller, reset reference point.
int _previous_toe_flag; // Toe off flag from controller, reset reference point.
int _contact_state;
int _contact_state_full;
int _previous_contact_state; // Previous contact state
// int _contact_timer;
bool buffer_first_time;
bool contact_change_flag;
// ComputeFootstuff
Eigen::Matrix<double,3,1> foot_pos[2]; // Contact point of foot
Eigen::Matrix<double,3,1> foot_vel[2]; // Velocity of contact point
Eigen::Matrix<double,3,1> foot_toe_pos[2]; // Position of toe
Eigen::Matrix<double,3,1> foot_toe_vel[2]; // Velocity of toe
Eigen::Matrix<double,12,1> foot_registered; // registered foot position/velocity, used as measurement
// Weight distribution between left and right leg
double _beta;

const static double timestep = 0.001;

/*-----------------------------------------------------------------------------------------*/
void makeProcessJacobian ()
{
  _A.setIdentity();
  _A.block(0,3,3,3).setIdentity();
  _A.block(0,3,3,3) *= timestep;
}

void makeObservationJacobian ()
{
  _C.setIdentity();
}

void dare(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &B, Eigen::Matrix<double,6,6> &P,int zDim = 6)
{
  Eigen::Matrix<double,6,6> Ainv = A.inverse();
  Eigen::Matrix<double,6,6> ABRB;
  if (zDim == 6)
  {
    ABRB = Ainv * B * _R.llt().solve(B.transpose());
  }
  else{
    ABRB = Ainv * B.topLeftCorner(6,zDim) * _R.topLeftCorner(zDim,zDim).llt().solve(B.topLeftCorner(6,zDim).transpose());
  }
  Eigen::Matrix<double,2*6,2*6> Z;
  Z.block(0,0,6,6) = Ainv;
  Z.block(0,6,6,6) = ABRB;
  Z.block(6,0,6,6) = _Q * Ainv;
  Z.block(6,6,6,6) = A.transpose() + _Q * ABRB;

  Eigen::ComplexEigenSolver <Eigen::Matrix<double,2*6,2*6> > ces;
  ces.compute(Z);

  Eigen::Matrix<std::complex<double>,2*6,1> eigVal = ces.eigenvalues();
  Eigen::Matrix<std::complex<double>,2*6,2*6> eigVec = ces.eigenvectors();

  Eigen::Matrix<std::complex<double>,2*6,6> unstableEigVec;

  int ctr = 0;
  for (int i = 0; i < 2*6; i++) {
    if (eigVal(i).real()*eigVal(i).real() + eigVal(i).imag()*eigVal(i).imag() > 1) {
      unstableEigVec.col(ctr) = eigVec.col(i);
      ctr++;
      if (ctr > 6)
        break;
    }
  }

  Eigen::Matrix<std::complex<double>,6,6> U21inv = unstableEigVec.block(0,0,6,6).inverse();
  Eigen::Matrix<std::complex<double>,6,6> PP = unstableEigVec.block(6,0,6,6) * U21inv;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P(i,j) = PP(i,j).real();
    }
  }
}

void computeKss(const Eigen::Matrix<double,6,6> &A, const Eigen::Matrix<double,6,6> &C, int zDim = 6)
{

  Eigen::Matrix<double,6,6> B = C.transpose();
  // Eigen::Matrix<double,6,6> P;
  dare(A.transpose(), B, _P, zDim); // A^T is used here
  _K.setZero();
  Eigen::Matrix<double,6,6> PB;
  Eigen::Matrix<double,6,6> BtPB_R;
  if (zDim == 6){
    PB = _P * B;
    BtPB_R = B.transpose() * PB + _R;
    _K = PB* BtPB_R.inverse()
;  }
  else
  {
    PB.topLeftCorner(6,zDim) = _P * B.topLeftCorner(6,zDim);
    BtPB_R.topLeftCorner(zDim,zDim) = B.topLeftCorner(6,zDim).transpose() * PB.topLeftCorner(6,zDim) + _R.topLeftCorner(zDim,zDim);
    _K.topLeftCorner(6,zDim) = PB.topLeftCorner(6,zDim)* BtPB_R.topLeftCorner(zDim,zDim).inverse();
  }
}

void setKss()
{
  makeProcessJacobian();
  makeObservationJacobian();
  //setQ(DSc_Q);
  //setR(DSc_R);
  computeKss(_A,_C, 6);
}

void computeAnkleRelated()
{
  int joint[2] = {PELV_S_JOINT_L_LEG_LAX, PELV_S_JOINT_R_LEG_LAX};
  int body[2] = {PELV_S_BODY_L_FOOT, PELV_S_BODY_R_FOOT};

  rs->makeSDFastState(Pose(_x.block(X0,0,3,1), _q), _x.block(V0,0,3,1), _w, _joints, _jointsd, sdFastState);
  set_state(rs->model, sdFastState.data());

  double com_to_ankle[3];
  double com_to_ref[3]; // Com to reference point.

  for (int side = 0; side < 2; side++) {
    rs->model->sdgetbtj(joint[side], com_to_ankle);

    for (int i = 0; i < 3; i++)
      com_to_ref[i] = com_to_ankle[i] + Foot::ankle_to_toe_offset[i];
    rs->model->sdpos(body[side], com_to_ref, foot_pos[side].data());
    rs->model->sdvel(body[side], com_to_ref, foot_vel[side].data());

  }

}

void set_hack_footpos()
{
  if (buffer_first_time)
  {
    buffer_first_time = false;
    _beta = 0.5; // Initialize beta here
  }
  if (contact_change_flag )
  {
    foot_registered.block(0,0,3,1) = foot_pos[LEFT];
    foot_registered.block(3,0,3,1) = foot_vel[LEFT];
    foot_registered.block(6,0,3,1) = foot_pos[RIGHT];
    foot_registered.block(9,0,3,1) = foot_vel[RIGHT];

    contact_change_flag = false;
  }
}

void initKF(const Eigen::Matrix<double,3,1> &root_pos, const Eigen::Matrix<double,3,1> &root_vel){

	buffer_first_time = true;
	contact_change_flag = true;
	_toe_flag = -1;

	Eigen::Matrix<double,6,6> P0;
	P0.setIdentity();
	P0 *= 0.01;
	_P = P0;

	_x.block(X0,0,3,1) = root_pos;
	_x.block(V0,0,3,1) = root_vel;
	_vimu = _x.block(V0,0,3,1); // This is not quite accurate, but only one time step, so does not matter
	setKss();

	// The following steps are additional step for next step measurement
	computeAnkleRelated(); // Compute ankle position and velocity
	set_hack_footpos();
	// Display in terminal
	std::cout << "===================================\n";
	std::cout << "Kinematic Filter QR" <<std::endl;
	std::cout << "Q = " << std::endl;
	std::cout <<_Q << std::endl;
	std::cout << "R = " << std::endl;
	std::cout <<_R << std::endl;
	std::cout << "===================================\n";

}

void makeInputs(const Eigen::Quaterniond &q, const Vec3 &w, const Vec3 &a, const Vec30 &joints, const Vec30 &jointsd)
{
  _q = q;
  _w = w;
  _a = a;
  _joints = joints;
  _jointsd = jointsd;
}


void setContactState (int cs, int toe_flag)
{
  setContactState(cs);
  _previous_toe_flag = _toe_flag; // store previous toe flag;
  _toe_flag = toe_flag;
}

void makeMeasurement(double left_fz, double right_fz)
{
  static double fz_hist[2] = {750, 750};
  left_fz = low_pass_filter(fz_hist[0], left_fz, 0.99);
  right_fz = low_pass_filter(fz_hist[1], right_fz, 0.99);

  _beta = left_fz / (left_fz + right_fz);

  if (!(_previous_contact_state == _contact_state_full))
  {
    _previous_contact_state = _contact_state_full;
    contact_change_flag = true;
    set_hack_footpos();
  }

  if (sqrt(left_fz*left_fz+right_fz*right_fz) < 50) // Total z force small
  {
    contact_change_flag = true; // Just update foot position until the robot drops from the air
    set_hack_footpos();
  }
  /*
  if (_toe_flag > _previous_toe_flag)// toe off
  {
    contact_change_flag = true;
    set_hack_footpos();
  }
  */
  _y = _beta*foot_registered.block(0,0,6,1) + (1.0 - _beta)*foot_registered.block(6,0,6,1);
  // Set measured velocity to zero
  _y.block(3,0,3,1).setZero();
}



/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------                              ------------------------------------*/
/*---------------------------           SL STUFF           ------------------------------------*/
/*---------------------------                              ------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------*/


static int init_state_est_lin_task(){
	_P.setIdentity();
	_Q.setIdentity();
	_R.setIdentity();
	return TRUE;
}

void KinematicFilter3::predictX()
{
  // IMU acceleration in world frame without gravity (a = R^{-1}*a_imu + g)
  Eigen::Matrix<double,3,1> acc;
  acc = _q*_imu_quat_offset.conjugate()*_a + _gravity;

  // IMU velocity in world frame
  _vimu += timestep*acc;
  // Position
  _x.block(X0,0,3,1) += timestep*_x.block(V0,0,3,1);// + 0.5*timestep*timestep*acc;
  // Velocity
  // V = R(-q)*(Vimu - \omega x imuoffset)
  _x.block(V0,0,3,1) = _vimu - _q*_w.cross(_imu_lin_offset);
}

void filterOneTimeStep_ss()
{
  predictX();
  computeAnkleRelated(); // Compute ankle position and velocity
  updateZ(6);
  computeInnovation(_y);
  updateX(6);
  // The following steps are additional step for next step measurement
  computeAnkleRelated(); // Compute ankle position and velocity
}


/*---------------------------------------------------------------------------------------------*/

static int run_state_est_lin_task(){

	makeInputs(imu_orientation, imu_angular_velocity, imu_linear_acceleration, joints, jointsd);
	setContactState(cState, toeFlag);

	makeMeasurement(l_foot_fz, r_foot_fz);
	filterOneTimeStep_ss();

	return TRUE;

}

/*---------------------------------------------------------------------------------------------*/

static int change_state_est_lin_task(){

	return TRUE;
}

/*---------------------------------------------------------------------------------------------*/

void add_state_est_lin_task(){

	addTask("state_est_lin", init_state_est_lin_task, run_state_est_lin_task, change_state_est_lin_task);
}
