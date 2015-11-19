/*
 * fall_task.c
 *
 *  Created on: Nov 18, 2015
 *      Author: perry
 */


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

/* defines */

/* local variables */
static double start_time = 0.0;


/* global functions */

/* local functions */
static int  init_fall_task(void);
static int  run_fall_task(void);
static int  change_fall_task(void);

/*****************************************************************************
******************************************************************************
Function Name	: add_sample_task
Date		: Feb 1999
Remarks:

adds the task to the task menu

******************************************************************************
Paramters:  (i/o = input/output)

none

*****************************************************************************/
void
add_fall_task( void )

{
  int i, j;

  addTask("Fall Task", init_fall_task,
	  run_fall_task, change_fall_task);

}

/*****************************************************************************
******************************************************************************
  Function Name	: init_sample_task
  Date		: Dec. 1997

  Remarks:

  initialization for task

******************************************************************************
  Paramters:  (i/o = input/output)

       none

 *****************************************************************************/
static int
init_fall_task(void)
{

  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: run_sample_task
  Date		: Dec. 1997

  Remarks:

  run the task from the task servo: REAL TIME requirements!

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static double currentForce = 0;

static int
run_fall_task(void)
{
	int i;
	double x_capture[4];
	for (i = 1; i <=2; i++){
		x_capture[i] = cog.x[i] + cog.xd[i]*sqrt(abs(cog.x[_Z_]/ gravity));
	}
	static int printed = FALSE;

	if (x_capture[_Y_] > cart_state[LEFT_FOOT].x[_Y_]+YTOE){
		if (printed == FALSE){
			printf("FALLING FORWARD!!!!");
			printed = TRUE;
		}
	} else {
		if (printed == TRUE){
			printed = FALSE;
			printf("NO LONGER IN FALL");
		}
	}

	static int printcounter =100;
	printcounter ++;
	if (printcounter > 100){
		printf("y_edge = %f, y_capture = %f \n", cart_state[LEFT_FOOT].x[_Y_]+YTOE ,x_capture[_Y_]);
		printcounter = 0;
	}

	  if (currentForce != 0){
		  uext_sim[B_TR].f[_Y_] = currentForce;
		  sendUextSim();
	  }


  return TRUE;
}

/*****************************************************************************
******************************************************************************
  Function Name	: change_sample_task
  Date		: Dec. 1997

  Remarks:

  changes the task parameters

******************************************************************************
  Paramters:  (i/o = input/output)

  none

 *****************************************************************************/
static int
change_fall_task(void)
{

  get_double("This is how to enter a double variable",currentForce,&currentForce);
  return TRUE;
}




