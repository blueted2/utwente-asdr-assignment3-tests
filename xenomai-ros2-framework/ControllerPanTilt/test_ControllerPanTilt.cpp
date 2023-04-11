/**********************************************************
 * This file is generated by 20-sim C++ Code Generator
 *
 *  file:  test_ControllerPanTilt.cpp
 *  subm:  ControllerPanTilt
 *  model: ControllerPanTilt
 *  expmt: 20-E9D4
 *  date:  April 7, 2023
 *  time:  10:14:43 AM
 *  user:  20-sim 5.0 Student License
 *  from:  Universiteit Twente
 *  build: 5.0.2.12127
 **********************************************************/

/* This file is a demo application of how the submodel function can
 * be used. It uses the global time variables that are used by both
 * the submodel and the integration method.
 *
 * PLEASE NOTE: THIS IS AN EXAMPLE WHERE ALL INPUTS ARE ZERO !
 * USE YOUR OWN INPUTS INSTEAD!! ALSO THE SUBMODEL MIGHT SIMPLY
 * NOT WORK CORRECTLY WITH INPUTS THAT ARE ZERO.
 */

#include <stdio.h>

/* 20-sim submodel class include file */
#include "ControllerPanTilt.h"

/* the main function */
int main()
{
	XXDouble u [4 + 1];
	XXDouble y [2 + 1];

	/* initialize the inputs and outputs with correct initial values */
	u[0] = 0.0;		/* MeasuredPan */
	u[1] = 0.0;		/* MeasuredTilt */
	u[2] = 0.0;		/* SetPointPan */
	u[3] = 0.0;		/* SetPointTilt */

	y[0] = 0.0;		/* SteeringPan */
	y[1] = 0.0;		/* SteeringTilt */


	ControllerPanTilt my20simSubmodel;

	/* initialize the submodel itself and calculate the outputs for t=0.0 */
	my20simSubmodel.Initialize(u, y, 0.0);
	printf("Time: %f\n", my20simSubmodel.GetTime() );

	/* simple loop, the time is incremented by the integration method */
	while (my20simSubmodel.state != ControllerPanTilt::finished)
	{
		/* call the submodel to calculate the output */
		my20simSubmodel.Calculate (u, y);
		printf("Time: %f\n", my20simSubmodel.GetTime() );
	}

	/* perform the final calculations */
	my20simSubmodel.Terminate (u, y);

	/* and we are done */
	return 0;
}

