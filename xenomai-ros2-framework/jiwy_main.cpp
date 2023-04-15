#include <stdio.h>
#include <unistd.h>
#include <iterator>
#include <signal.h>
#include <vector>
#include <sys/syscall.h>

#include "framework/multiCommClass.h"
#include "framework/runnableClass.h"
#include "framework/superThread.h"
#include "framework/icoCommClass.h"

#include "ControllerPanTilt/ControllerPanTilt.h"

#include "fpga_indices.h"

#define U_MEASURED_PAN_INDEX    0
#define U_MEASURED_TILT_INDEX   1
#define U_SETPOINT_PAN_INDEX    2
#define U_SETPOINT_TILT_INDEX   3

#define Y_STEERING_PAN_INDEX    0
#define Y_STEERING_TILT_INDEX   1

// XDDP ports

// send setpoint to controller
#define XDDP_SETPOINT_PORT_TILT   10 
#define XDDP_SETPOINT_PORT_PAN    15

// log position and output back to non-rt
#define XDDP_POSITION_LOG_PORT  26
#define XDDP_OUTPUT_LOG_PORT    22


// conversion functions for IcoComm
void readConvert(const double *src, double *dst) {

    // catch underflow and convert to negative values
    if (src[0] > 8192) dst[0] = src[0] - 16384; else dst[0] = src[0];
    if (src[3] > 8192) dst[3] = src[3] - 16384; else dst[3] = src[3];

    // convert encoder values to radians
    dst[0] = dst[0] / 2000 * 2 * 3.1415;
    dst[3] = dst[3] / 1250 * 2 * 3.1415 / 4;
}

void writeConvert(const double *src, double *dst) { 

    // scale up normalized values
    dst[0] = src[0] * 4096;

    // additional scaling for pan due to 4:1 gear ratio
    dst[2] = src[2] * 4096 * 4;
}


volatile bool exitbool = false;

void exit_handler(int s)
{
    printf("Caught signal %d\n", s);
    exitbool = true;
}

int main()
{
    //CREATE CNTRL-C HANDLER
    signal(SIGINT, exit_handler);

    printf("Press Ctrl-C to stop program\n"); // Note: this will 
        // not kill the program; just jump out of the wait loop. Hence,
        // you can still do proper clean-up. You are free to alter the
        // way of determining when to stop (e.g., run for a fixed time).


    // CONFIGURE, CREATE AND START THREADS HERE

    /*

    +---------+-------+--------------+----+------------------+
    |         | Index | Description  |    |       Port       |
    +=========+=======+==============+====+==================+
    | Inputs  |   0   | MeasuredPan  | <- | IcoComm - P2_ENC |
    |         +-------+--------------+----+------------------+
    |         |   1   | MeasuredTilt | <- | IcoComm - P1_ENC |
    |         +-------+--------------+----+------------------+
    |         |   2   | SetPointPan  | <- |    XDDP - 15     |
    |         +-------+--------------+----+------------------+
    |         |   3   | SetPointTilt | <- |    XDDP - 10     |
    +=========+=======+==============+====+==================+
    | Outputs |   0   | SteeringPan  | -> | IcoComm - P2_PWM |
    |         +-------+--------------+----+------------------+
    |         |   1   | SteeringTilt | -> | IcoComm - P1_PWM |
    +---------+-------+--------------+----+------------------+

    */
   

    // configure where motor outputs get written
    int sendParameters[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };

    sendParameters[P2_PWM] = Y_STEERING_PAN_INDEX;
    sendParameters[P1_PWM] = Y_STEERING_TILT_INDEX;

    // configure from where pan/tilt values are read
    int receiveParameters[12] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

    receiveParameters[P2_ENC] = U_MEASURED_PAN_INDEX;
    receiveParameters[P1_ENC] = U_MEASURED_TILT_INDEX;
    
    auto icoComm = new IcoComm(
        sendParameters,
        receiveParameters
    );  

    icoComm->setReadConvertFcn(readConvert);
    icoComm->setWriteConvertFcn(writeConvert);


    // Configure where setpoint values get stored when received.
    // Originally this was a single 2-element array,
    // but this had to be split as the ros2-xenomai listener node
    // only supports sending scalar values
    int xddp_uParam_Setpoint_tilt[1] = {
        U_SETPOINT_TILT_INDEX
    };

    int xddp_uParam_Setpoint_pan[1] = {
        U_SETPOINT_PAN_INDEX
    };

    // controller gets position inputs from FPGA/SPI/IcoComm and setpoint inputs from ROS/XDDPComm
    frameworkComm *uPorts[] = {
        icoComm,
        new XDDPComm(XDDP_SETPOINT_PORT_TILT, -1, 1, xddp_uParam_Setpoint_tilt),
        new XDDPComm(XDDP_SETPOINT_PORT_PAN, -1, 1, xddp_uParam_Setpoint_pan)
    };

    // controller outputs motor power to FPGA/SPI/IcoComm
    frameworkComm *yPorts[] = {
        icoComm
    };

    ControllerPanTilt *controller_pan_tilt = new ControllerPanTilt();
    controller_pan_tilt->SetFinishTime(0.0); // run forever

    runnable *controller = new wrapper<ControllerPanTilt>(
        controller_pan_tilt,
        uPorts,
        yPorts,
        3,
        1
    );
    
    // controller->setVerbose(true);

    // 2 setpoint inputs, 2 position inputs and 2 motor outputs
    controller->setSize(4, 2);

    xenoThread controller_thread = xenoThread(controller); 
    
    controller_thread.init(1000000, 99, 1);
    controller_thread.start("controller");


    // WAIT FOR CNTRL-C
    timespec t = {.tv_sec=0, .tv_nsec=100000000}; // 1/10 second

    while (!exitbool)
    {
        // Let the threads do the real work
        nanosleep(&t, NULL);
        // Wait for Ctrl-C to exit
    }
    printf("Ctrl-C was pressed: Stopping gracefully...\n");

    //CLEANUP HERE

    // cleanup is mostly redundant as the OS will clean up after program termination
    // however, it is good practice to do it anyway
    controller_thread.stopThread();
    controller->~runnable();

    return 0;
}