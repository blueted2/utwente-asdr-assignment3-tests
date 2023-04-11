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
#define XDDP_SETPOINT_PORT   20 

// log position and output back to non-rt
#define XDDP_POSITION_LOG_PORT  21
#define XDDP_OUTPUT_LOG_PORT    22


// conversion functions for IcoComm
void readConvert(const double *src, double *dst) { 
    dst[0] = src[0];
    dst[1] = src[1];
}

void writeConvert(const double *src, double *dst) { 
    dst[0] = src[0];
    dst[1] = src[1];
}

class ControllerPanTiltRunnable : public wrapper<ControllerPanTilt>
{
private:
    frameworkComm *uPorts[2];
    frameworkComm *yPorts[2];
public:
    // TODO: pass xddp ports as arguments instead of using hardcoded defines
    ControllerPanTiltRunnable() : wrapper<ControllerPanTilt>(
        new ControllerPanTilt, uPorts, yPorts, 2, 2
    ) {
        
        // configure where motor outputs get written
        int sendParameters[8] = { 
            [0 ... 7]  = -1, 
            [P1_PWM] = Y_STEERING_PAN_INDEX,    // pan output goes to P1_PWM
            [P2_PWM] = Y_STEERING_TILT_INDEX    // tilt output goes to P2_PWM
        };


        // configure from where pan/tilt values are read
        int receiveParameters[12] = { 
            [0 ... 11] = -1, 
            [P1_ENC] = U_MEASURED_PAN_INDEX,    // P1_ENC goes to pan input
            [P2_ENC] = U_MEASURED_TILT_INDEX    // P2_ENC goes to tilt input
        };
        
        auto icoComm = new IcoComm(
            sendParameters,
            receiveParameters
        );

        icoComm->setReadConvertFcn(readConvert);
        icoComm->setWriteConvertFcn(writeConvert);


        // configure where setpoint values get stored when received
        int xddp_uParam_Setpoint[2] = {
            U_SETPOINT_PAN_INDEX,   // first value is pan setpoint
            U_SETPOINT_TILT_INDEX   // second value is tilt setpoint
        };

        // // configure encoder position logging
        // int xddp_yParam_Logging[2] = {
        //     U_MEASURED_PAN_INDEX,   // first value is pan position
        //     U_MEASURED_TILT_INDEX   // second value is tilt position
        // };

        // configure motor output logging
        int xddp_yParam_Logging[2] = {
            Y_STEERING_PAN_INDEX,   // first value is pan output
            Y_STEERING_TILT_INDEX   // second value is tilt output
        };

        // controller gets position inputs from FPGA/SPI/IcoComm and setpoint inputs from ROS/XDDPComm
        this->uPorts[0] = icoComm;
        this->uPorts[1] = new XDDPComm(XDDP_SETPOINT_PORT, -1, 2, xddp_uParam_Setpoint);

        // controller outputs motor power to FPGA/SPI/IcoComm and logs position and output to ROS/XDDPComm
        this->yPorts[0] = icoComm;
        this->yPorts[1] = new XDDPComm(XDDP_POSITION_LOG_PORT, -1, 2, xddp_yParam_Logging);

    }
};



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
    | Inputs  |   0   | MeasuredPan  | <- | IcoComm - P1_ENC |
    |         +-------+--------------+----+------------------+
    |         |   1   | MeasuredTilt | <- | IcoComm - P2_ENC |
    |         +-------+--------------+----+------------------+
    |         |   2   | SetPointPan  | <- |    XDDP - 0      |
    |         +-------+--------------+----+------------------+
    |         |   3   | SetPointTilt | <- |    XDDP - 1      |
    +=========+=======+==============+====+==================+
    | Outputs |   0   | SteeringPan  | -> | IcoComm - P1_PWM |
    |         +-------+--------------+----+------------------+
    |         |   1   | SteeringTilt | -> | IcoComm - P2_PWM |
    +---------+-------+--------------+----+------------------+

    */

   {

    // // configure where motor outputs get written
    // int sendParameters[8] = { 
    //     [0 ... 7]  = -1, 
    //     [P1_PWM] = Y_STEERING_PAN_INDEX,    // pan output goes to P1_PWM
    //     [P2_PWM] = Y_STEERING_TILT_INDEX    // tilt output goes to P2_PWM
    // };


    // // configure from where pan/tilt values are read
    // int receiveParameters[12] = { 
    //     [0 ... 11] = -1, 
    //     [P1_ENC] = U_MEASURED_PAN_INDEX,    // P1_ENC goes to pan input
    //     [P2_ENC] = U_MEASURED_TILT_INDEX    // P2_ENC goes to tilt input
    // };
    
    // auto icoComm = new IcoComm(
    //     sendParameters,
    //     receiveParameters
    // );

    // // configure where setpoint values get stored when received
    // int xddp_uParam_Setpoint[2] = {
    //     U_SETPOINT_PAN_INDEX,   // first value is pan setpoint
    //     U_SETPOINT_TILT_INDEX   // second value is tilt setpoint
    // };

    // // // configure encoder position logging
    // // int xddp_yParam_Logging[2] = {
    // //     U_MEASURED_PAN_INDEX,   // first value is pan position
    // //     U_MEASURED_TILT_INDEX   // second value is tilt position
    // // };

    // // configure motor output logging
    // int xddp_yParam_Logging[2] = {
    //     Y_STEERING_PAN_INDEX,   // first value is pan output
    //     Y_STEERING_TILT_INDEX   // second value is tilt output
    // };

    // // controller gets position inputs from FPGA/SPI/IcoComm and setpoint inputs from ROS/XDDPComm
    // frameworkComm *uPorts[] = {
    //     icoComm,
    //     new XDDPComm(XDDP_SETPOINT_PORT, -1, 2, xddp_uParam_Setpoint)
    // };

    // // controller outputs motor power to FPGA/SPI/IcoComm and logs position and output to ROS/XDDPComm
    // frameworkComm *yPorts[] = {
    //     icoComm,
    //     new XDDPComm(XDDP_POSITION_LOG_PORT, -1, 2, xddp_yParam_Logging),
    // };

    // runnable *controller = new wrapper<ControllerPanTilt>(
    //     new ControllerPanTilt,
    //     uPorts,
    //     yPorts,
    //     2,
    //     2
    // );

   }

    runnable *controller = new ControllerPanTiltRunnable();

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


    return 0;
}