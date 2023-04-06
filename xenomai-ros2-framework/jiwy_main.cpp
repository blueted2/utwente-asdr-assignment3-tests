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

#include "controllers/ControllerPan.h"

// #include <Your 20-sim-code-generated h-file?> Don't forget to compile the cpp file by adding it to CMakeLists.txt


enum output_ports {
    P1_PWM   = 0, 
    P1_OUT_1 = 1,
    P2_PWM   = 2, 
    P2_OUT_1 = 3,
    P3_PWM   = 4, 
    P3_OUT_1 = 5,
    P4_PWM   = 6, 
    P4_OUT_1 = 7
};

enum input_ports {
    P1_ENC   = 0, 
    P1_IN_1  = 1,
    P1_IN_2  = 2,
    P2_ENC   = 3, 
    P2_IN_1  = 4,
    P2_IN_2  = 5,
    P3_ENC   = 6, 
    P3_IN_1  = 7,
    P3_IN_2  = 8,
    P4_ENC   = 9, 
    P4_IN_1  = 10,
    P4_IN_2  = 11
};

#define XDDP_PAN_SETPOINT_PORT   20
#define XDDP_TILT_SETPOINT_PORT  30

#define U_MEASURED_INDEX 0
#define U_SETPOINT_INDEX 1

#define Y_STEERING_INDEX 0

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

    // bind P1_PWM to u[0] and Y_STEERING_INDEX to y[0]
    int sendParameters[8]     = { [0 ... 7]  = -1, [P1_PWM] = Y_STEERING_INDEX };
    int receiveParameters[12] = { [0 ... 11] = -1, [P1_ENC] = U_MEASURED_INDEX };
    
    auto icoComm = new IcoComm(
        sendParameters,
        receiveParameters
    );

    // bind 
    int xddp_uParam_Setpoint [1] = { 1 } ;

    frameworkComm *pan_controller_uPorts[] = {
        icoComm,
        new XDDPComm(XDDP_PAN_SETPOINT_PORT, -1, 1, xddp_uParam_Setpoint)
    };

    frameworkComm *pan_controller_yPorts[] = {
        icoComm
    };

    
    runnable *pan_controller = new wrapper<ControllerPan>(
        new ControllerPan,
        pan_controller_uPorts,
        pan_controller_yPorts,
        2,
        1
    );

    xenoThread *pan_controller_thread = new xenoThread(pan_controller); 


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