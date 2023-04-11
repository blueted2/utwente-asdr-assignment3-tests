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

#include "fpga_indices.h"



#define U_MEASURED_PAN_INDEX    0
#define U_MEASURED_TILT_INDEX   1
#define U_SETPOINT_PAN_INDEX    2
#define U_SETPOINT_TILT_INDEX   3

#define Y_STEERING_PAN_INDEX    0
#define Y_STEERING_TILT_INDEX   1


// XDDP ports
#define XDDP_OUTPUT_PORT   20
#define XDDP_INPUT_PORT    21


class IcoCommForwarder : public runnable
{
private:
    IcoComm *icoComm;

    double spi_output[8];
    double spi_input[12];

    int xddp_output_port = 0;
    int xddp_input_port = 0;

    frameworkComm outputComm;
    frameworkComm inputComm;

public:
    IcoCommForwarder(int xddp_output_port, int xddp_input_port)
    {
        this->xddp_output_port = xddp_output_port;
        this->xddp_input_port = xddp_input_port;

        int sendParameters[8] = {
            0, 1, 2, 3, 4, 5, 6, 7
        };

        int receiveParameters[12] = {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
        };

        // initialize icoComm
        icoComm = new IcoComm(
            sendParameters,
            receiveParameters
        );

        // initialize sendComm
        outputComm = XDDPComm(xddp_output_port, -1, 8, sendParameters);
        inputComm = XDDPComm(xddp_input_port, -1, 12, receiveParameters);
    }

    ~IcoCommForwarder()
    {
        delete icoComm;
    }

    void prerun() override
    {
        printf("prerun\n");
    }

    void postrun() override
    {
        printf("postrun\n");
    }

    void run() override
    {
        printf("run\n");
    }

    void step() override
    {
        // get values to forward from xddp
        outputComm.receive(spi_output);
        // forward values to icoComm
        icoComm->send(spi_output);

        // get values to send back from icoComm
        icoComm->receive(spi_input);
        // send values back to xddp
        inputComm.send(spi_input);
    }

    void setSize(int uSize, int ySize) override 
    {
        printf("setSize\n");
    }

    void enableGPIO(int pin) override 
    {
        printf("enableGPIO\n");
    }

    void setVerbose(bool verbose) override 
    {
        printf("setVerbose\n");
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

    // create icoCommForwarder
    IcoCommForwarder icoCommForwarder(XDDP_OUTPUT_PORT, XDDP_INPUT_PORT);

    xenoThread controller_thread = xenoThread(&icoCommForwarder); 
    controller_thread.init(1000000, 99, 1);
    controller_thread.start("forwarder");


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