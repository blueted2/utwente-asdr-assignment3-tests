#include "communicationClass.h"
#include "multiCommClass.h"
#include <rtdm/ipc.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include "ico_io.h"
#include "icoCommClass.h"


void IcoConnection::initSocket()
{
    // Not really a socket to be initalized, but at least we do some initialization here
    protocol = "SPI";

    icoIo = new IcoIO();
    if (!icoIo->open_spi())
    {
        printf("Error opening SPI device. Note: Only create one instance of IcoConnection and use it as\n");
        printf("both ReceiveClass and SendClass in the class wrapper.\n");
    }

    icoIo->init();

    printf("Done setting up the SPI connection");
}


IcoConnection::IcoConnection():
    writeBuffer{0,0,0,0,0,0,0,0}
{
    printf("IcoConnection constructor\n");
    initSocket();
}

IcoConnection::~IcoConnection()
{
    printf("Close SPI connection and close port: %d\n", receivePort);
    delete icoIo;
}

/* Overrides sendDoubleArray from XenoCommunication, since the sending/receiving here is totally different than for XDDP/IDDP channels */
int IcoConnection::sendDoubleArray(double toSend[], int port, int size)
{
    // port argument is ignored
    
    // In this function we just prepare data so that it can be sent together with reading new encoder
    // data (saves a lot of time since starting SPI communication is slow)

    // The ICO_IO firmware in the FPGA does not accept floats. Instead, we convert it here to a packed 
    // data struct which represents the same information as the doubles. This assumes that toSend is an array of size 8,
    // being:
    // [ pwm1, val1, pwm2, val2, pwm3, val3, pwm4, val4], where:
    // - pwmX is an integer value (represented as double of course), -2047 .. +2047 representing the required motor power for motor X (-100%-100%)
    // - val1 is some value; if nonzero the output PX_OUT_1 becomes high; otherwise low.

    if (size!=8)
    {
        printf("Unsupported array size %i in %s",size, __FUNCTION__);
        perror("Unsupported array size in IcoConnection::sendDoubleArray");
    }

    writeBuffer.pwm1 = (int)toSend[0];
    writeBuffer.val1 = (toSend[1]!=0)?true:false;
    writeBuffer.pwm2 = (int)toSend[2];
    writeBuffer.val2 = (toSend[3]!=0)?true:false;
    writeBuffer.pwm3 = (int)toSend[4];
    writeBuffer.val3 = (toSend[5]!=0)?true:false;
    writeBuffer.pwm4 = (int)toSend[6];
    writeBuffer.val4 = (toSend[7]!=0)?true:false;
    // writeBuffer is now ready to be sent along with the next encoder read
    if (verbose) 
    {
        printf("Preparing PWM/OUT data for SPI sending with next encoder read:\n");
        printf("PWM = %5d, %5d, %5d, %5d\n",writeBuffer.pwm1, writeBuffer.pwm2, writeBuffer.pwm3, writeBuffer.pwm4);
        printf("OUT = %5d, %5d, %5d, %5d\n",writeBuffer.val1, writeBuffer.val2, writeBuffer.val3, writeBuffer.val4);
    }
    return 1;
}


int IcoConnection::receiveDoubleArray(double received[], int size)
{
    // This function reads the encoder signals as wel as sends the most recent pwm/output values in one operation.

    // The ICO_IO firmware in the FPGA does not return doubles. Instead, it gives integers/booleans; we convert it here
    /// into a double array which represents the same information as the data from the FPGA. This assumes received is an
    // array of size 12, being:
    // [ enc1, in1_1, in1_2, enc2, in2_1, in2_2, enc3, in3_1, in3_2, enc4, in4_1, in4_2 ], 
    // where
    // - encX is an integer count (stored as a double) of the encoder pulses (14 bits unsigned int, i.e., 0...16383 ) (may wrap around)
    // - inX_Y is 0.0 or 1.0 depending on the logical state of input X.Y.

    if (size!=12)
    {
        printf("Unsupported array size %i in %s",size, __FUNCTION__);
        perror("Unsupported array size in IcoConnection::receiveDoubleArray");
    }

    // Do the read/write action
    icoIo->update_io(writeBuffer, &readBuffer);

    // Expandthe data from the struct into the DoubleArray
    received[0] = readBuffer.channel1;
    received[1] = readBuffer.channel1_1;
    received[2] = readBuffer.channel1_2;
    received[3] = readBuffer.channel2;
    received[4] = readBuffer.channel2_1;
    received[5] = readBuffer.channel2_2;
    received[6] = readBuffer.channel3;
    received[7] = readBuffer.channel3_1;
    received[8] = readBuffer.channel3_2;
    received[9] = readBuffer.channel4;
    received[10]= readBuffer.channel4_1;
    received[11]= readBuffer.channel4_2;

    if (verbose) 
    {
        printf("Encoders: p1: %d, p2: %d, p3: %d, p4: %d\n", readBuffer.channel1, readBuffer.channel2, readBuffer.channel3, readBuffer.channel4);
        printf("Aux inputs: p1_1: %d, p1_2: %d, p2_1: %d, p2_2: %d, p3_1: %d, p3_2: %d, p4_1: %d, p4_2: %d\n\n",
            readBuffer.channel1_1, readBuffer.channel1_2, readBuffer.channel2_1, readBuffer.channel2_2, 
            readBuffer.channel3_1, readBuffer.channel3_2, readBuffer.channel4_1, readBuffer.channel4_2);
    }

    return 1;
}



/***********************************************************
 * IcoComm
 */
IcoComm::IcoComm(int _sendParameters[], int _receiveParameters[])
{
    printf("Constructing IcoComm...\n");
    sendSize = 8; // Fixed because the whole protocol of talking with the ico_io is fixed.
    receiveSize = 12;
    sendParameters = _sendParameters;
    receiveParameters = _receiveParameters;
    xenoCommClass = new IcoConnection();
}

IcoComm::~IcoComm()
{
    printf("Destructing IcoComm...\n");
    delete xenoCommClass;
}
