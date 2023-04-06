#include "multiCommClass.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

IDDPComm::IDDPComm(int ownPort, int destPort, int _size, int _parameters[])
{
    sendSize = _size;
    receiveSize = _size;
    destinationPort = destPort;
    sendParameters = _parameters; // For IDDPComm, we use either send or receive parameters, so they can safely share one constructor argument.
    receiveParameters = _parameters;
    xenoCommClass = new IDDPconnection(ownPort);
}

IDDPComm::~IDDPComm()
{
    xenoCommClass->~xenoCommunication(); // Shouldn't this be `delete xenoCommClass`?
}

XDDPComm::XDDPComm(int ownPort, int destPort, int _size, int _parameters[])
{
    sendSize = _size;
    receiveSize = _size;
    destinationPort = destPort;
    sendParameters = _parameters;
    receiveParameters = _parameters;
    xenoCommClass = new XDDPconnection(ownPort);
}

XDDPComm::~XDDPComm()
{
    xenoCommClass->~xenoCommunication();
}


frameworkComm::frameworkComm():
    readConvertFcn(nullptr),
    writeConvertFcn(nullptr)
{
}

void frameworkComm::setVerbose(bool _verbose)
{
    verbose = _verbose;
    xenoCommClass->setVerbose(verbose);
}

int frameworkComm::send(double value[])
{
    //printf("send function of IDDP\n");
    double toSend[sendSize]={0};
    double toSendConverted[sendSize];
    int ret;
    for (int i = 0; i < sendSize; i++)
    {
        if (sendParameters[i]!=-1)
            toSend[i] = value[sendParameters[i]];
        //printf("value on %d is: %f\n", sendParameters[i], value[sendParameters[i]]);
    }
    if (writeConvertFcn != nullptr)
    {
        writeConvertFcn(toSend, toSendConverted);
    } else
    {
        memcpy(toSendConverted, toSend, sizeof(toSendConverted));
    }
    if (verbose)
    {
        printf("Send - Original and Converted values of output array:\n");
        for (int i = 0; i < sendSize; i++)
        {
            printf("%11f -> %11f\n", toSend[i], toSendConverted[i]);
        }
    }
    ret = xenoCommClass->sendDoubleArray(toSendConverted, destinationPort, sendSize);
    return ret;
}

int frameworkComm::receive(double value[])
{
    double received[receiveSize];
    double receivedConverted[receiveSize];
    int ret;
    ret = xenoCommClass->receiveDoubleArray(received, receiveSize);

    if (readConvertFcn != nullptr)
    {
        readConvertFcn(received, receivedConverted);
    } else
    {
        memcpy(receivedConverted, received, sizeof(receivedConverted));
    }

    if (verbose)
    {
        printf("Receive - Original and Converted values of input array:\n");
        for (int i = 0; i < sendSize; i++)
        {
            printf("%11f -> %11f\n", received[i], receivedConverted[i]);
        }
    }
    

    if (ret > 0)
    {
        for (int i = 0; i < receiveSize; i++)
        {
            if (receiveParameters[i]!=-1)
                value[receiveParameters[i]] = receivedConverted[i];
        }
    }
    return ret;
}

void frameworkComm::setReadConvertFcn(ReadWriteConversionFcn *f)
{
    readConvertFcn = f; 
}

void frameworkComm::setWriteConvertFcn(ReadWriteConversionFcn *f)
{
    writeConvertFcn = f;
}

