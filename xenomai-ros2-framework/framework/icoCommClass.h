#ifndef ICO_COMM_CLASS_H
#define ICO_COMM_CLASS_H
#include "multiCommClass.h"
#include "communicationClass.h"
#include "ico_io.h"

class IcoComm: public frameworkComm
{
protected:
public:
    IcoComm(int _sendParameters[], int _receiveParameters[]);
    ~IcoComm();
};


class IcoConnection: public xenoCommunication
{
public:
    IcoConnection();
    ~IcoConnection();
    int sendDoubleArray(double toSend[], int port, int size) override;
    int receiveDoubleArray(double received[], int size) override;

private:
    void initSocket();
    IcoIO *icoIo;
    IcoIO::IcoWrite writeBuffer;
    IcoIO::IcoRead readBuffer;
};


#endif /* ICO_COMM_CLASS_H */
