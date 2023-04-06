#ifndef MULTICOMMCLASS_H
#define MULTICOMMCLASS_H

#include "communicationClass.h"


/** @typedef ReadWriteConversionFcn
 * @brief A new 'variable type', which is a function. Now we can easily make pointers to this type of function
 * This variable type is used for ReadConvert and WriteConvert functions, for implementing the
 * Measurements and Actuation blocks.
 * Defining a ReadConvert or WriteConvert function can simply be done like this:
 *    void MyReadConvert(const double *src, double *dst) { ... }
 * \sa setReadConvertFcn, setWriteConvertFcn
 */
typedef void ReadWriteConversionFcn (const double*, double*);

/**
 * @brief Class which provides top level access to XDDP/IDDP
 * TODO: Integrate with xenoCommunication classes. 
 * 
 */
class frameworkComm
{
public:
    /**
     * @brief Send value to predetermined port and with predetermined parameters. 
     * 
     * @param value value which needs to be send.
     * @return int: size of values send. 
     */
    int send(double value[]);

    /**
     * @brief Receive function which receives an array of data and writes it over the specified indexes of the output value. 
     * Port is pre-assigned. 
     * @param value output array. 
     * @param parameters array filled with indexes of where the receives array is put. (in order of received array)
     * @return int: size of received values. 
     */
    // int receive(double value[], int parameters[]);

    /**
     * @brief receive value to predetermined port and with predetermined parameters. 
     * Port is pre-assigned. 
     * @param value array which is filled with values according to predetermined parameters. 
     * @return int: size of received values. 
     */
    int receive(double value[]);
    
    /**
     * @brief Set the function to be Verbose or not. 
     * 
     * @param verbose value of verbosity. 
     */
    void setVerbose(bool verbose);

    /** 
     * @brief Bind a ReadConvert function to this communication class
     * This is to provide Measurement&Actuation functionality to the class. The ReadConvert
     * function could be used to perform scaling or filtering to the incoming signals
     * prior to handing them over to the u[] variable of the 20-sim model.
     * 
     * Example: 
     * @code
     * void MyReadConvert(const double* src, double *dst)
     * {
     *     static double lastKnownGoodValue = 0;
     *     // We assume that src is a 3 element double array, which has:
     *     // src[0] = temperature reading in degrees Fahrenheit (scaling) 
     *     //          -> We want the output in degrees Celcius
     *     // src[1] = some measurement with unreliable communication; once
     *     //          in a while a (faulty) 0 is received; this should be 
     *     //          filtered and the previous value should be passed (filtering)
     *     // src[2] = some good measurement which does not need 
     *     //          scaling neither filtering
     *     //
     *     dst[0] = (src[0]-32.0) / 1.8; // scaling
     * 
     *     if ( src[1]!=0) // filtering
     *     {
     *        dst[1] = src[1];
     *        lastKnownGoodValue = src[1];
     *     } else
     *     {
     *        dst[1] = lastKnownGoodValue;
     *     }
     * 
     *     dst[2] = src[2]; // passing
     * }
     * 
     * // Using the function (typically in some main/setup function):
     * IcoComm icoComm = new IcoComm(...);
     * icoComm.SetReadConvertFcn(&MyReadConvert);
     * @endcode
     */
    void setReadConvertFcn(ReadWriteConversionFcn *f);

    /** 
     * @brief Bind a WriteConvert function to this communication class
     * \sa setReadConvertFcn
     */ 
    void setWriteConvertFcn(ReadWriteConversionFcn *f);


    virtual ~frameworkComm() {}

    frameworkComm();
    
protected:
    int *sendParameters;
    int *receiveParameters;
    int destinationPort;
    int receiveSize;
    int sendSize;
    xenoCommunication *xenoCommClass;
    bool verbose = false;
    ReadWriteConversionFcn *readConvertFcn;
    ReadWriteConversionFcn *writeConvertFcn;
};

class IDDPComm : public frameworkComm
{
public:
    IDDPComm(int ownPort, int destPort, int _size, int _parameters[]);
    ~IDDPComm();
};

class XDDPComm : public frameworkComm
{
public:
    XDDPComm(int ownPort, int destPort, int _size, int _parameters[]);
    ~XDDPComm();
};

#endif