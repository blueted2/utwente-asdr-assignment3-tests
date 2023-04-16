#include<stdio.h>

#define PI 3.1415

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
    dst[0] = src[0] * 2048;

    // additional scaling for pan due to 4:1 gear ratio
    dst[2] = src[2] * 2048 * 4;
}

void read_test(double tilt_steps, double pan_steps, double expected_tilt, double expected_pan) {
    double read_src[12];
    double read_dst[12];

    read_src[0] = tilt_steps;
    read_src[3] = pan_steps;

    readConvert(read_src, read_dst);
    // print expected and actual values
    printf("Expected: tilt: %.2f, pan: %.2f\n", expected_tilt, expected_pan);
    printf("Actual: tilt  : %.2f, pan: %.2f\n", read_dst[0], read_dst[3]);
}

void write_test(double motor_tilt, double motor_pan, double motor_tilt_out, double motor_pan_out) {
    double write_src[8];
    double write_dst[8];

    write_src[0] = motor_tilt;
    write_src[2] = motor_pan;


    writeConvert(write_src, write_dst);
    // print expected and actual values
    printf("Expected: tilt: %.2f, pan: %.2f\n", motor_tilt_out, motor_pan_out);
    printf("Actual  : tilt: %.2f, pan: %.2f\n", write_dst[0], write_dst[2]);
}


int main(void) {
    // zero should stay zero
    read_test(0, 0, 0, 0);

    // one revolution on each axis
    read_test(2000, 1250 * 4, 2 * PI, 2 * PI);

    // one revolution on each axis, negative
    // "max_value - value" is used to convert to negative values
    read_test(16384 - 2000, 16384 - 1250 * 4, -2 * PI, -2 * PI);

    // zero should stay zero
    write_test(0, 0, 0, 0);

    // max output
    // 2048 is the max value for the motor, however here we can go past this value
    // this doesn't matter as the value is limited to 2047 in the motor driver
    write_test(1, 1, 2048, 2048 * 4);

    // max output, negative
    write_test(-1, -1, -2048, -2048 * 4);

    return 0;

}