#pragma once

#include <rtdm/rtdm.h>
#include "/home/pi/spi-bcm283x-rtdm/include/spi-bcm283x-rtdm.h"

class IcoIO 
{
public:
struct IcoWrite {
    int16_t pwm1;
    bool    val1;
    int16_t pwm2;
    bool    val2;
    int16_t pwm3;
    bool    val3;
    int16_t pwm4;
    bool    val4;
};

struct IcoRead {
    int channel1;
    int channel2;
    int channel3;
    int channel4;
    bool channel1_1;
    bool channel1_2;
    bool channel2_1;
    bool channel2_2;
    bool channel3_1;
    bool channel3_2;
    bool channel4_1;
    bool channel4_2;
};
    IcoIO();
    ~IcoIO();



    void setPWM(uint8_t channel, int16_t value);
    int readEnc(uint8_t channel);
    size_t update_io(IcoWrite write_val, IcoRead* read_val);

    /**
     * @brief Open SPI port
     * 
     * @return int: 0 on failure; 1 on success
     */
    int open_spi();

    /**
     * @brief Close SPI Port
     * 
     * @return int: 1 (success) (cannot fail)
     */
    int close_spi();
    int init();


private:
    enum OP_CODE {
        NOOP = 0,
        INIT = 1,
        PWM = 2,
        ENC = 3,
        ALL = 4
    };

    struct WriteValue {
        unsigned int pwm_val: 14;
        unsigned int pwm_dir: 1;
        unsigned int pin_val: 1;
    };

    union WriteBytes {
        WriteValue n;
        uint8_t s[2];
    };

    struct WriteArray {
        unsigned int pwm_val1: 14;
        unsigned int pwm_dir1: 1;
        unsigned int pin_val1: 1;
        unsigned int pwm_val2: 14;
        unsigned int pwm_dir2: 1;
        unsigned int pin_val2: 1;
        unsigned int pwm_val3: 14;
        unsigned int pwm_dir3: 1;
        unsigned int pin_val3: 1;
        unsigned int pwm_val4: 14;
        unsigned int pwm_dir4: 1;
        unsigned int pin_val4: 1;
    };

    union WriteArrayBytes {
        WriteArray in;
        uint8_t out[6];
    };

    struct ReadValue {
        unsigned int enc: 14;
        unsigned int pin1: 1;
        unsigned int pin2: 1;
    };

    union ReadBytes {
        ReadValue v;
        uint8_t s[2];
    };

    WriteValue write_value_;
    WriteBytes write_bytes_;
    ReadValue read_value_;
    ReadBytes read_bytes_;

    uint8_t tx_buffer_[12];
    uint8_t rx_buffer_[12];
    int device_handle_;
    int create_pwm_command(uint8_t channel, int16_t value);
    int create_all_command(uint8_t *buffer, IcoWrite data);
    WriteValue calc_value(int pwm_val, bool pin_val);
    WriteBytes create_pwm_bytes(int16_t pwm_val, bool pin_val);

};
