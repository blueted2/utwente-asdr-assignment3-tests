#include "ico_io.h"
#include <cstdio>
#include <errno.h>
#include <string.h>
#include <time.h>

#define BILLION  1000000000L;
IcoIO::IcoIO() : device_handle_{-1}
{
    printf("%s: Constructing IcoIO\n", __FUNCTION__);
}

IcoIO::~IcoIO()
{
    printf("%s: Destructing IcoIO\n", __FUNCTION__);
    close_spi();
}


int IcoIO::open_spi()
{
    int res;
    int value;

    res = open("/dev/rtdm/spidev0.0", O_RDWR);
    if (res < 0) 
    {
        printf("%s: Could not open spi device, open has failed with: %d(%s)\n",__FUNCTION__, errno, strerror(errno));
        return 0;
    } else {
        printf("%s: Device opened.\n", __FUNCTION__);
        device_handle_ = res;
    }

    // configure device
    value = BCM283X_SPI_BIT_ORDER_LSBFIRST;
    res = ioctl(device_handle_, BCM283X_SPI_SET_BIT_ORDER, &value);
    if (res < 0)
    {
        printf("%s: Could not configure bit order, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        device_handle_ = -1;
        return 0;
    } 

    value = BCM283X_SPI_DATA_MODE_0;
    res = ioctl(device_handle_, BCM283X_SPI_SET_DATA_MODE, &value);
    if (res < 0)
    {
        printf("%s: Could not configure data mode, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        device_handle_ = -1;
        return 0;
    }

    // value = BCM283X_SPI_SPEED_30kHz;
    value = BCM283X_SPI_SPEED_31MHz;
    res = ioctl(device_handle_, BCM283X_SPI_SET_SPEED, &value);
    if (res < 0)
    {
        printf("%s: Could not configure bus speed, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        device_handle_ = -1;
        return 0;
    }

    value = BCM283X_SPI_CS_POL_LOW;
    res = ioctl(device_handle_, BCM283X_SPI_SET_CS_POLARITY, &value);
    if (res < 0) 
    {
        printf("%s: Could not configure chip select polarity, ioctl has failed with %d (%s).\n", __FUNCTION__, errno, strerror(errno));
        device_handle_ = -1;
        return 0;
    }
    return 1;
}

int IcoIO::close_spi()
{
    if (device_handle_ != -1)
    {
        close(device_handle_);
        device_handle_ = -1;
    }
    return 1;
}
int IcoIO::init()
{
    ssize_t size;
    int spi_frame_size;
    uint8_t tx_buffer[] = {
        OP_CODE::INIT, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };
    uint8_t rx_buffer[]={0, }; 

    spi_frame_size=sizeof(tx_buffer);
    size = write(device_handle_, (const void *) tx_buffer, spi_frame_size);
    size = read(device_handle_, (void *) rx_buffer, spi_frame_size); 
    return size;
}


void IcoIO::setPWM(uint8_t channel, int16_t value)
{
    int spi_frame_size;
    uint8_t tx_buffer[4];
    uint8_t rx_buffer[]={0, 0, 0, 0}; 
    ssize_t size;

    create_pwm_command(channel, value);
    spi_frame_size=sizeof(tx_buffer_);

    /* Write from tx buffer */
    size = write(device_handle_, (const void *) tx_buffer_, spi_frame_size);
    // printf("send: ");
    // for(int i=0;i<spi_frame_size;i++)		
    //     printf("0x%02x ", tx_buffer_[i]);
    // printf("\n");

    /* Receive to rx buffer */
    size = read(device_handle_, (void *) rx_buffer_, spi_frame_size); 
    // printf("read: ");
    // for(int i=0;i<spi_frame_size;i++)		
    //     printf("0x%02x ", rx_buffer_[i]);
    // printf("\n");
}

int IcoIO::readEnc(uint8_t channel)
{
    struct timespec start, stop;
    double accum;
    ssize_t size;
    int spi_frame_size;

    uint8_t enc_cmd[12] = {
        OP_CODE::ENC, channel, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    uint8_t fake_cmd[12] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    spi_frame_size=sizeof(enc_cmd);

    /* Write from tx buffer */
    // clock_gettime( CLOCK_REALTIME, &start);
    size = write(device_handle_, (const void *) enc_cmd, spi_frame_size);
    // clock_gettime( CLOCK_REALTIME, &stop);
    // accum = ( stop.tv_sec - start.tv_sec )
    //          + (double)( stop.tv_nsec - start.tv_nsec )
    //            / (double)BILLION;
    // printf( "Write: %lf\n", accum );
    // printf("send: ");
    // for(int i=0;i<spi_frame_size;i++)		
    //     printf("0x%02x ", enc_cmd[i]);
    // printf("\n");

    // size = write(device_handle_, (const void *) reader, spi_frame_size);
    /* Receive to rx buffer */
    // clock_gettime( CLOCK_REALTIME, &start);
    size = write(device_handle_, (const void *) fake_cmd, spi_frame_size);
    size = read(device_handle_, (void *) rx_buffer_, sizeof(rx_buffer_)); 
    // clock_gettime( CLOCK_REALTIME, &stop);
    // accum = ( stop.tv_sec - start.tv_sec )
    //          + (double)( stop.tv_nsec - start.tv_nsec )
    //            / (double)BILLION;
    // printf( "Read: %lf\n", accum );
    
    printf("read: ");
    for(int i=0;i<sizeof(rx_buffer_);i++)		
        printf("0x%02x ", rx_buffer_[i]);
    printf(".\n");


    ReadBytes read_value;
    read_value.s[0] = static_cast<char>(rx_buffer_[5]);
    read_value.s[1] = static_cast<char>(rx_buffer_[6]);

    int16_t result = static_cast<int16_t>(read_value.v.enc);
    printf("Encvalue: %d | %d\n", read_value.v.enc, result);
    return read_value.v.enc;
}


size_t IcoIO::update_io(IcoWrite write_val, IcoRead* read_val)
{
    struct timespec start, stop;
    double accum;
    
    int spi_frame_size;
    ssize_t size;
    uint8_t tx_buffer[12];
    int ret = create_all_command(tx_buffer, write_val);
    spi_frame_size=sizeof(tx_buffer);


    /* Write from tx buffer */
    size = write(device_handle_, (const void *) tx_buffer, spi_frame_size);
    // printf("send: ");
    // for(int i=0;i<spi_frame_size;i++)		
    //     printf("0x%02x ", tx_buffer[i]);
    // printf("\n");


    uint8_t enc_cmd[12] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    size = write(device_handle_, (const void *) enc_cmd, 12);


    /* Receive to rx buffer */
    size = read(device_handle_, (void *) rx_buffer_, spi_frame_size); 
    printf("read: ");
    for(int i=0;i<spi_frame_size;i++)		
        printf("0x%02x ", rx_buffer_[i]);
    printf("\n");
    
    ReadBytes enc_bytes1;
    ReadBytes enc_bytes2;
    ReadBytes enc_bytes3;
    ReadBytes enc_bytes4;

    enc_bytes1.s[0] = static_cast<char>(rx_buffer_[1]);
    enc_bytes1.s[1] = static_cast<char>(rx_buffer_[2]);
    enc_bytes2.s[0] = static_cast<char>(rx_buffer_[3]);
    enc_bytes2.s[1] = static_cast<char>(rx_buffer_[4]);
    enc_bytes3.s[0] = static_cast<char>(rx_buffer_[5]);
    enc_bytes3.s[1] = static_cast<char>(rx_buffer_[6]);
    enc_bytes4.s[0] = static_cast<char>(rx_buffer_[7]);
    enc_bytes4.s[1] = static_cast<char>(rx_buffer_[8]);

    read_val->channel1 = enc_bytes1.v.enc;
    read_val->channel1_1 = enc_bytes1.v.pin1;
    read_val->channel1_2 = enc_bytes1.v.pin2;
    read_val->channel2 = enc_bytes2.v.enc;
    read_val->channel2_1 = enc_bytes2.v.pin1;
    read_val->channel2_2 = enc_bytes2.v.pin2;
    read_val->channel3 = enc_bytes3.v.enc;
    read_val->channel3_1 = enc_bytes3.v.pin1;
    read_val->channel3_2 = enc_bytes3.v.pin2;
    read_val->channel4 = enc_bytes4.v.enc;
    read_val->channel4_1 = enc_bytes4.v.pin1;
    read_val->channel4_2 = enc_bytes4.v.pin2;
    return size;
}

int IcoIO::create_pwm_command(uint8_t channel, int16_t value)
{   
    WriteValue write_value;
    WriteBytes write_bytes;
    if (value < 0)
    {
        write_value.pwm_dir = 0;
        if( value < -2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = -value;
        }
    } else {
        write_value.pwm_dir = 1;
        if (value > 2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = value;
        }
    }

    write_bytes.n = write_value;

    int spi_frame_size;
    uint8_t pwm_cmd[12] = {
        OP_CODE::PWM, channel, write_bytes.s[0], write_bytes.s[1],
        0, 0, 0, 0,
        0, 0, 0, 0
    };
    memcpy(tx_buffer_, pwm_cmd, sizeof(tx_buffer_));
    return 1;
}

int IcoIO::create_all_command(uint8_t *buffer, IcoWrite data)
{
    WriteValue val1 = calc_value(data.pwm1, data.val1);
    WriteValue val2 = calc_value(data.pwm2, data.val2);
    WriteValue val3 = calc_value(data.pwm3, data.val3);
    WriteValue val4 = calc_value(data.pwm4, data.val4);
    WriteArray write_array;
    // printf("Converted: %d, %d, %d, %d\n", val1.value, val2.value, val3.value, val4.value);

    write_array.pwm_val1 = val1.pwm_val;
    write_array.pwm_dir1 = val1.pwm_dir;
    write_array.pin_val1 = val1.pin_val;
    write_array.pwm_val2 = val2.pwm_val;
    write_array.pwm_dir2 = val2.pwm_dir;
    write_array.pin_val2 = val2.pin_val;
    write_array.pwm_val3 = val3.pwm_val;
    write_array.pwm_dir3 = val3.pwm_dir;
    write_array.pin_val3 = val3.pin_val;
    write_array.pwm_val4 = val4.pwm_val;
    write_array.pwm_dir4 = val4.pwm_dir;
    write_array.pin_val4 = val4.pin_val;

    WriteArrayBytes write_bytes;
    write_bytes.in = write_array;
    uint8_t msg[12] = {
        OP_CODE::ALL, 0, 
        write_bytes.out[0], write_bytes.out[1],
        write_bytes.out[2], write_bytes.out[3],
        write_bytes.out[4], write_bytes.out[5],
        write_bytes.out[6], write_bytes.out[7],
        0, 0 
    };
    // printf("msg: ");
    // for(int i=0;i<8;i++)		
    //     printf("0x%02x ", msg[i]);
    // printf("\n");
    memcpy(buffer, msg, 12);
    return 1;
}

IcoIO::WriteBytes IcoIO::create_pwm_bytes(int16_t pwm_val, bool pin_val)
{
    WriteValue write_value;
    WriteBytes write_bytes;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if( pwm_val < -2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = -pwm_val;
        }
    } else {
        write_value.pwm_dir = 1;
        if (pwm_val > 2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = pwm_val;
        }
    }
    write_value.pin_val = static_cast<unsigned int>(pin_val);
    write_bytes.n = write_value;
    return write_bytes;
}

IcoIO::WriteValue IcoIO::calc_value(int pwm_val, bool pin_val)
{
    WriteValue write_value;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if( pwm_val < -2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = -pwm_val;
        }
    } else {
        write_value.pwm_dir = 1;
        if (pwm_val > 2047)
        {
            write_value.pwm_val = 2047;
        } else {
            write_value.pwm_val = pwm_val;
        }
    }
    write_value.pin_val = static_cast<unsigned int>(pin_val);
    return write_value;
}
