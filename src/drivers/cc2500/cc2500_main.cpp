#include "cc2500.h"
#include <drivers/device/spi.h>
#include <px4_config.h>
#include <stm32_gpio.h>


#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <nuttx/config.h>

#include <uORB/topics/cc2500_message.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_gps_position.h>


extern "C" __EXPORT int cc2500_main(int argc, char *argv[]);

#define cc2500_SPI_BUS_SPEED 2*1000*1000


#define cc2500_spi_bus 4
#define cc2500_spi_bus_cs 2   //对应的CS

#define CC2500_MAX_EXCHANGE_DATA 255


#define SPI_CS_LOW           px4_arch_gpiowrite(GPIO_PORTC|GPIO_PIN14,0)
#define SPI_CS_HIGH          px4_arch_gpiowrite(GPIO_PORTC|GPIO_PIN14,1);
#define WRITE_SINGLE     0x00
#define WRITE_BURST      0x40
#define READ_SINGLE      0x80
#define READ_BURST       0xC0


#ifndef __TI_CC_CC1100_CC2500_H
#define __TI_CC_CC1100_CC2500_H
// Configuration Registers  配置寄存器
#define TI_CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define TI_CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define TI_CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define TI_CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define TI_CCxxx0_SYNC1        0x04        // Sync word, high byte
#define TI_CCxxx0_SYNC0        0x05        // Sync word, low byte
#define TI_CCxxx0_PKTLEN       0x06        // Packet length
#define TI_CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define TI_CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define TI_CCxxx0_ADDR         0x09        // Device address
#define TI_CCxxx0_CHANNR       0x0A        // Channel number
#define TI_CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define TI_CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define TI_CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define TI_CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define TI_CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define TI_CCxxx0_MDMCFG4      0x10        // Modem configuration
#define TI_CCxxx0_MDMCFG3      0x11        // Modem configuration
#define TI_CCxxx0_MDMCFG2      0x12        // Modem configuration
#define TI_CCxxx0_MDMCFG1      0x13        // Modem configuration
#define TI_CCxxx0_MDMCFG0      0x14        // Modem configuration
#define TI_CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define TI_CCxxx0_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation config
#define TI_CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define TI_CCxxx0_AGCCTRL2     0x1B        // AGC control
#define TI_CCxxx0_AGCCTRL1     0x1C        // AGC control
#define TI_CCxxx0_AGCCTRL0     0x1D        // AGC control
#define TI_CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define TI_CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define TI_CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define TI_CCxxx0_FREND1       0x21        // Front end RX configuration
#define TI_CCxxx0_FREND0       0x22        // Front end TX configuration
#define TI_CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define TI_CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define TI_CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define TI_CCxxx0_FSTEST       0x29        // Frequency synthesizer cal control
#define TI_CCxxx0_PTEST        0x2A        // Production test
#define TI_CCxxx0_AGCTEST      0x2B        // AGC test
#define TI_CCxxx0_TEST2        0x2C        // Various test settings
#define TI_CCxxx0_TEST1        0x2D        // Various test settings
#define TI_CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands  命令滤波
#define TI_CCxxx0_SRES         0x30        // Reset chip.
#define TI_CCxxx0_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define TI_CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define TI_CCxxx0_SCAL         0x33        // Calibrate freq synthesizer & disable
#define TI_CCxxx0_SRX          0x34        // Enable RX.
#define TI_CCxxx0_STX          0x35        // Enable TX.
#define TI_CCxxx0_SIDLE        0x36        // Exit RX / TX
#define TI_CCxxx0_SAFC         0x37        // AFC adjustment of freq synthesizer
#define TI_CCxxx0_SWOR         0x38        // Start automatic RX polling sequence
#define TI_CCxxx0_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define TI_CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define TI_CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define TI_CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define TI_CCxxx0_SNOP         0x3D        // No operation.

// Status registers  状态寄存器
#define TI_CCxxx0_PARTNUM      0x30        // Part number
#define TI_CCxxx0_VERSION      0x31        // Current version number
#define TI_CCxxx0_FREQEST      0x32        // Frequency offset estimate
#define TI_CCxxx0_LQI          0x33        // Demodulator estimate for link quality
#define TI_CCxxx0_RSSI         0x34        // Received signal strength indication
#define TI_CCxxx0_MARCSTATE    0x35        // Control state machine state
#define TI_CCxxx0_WORTIME1     0x36        // High byte of WOR timer
#define TI_CCxxx0_WORTIME0     0x37        // Low byte of WOR timer
#define TI_CCxxx0_PKTSTATUS    0x38        // Current GDOx status and packet status
#define TI_CCxxx0_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define TI_CCxxx0_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define TI_CCxxx0_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define TI_CCxxx0_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define TI_CCxxx0_PATABLE      0x3E
#define TI_CCxxx0_TXFIFO       0x3F
#define TI_CCxxx0_RXFIFO       0x3F

// Masks for appended status bytes
#define TI_CCxxx0_LQI_RX       0x01        // Position of LQI byte
#define TI_CCxxx0_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define TI_CCxxx0_WRITE_BURST  0x40
#define TI_CCxxx0_READ_SINGLE  0x80
#define TI_CCxxx0_READ_BURST   0xC0

#endif



namespace cc2500
{
    CC2500	*g_cc2500;
}

CC2500::CC2500(int bus, spi_dev_e device):
                            SPI("cc2500",nullptr, bus, device, SPIDEV_MODE0, cc2500_SPI_BUS_SPEED),
                            _task_should_exit(false),
                            cc2500_spi_bus_high(false)
{

}

CC2500::~CC2500()
{
    _task_should_exit = true;
}

int
CC2500::init(){

    int ret ;
    ret = TI_CC_SPISetup();
    
    if (ret != OK) {
        DEVICE_DEBUG("SPI init failed");
        return -EIO;
    }
  
    
    TI_CC_PowerupResetCCxxxx();
    writeRFSettings();
    //TI_CC_SPIStrobe(TI_CCxxx0_SRX);
    
    ASSERT(_cc2500_task == -1);
    /* start the task */
	_cc2500_task = px4_task_spawn_cmd("cc2500",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT + 5,
                                    1800,
                                    (px4_main_t)&CC2500::task_main_trampoline,
                                    nullptr);

    if (_cc2500_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}



void
CC2500::task_main_trampoline()
{
    cc2500::g_cc2500->run();
}


void
CC2500::run()
{
    bool cc2500_message_updated = false;
    bool global_pos_updated = false;
    bool home_position_updated = false;
    bool mission_result__updated = false;
    bool ctrl_state_updated = false;
    bool gps_pos_updated = false;

    char p_buf[30] = {18,1,0,1,2,3,4,5};
    uint32_t temp32;
    uint16_t temp16;
    char *p;
    math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
    

    int _cc2500_message_pub = orb_subscribe(ORB_ID(cc2500_message));
    memset(&_cc2500_message, 0, sizeof(_cc2500_message));

    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    memset(&_global_pos, 0, sizeof(_global_pos));

    int gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    memset(&_gps_pos, 0, sizeof(_gps_pos));

    int home_pos_sub = orb_subscribe(ORB_ID(home_position));
    memset(&_home_pos, 0, sizeof(_home_pos));

    int mission_result_pub = orb_subscribe(ORB_ID(mission_result));
    memset(&_mission_result, 0, sizeof(_mission_result));

    int	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));;
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));


 //   bool flag_pwm = false;
    //char read_byte;
    while(!_task_should_exit)
    {
        usleep(50000);

        orb_check(_cc2500_message_pub, &cc2500_message_updated);  
        if (cc2500_message_updated) {
            orb_copy(ORB_ID(cc2500_message), _cc2500_message_pub, &_cc2500_message);
            _cc2500_send_location_data_packet = _cc2500_message.cc2500_send_location_data_packet;
        }

        orb_check(global_pos_sub, &global_pos_updated);
        if (global_pos_updated) {
            orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &_global_pos);
        }

        orb_check(gps_pos_sub, &gps_pos_updated);
        if (gps_pos_updated) {
            orb_copy(ORB_ID(vehicle_gps_position), gps_pos_sub, &_gps_pos);
        }


        orb_check(home_pos_sub, &home_position_updated);
        if (home_position_updated) {
            orb_copy(ORB_ID(home_position), home_pos_sub, &_home_pos);

            home_position_altitude = _home_pos.alt;
        }

        orb_check(mission_result_pub, &mission_result__updated);
        if (mission_result__updated) {
            orb_copy(ORB_ID(mission_result), mission_result_pub, &_mission_result);
        } 
        
        orb_check(_ctrl_state_sub, &ctrl_state_updated);
        
        if (ctrl_state_updated) {
            orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
            math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
            _R = q_att.to_dcm();
            math::Vector<3> euler_angles;
            euler_angles = _R.to_euler();
            _yaw = euler_angles(2);

            if(_yaw < 0)
            {
              _yaw = 2 * 3.1415926f + _yaw;
            }
        }


        /*if(global_pos_updated)
        {
          location_data_packet.lattitude = (uint32_t)((float)(_global_pos.lat) * 2147483648.0f / 3.1415926f);
          location_data_packet.longtitude = (uint32_t)((float)(_global_pos.lon) * 2147483648.0f / 3.1415926f);
          location_data_packet.global_altitude = (uint32_t)(_global_pos.alt * 100.0f);
          location_data_packet.altitude = (uint32_t)((float)(_global_pos.alt - home_position_altitude) * 100.0f);
          location_data_packet.yaw = (uint16_t)((_global_pos.yaw) * 65536.0f / (2.0f * 3.1415926f));
        }*/
        
//        temp32 = (uint32_t)((float)(_global_pos.lon) / 180.0f * 2147483648.0f);

        temp32 = (uint32_t)((float)(_gps_pos.lon / 1e7) / 180.0f * 2147483648.0f);
        p = (char*)(&temp32);
        for(int i=0; i<4; i++)
        {
          p_buf[2+i] = (*(p+i));
        }

//        temp32 = (uint32_t)((float)(_global_pos.lat) /180.0f * 2147483648.0f);
        temp32 = (uint32_t)((float)(_gps_pos.lat / 1e7) /180.0f * 2147483648.0f);
        p = (char*)(&temp32);
        for(int i=0; i<4; i++)
        {
          p_buf[6+i] = (*(p+i));
        }
 
        temp32 = (uint32_t)(_global_pos.alt * 100.0f);
        p = (char*)(&temp32);
        for(int i=0; i<4; i++)
        {
          p_buf[10+i] = (*(p+i));
        }        

        
        temp16 = (uint16_t)((_yaw) * 65536.0f / (2.0f * 3.1415926f));
        p = (char*)(&temp16);
        p_buf[14] = *p;
        p_buf[15] = *(p+1);
        
        //temp32 = (uint32_t)((float)(_global_pos.alt - home_position_altitude) * 100.0f);
        //_mission_result.seq_reached = 2;

        temp16 = (uint16_t)(_mission_result.seq_reached);
        p = (char*)(&temp16);
        for(int i=0; i<4; i++)
        {
          p_buf[16+i] = (*(p+i));
        }    

        if(_cc2500_send_location_data_packet)
        {
          //::printf("yuwenbin........................test.......\n");
           RFSendPacket(p_buf,30);
        }
        

        //read_byte = TI_CC_SPIReadReg(TI_CCxxx0_MDMCFG2);
        //::printf("yuewenbin..........test........%d\n",read_byte);

 /*       if(flag_pwm)
        {
          flag_pwm = false;
          px4_arch_gpiowrite((GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14),true);
        }
        else
        {
          flag_pwm = true;
          px4_arch_gpiowrite((GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14),false);
        }*/
    }

}

void
CC2500::Wait_CC_Ready(void)
{
  uint32_t delaytime; 

  px4_arch_configgpio(GPIO_INPUT|GPIO_PORTE|GPIO_PIN5|GPIO_FLOAT|GPIO_SPEED_50MHz);  //
  delaytime = hrt_absolute_time() + 2000;     //延迟2ms
  while(stm32_gpioread(GPIO_PORTE|GPIO_PIN5))
  {
    if((delaytime-hrt_absolute_time())&0x80000000)
    break;    
  }
  px4_arch_configgpio(GPIO_SPI4_MISO);

}


// Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
// it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
// delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
void
CC2500::TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}

uint8_t
CC2500::SPI_CC_SendByte(uint8_t byte)
{
    Wait_CC_Ready();
    transfer(&byte,&byte,1);

  return byte;
}

uint8_t
CC2500::TI_CC_SPISetup(void)
{
  uint8_t ret;
  ret = SPI::init();
  
  if (ret != OK) {
      DEVICE_DEBUG("SPI init failed");
      //return -EIO;
  }
  
  px4_arch_configgpio(GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN14|GPIO_FLOAT|GPIO_SPEED_50MHz);    //配置PC14作为CS线

  return OK;

}

void
CC2500::TI_CC_SPIWriteReg(char addr, char value)
{
  uint8_t data[CC2500_MAX_EXCHANGE_DATA];


  data[0] = (uint8_t)(addr | WRITE_SINGLE);
  data[1] = (uint8_t)(value);

  Wait_CC_Ready();
  transfer(data,data,2);

}

void
CC2500::TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
  uint8_t data[CC2500_MAX_EXCHANGE_DATA];
  data[0] = (uint8_t)(addr | WRITE_BURST);

  for(int i = 0;i <count; i++)
  {
    data[i+1] = (uint8_t)(buffer[i]);
  }
  
  Wait_CC_Ready();

  transfer(data,data,count+1);
}

char
CC2500::TI_CC_SPIReadReg(char addr)
{
  uint8_t data[CC2500_MAX_EXCHANGE_DATA];

  data[0] = addr | READ_SINGLE;
  data[1] = 0;
  Wait_CC_Ready();

  transfer(data,data, 2);


  return (char)(data[1]);
}

void
CC2500::TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  uint8_t data[CC2500_MAX_EXCHANGE_DATA];
  data[0] = (uint8_t)(addr | READ_BURST);

  for(int i = 0;i <count; i++)
  {
    data[i+1] = 0;
  }
  
  Wait_CC_Ready();

  transfer(data,data,count+1);

  for(int i = 0;i <count; i++)
  {
    buffer[i] = data[i+1];
  }

}

// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
char
CC2500::TI_CC_SPIReadStatus(char addr)
{
  uint8_t data[2];

  data[0] = (uint8_t)(addr | READ_SINGLE);
  data[1] = (uint8_t)(0);

  transfer(data,data,2);

  return (char)(data[1]);
}

void
CC2500::TI_CC_SPIStrobe(char strobe)
{
  Wait_CC_Ready();
  SPI_CC_SendByte(strobe);
}

void
CC2500::TI_CC_PowerupResetCCxxxx(void)
{
  
  SPI_CS_HIGH;
  TI_CC_Wait(3000);
  SPI_CS_LOW;
  TI_CC_Wait(6000);
  SPI_CS_HIGH;
  TI_CC_Wait(4500);

  SPI_CS_LOW;        // /CS enable
  Wait_CC_Ready();
  TI_CC_Wait(4500);
  SPI_CC_SendByte(TI_CCxxx0_SRES);
  TI_CC_Wait(4500);
  SPI_CS_HIGH;
}

void
CC2500::GDO0_Init(void)
{

}

void
CC2500::writeRFSettings(void)   //2.4Gs
{
  GDO0_Init();
  
  char PaTabel[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG2,   0x1B);  // GDO2 output pin config.�����½��ؽ����������ر���ȡ
  TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG0,   0x06);  // GDO0 output pin config.��ͬ���ʻ㱻����/�յ�ʱ�����������ݰ�ĩ�˷�����
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length.���ݰ�����
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL1, 0x05);  // Packet automation control.״̬�ֽ�RSSI��LQI��CRC����OK��Ǹ��������ݰ�β�ˣ���ַ��飬�޹㲥
  TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL0, 0x05);  // Packet automation control.����CRCУ׼���ɱ䳤���ݰ���ͬ���ʻ���һ��λ�������ݰ�����
  TI_CC_SPIWriteReg(TI_CCxxx0_ADDR,     0x01);  // Device address.���ݰ�����ʱʹ�õĵ�ַ
  TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.�ŵ���
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL1,  0x06); // Freq synthesizer control.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ2,    0x5D); // Freq control word, high byte ����Ϊ2438MHz
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ1,    0x93); // Freq control word, mid byte.
  TI_CC_SPIWriteReg(TI_CCxxx0_FREQ0,    0xB1); // Freq control word, low byte.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG4,  0x78); // Modem configuration.   ���ƽ��������
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG3,  0x93); // Modem configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG2,  0x03); // Modem configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG1,  0x22); // Modem configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_DEVIATN,  0x44); // Modem dev (when FSK mod en)
  TI_CC_SPIWriteReg(TI_CCxxx0_MCSM1 ,   0x3F); //MainRadio Cntrl State Machine
  TI_CC_SPIWriteReg(TI_CCxxx0_MCSM0 ,   0x18); //MainRadio Cntrl State Machine ���ݰ����պ���뷢��״̬���ź���RSSI���£������ŵ����ز�������
  TI_CC_SPIWriteReg(TI_CCxxx0_FOCCFG,   0x16); // Freq Offset Compens. Config
  TI_CC_SPIWriteReg(TI_CCxxx0_BSCFG,    0x6C); //  Bit synchronization config.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL2, 0x43); // AGC control.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL1, 0x40); // AGC control.
  TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL0, 0x91); // AGC control.
  TI_CC_SPIWriteReg(TI_CCxxx0_FREND1,   0x56); // Front end RX configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL3,   0xA9); // Frequency synthesizer cal.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
  TI_CC_SPIWriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
  TI_CC_SPIWriteReg(TI_CCxxx0_TEST0,    0x0B);  // Various test settings.
  
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE,PaTabel,8);

  TI_CC_SPIStrobe(TI_CCxxx0_SFRX);
  TI_CC_SPIStrobe(TI_CCxxx0_SFTX);
  TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);


}

void
CC2500::RFSendPacket(char *txBuffer, char size)
{
  TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);
  TI_CC_SPIStrobe(TI_CCxxx0_SFTX);

  TI_CC_SPIWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, size); // Write TX data

  TI_CC_SPIStrobe(TI_CCxxx0_STX);
  
}



//-----------------------------------------------------------------------------
//  char RFReceivePacket(char *rxBuffer, char *length)
//
//  DESCRIPTION:
//  Receives a packet of variable length (first byte in the packet must be the
//  length byte).  The packet length should not exceed the RXFIFO size.  To use
//  this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
//  is assumed that the function is called after it is known that a packet has
//  been received; for example, in response to GDO0 going low when it is
//  configured to output packet reception status.
//
//  The RXBYTES register is first read to ensure there are bytes in the FIFO.
//  This is done because the GDO signal will go high even if the FIFO is flushed
//  due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      char *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      char *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  RETURN VALUE:
//      char
//          0x80:  CRC OK
//          0x00:  CRC NOT OK (or no pkt was put in the RXFIFO due to filtering)
//-----------------------------------------------------------------------------
char
CC2500::RFReceivePacket(char *rxBuffer, char *length, unsigned char *RSSI)
{
  char status[2];
  char pktLen;

  if ((TI_CC_SPIReadStatus(TI_CCxxx0_RXBYTES) & TI_CCxxx0_NUM_RXBYTES))
  {
    pktLen = TI_CC_SPIReadReg(TI_CCxxx0_RXFIFO); // Read length byte

    if (pktLen <= *length)                  // If pktLen size <= rxBuffer
    {
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, rxBuffer, pktLen); // Pull data
      *length = pktLen;                     // Return the actual size
      TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, status, 2);
                                            // Read appended status bytes
      *RSSI = status[0];
      
      TI_CC_SPIStrobe(TI_CCxxx0_SFRX);
      TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);
      return (char)(status[TI_CCxxx0_LQI_RX]&TI_CCxxx0_CRC_OK);
      
    }                                       // Return CRC_OK bit
    else
    {
      *length = pktLen;                     // Return the large size
      TI_CC_SPIStrobe(TI_CCxxx0_SFRX);      // Flush RXFIFO
      TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);
      return 0;                             // Error
    }
  }
  else
      return 0;                             // Error
}



int
cc2500_main(int argc, char *argv[])
{

	if (!strcmp(argv[1], "start")) {
        
        if (cc2500::g_cc2500 != nullptr) {
            PX4_WARN("already running");
            return 1;
        }

        cc2500::g_cc2500= new CC2500(cc2500_spi_bus, (spi_dev_e)(2));   //bus: 使能SPI_CS

        if(cc2500::g_cc2500->init() != OK){
            delete cc2500::g_cc2500;
        }
    }


    return 1;
}