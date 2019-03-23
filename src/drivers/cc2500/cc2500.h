

#ifndef CC2500_H
#define CC2500_H


#include <drivers/device/spi.h>
#include <uORB/topics/cc2500_message.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_gps_position.h>

class CC2500  : public device::SPI
{
public:
    
    CC2500(int bus, spi_dev_e device);
    ~CC2500();

    int       init();
    int		  _cc2500_task{-1};
 
    static void	task_main_trampoline();

    void        run();

    bool		_task_should_exit;

    bool         cc2500_spi_bus_high;


private:
    int         _device_type;

    uint8_t     TI_CC_SPISetup(void);
    void        TI_CC_PowerupResetCCxxxx(void);
    void        TI_CC_SPIWriteReg(char, char);
    void        TI_CC_SPIWriteBurstReg(char, char*, char);
    char        TI_CC_SPIReadReg(char);
    void        TI_CC_SPIReadBurstReg(char, char *, char);
    char        TI_CC_SPIReadStatus(char);
    void        TI_CC_SPIStrobe(char);
    void        TI_CC_Wait(unsigned int);
    uint8_t     SPI_CC_SendByte(uint8_t byte);
    void        Wait_CC_Ready(void);


    void writeRFSettings(void);
    void RFSendPacket(char *, char);
    char RFReceivePacket(char *, char *, unsigned char *);
    void GDO0_Init(void);
    typedef struct _CC2500_RF_SETTINGS_
    {
        uint8_t IOCFG2;    // GDO2 output pin configuration
        uint8_t IOCFG1;
        uint8_t IOCFG0;    // GDO0 output pin configuration
        uint8_t FIFOTHR;
        uint8_t  SYNC1;
        uint8_t  SYNC0;
        uint8_t PKTLEN;    // Packet length.
        uint8_t PKTCTRL1;  // Packet automation control.
        uint8_t PKTCTRL0;  // Packet automation control.
        uint8_t ADDR;      // Device address.
        uint8_t CHANNR;
            
        uint8_t FSCTRL1;   // Frequency synthesizer control.
        uint8_t FSCTRL0;   // Frequency synthesizer control.
        uint8_t FREQ2;     // Frequency control word, high u8.
        uint8_t FREQ1;     // Frequency control word, middle u8.
        uint8_t FREQ0;     // Frequency control word, low u8.
        
        uint8_t MDMCFG4;   // Modem configuration.
        uint8_t MDMCFG3;   // Modem configuration.
        uint8_t MDMCFG2;   // Modem configuration.
        uint8_t MDMCFG1;   // Modem configuration.
        uint8_t MDMCFG0;   // Modem configuration.
        
        uint8_t DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
        uint8_t MCSM2;     // Main Radio Control State Machine configuration.
        uint8_t MCSM1;     // Main Radio Control State Machine configuration.	
        uint8_t MCSM0;     // Main Radio Control State Machine configuration.
        uint8_t FOCCFG;    // Frequency Offset Compensation Configuration.
        uint8_t BSCFG;     // Bit synchronization Configuration.
        uint8_t AGCCTRL2 ;
        uint8_t AGCCTRL1 ;
        uint8_t AGCCTRL0 ;
        uint8_t WOREVT1;
        uint8_t WOREVT0;	
        uint8_t WORCTRL;
        uint8_t FREND1;    // Front end RX configuration.
        uint8_t FREND0;    // Front end RX configuration.
        uint8_t FSCAL3;    // Frequency synthesizer calibration.
        uint8_t FSCAL2;    // Frequency synthesizer calibration.
        uint8_t FSCAL1;    // Frequency synthesizer calibration.
        uint8_t FSCAL0;    // Frequency synthesizer calibration.
        uint8_t RCCTRL1;
        uint8_t  RCCTRL0;
        uint8_t FSTEST;    // Frequency synthesizer calibration control
        uint8_t PTEST;
        uint8_t AGCTEST;
        uint8_t TEST2;     // Various test settings.
        uint8_t TEST1;     // Various test settings.
        uint8_t TEST0;     // Various test settings.
    
    }CC2500_RF_SETTINGS;

    cc2500_message_s    _cc2500_message;
    bool _cc2500_send_location_data_packet = false;
    
    vehicle_global_position_s   _global_pos;
    vehicle_gps_position_s       _gps_pos;
    home_position_s			    _home_pos;
    mission_result_s			_mission_result; 

    struct location_data_packet_s{
        uint32_t lattitude;   //纬度
        uint32_t longtitude;  //经度
        uint32_t altitude;    //相对高度
        uint16_t yaw;          //航向角
        uint32_t global_altitude; //绝对高度
    };
    location_data_packet_s location_data_packet;

    float home_position_altitude;

    struct control_state_s				_ctrl_state;		/**< vehicle attitude */
    float _yaw;
    
};



#endif

