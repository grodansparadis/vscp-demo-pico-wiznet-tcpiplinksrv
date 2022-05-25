
// Command to put core 2 into idle
static const uint32_t _GOTOSLEEP = 0xC0DED02E;


#define ENABLE_UDP_HEARTBEAT
#define ENABLE_MULTICAST_HEARTBEAT
#define ENABLE_NTP
//#define ENABLE_DHCP

// Defines for flash variable storage
#define FLASH_EEPROM_SIM_START  (uint8_t *)(0x1FF000)    // Start of EEPROM simulated flash 4096 bytes, (2MB flash)   
#define FLASH_CTRL_BYTE_MSB     0xFE    // MSB of flash control byte
#define FLASH_CTRL_BYTE_LSB     0xFF    // LSB of flash control byte   

#define NTP_SERVER            "pool.ntp.org"  // https://www.pool.ntp.org/zone/@

// Register definitions

#define REG_DEVICE_ZONE       0x00000000		// Zone for device.
#define REG_DEVICE_SUBZONE    0x00000001		// Subzone for device.
#define REG_LED_CTRL          0x00000002		// LED Control register.
#define REG_LED_STATUS        0x00000003    // LED Status register.

#define REG_SERIAL_CTRL       0x00000004    // Serial channel control register.
#define REG_SERIAL_STATUS     0x00000005    // Serial channel status register.

#define REG_IO_CTRL           0x00000010    // I/O Control register.
#define REG_IO_STATUS         0x00000011    // I/O Status register.

#define REG_TEMP_CTRL         0x00000020    // Temp sensor control register  ADC4
#define REG_TEMP_RAW_MSB      0x00000021    // Temperature raw MSB
#define REG_TEMP_RAW_LSB      0x00000022    // Temperature raw LSB
#define REG_TEMP_CORR_MSB     0x00000023    // Temperature correction factor MSB
#define REG_TMP_CORR_LSB      0x00000024    // Temperature correction factor LSB
#define REG_TMP_INTERVAL      0x00000025    // Temperature report interval

#define REG_NTP_TIME_ZONE     0x00000030    // NTP time zone

#define REG_DM_START          0xFFFE0000    // Start for decision matrix.
#define REG_STANDARD_START    0xFFFF0000		// Start for level II standard registers.


// Decision matrix action definitions.

#define DM_ACTION_NOOP          0x0000        // No operation.
#define DM_ACTION_LED_CTRL      0x0001			  // Arg '0': LED off, Arg '1': LED on.
#define DM_ACTION_IO_CTRL       0x0002			  // Control I/o pins.
#define DM_ACTION_REPORT_IO     0x0003        // Send I/O status events
#define DM_ACTION_REPORT_TEMP   0x0004        // Write action data to serial channel.
#define DM_ACTION_REPORT_ADC    0x0005        // Report ADC value. Arg is channel 0-3
#define DM_ACTION_NTP           0x0006        // Check time from NTP server

void idle_other_core(void);
void resume_other_core(void);

void init_persistent_storage(void);