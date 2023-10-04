#define sbi(p,n) ((p) |= (1UL << (n)))
#define cbi(p,n) ((p) &= ~(1 << (n)))

#define CMD_LMS8FE_HELLO                   0x00

//#define CMD_GET_INFO                0xe1
#define CMD_LMS8FE_GET_INFO           0x11
#define CMD_LMS8FE_SET_CONFIG_FULL    0x12
#define CMD_LMS8FE_GET_CONFIG_FULL    0x13
#define CMD_LMS8FE_RESET              0x14
//milans 221128
#define CMD_LMS8FE_LMS8_ENABLE        0x15
#define CMD_LMS8FE_SELECT_CHANNEL     0x16

//milans 220419
//#define CMD_LMS8FE_DIODE            0xf1
#define CMD_LMS8FE_DIODE            0x71

//milans 220506
//#define CMD_LMS8FE_DIODESPI         0xf2
#define CMD_LMS8FE_DIODESPI         0x72

//milans 220520
//SC1905 Control
//#define RFE_CMD_SC1905_SPI_MESSAGE_MEMORY  0x91
//#define RFE_CMD_SC1905_RESET               0x92
//#define RFE_CMD_SC1905_SPI_SPECIAL_COMMAND 0x93
//#define RFE_CMD_SC1905_SPI_EEPROM          0x94

#define CMD_SC1905_SPI_MESSAGE_MEMORY  0x61
#define CMD_SC1905_RESET               0x62
#define CMD_SC1905_SPI_SPECIAL_COMMAND 0x63
#define CMD_SC1905_SPI_EEPROM          0x64

#define LMS8001_CMD_MASK 0x80

//milans 220614
//#define CMD_LMS8001_RST 0x20 | 0x80
//#define CMD_LMS8001_RST 0xA0
//#define CMD_LMS8001_RD 0x26 | 0x80
//#define CMD_LMS8001_RD 0xA6
