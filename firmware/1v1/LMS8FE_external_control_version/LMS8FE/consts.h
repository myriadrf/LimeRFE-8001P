//milans 221223
//#define CHAIN_SIZE              6  // chain bytes
#define CHAIN_SIZE              8  // chain bytes
//#define STATE_SIZE              7  // board state bytes (chain bytes + MCU STATE byte)
//milans 221223
//#define STATE_SIZE              8  // board state bytes (chain bytes + MCU state byte + MISC byte)
#define STATE_SIZE              10  // board state bytes (chain bytes + MCU state byte + MISC byte)

#define BUFFER_SIZE             16  // communication buffer size 16 bytes
#define BUFFER_SIZE_MODE         2  // communication buffer size 2 bytes for CMD_MODE
#define BUFFER_SIZE_LMS8001     64  // communication buffer size 64 bytes for communication with LMS8001

//#define RESETN_SC1904_PIN       11  // RP2040 GPIO pin connected to SC1904 RESETN PIN - This is just for test, on the LMS8FE board it will be connected to a shift register in the daisy chain
//#define RESETN_LMS8001_PIN      10  // RP2040 GPIO pin connected to LMS8001 RESETN PIN - This is just for test, on the LMS8FE board it will be connected to a shift register in the daisy chain
// RP2040 GPIO pin connected to SC1904 LOADENB PIN - This is just for test, on the LMS8FE board, there is no direct connection.
// Spare MCU pins are connected to a header. SC1905 control pins are connected to another header. In the current version the only way
// to establish connection is to connect a wire between the appropriate pins of these headers.
//#define LOADENB_SC1904_PIN      10  // Change this!!!
//#define LOADENB_SC1904_1_PIN    13
//#define LOADENB_SC1904_2_PIN    23
//milans 221223
//#define LOADENB_SC1905_1_PIN    13
//#define LOADENB_SC1905_2_PIN    23
#define LOADENB_SC1905_1_PIN    16
#define LOADENB_SC1905_2_PIN    17

#define LMS64C_CTRL_HEADER_SIZE  8

#define PERIPH_SC1905    0
#define PERIPH_LMS8001   1
#define PERIPH_ADF4002   2

//get info
#define FW_VER     1
#define DEV_TYPE  LMS_DEV_LMS8FE
#define LMS_PROTOCOL_VER 1
#define HW_VER    0
#define EXP_BOARD   EXP_BOARD_UNSUPPORTED
