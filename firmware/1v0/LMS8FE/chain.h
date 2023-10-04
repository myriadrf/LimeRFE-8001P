#include "consts.h"

#define SPI_2_MCU_DIR_OUT_in_BYTE   0
#define SPI_2_MCU_DIR_OUT_in_BIT    0

#define LMS8001_1_SSENn_BYTE        0
#define LMS8001_1_SSENn_BIT         1

#define LMS8001_2_SSENn_BYTE        0
#define LMS8001_2_SSENn_BIT         2

#define EXT_PLL_SSENn_BYTE          0
#define EXT_PLL_SSENn_BIT           3

#define LMS8001_1_RESETn_BYTE       0
#define LMS8001_1_RESETn_BIT        4

#define LMS8001_2_RESETn_BYTE       0
#define LMS8001_2_RESETn_BIT        5

#define SC1905_1_SSENn_BYTE         0
#define SC1905_1_SSENn_BIT          6

#define SC1905_2_SSENn_BYTE         0
#define SC1905_2_SSENn_BIT          7


#define GPIO_SEL_A_LMS8001_BYTE     1
#define GPIO_SEL_A_LMS8001_BIT      0

#define SC1905_1_RESETn_BYTE        1
#define SC1905_1_RESETn_BIT         1

#define SC1905_2_RESETn_BYTE        1
#define SC1905_2_RESETn_BIT         2


#define BYPASS_AMP1_BYTE            2
#define BYPASS_AMP1_BIT             0

#define DISABLE_AMP1_BYTE           2
#define DISABLE_AMP1_BIT            1

#define BYPASS_AMP2_BYTE            2
#define BYPASS_AMP2_BIT             2

#define DISABLE_AMP2_BYTE           2
#define DISABLE_AMP2_BIT            3


#define PA1_A_EN_BYTE               3
#define PA1_A_EN_BIT                0

#define PA1_B_EN_BYTE               3
#define PA1_B_EN_BIT                1

#define PA2_A_EN_BYTE               3
#define PA2_A_EN_BIT                2

#define PA2_B_EN_BYTE               3
#define PA2_B_EN_BIT                3

#define LNA1_EN_BYTE                3
#define LNA1_EN_BIT                 4

#define LNA2_EN_BYTE                3
#define LNA2_EN_BIT                 5

#define DA1_EN_BYTE                 3
#define DA1_EN_BIT                  6

#define DA2_EN_BYTE                 3
#define DA2_EN_BIT                  7


#define PA1_A_B_CTRL_BYTE           4
#define PA1_A_B_CTRL_BIT            0

#define PA2_A_B_CTRL_BYTE           4
#define PA2_A_B_CTRL_BIT            1

#define PA1_CPL_D0_BYTE             4
#define PA1_CPL_D0_BIT              2

#define PA1_CPL_D1_BYTE             4
#define PA1_CPL_D1_BIT              3

#define PA1_CPL_D2_BYTE             4
#define PA1_CPL_D2_BIT              4

#define PA1_CPL_D3_BYTE             4
#define PA1_CPL_D3_BIT              5

#define PA1_CPL_D4_BYTE             4
#define PA1_CPL_D4_BIT              6

#define PA1_CPL_D5_BYTE             4
#define PA1_CPL_D5_BIT              7


#define PA1_CPL_D6_BYTE             5
#define PA1_CPL_D6_BIT              0

#define PA2_CPL_D0_BYTE             5
#define PA2_CPL_D0_BIT              1

#define PA2_CPL_D1_BYTE             5
#define PA2_CPL_D1_BIT              2

#define PA2_CPL_D2_BYTE             5
#define PA2_CPL_D2_BIT              3

#define PA2_CPL_D3_BYTE             5
#define PA2_CPL_D3_BIT              4

#define PA2_CPL_D4_BYTE             5
#define PA2_CPL_D4_BIT              5

#define PA2_CPL_D5_BYTE             5
#define PA2_CPL_D5_BIT              6

#define PA2_CPL_D6_BYTE             5
#define PA2_CPL_D6_BIT              7

#define MCU_BYTE                    6
#define MCU_TXRX_1_BIT              0
#define MCU_TXRX_2_BIT              1

#define MISC_BYTE                   7
#define MISC_CHANNEL_BIT            0

void setChainBit(uint8_t targetByte, uint8_t targetBit, uint8_t value, unsigned char* currentState, unsigned char* tmpState);
void resetData();
//void shiftData(unsigned char* data, int dataSize, unsigned char* currentState);
//void shiftData(unsigned char* data, unsigned char* currentState);
void shiftData(unsigned char* data);
void myShiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, unsigned char* val, unsigned int numOfBytes);
