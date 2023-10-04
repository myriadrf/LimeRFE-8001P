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

//milans 221223 - This is not used on 1v1
//#define GPIO_SEL_A_LMS8001_BYTE     1
//#define GPIO_SEL_A_LMS8001_BIT      0

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

//milans 221223 - PA was renamed to DA
//#define PA1_A_EN_BYTE               3
//#define PA1_A_EN_BIT                0
#define DA1_A_EN_BYTE               3
#define DA1_A_EN_BIT                0

//#define PA1_B_EN_BYTE               3
//#define PA1_B_EN_BIT                1
#define DA1_B_EN_BYTE               3
#define DA1_B_EN_BIT                1

//#define PA2_A_EN_BYTE               3
//#define PA2_A_EN_BIT                2
#define DA2_A_EN_BYTE               3
#define DA2_A_EN_BIT                2

//#define PA2_B_EN_BYTE               3
//#define PA2_B_EN_BIT                3
#define DA2_B_EN_BYTE               3
#define DA2_B_EN_BIT                3

#define LNA1_EN_BYTE                3
#define LNA1_EN_BIT                 4

#define LNA2_EN_BYTE                3
#define LNA2_EN_BIT                 5

//milans 221223 - DA was renamed to PDA
//#define DA1_EN_BYTE                 3
//#define DA1_EN_BIT                  6
#define PDA1_EN_BYTE                 3
#define PDA1_EN_BIT                  6

//#define DA2_EN_BYTE                 3
//#define DA2_EN_BIT                  7
#define PDA2_EN_BYTE                 3
#define PDA2_EN_BIT                  7


//milans 221223 - PA was renamed to DA
//#define PA1_A_B_CTRL_BYTE           4
//#define PA1_A_B_CTRL_BIT            0
#define DA1_A_B_CTRL_BYTE           4
#define DA1_A_B_CTRL_BIT            0

//milans 221223
#define ORX1_ATT_D0_BYTE            4
#define ORX1_ATT_D0_BIT             1
#define ORX1_ATT_D1_BYTE            4
#define ORX1_ATT_D1_BIT             2
#define ORX1_ATT_D2_BYTE            4
#define ORX1_ATT_D2_BIT             3
#define ORX1_ATT_D3_BYTE            4
#define ORX1_ATT_D3_BIT             4
#define ORX1_ATT_D4_BYTE            4
#define ORX1_ATT_D4_BIT             5
#define ORX1_ATT_D5_BYTE            4
#define ORX1_ATT_D5_BIT             6
#define ORX1_ATT_D6_BYTE            4
#define ORX1_ATT_D6_BIT             7

//milans 221223 - PA was renamed to DA
//#define PA2_A_B_CTRL_BYTE           4
//#define PA2_A_B_CTRL_BIT            1
#define DA2_A_B_CTRL_BYTE           5
#define DA2_A_B_CTRL_BIT            0

#define ORX2_ATT_D0_BYTE            5
#define ORX2_ATT_D0_BIT             1
#define ORX2_ATT_D1_BYTE            5
#define ORX2_ATT_D1_BIT             2
#define ORX2_ATT_D2_BYTE            5
#define ORX2_ATT_D2_BIT             3
#define ORX2_ATT_D3_BYTE            5
#define ORX2_ATT_D3_BIT             4
#define ORX2_ATT_D4_BYTE            5
#define ORX2_ATT_D4_BIT             5
#define ORX2_ATT_D5_BYTE            5
#define ORX2_ATT_D5_BIT             6
#define ORX2_ATT_D6_BYTE            5
#define ORX2_ATT_D6_BIT             7


#define TX1_ATT_D0_BYTE             6
#define TX1_ATT_D0_BIT              1
#define TX1_ATT_D1_BYTE             6
#define TX1_ATT_D1_BIT              2
#define TX1_ATT_D2_BYTE             6
#define TX1_ATT_D2_BIT              3
#define TX1_ATT_D3_BYTE             6
#define TX1_ATT_D3_BIT              4
#define TX1_ATT_D4_BYTE             6
#define TX1_ATT_D4_BIT              5
#define TX1_ATT_D5_BYTE             6
#define TX1_ATT_D5_BIT              6
#define TX1_ATT_D6_BYTE             6
#define TX1_ATT_D6_BIT              7

#define TX2_ATT_D0_BYTE             7
#define TX2_ATT_D0_BIT              1
#define TX2_ATT_D1_BYTE             7
#define TX2_ATT_D1_BIT              2
#define TX2_ATT_D2_BYTE             7
#define TX2_ATT_D2_BIT              3
#define TX2_ATT_D3_BYTE             7
#define TX2_ATT_D3_BIT              4
#define TX2_ATT_D4_BYTE             7
#define TX2_ATT_D4_BIT              5
#define TX2_ATT_D5_BYTE             7
#define TX2_ATT_D5_BIT              6
#define TX2_ATT_D6_BYTE             7
#define TX2_ATT_D6_BIT              7

//milans 221223
/* 
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
*/

//milans 221223
//#define MCU_BYTE                    6
#define MCU_BYTE                    8
#define MCU_TXRX_1_BIT              0
#define MCU_TXRX_2_BIT              1

//milans 221223
//#define MISC_BYTE                   7
#define MISC_BYTE                   9
#define MISC_CHANNEL_BIT            0

void setChainBit(uint8_t targetByte, uint8_t targetBit, uint8_t value, unsigned char* currentState, unsigned char* tmpState);
void resetData();
//void shiftData(unsigned char* data, int dataSize, unsigned char* currentState);
//void shiftData(unsigned char* data, unsigned char* currentState);
void shiftData(unsigned char* data);
void myShiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, unsigned char* val, unsigned int numOfBytes);
