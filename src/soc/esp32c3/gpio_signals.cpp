// ESP32-C3 GPIO matrix signal names, generated from ESP-IDF v5.3.1
// components/soc/esp32c3/include/soc/gpio_sig_map.h
#include "soc/esp32c3/gpio.hpp"

namespace esp32sim::c3 {

const SignalName kOutSignals[] = {
    {0, "SPIQ"}, {1, "SPID"}, {2, "SPIHD"}, {3, "SPIWP"},
    {4, "SPICLK"}, {5, "SPICS0"}, {6, "U0TXD"}, {7, "U0RTS"},
    {8, "U0DTR"}, {9, "U1TXD"}, {10, "U1RTS"}, {11, "U1DTR"},
    {12, "I2S_MCLK"}, {13, "I2SO_BCK"}, {14, "I2SO_WS"}, {15, "I2SO_SD"},
    {16, "I2SI_BCK"}, {17, "I2SI_WS"}, {18, "GPIO_BT_PRIORITY"}, {19, "GPIO_BT_ACTIVE"},
    {28, "CPU_GPIO_OUT0"}, {29, "CPU_GPIO_OUT1"}, {30, "CPU_GPIO_OUT2"}, {31, "CPU_GPIO_OUT3"},
    {32, "CPU_GPIO_OUT4"}, {33, "CPU_GPIO_OUT5"}, {34, "CPU_GPIO_OUT6"}, {35, "CPU_GPIO_OUT7"},
    {36, "USB_JTAG_TCK"}, {37, "USB_JTAG_TMS"}, {38, "USB_JTAG_TDI"}, {39, "USB_JTAG_TDO"},
    {40, "USB_EXTPHY_VP"}, {41, "USB_EXTPHY_VM"}, {42, "USB_EXTPHY_RCV"}, {43, "USB_EXTPHY_VMO"},
    {44, "USB_EXTPHY_SUSPND"}, {45, "EXT_ADC_START"}, {46, "LEDC_LS_SIG_OUT1"}, {47, "LEDC_LS_SIG_OUT2"},
    {48, "LEDC_LS_SIG_OUT3"}, {49, "LEDC_LS_SIG_OUT4"}, {50, "LEDC_LS_SIG_OUT5"}, {51, "RMT_SIG_OUT0"},
    {52, "RMT_SIG_OUT1"}, {53, "I2CEXT0_SCL"}, {54, "I2CEXT0_SDA"}, {55, "GPIO_SD0"},
    {56, "GPIO_SD1"}, {57, "GPIO_SD2"}, {58, "GPIO_SD3"}, {59, "I2SO_SD1"},
    {63, "FSPICLK"}, {64, "FSPIQ"}, {65, "FSPID"}, {66, "FSPIHD"},
    {67, "FSPIWP"}, {68, "FSPICS0"}, {69, "FSPICS1"}, {70, "FSPICS2"},
    {71, "FSPICS3"}, {72, "FSPICS4"}, {73, "FSPICS5"}, {74, "TWAI_RX"},
    {75, "TWAI_BUS_OFF_ON"}, {76, "TWAI_CLKOUT"}, {77, "BT_AUDIO0_IRQ"}, {78, "BT_AUDIO1_IRQ"},
    {79, "BT_AUDIO2_IRQ"}, {80, "RW_WAKEUP_REQ"}, {81, "BLE_AUDIO1_IRQ"}, {82, "BLE_AUDIO2_IRQ"},
    {83, "PCMFSYNC"}, {84, "PCMCLK"}, {85, "PCMDOUT"}, {86, "BLE_AUDIO_SYNC0_P"},
    {87, "BLE_AUDIO_SYNC1_P"}, {88, "BLE_AUDIO_SYNC2_P"}, {89, "ANT_SEL0"}, {90, "ANT_SEL1"},
    {91, "ANT_SEL2"}, {92, "ANT_SEL3"}, {93, "ANT_SEL4"}, {94, "ANT_SEL5"},
    {95, "ANT_SEL6"}, {96, "ANT_SEL7"}, {101, "SYNCERR"}, {102, "SYNCFOUND_FLAG"},
    {103, "EVT_CNTL_IMMEDIATE_ABORT"}, {105, "DATA_EN"}, {106, "DATA"}, {107, "PKT_TX_ON"},
    {108, "PKT_RX_ON"}, {109, "RW_TX_ON"}, {110, "RW_RX_ON"}, {111, "EVT_REQ_P"},
    {112, "EVT_STOP_P"}, {113, "BT_MODE_ON"}, {114, "GPIO_LC_DIAG0"}, {115, "GPIO_LC_DIAG1"},
    {116, "GPIO_LC_DIAG2"}, {117, "CH_IDX"}, {119, "UPDATE_RX"}, {120, "RX_STATUS"},
    {121, "CLK_GPIO"}, {122, "NBT_BLE"}, {123, "CLK_OUT_OUT1"}, {124, "CLK_OUT_OUT2"},
    {125, "CLK_OUT_OUT3"}, {126, "SPICS1"}, {127, "USB_JTAG_TRST"}, {128, "SIG_GPIO"},
};

const SignalName kInSignals[] = {
    {0, "SPIQ"}, {1, "SPID"}, {2, "SPIHD"}, {3, "SPIWP"},
    {6, "U0RXD"}, {7, "U0CTS"}, {8, "U0DSR"}, {9, "U1RXD"},
    {10, "U1CTS"}, {11, "U1DSR"}, {12, "I2S_MCLK"}, {13, "I2SO_BCK"},
    {14, "I2SO_WS"}, {15, "I2SI_SD"}, {16, "I2SI_BCK"}, {17, "I2SI_WS"},
    {28, "CPU_GPIO_IN0"}, {29, "CPU_GPIO_IN1"}, {30, "CPU_GPIO_IN2"}, {31, "CPU_GPIO_IN3"},
    {32, "CPU_GPIO_IN4"}, {33, "CPU_GPIO_IN5"}, {34, "CPU_GPIO_IN6"}, {35, "CPU_GPIO_IN7"},
    {51, "RMT_SIG_IN0"}, {52, "RMT_SIG_IN1"}, {53, "I2CEXT0_SCL"}, {54, "I2CEXT0_SDA"},
    {63, "FSPICLK"}, {64, "FSPIQ"}, {65, "FSPID"}, {66, "FSPIHD"},
    {67, "FSPIWP"}, {68, "FSPICS0"}, {77, "PCMFSYNC"}, {78, "PCMCLK"},
    {79, "PCMDIN"}, {97, "SIG_IN_FUNC_97"}, {98, "SIG_IN_FUNC_98"}, {99, "SIG_IN_FUNC_99"},
    {100, "SIG_IN_FUNC_100"}, {104, "LINKLBL"}, {118, "RX_WINDOW"},
};

const size_t kOutSignalCount = sizeof kOutSignals / sizeof kOutSignals[0];
const size_t kInSignalCount = sizeof kInSignals / sizeof kInSignals[0];

}  // namespace esp32sim::c3
