// ESP32-C6 GPIO matrix output signal names, generated from ESP-IDF v5.3.1
// components/soc/esp32c6/include/soc/gpio_sig_map.h
#include "soc/esp32c6/gpio_pads.hpp"

namespace esp32sim::c6 {

const c3::SignalName kOutSignals[] = {
    {0, "LEDC_LS_SIG_OUT0"}, {1, "LEDC_LS_SIG_OUT1"}, {2, "LEDC_LS_SIG_OUT2"}, {3, "LEDC_LS_SIG_OUT3"},
    {4, "LEDC_LS_SIG_OUT4"}, {5, "LEDC_LS_SIG_OUT5"}, {6, "U0TXD"}, {7, "U0RTS"},
    {8, "U0DTR"}, {9, "U1TXD"}, {10, "U1RTS"}, {11, "U1DTR"},
    {12, "I2S_MCLK"}, {13, "I2SO_BCK"}, {14, "I2SO_WS"}, {15, "I2SO_SD"},
    {16, "I2SI_BCK"}, {17, "I2SI_WS"}, {18, "I2SO_SD1"}, {19, "USB_JTAG_TRST"},
    {20, "CPU_TESTBUS0"}, {21, "CPU_TESTBUS1"}, {22, "CPU_TESTBUS2"}, {23, "CPU_TESTBUS3"},
    {24, "CPU_TESTBUS4"}, {25, "CPU_TESTBUS5"}, {26, "CPU_TESTBUS6"}, {27, "CPU_TESTBUS7"},
    {28, "CPU_GPIO_OUT0"}, {29, "CPU_GPIO_OUT1"}, {30, "CPU_GPIO_OUT2"}, {31, "CPU_GPIO_OUT3"},
    {32, "CPU_GPIO_OUT4"}, {33, "CPU_GPIO_OUT5"}, {34, "CPU_GPIO_OUT6"}, {35, "CPU_GPIO_OUT7"},
    {36, "USB_JTAG_TCK"}, {37, "USB_JTAG_TMS"}, {38, "USB_JTAG_TDI"}, {39, "USB_JTAG_TDO"},
    {40, "USB_EXTPHY_OEN"}, {41, "USB_EXTPHY_SPEED"}, {42, "USB_EXTPHY_VPO"}, {43, "USB_EXTPHY_VMO"},
    {44, "USB_EXTPHY_SUSPND"}, {45, "I2CEXT0_SCL"}, {46, "I2CEXT0_SDA"}, {47, "PARL_TX_DATA0"},
    {48, "PARL_TX_DATA1"}, {49, "PARL_TX_DATA2"}, {50, "PARL_TX_DATA3"}, {51, "PARL_TX_DATA4"},
    {52, "PARL_TX_DATA5"}, {53, "PARL_TX_DATA6"}, {54, "PARL_TX_DATA7"}, {55, "PARL_TX_DATA8"},
    {56, "PARL_TX_DATA9"}, {57, "PARL_TX_DATA10"}, {58, "PARL_TX_DATA11"}, {59, "PARL_TX_DATA12"},
    {60, "PARL_TX_DATA13"}, {61, "PARL_TX_DATA14"}, {62, "PARL_TX_DATA15"}, {63, "FSPICLK"},
    {64, "FSPIQ"}, {65, "FSPID"}, {66, "FSPIHD"}, {67, "FSPIWP"},
    {68, "FSPICS0"}, {69, "SDIO_TOHOST_INT"}, {70, "PARL_TX_CLK"}, {71, "RMT_SIG_OUT0"},
    {72, "RMT_SIG_OUT1"}, {73, "MODEM_DIAG2"}, {74, "MODEM_DIAG3"}, {75, "TWAI0_CLKOUT"},
    {76, "MODEM_DIAG5"}, {77, "MODEM_DIAG6"}, {78, "MODEM_DIAG7"}, {79, "TWAI1_CLKOUT"},
    {80, "MODEM_DIAG9"}, {81, "EXTERN_PRIORITY_O"}, {82, "EXTERN_ACTIVE_O"}, {83, "GPIO_SD0"},
    {84, "GPIO_SD1"}, {85, "GPIO_SD2"}, {86, "GPIO_SD3"}, {87, "PWM0_OUT0A"},
    {88, "PWM0_OUT0B"}, {89, "PWM0_OUT1A"}, {90, "PWM0_OUT1B"}, {91, "PWM0_OUT2A"},
    {92, "PWM0_OUT2B"}, {93, "ANT_SEL0"}, {94, "ANT_SEL1"}, {95, "ANT_SEL2"},
    {96, "ANT_SEL3"}, {97, "SIG_IN_FUNC97"}, {98, "SIG_IN_FUNC98"}, {99, "SIG_IN_FUNC99"},
    {100, "SIG_IN_FUNC100"}, {101, "FSPICS1"}, {102, "FSPICS2"}, {103, "FSPICS3"},
    {104, "FSPICS4"}, {105, "FSPICS5"}, {106, "MODEM_DIAG21"}, {107, "MODEM_DIAG22"},
    {108, "MODEM_DIAG23"}, {109, "MODEM_DIAG24"}, {110, "MODEM_DIAG25"}, {111, "MODEM_DIAG26"},
    {112, "MODEM_DIAG27"}, {113, "MODEM_DIAG28"}, {114, "SPICLK"}, {115, "SPICS0"},
    {116, "SPICS1"}, {117, "GPIO_TASK_MATRIX_OUT0"}, {118, "GPIO_TASK_MATRIX_OUT1"}, {119, "GPIO_TASK_MATRIX_OUT2"},
    {120, "GPIO_TASK_MATRIX_OUT3"}, {121, "SPIQ"}, {122, "SPID"}, {123, "SPIHD"},
    {124, "SPIWP"}, {125, "CLK_OUT_OUT1"}, {126, "CLK_OUT_OUT2"}, {127, "CLK_OUT_OUT3"},
};
const size_t kOutSignalCount = sizeof kOutSignals / sizeof kOutSignals[0];

}  // namespace esp32sim::c6
