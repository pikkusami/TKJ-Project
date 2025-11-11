/* =========================
 *  Board / pin macros
 * ========================= */

/**
 * @defgroup board_pins Board and pin definitions
 * @brief Default GPIO assignments for peripherals on the JTKJ HAT.
 *
 * @details
 * These macros define the GPIO pin numbers for peripherals and interfaces
 * used by the SDK. They can be referenced directly or overridden at compile time
 * if custom wiring is used.
 *
 * **Default pinout:**
 * | Function | Macro | GPIO | Notes |
 * |-----------|--------|------|-------|
 * | I²C SDA   | @ref DEFAULT_I2C_SDA_PIN | 12 | Default I²C data |
 * | I²C SCL   | @ref DEFAULT_I2C_SCL_PIN | 13 | Default I²C clock |
 * | UART0     | @ref DEFAULT_UART_0      | 0  | Primary UART port |
 * | UART1     | @ref DEFAULT_UART_1      | 1  | Secondary UART port |
 * | SW1       | @ref SW1_PIN / @ref BUTTON1 | 2  | User button 1 |
 * | SW2       | @ref SW2_PIN / @ref BUTTON2 | 22 | User button 2 |
 * | Red LED   | @ref RED_LED_PIN / @ref LED1 | 14 | Onboard LED |
 * | RGB LED R | @ref RGB_LED_R | 18 | RGB red channel |
 * | RGB LED G | @ref RGB_LED_G | 19 | RGB green channel |
 * | RGB LED B | @ref RGB_LED_B | 20 | RGB blue channel |
 * | Buzzer    | @ref BUZZER_PIN | 17 | Active buzzer |
 * | PDM DATA  | @ref PDM_DATA | 16 | Microphone data line |
 * | PDM CLK   | @ref PDM_CLK | 15 | Microphone clock line |
 * | VEML6030 Interrupt | @ref VEML6030_INTERRUPT | 9  | Light sensor INT pin |
 * | HDC2021 Interrupt  | @ref HDC2021_INTERRUPT | 21 | Temp/humidity INT pin |
 * | ICM42670 Interrupt | @ref ICM42670_INT | 6  | IMU INT1 pin |
 *
 * @note These pin numbers correspond to the Raspberry Pi Pico GPIO numbers.
 * @{
 */

#define DEFAULT_I2C_SDA_PIN                     12
#define DEFAULT_I2C_SCL_PIN                     13

#define DEFAULT_UART_0                          0
#define DEFAULT_UART_1                          1

#define SW1_PIN                                 2
#define SW2_PIN                                 22
#define BUTTON1                                 SW1_PIN
#define BUTTON2                                 SW2_PIN

#define RED_LED_PIN                             14
#define LED1                                    RED_LED_PIN

#define RGB_LED_R                               18
#define RGB_LED_G                               19
#define RGB_LED_B                               20

#define BUZZER_PIN                              17

#define PDM_DATA                                16
#define PDM_CLK                                 15

#define VEML6030_INTERRUPT                      9
#define HDC2021_INTERRUPT                       21
#define ICM42670_INT                            6

/** @} */ /* end of group board_pins */
