/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* GPIO_AD_25 (coord M15), LPUART1_RXD/J32[2] */
/* Routed pin properties */
#define BOARD_INITPINS_LPUART1_RXD_PERIPHERAL                            LPUART1   /*!< Peripheral name */
#define BOARD_INITPINS_LPUART1_RXD_SIGNAL                                    RXD   /*!< Signal name */

/* GPIO_AD_24 (coord L13), LPUART1_TXD/J31[2] */
/* Routed pin properties */
#define BOARD_INITPINS_LPUART1_TXD_PERIPHERAL                            LPUART1   /*!< Peripheral name */
#define BOARD_INITPINS_LPUART1_TXD_SIGNAL                                    TXD   /*!< Signal name */

/* GPIO_AD_34 (coord J16), 1588_EVENT0_IN/SD1_VSELECT */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_VSELECT_PERIPHERAL                             USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_VSELECT_SIGNAL                          usdhc_vselect   /*!< Signal name */

/* GPIO_SD_B1_00 (coord B16), SD1_CMD/J15[3]/WIFI_SDIO_CMD/J54[11]/NVCC_SD/U19G[D14] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_CMD_PERIPHERAL                                 USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_CMD_SIGNAL                                  usdhc_cmd   /*!< Signal name */

/* GPIO_SD_B1_01 (coord D15), SD1_CLK/J15[5]/WIFI_SDIO_CLK/J54[9] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_CLK_PERIPHERAL                                 USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_CLK_SIGNAL                                  usdhc_clk   /*!< Signal name */

/* GPIO_SD_B1_02 (coord C15), SD1_D0/J15[7]/WIFI_SDIO_D0/J54[13]/NVCC_SD/U19G[D14] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_D0_PERIPHERAL                                  USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_D0_SIGNAL                                  usdhc_data   /*!< Signal name */
#define BOARD_INITPINS_SD1_D0_CHANNEL                                         0U   /*!< Signal channel */

/* GPIO_SD_B1_03 (coord B17), SD1_D1/J15[8]/WIFI_SDIO_D1/J54[15] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_D1_PERIPHERAL                                  USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_D1_SIGNAL                                  usdhc_data   /*!< Signal name */
#define BOARD_INITPINS_SD1_D1_CHANNEL                                         1U   /*!< Signal channel */

/* GPIO_SD_B1_04 (coord B15), SD1_D2/J15[1]/WIFI_SDIO_D2/J54[17] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_D2_PERIPHERAL                                  USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_D2_SIGNAL                                  usdhc_data   /*!< Signal name */
#define BOARD_INITPINS_SD1_D2_CHANNEL                                         2U   /*!< Signal channel */

/* GPIO_SD_B1_05 (coord A16), SD1_D3/J15[2]/WIFI_SDIO_D3/J54[19] */
/* Routed pin properties */
#define BOARD_INITPINS_SD1_D3_PERIPHERAL                                  USDHC1   /*!< Peripheral name */
#define BOARD_INITPINS_SD1_D3_SIGNAL                                  usdhc_data   /*!< Signal name */
#define BOARD_INITPINS_SD1_D3_CHANNEL                                         3U   /*!< Signal channel */

/* GPIO_AD_35 (coord G17), 1588_EVENT0_OUT/SD_PWREN_B */
/* Routed pin properties */
#define BOARD_INITPINS_SD_PWREN_B_PERIPHERAL                              GPIO10   /*!< Peripheral name */
#define BOARD_INITPINS_SD_PWREN_B_SIGNAL                                 gpio_io   /*!< Signal name */
#define BOARD_INITPINS_SD_PWREN_B_CHANNEL                                     2U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SD_PWREN_B_GPIO                                    GPIO10   /*!< GPIO peripheral base pointer */
#define BOARD_INITPINS_SD_PWREN_B_GPIO_PIN                                    2U   /*!< GPIO pin number */
#define BOARD_INITPINS_SD_PWREN_B_GPIO_PIN_MASK                       (1U << 2U)   /*!< GPIO pin mask */

/* GPIO_AD_32 (coord K16), ENET_MDC/U7[12]/SD1_CD_B/J15[9] */
/* Routed pin properties */
#define BOARD_INITPINS_ENET_MDC_PERIPHERAL                             CM7_GPIO3   /*!< Peripheral name */
#define BOARD_INITPINS_ENET_MDC_SIGNAL                           gpio_mux_io_cm7   /*!< Signal name */
#define BOARD_INITPINS_ENET_MDC_CHANNEL                                      31U   /*!< Signal channel */

/* GPIO_AD_08 (coord R15), LED_RED */
/* Routed pin properties */
#define BOARD_INITPINS_USER_LED_RED_PERIPHERAL                             GPIO9   /*!< Peripheral name */
#define BOARD_INITPINS_USER_LED_RED_SIGNAL                               gpio_io   /*!< Signal name */
#define BOARD_INITPINS_USER_LED_RED_CHANNEL                                   7U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_USER_LED_RED_GPIO                                   GPIO9   /*!< GPIO peripheral base pointer */
#define BOARD_INITPINS_USER_LED_RED_GPIO_PIN                                  7U   /*!< GPIO pin number */
#define BOARD_INITPINS_USER_LED_RED_GPIO_PIN_MASK                     (1U << 7U)   /*!< GPIO pin mask */

/* GPIO_EMC_B2_18 (coord N3), LED_GREEN */
/* Routed pin properties */
#define BOARD_INITPINS_USER_LED_GREEN_PERIPHERAL                           GPIO8   /*!< Peripheral name */
#define BOARD_INITPINS_USER_LED_GREEN_SIGNAL                             gpio_io   /*!< Signal name */
#define BOARD_INITPINS_USER_LED_GREEN_CHANNEL                                28U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_USER_LED_GREEN_GPIO                                 GPIO8   /*!< GPIO peripheral base pointer */
#define BOARD_INITPINS_USER_LED_GREEN_GPIO_PIN                               28U   /*!< GPIO pin number */
#define BOARD_INITPINS_USER_LED_GREEN_GPIO_PIN_MASK                  (1U << 28U)   /*!< GPIO pin mask */

/* GPIO_AD_10 (coord R17), LED_BLUE */
/* Routed pin properties */
#define BOARD_INITPINS_USER_LED_BLUE_PERIPHERAL                            GPIO9   /*!< Peripheral name */
#define BOARD_INITPINS_USER_LED_BLUE_SIGNAL                              gpio_io   /*!< Signal name */
#define BOARD_INITPINS_USER_LED_BLUE_CHANNEL                                  9U   /*!< Signal channel */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_USER_LED_BLUE_GPIO                                  GPIO9   /*!< GPIO peripheral base pointer */
#define BOARD_INITPINS_USER_LED_BLUE_GPIO_PIN                                 9U   /*!< GPIO pin number */
#define BOARD_INITPINS_USER_LED_BLUE_GPIO_PIN_MASK                    (1U << 9U)   /*!< GPIO pin mask */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);                    /* Function assigned for the Cortex-M7F */

/*!
 * @brief configure reset for wifi.
 *
 */
void BOARD_InitM2WifiResetPins(void);         /* Function assigned for the Cortex-M7F */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
