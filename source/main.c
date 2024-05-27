// SPDX-FileCopyrightText: 2024 Infineon Technologies AG
// SPDX-License-Identifier: MIT

/**
 * \file main.c
 * \brief Main function starting up FreeRTOS for NBT pass-through mode usecase.
 */
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "infineon/ifx-apdu.h"
#include "infineon/ifx-logger.h"
#include "infineon/logger-printf.h"
#include "infineon/logger-cyhal-rtos.h"
#include "infineon/ifx-protocol.h"
#include "infineon/i2c-cyhal.h"
#include "infineon/ifx-t1prime.h"
#include "infineon/nbt-cmd.h"

#include "nbt-utilities.h"

#ifndef NBT_PIN_IRQ
/**
 * \brief NBT IRQ pin required to get pass-through notifications.
 */
#define NBT_PIN_IRQ P6_2
#endif

/**
 * \brief String used as source information for logging.
 */
#define LOG_TAG "NBT example"

/**
 * \brief NBT framework logger.
 */
ifx_logger_t logger_implementation;

/**
 * \brief ModusToolbox CYHAL I2C driver for communication with NBT.
 */
static cyhal_i2c_t i2c_device;

/**
 * \brief Adapter between ModusToolbox CYHAL I2C driver and NBT library framework.
 */
static ifx_protocol_t driver_adapter;

/**
 * \brief Communication protocol stack for NBT library framework.
 */
static ifx_protocol_t communication_protocol;

/**
 * \brief NBT abstraction.
 */
static nbt_cmd_t nbt;

/**
 * \brief FreeRTOS mutex waiting for NBT interrupts.
 */
static SemaphoreHandle_t nbt_irq_sleeper;

/**
 * \brief Interrupt handler for NBT IRQ pin.
 *
 * \param[in] handler_arg ignored.
 * \param[in] event ignored.
 */
static void nbt_irq(void *handler_arg, cyhal_gpio_event_t event)
{
    (void) handler_arg;
    (void) event;

    static BaseType_t xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(nbt_irq_sleeper, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * \brief T=1' interrupt handler.
 *
 * \param[in] self ignored.
 * \param[in] timeout_us Timeout until interrupt has to have triggered (in [us]).
 * \return ifx_status_t \c IFX_T1PRIME_IRQ_TRIGGERED if successful, any other value in case of error.
 * \see nbt_irq_sleeper
 */
static ifx_status_t nbt_t1prime_irq(ifx_protocol_t *self, uint32_t timeout_us)
{
    (void) self;

    uint64_t ms = (uint64_t) ceil(((double) timeout_us) / 1000.0);
    if (xSemaphoreTake(nbt_irq_sleeper, pdMS_TO_TICKS(ms)) != pdPASS)
    {
        return IFX_ERROR(LIB_T1PRIME, IFX_T1PRIME_IRQ, IFX_T1PRIME_IRQ_NOT_TRIGGERED);
    }
    return IFX_T1PRIME_IRQ_TRIGGERED;
}

/**
 * \brief Callback data for nbt_irq().
 */
static cyhal_gpio_callback_data_t nbt_irq_data = {.callback = nbt_irq, .callback_arg = NULL};

/**
 * \brief Callback used to handle APDU data received via pass-through mode.
 *
 * \details Can be used to implement application logic for usecase.
 *
 * \details If additional data (besides the status word) needs to be sent back via pass-through mode, `response->data` must be dynamically allocated
 * and will be freed by the actual task.
 *
 * \param[in] command Command APDU as received via pass-through mode
 * \param[out] response_buffer Buffer to store command response in.
 * \return ifx_status_t \c IFX_SUCCESS if successful, any other value in case of error.
 */
ifx_status_t apdu_handler(ifx_apdu_t *command, ifx_apdu_response_t *response_buffer)
{
    // Validate parameters
    if ((command == NULL) || (response_buffer == NULL))
    {
        return IFX_ERROR(LIB_PROTOCOL, IFX_PROTOCOL_TRANSCEIVE, IFX_ILLEGAL_ARGUMENT);
    }

    // Clear response
    response_buffer->len = 0U;
    response_buffer->data = NULL;

    // Handle command
    static bool selected = false;
    switch (command->cla)
    {
    case 0x00U:
        switch (command->ins)
        {
        // SELECT
        case 0xA4U:
            if ((command->p1 == 0x04U) && (command->p2 == 0x00U))
            {
                const uint8_t expected_aid[] = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U};
                if ((command->lc == sizeof(expected_aid)) && (memcmp(expected_aid, command->data, sizeof(expected_aid)) == 0))
                {
                    ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_DEBUG, "Successfully selected custom application");
                    selected = true;
                    response_buffer->sw = 0x9000U;
                }
                else
                {
                    ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (File not found)");
                    response_buffer->sw = 0x6A82U;
                }
            }
            else
            {
                ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Wrong parameter(s) P1-P2)");
                response_buffer->sw = 0x6B00;
            }
            break;

        // SET_LED
        case 0x00U:
            if (command->p1 == 0x00U)
            {
                if (command->lc == 0x00U)
                {
                    switch (command->p2)
                    {
                    case 0x00U:
                        if (!selected)
                        {
                            // clang-format off
                            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Instruction code not supported or invalid)");
                            // clang-format on
                            response_buffer->sw = 0x6D00U;
                        }
                        else
                        {
                            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_DEBUG, "Turning off LED");
                            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
                            response_buffer->sw = 0x9000U;
                        }
                        break;
                    case 0x01U:
                        if (!selected)
                        {
                            // clang-format off
                            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Instruction code not supported or invalid)");
                            // clang-format on
                            response_buffer->sw = 0x6D00U;
                        }
                        else
                        {
                            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_DEBUG, "Turning on LED");
                            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
                            response_buffer->sw = 0x9000U;
                        }
                        break;
                    default:
                        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Wrong parameter(s) P1-P2)");
                        response_buffer->sw = 0x6B00U;
                        break;
                    }
                }
                else
                {
                    // clang-format off
                    ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (The parameters in the data field are incorrect)");
                    // clang-format on
                    response_buffer->sw = 0x6A80U;
                }
            }
            else
            {
                ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Wrong parameter(s) P1-P2)");
                response_buffer->sw = 0x6B00U;
            }
            break;
        default:
            // clang-format off
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Instruction code not supported or invalid)");
            // clang-format on
            response_buffer->sw = 0x6D00U;
            break;
        }
        break;
    default:
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_WARN, "Invalid APDU received via NBT pass-through mode (Class not supported)");
        response_buffer->sw = 0x6E00U;
        break;
    }
    return IFX_SUCCESS;
}

/**
 * \brief Configures NBT for pass-through mode usecase.
 *
 * \param[in] nbt NBT abstraction for communication.
 * \return ifx_status_t \c IFX_SUCCESS if successful, any other value in case of error.
 */
ifx_status_t nbt_configure_pt(nbt_cmd_t *nbt)
{
    if (nbt == NULL)
    {
        return IFX_ERROR(LIB_NBT_APDU, NBT_SET_CONFIGURATION, IFX_ILLEGAL_ARGUMENT);
    }

    // Cache and clear T=1' interrupt handler in case NBT currently not configured correctly
    ifx_t1prime_irq_handler_t irq_cache = NULL;
    ifx_status_t status = ifx_t1prime_get_irq_handler(nbt->protocol, &irq_cache);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not cache current T=1' interrupt handler");
        return status;
    }
    status = ifx_t1prime_set_irq_handler(nbt->protocol, NULL);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not clear current T=1' interrupt handler");
        return status;
    }

    // Update file access policy
    const nbt_file_access_policy_t fap_cc = {.file_id = NBT_FILEID_CC,
                                             .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                             .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                             .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                             .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_ndef = {.file_id = NBT_FILEID_NDEF,
                                               .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                               .i2c_write_access_condition = NBT_ACCESS_ALWAYS,
                                               .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                               .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_fap = {.file_id = NBT_FILEID_FAP,
                                              .i2c_read_access_condition = NBT_ACCESS_ALWAYS,
                                              .i2c_write_access_condition = NBT_ACCESS_ALWAYS,
                                              .nfc_read_access_condition = NBT_ACCESS_ALWAYS,
                                              .nfc_write_access_condition = NBT_ACCESS_ALWAYS};
    const nbt_file_access_policy_t fap_proprietary1 = {.file_id = NBT_FILEID_PROPRIETARY1,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_proprietary2 = {.file_id = NBT_FILEID_PROPRIETARY2,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_proprietary3 = {.file_id = NBT_FILEID_PROPRIETARY3,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t fap_proprietary4 = {.file_id = NBT_FILEID_PROPRIETARY4,
                                                       .i2c_read_access_condition = NBT_ACCESS_NEVER,
                                                       .i2c_write_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_read_access_condition = NBT_ACCESS_NEVER,
                                                       .nfc_write_access_condition = NBT_ACCESS_NEVER};
    const nbt_file_access_policy_t *faps[] = {&fap_cc, &fap_ndef, &fap_fap, &fap_proprietary1, &fap_proprietary2, &fap_proprietary3, &fap_proprietary4};
    const struct nbt_configuration configuration = {.fap = (nbt_file_access_policy_t **) faps,
                                                    .fap_len = sizeof(faps) / sizeof(struct nbt_configuration *),
                                                    .communication_interface = NBT_COMM_INTF_NFC_ENABLED_I2C_ENABLED,
                                                    .irq_function = NBT_GPIO_FUNCTION_NFC_I2C_PASS_THROUGH_IRQ_OUTPUT};
    status = nbt_configure(nbt, &configuration);

    // Re-set T=1' interrupt handler
    ifx_t1prime_set_irq_handler(nbt->protocol, irq_cache);
    return status;
}

/**
 * \brief Example FreeRTOS task showing how to use NBT pass-through mode functionality.
 *
 * \details
 *   * Opens communication channel to NBT.
 *   * Configures NBT for pass-through mode usecase.
 *   * Sets LED default state (OFF).
 *   * Waits for NBT interrupt indicating pass through data requested.
 *   * Reads command from NBT.
 *   * Updates LED state based on given configuration (via apdu_handler()).
 *   * Writes response to NBT.
 *   * Waits for NBT interrupt again.
 *
 * \see nbt_configure_pt()
 */
void nbt_pt_task(void *arg)
{
    // Activate communication channel to NBT
    uint8_t *atpo = NULL;
    size_t atpo_len = 0U;
    ifx_status_t status = ifx_protocol_activate(&communication_protocol, &atpo, &atpo_len);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_FATAL, "Could not open communication channel to NBT");
        goto cleanup;
    }

    // Set NBT to PT configuration
    status = nbt_configure_pt(&nbt);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_FATAL, "Could not set NBT to PT configuration");
        goto cleanup;
    }

    // Late enable T=1' interrupt in case previous configuration did not use feature
    if (ifx_error_check(ifx_t1prime_set_irq_handler(nbt.protocol, &nbt_t1prime_irq)))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_FATAL, "Could not configure T=1' interrupt");
        goto cleanup;
    }

    // Clear "old" interrupt just in case
    xSemaphoreTake(nbt_irq_sleeper, 0U);

    while (1)
    {
        // Wait for NBT to signalize pass-trough data is available
        while (xSemaphoreTake(nbt_irq_sleeper, portMAX_DELAY) != pdPASS)
            ;

        // Read command from NBT
        ifx_apdu_t command;
        ifx_status_t status = nbt_get_passthrough_apdu(&nbt, &command);
        if (ifx_error_check(status))
        {
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not fetch pass-through data from NBT");
            continue;
        }

        // Prepare response depending on command
        ifx_apdu_response_t command_response = {0};
        status = apdu_handler(&command, &command_response);
        ifx_apdu_destroy(&command);
        if (ifx_error_check(status))
        {
            // clang-format off
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Error occurred in pass-through APDU command handler - please check your implementation");
            // clang-format off
            CY_ASSERT(0);
        }

        // Send response via NBT
        status = nbt_set_passthrough_response(&nbt, &command_response);
        ifx_apdu_response_destroy(&command_response);
        if (ifx_error_check(status))
        {
            ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not send pass-through response to NBT");
            xSemaphoreTake(nbt_irq_sleeper, 0U);
            continue;
        }
    }

cleanup:
    cyhal_i2c_free(&i2c_device);
    ifx_protocol_destroy(&communication_protocol);
    nbt_destroy(&nbt);
    vTaskDelete(NULL);
}

/**
 * \brief Main function starting NBT asynchronous data transfer usecase via FreeRTOS tasks.
 * \details Prepares ModusToolbox and NBT framwework components and starts actual Pass-Through Mode task.
 * \see nbt_pt_task
 */
int main(void)
{
    ///////////////////////////////////////////////////////////////////////////
    // ModusTooblbox start-up boilerplate
    ///////////////////////////////////////////////////////////////////////////
    cy_rslt_t result;
#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    __enable_irq();

    ///////////////////////////////////////////////////////////////////////////
    // ModusTooblbox component configuration
    ///////////////////////////////////////////////////////////////////////////

    // RetargetIO for logging data via serial connection
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    printf("\x1b[2J\x1b[;H");
    printf("****************** "
           "NBT: Pass Through "
           "****************** \r\n\n");

    // I2C driver for communication with NBT
    cyhal_i2c_cfg_t i2c_cfg = {.is_slave = false, .address = 0x00U, .frequencyhal_hz = 400000U};
    result = cyhal_i2c_init(&i2c_device, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&i2c_device, &i2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // User LED displaying current state
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // NBT interrupt to get pass-through data notifications
    result = cyhal_gpio_init(NBT_PIN_IRQ, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    nbt_irq_sleeper = xSemaphoreCreateBinary();
    if (nbt_irq_sleeper == NULL)
    {
        CY_ASSERT(0);
    }
    cyhal_gpio_register_callback(NBT_PIN_IRQ, &nbt_irq_data);
    cyhal_gpio_enable_event(NBT_PIN_IRQ, CYHAL_GPIO_IRQ_RISE, configMAX_PRIORITIES - 1U, true);

    ///////////////////////////////////////////////////////////////////////////
    // NBT library configuration
    ///////////////////////////////////////////////////////////////////////////

    // Logging framework
    ifx_status_t status = logger_printf_initialize(&logger_implementation);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = ifx_logger_set_level(&logger_implementation, IFX_LOG_DEBUG);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = logger_cyhal_rtos_initialize(ifx_logger_default, &logger_implementation);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = ifx_logger_set_level(ifx_logger_default, IFX_LOG_DEBUG);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }
    status = logger_cyhal_rtos_start(ifx_logger_default, NULL);
    if (ifx_error_check(status))
    {
        CY_ASSERT(0);
    }

    // I2C driver adapter
    status = i2c_cyhal_initialize(&driver_adapter, &i2c_device, NBT_DEFAULT_I2C_ADDRESS);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize I2C driver adapter");
        CY_ASSERT(0);
    }

    // Communication protocol (data link layer)
    status = ifx_t1prime_initialize(&communication_protocol, &driver_adapter);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize NBT communication protocol");
        CY_ASSERT(0);
    }
    ifx_protocol_set_logger(&communication_protocol, ifx_logger_default);

    // NBT command abstraction
    status = nbt_initialize(&nbt, &communication_protocol, ifx_logger_default);
    if (ifx_error_check(status))
    {
        ifx_logger_log(ifx_logger_default, LOG_TAG, IFX_LOG_ERROR, "Could not initialize NBT abstraction");
        CY_ASSERT(0);
    }

    ///////////////////////////////////////////////////////////////////////////
    // FreeRTOS start-up
    ///////////////////////////////////////////////////////////////////////////
    xTaskCreate(nbt_pt_task, (char *) "NBT example", 2048U, 0U, configMAX_PRIORITIES - 1U, NULL);
    vTaskStartScheduler();

    ///////////////////////////////////////////////////////////////////////////
    // Cleanup (should not be reached)
    ///////////////////////////////////////////////////////////////////////////
    vSemaphoreDelete(nbt_irq_sleeper);
    cy_retarget_io_deinit();
    ifx_logger_destroy(&logger_implementation);

    CY_ASSERT(0);
    return -1;
}
