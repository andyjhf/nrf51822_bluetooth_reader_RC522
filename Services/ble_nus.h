/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_srv_nus Nordic UART Service - Experimental
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service implementation.
 *
 * @details  The Nordic UART Service is a simple GATT based service with a TX and RX
 *           characteristics. Data received from the peer will be passed to the application and the
 *           data received from the application of this service will be sent to the peer as Handle
 *           Value Notifications. This module is intended to demonstrate how custom GATT based
 *           service and characteristics can be implemented using the S110 SoftDevice. This service
 *           is used by the application to send and receive ASCII text strings to and from the
 *           peer.
 *
 * @note     The application must propagate S110 SoftDevice events to the Nordic UART Service module
 *           by calling the ble_nus_on_ble_evt() function from the @ref ble_stack_handler callback.
 */

#ifndef BLE_NUS_H__
#define BLE_NUS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_NUS_SERVICE            			0xfee7                       /**< The UUID of the Nordic UART Service. */
#define BLE_UUID_NUS_WRITE_CHARACTERISTIC  		0xfec7                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_INDICATE_CHARACTERISTIC  0xfec8                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_READ_CHARACTERISTIC  		0xfec9                       /**< The UUID of the TX Characteristic. */

#define BLE_NUS_MAX_DATA_LEN            (GATT_MTU_SIZE_DEFAULT - 3)  /**< Maximum length of data (in bytes) that can be transmitted by the Nordic UART service module to the peer. */

#define BLE_NUS_MAX_RX_CHAR_LEN         BLE_NUS_MAX_DATA_LEN         /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_NUS_MAX_TX_CHAR_LEN         20                           /**< Maximum length of the TX Characteristic (in bytes). */

/**@brief Blood Pressure Service event type. */
typedef enum
{
    BLE_NUS_EVT_INDICATION_ENABLED,                                         /**< Blood Pressure value indication enabled event. */
    BLE_NUS_EVT_INDICATION_DISABLED,                                        /**< Blood Pressure value indication disabled event. */
    BLE_NUS_EVT_INDICATION_CONFIRMED                                        /**< Confirmation of a blood pressure measurement indication has been received. */
} ble_nus_evt_type_t;

/**@brief Blood Pressure Service event. */
typedef struct
{
    ble_nus_evt_type_t evt_type;                                            /**< Type of event. */
} ble_nus_evt_t;

/**@brief Blood Pressure Service event. */
typedef struct
{
//	uint8_t state;
	uint8_t data[512];
	uint16_t startLength;
	uint16_t endLength;
	uint16_t sendLength;
} ble_nus_send_data_t;

typedef struct
{
	uint8_t data[512];
	uint16_t Length;
	uint16_t recvLen;
} ble_nus_recv_data_t;

// Forward declaration of the ble_nus_t type. 
typedef struct ble_nus_s ble_nus_t;

/**@brief Nordic UART Service event handler type. */
typedef void (*ble_nus_data_handler_t) (ble_nus_t * p_nus, uint8_t * data, uint16_t length);

typedef void (*ble_nus_evt_handler_t) (ble_nus_t * p_bps, ble_nus_evt_t * p_evt);

/**@brief   Nordic UART Service init structure.
 *
 * @details This structure contains the initialization information for the service. The application
 *          needs to fill this structure and pass it to the service using the @ref ble_nus_init
 *          function.
 */
typedef struct
{
    ble_nus_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
		ble_nus_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the Blood Pressure Service. */
} ble_nus_init_t;

/**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
typedef struct ble_nus_s
{
    uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic. (as provided by the S110 SoftDevice)*/
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic. (as provided by the S110 SoftDevice)*/
		ble_gatts_char_handles_t read_handles;
	uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the S110 SoftDevice). This will be BLE_CONN_HANDLE_INVALID if not in a connection. */
//    bool                     is_indicate_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_nus_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
		ble_nus_evt_handler_t    evt_handler;                               /**< Event handler to be called for handling events in the Blood Pressure Service. */
		ble_nus_send_data_t 		 send_data;
		ble_nus_recv_data_t			recv_data;
} ble_nus_t;

/**@brief       Function for initializing the Nordic UART Service.
 *
 * @param[out]  p_nus       Nordic UART Service structure. This structure will have to be supplied
 *                          by the application. It will be initialized by this function and will
 *                          later be used to identify this particular service instance.
 * @param[in]   p_nus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 *              This function returns NRF_ERROR_NULL if either of the pointers p_nus or p_nus_init
 *              is NULL.
 */
uint32_t ble_nus_init(ble_nus_t * p_nus, const ble_nus_init_t * p_nus_init);

/**@brief       Nordic UART Service BLE event handler.
 *
 * @details     The Nordic UART service expects the application to call this function each time an
 *              event is received from the S110 SoftDevice. This function processes the event if it
 *              is relevant for it and calls the Nordic UART Service event handler of the
 *              application if necessary.
 *
 * @param[in]   p_nus      Nordic UART Service structure.
 * @param[in]   p_ble_evt  Event received from the S110 SoftDevice.
 */
void ble_nus_on_ble_evt(ble_nus_t * p_nus, ble_evt_t * p_ble_evt);

/**@brief       Function for sending a string to the peer.
 *
 * @details     This function will send the input string as a RX characteristic notification to the
 *              peer.
  *
 * @param[in]   p_nus          Pointer to the Nordic UART Service structure.
 * @param[in]   string         String to be sent.
 * @param[in]   length         Length of string.
 *
 * @return      NRF_SUCCESS if the DFU Service has successfully requested the S110 SoftDevice to
 *              send the notification. Otherwise an error code.
 *              This function returns NRF_ERROR_INVALID_STATE if the device is not connected to a
 *              peer or if the notification of the RX characteristic was not enabled by the peer.
 *              It returns NRF_ERROR_NULL if the pointer p_nus is NULL.
 */
uint32_t ble_nus_send(ble_nus_t * p_nus);
uint32_t ble_nus_is_indication_enabled(ble_nus_t * p_nus, bool * p_indication_enabled);
#endif // BLE_NUS_H__

/** @} */
