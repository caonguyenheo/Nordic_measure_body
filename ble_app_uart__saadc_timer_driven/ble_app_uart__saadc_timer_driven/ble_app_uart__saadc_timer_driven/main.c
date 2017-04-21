/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "boards.h"
#include "nrf_gpio.h"
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                       /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256 //256                                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256  //256                                       /**< UART RX buffer size. */

#define SAADC_SAMPLES_IN_BUFFER         8    //8
#define SAADC_SAMPLE_RATE_DIVIDER				0x2000;
#define number                          5
#define VCC                             3.19
#define resistor												9890  //9910.0//10000
#define resistor_medium									150
#define SAADC_BURST_MODE                 1
//#define total                            6.1
uint16_t frequency = 2000;
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

volatile uint8_t state = 1;

static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
//static uint32_t                m_adc_evt_counter ;

static uint8_t 			m_adc_channel_enabled = 6; 
//static nrf_saadc_channel_config_t channel_0_config;
//static nrf_saadc_channel_config_t channel_1_config;
//static nrf_saadc_channel_config_t channel_2_config;
//static nrf_saadc_channel_config_t	channel_3_config;
//static nrf_saadc_channel_config_t	channel_4_config;
//static nrf_saadc_channel_config_t	channel_5_config;
static nrf_saadc_channel_config_t channel_0_config;
static nrf_saadc_channel_config_t channel_1_config;
static nrf_saadc_channel_config_t channel_2_config;
static nrf_saadc_channel_config_t channel_3_config;
static nrf_saadc_channel_config_t channel_4_config;
static nrf_saadc_channel_config_t channel_5_config;
const nrf_drv_rtc_t            rtc = NRF_DRV_RTC_INSTANCE(0);
void saadc_init(void);

//const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
		
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,//APP_IRQ_PRIORITY_HIGH//APP_IRQ_PRIORITY_LOW
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
		uint32_t timeout = 5000;//5000
		while (0 == nrf_saadc_event_check(NRF_SAADC_EVENT_END) && timeout > 0)
		{
				timeout--;
		}
		nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
//    uint32_t ticks = SAADC_SAMPLE_RATE_DIVIDER;
	  uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer,25000);//25000
//		uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, frequency);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();
    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}
//void saadc_sampling_event_disable(void)
//{
//    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
//    APP_ERROR_CHECK(err_code);
//}
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
		char buffer[256] = {0}; //buffer_1[256] = {0};
    unsigned int message_length = 0;// message_value = 0;
		static ret_code_t err_code;
//		int i = 0;
		float value_adc_before = 0, value_w = 0, value_h = 0;;
		unsigned int value_th = 0, value_hip_s = 0, value_waist_s = 0;
		float flexV = 0, flexV1 = 0, flexV2 = 0;
		float R_DIV = 10000;///9870.0
		int solandoc = 0 ;
		static float th[2][691] ={{800, 801, 802, 803,804, 805, 806, 807, 808,809,810,811,812,813,814,815,816,817,818,819,820,821,822,823,824,825,826,827,828,829,830,831,832,833,834,835,836,837,838,839,840,841,842,843,844,845,846,847,848,849,850,851,852,853,854,855,856,857,858,859,860,861,862, 863, 864,865,866,867,868,869,870,871,872,873,874,875,876,877,878,879,880,881,882,883,884,885,886,887,888,889,890,891,892,893,894,895,896,897,898,899,900,901,902,903,904,905,906,907,908,909,910,911,912,913,914,915,916,917,918,919,920,921,922,923,924,925,926,927,928,929,930,931,	932,	933,	934,	935,	936,	937,	938,	939,	940,	941,	942,	943,944,945,946,947,948,949,950,951,952,953,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,977,978,979,980,981,982,983,984,985,986,987,988,989,990,991,992,993,994,995,996,997,998,999,1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015,1016,1017,1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029,1030,1031,1032,1033,1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047,1048,1049,1050,1051,1052,1053,1054,1055,1056,1057,1058,1059,1060,1061,1062,1063,1064,1065,1066,1067,1068,1069,1070,1071,1072,1073,1074,1075,1076,1077,1078,1079,1080,1081,1082,1083,1084,1085,1086,1087,1088,1089,1090,1091,1092,1093,1094,1095,1096,1097,1098,1099,1100,1101,1102,1103,1104,1105,1106,1107,1108,1109,1110,1111,1112,1113,1114,1115,1116,1117,1118,1119,1120,1121,1122,1123,1124,1125,1126,1127,1128,1129,1130,1131,1132,1133,1134,1135,1136,1137,1138,1139,1140,1141,1142,1143,1144,1145,1146,1147,1148,1149,1150,1151,1152,1153,1154,1155,1156,1157,1158,1159,1160,1161,1162,1163,1164,1165,1166,1167,1168,1169,1170,1171,1172,1173,1174,1175,1176,1177,1178,1179,1180,1181,1182,1183,1184,1185,1186,1187,1188,1189,1190,1191,1192,1193,1194,1195,1196,1197,1198,1199,1200,1201,1202,1203,1204,1205,1206,1207,1208,1209,1210,1211,1212,1213,1214,1215,1216,1217,1218,1219,1220,1221,1222,1223,1224,1225,1226,1227,1228,1229,1230,1231,1232,1233,1234,1235,1236,1237,1238,1239,1240,1241,1242,1243,1244,1245,1246,1247,1248,1249,1250,1251,1252,1253,1254,1255,1256,1257,1258,1259,1260,1261,1262,1263,1264,1265,1266,1267,1268,1269	,1270,1271,1272,1273,1274,1275,1276,1277,1278,1279,1280,1281,1282,1283,1284,1285,1286,1287,1288,1289,1290,1291,1292,1293,1294,1295,1296,1297,1298,1299,1300,1301,1302,1303,1304,1305,1306,1307,1308,1309,1310,1311,1312,1313,1314,1315,1316,1317,1318,1319,1320,1321,1322,1323,1324,1325,1326,1327,1328,1329,1330,1331,1332,1333,1334,1335,1336,1337,1338,1339,1340,1341,1342,1343,1344,1345,1346,1347,1348,1349,1350,1351,1352,1353,1354,1355,1356,1357,1358,1359,1360,1361,1362,1363,1364,1365,1366,1367,1368,1369,1370,1371,1372,1373,1374,1375,1376,1377,1378,1379,1380,1381,1382,1383,1384,1385,1386,1387,1388,1389,1390,1391,1392,1393,1394,1395,1396,1397,1398,1399,1400,1401,1402,1403,1404,1405,1406,1407,1408,1409,1410,1411,1412,1413,1414,1415,1416,1417,1418,1419,1420,1421,1422,1423,1424,1425,1426,1427,1428,1429,1430,1431,1432,1433,1434,1435,1436,1437,1438,1439,1440,1441,1442,1443,1444,1445,1446,1447,1448,1449,1450,1451,1452,1453,1454,1455,1456,1457,1458,1459,1460,1461,1462,1463,1464,1465,1466,1467,1468,1469,1470,1471,1472,1473,1474,1475,1476,1477,1478,1479,1480,1481,1482,1483,1484,1485,1486,1487,1488,1489,1490},
											{50.01,50.02,50.02,50.03,50.04,50.05,50.06,50.07,50.08,50.09,50.10,50.11,50.12,50.13,50.14,50.15,50.16,50.17,50.18,50.19,50.20,50.21,50.22,50.23,50.24,50.25,50.26,50.27,50.28,50.29,50.30,50.31,50.32,50.33,50.34,50.35,50.36,50.37,50.38,50.39,50.40,50.41,50.42,50.43,50.44,50.45,50.46,50.47,50.48,50.49,50.50,50.51,50.52,50.53,50.54,50.55,50.56,50.57,50.58,50.59,50.60,50.61,50.62,50.63,50.64,50.65,50.66,50.67,50.68,50.69,50.70,50.71,50.72,50.73,50.74,50.75,50.76,50.77,50.78,50.79,50.80,50.81,50.82,50.83,50.84,50.85,50.86,50.87,50.88,50.89,50.90,50.91,50.92,50.93,50.94,50.95,50.96,50.97,50.98,50.99,51.00,51.01,51.02,51.03,51.04,51.05,51.06,51.07,51.08,51.09,51.10,51.11,51.12,51.13,51.14,51.15,51.16,51.17,51.18,51.19,51.20,51.21,51.22,51.23,51.24,51.25,51.26,51.27,51.28,51.29,51.30,51.31,51.32,51.33,51.34,51.35,51.36,51.37,51.38,51.39,51.40,51.41,51.42,51.43,51.44,51.45,51.46,51.47,51.48,51.49,51.50,51.51,51.52,51.53,51.54,51.55,51.56,51.57,51.58,51.59,51.60,51.61,51.62,51.63,51.64,51.65,51.66,51.67,51.68,51.69,51.70,51.71,51.72,51.73,51.74,51.75,51.76,51.77,51.78,51.79,51.80,51.81,51.82,51.83,51.84,51.85,51.86,51.87,51.88,51.89,51.90,51.91,51.92,51.93,51.94,51.95,51.96,51.97,51.98,51.99,52.00,52.01,52.02,52.03,52.04,52.05,52.06,52.07,52.08,52.09,52.10,52.11,52.12,52.13,52.14,52.15,52.16,52.17,52.18,52.19,52.20,52.21,52.22,52.23,52.24,52.25,52.26,52.27,52.28,52.29,52.30,	52.31,52.32,52.33,52.34,52.35,52.36,52.37,52.38,52.39,52.40,52.41,52.42,52.43,52.44,52.45,52.46,52.47,52.48,52.49,52.50,52.51,52.52,52.53,52.54,52.55,52.56,52.57,52.58,52.59,52.60,52.61,52.62,52.63,52.64,52.65,52.66,52.67,52.68,52.69,52.70,52.71,52.72,52.73,52.74,52.75,52.76,52.77,52.78,52.79,52.80,52.81,52.82,52.83,52.84,52.85,52.86,52.87,52.88,52.89,52.90,52.91,52.92,52.93,52.94,52.95,52.96,52.97,52.98,52.99,53.00,53.01,53.02,53.03,53.04,53.05,53.06,53.07,53.08,53.09,53.10,53.11,53.12,53.13,53.14,53.15,53.16,53.17,53.18,53.19,53.20,53.21,53.22,53.23,53.24,53.25,53.26,53.27,53.28,53.29,53.30,53.31,53.32,53.33,53.34,53.35,53.36,53.37,53.38,53.39,53.40,53.41,53.42,53.43,53.44,53.45,53.46,53.47,53.48,53.49,53.50,53.51,53.52,53.53,53.54,53.55,53.56,53.57,53.58,53.59,53.60,53.61,53.62,53.63,53.64,53.65,53.66,53.67,53.68,53.69,53.70,53.71,53.72,53.73,53.74,53.75,53.76,53.77,53.78,53.79,53.80,53.81,53.82,53.83,53.84,53.85,53.86,53.87,53.88,53.89,53.90,53.91,53.92,53.93,53.94,53.95,53.96,53.97,53.98,53.99,54.00,54.01,54.02,54.03,54.04,54.05,54.06,54.07,54.08,54.09,54.10,54.11,54.12,54.13,54.14,54.15,54.16,54.17,54.18,54.19,54.20,54.21,54.22,54.23,54.24,54.25,54.26,54.27,54.28,54.29,54.30,54.31,54.32,54.33,54.34,54.35,54.36,54.37,54.38,54.39,54.40,54.41,54.42,54.43,54.44,54.45,54.46,54.47,54.48,54.49,54.50,54.51,54.52,54.53,54.54,54.55,54.56,54.57,54.58,54.59,54.60,54.61,54.62,54.63,54.64,54.65,54.66,54.67,54.68,54.69,54.70,54.71,54.72,54.73,54.74,54.75,54.76,54.77,54.78,54.79,54.80,54.81,54.82,54.83,54.84,54.85,54.86,54.87,54.88,54.89,54.90,54.91,54.92,54.93,54.94,54.95,54.96,54.97,54.98,54.99,55.00,55.01,55.02,55.03,55.04,55.05,55.06,55.07,55.08,55.09,55.10,55.11,55.13,55.14,55.15,55.16,55.17,55.18,55.19,55.20,55.21,55.22,55.23,55.24,55.25,55.26,55.27,55.28,55.29,55.30,55.31,55.32,55.33,55.34,55.35,55.36,55.37,55.38,55.39,55.40,55.41,55.42,55.43,55.44,55.45,55.46,55.47,55.48,55.49,55.50,55.51,55.52,55.53,55.54,55.55,55.56,55.57,55.58,55.59,55.60,55.61,55.62,55.63,55.64,55.65,55.66,55.67,55.68,55.69,55.70,55.71,55.72,55.73,55.74,55.75,55.76,55.77,55.78,55.79,55.80,55.81,55.82,55.83,55.84,55.85,55.86,55.87,55.88,55.89,55.90,55.91,55.92,55.93,55.94,55.95,55.96,55.97,55.98,55.99,56.00,56.01,56.02,56.03,56.04,56.05,56.06,56.07,56.08,56.09,56.10,56.11,56.12,56.13,56.14,56.15,56.16,56.17,56.18,56.19,56.20,56.21,56.22,56.23,56.24,56.25,56.26,56.27,56.28,56.29,56.30,56.31,56.32,56.33,56.34,56.35,56.36,56.37,56.38,56.39,56.40,56.41,56.42,56.43,56.44,56.45,56.46,56.47,56.48,56.49,56.50,56.51,56.5,56.53,56.54,56.55,56.56,56.57,56.58,56.59,56.60,56.61,56.62,56.63,56.64,56.65,56.66,56.67,56.68,56.69,56.70,56.71,56.72,56.73,56.74,56.75,56.76,56.77,56.78,56.79,56.80,56.81,56.82,56.83,56.84,56.85,56.86,56.87,56.88,56.89}};
												
		static float hip[2][211]={{2990,2991,2992,2993,2994,2995,2996,2997,2998,2999,3000,3001,3002,3003,3004,3005,3006,3007,3008,3009,3010,3011,3012,3013,3014,3015,3016,3017,3018,3019,3020,3021,3022,3023,3024,3025,3026,3027,3028,3029,3030,3031,3032,3033,3034,3035,3036,3037,3038,3039,3040,3041,3042,3043,3044,3045,3046,3047,3048,3049,3050,3051,3052,3053,3054,3055,3056,3057,3058,3059,3060,3061,3062,3063,3064,3065,3066,3067,3068,3069,3070,3071,3072,3073,3074,3075,3076,3077,3078,3079,3080,3081,3082,3083,3084,3085,3086,3087,3088,3089,3090,3091,3092,3093,3094,3095,3096,3097,3098,3099,3100,3101,3102,3103,3104,3105,3106,3107,3108,3109,3110,3111,3112,3113,3114,3115,3116,3117,3118,3119,3120,3121,3122,3123,3124,3125,3126,3127,3128,3129,3130,3131,3132,3133,3134,3135,3136,3137,3138,3139,3140,3141,3142,3143,3144,3145,3146,3147,3148,3149,3150,3151,3152,3153,3154,3155,3156,3157,3158,3159,3160,3161,3162,3163,3164,3165,3166,3167,3168,3169,3170,3171,3172,3173,3174,3175,3176,3177,3178,3179,3180,3181,3182,3183,3184,3185,3186,3187,3188,3189,3190,3191,3192,3193,3194,3195,3196,3197,3198,3199,3200},
														{92.01,92.02,92.02,92.03,92.04,92.05,92.06,92.07,92.08,92.09,92.10,92.11,92.12,92.13,92.14,92.15,92.16,92.17,92.18,92.19,92.20,92.21,92.22,92.23,92.24,92.25,92.26,92.27,92.28,92.29,92.30,92.31,92.32,92.33,92.34,92.35,92.36,92.37,92.38,92.39,92.40,92.41,92.42,92.43,92.44,92.45,92.46,92.47,92.48,92.49,92.50,92.51,92.52,92.53,92.54,92.55,92.56,92.57,92.58,92.59,92.60,92.61,92.62,92.63,92.64,92.65,92.66,92.67,92.68,92.69,92.70,92.71,92.72,92.73,92.74,92.75,92.76,92.77,92.78,92.79,92.80,92.81,92.82,92.83,92.84,92.85,92.86,92.87,92.88,92.89,92.90,92.91,92.92,92.93,92.94,92.95,92.96,92.97,92.98,92.99,93.00,93.01,93.02,93.03,93.04,93.05,93.06,93.07,93.08,93.09,93.10,93.11,93.12,93.13,93.14,93.15,93.16,93.17,93.18,93.19,93.20,93.21,93.22,93.23,93.24,93.25,93.26,93.27,93.28,93.29,93.30,93.31,93.32,93.33,93.34,93.35,93.36,93.37,93.38,93.39,93.40,93.41,93.42,93.43,93.44,93.45,93.46,93.47,93.48,93.49,93.50,93.51,93.52,93.53,93.54,93.55,93.56,93.57,93.58,93.59,93.60,93.61,93.62,93.63,93.64,93.65,93.66,93.67,93.68,93.69,93.70,93.71,93.72,93.73,93.74,93.75,93.76,93.77,93.78,93.79,93.80,93.81,93.82,93.83,93.84,93.85,93.86,93.87,93.88,93.89,93.90,93.91,93.92,93.93,93.94,93.95,93.96,93.97,93.98,93.99,94.00,94.01,94.02,94.03,94.04,94.05,94.06,94.07,94.08,94.08,94.09}};
															
		static float waist[2][711]={{2190,	2191,	2192,	2193,	2194,	2195,	2196,	2197,	2198,	2199,	2200,	2201,	2202,	2203,	2204,	2205,	2206,	2207,	2208,	2209,	2210,	2211,	2212,	2213,	2214,	2215,	2216,	2217,	2218,	2219,	2220,	2221,	2222,	2223,	2224,	2225,	2226,	2227,	2228,	2229,	2230,	2231,	2232,	2233,	2234,	2235,	2236,	2237,	2238,	2239,	2240,	2241,	2242,	2243,	2244,	2245,	2246,	2247,	2248,	2249,	2250,	2251,	2252,	2253,	2254,	2255,	2256,	2257,	2258,	2259,	2260,	2261,	2262,	2263,	2264,	2265,	2266,	2267,	2268,	2269,	2270,	2271,	2272,	2273,	2274,	2275,	2276,	2277,	2278,	2279,	2280,	2281,	2282,	2283,	2284,	2285,	2286,	2287,	2288,	2289,	2290,	2291,	2292,	2293,	2294,	2295,	2296,	2297,	2298,	2299,	2300,	2301,	2302,	2303,	2304,	2305,	2306,	2307,	2308,	2309,	2310,	2311,	2312,	2313,	2314,	2315,	2316,	2317,	2318,	2319,	2320,	2321,	2322,	2323,	2324,	2325,	2326,	2327,	2328,	2329,	2330,	2331,	2332,	2333,	2334,	2335,	2336,	2337,	2338,	2339,	2340,	2341,	2342,	2343,	2344,	2345,	2346,	2347,	2348,	2349,	2350,	2351,	2352,	2353,	2354,	2355,	2356,	2357,	2358,	2359,	2360,	2361,	2362,	2363,	2364,	2365,	2366,	2367,	2368,	2369,	2370,	2371,	2372,	2373,	2374,	2375,	2376,	2377,	2378,	2379,	2380,	2381,	2382,	2383,	2384,	2385,	2386,	2387,	2388,	2389,	2390,	2391,	2392,	2393,	2394,	2395,	2396,	2397,	2398,	2399,	2400,	2401,	2402,	2403,	2404,	2405,	2406,	2407,	2408,	2409,	2410,	2411,	2412,	2413,	2414,	2415,	2416,	2417,	2418,	2419,	2420,	2421,	2422,	2423,	2424,	2425,	2426,	2427,	2428,	2429,	2430,	2431,	2432,	2433,	2434,	2435,	2436,	2437,	2438,	2439,	2440,	2441,	2442,	2443,	2444,	2445,2446,	2447,	2448,	2449,	2450,	2451,	2452,	2453,	2454,	2455,	2456,	2457,	2458,	2459,	2460,	2461,	2462,	2463,	2464,	2465,	2466,	2467,	2468,	2469,	2470,	2471,	2472,	2473,	2474,	2475,	2476,	2477,	2478,	2479,	2480,	2481,	2482,	2483,	2484,	2485,	2486,	2487,	2488,	2489,	2490,	2491,	2492,	2493,	2494,	2495,	2496,	2497,	2498,	2499,	2500,	2501,	2502,	2503,	2504,	2505,	2506,	2507,	2508,	2509,	2510,	2511,	2512,	2513,	2514,	2515,	2516,	2517,	2518,	2519,	2520,	2521,	2522,	2523,	2524,	2525,	2526,	2527,	2528,	2529,	2530,	2531,	2532,	2533,	2534,	2535,	2536,	2537,	2538,	2539,	2540,	2541,	2542,	2543,	2544,	2545,	2546,	2547,	2548,	2549,	2550,	2551,	2552,	2553,	2554,	2555,	2556,	2557,	2558,	2559,	2560,	2561,	2562,	2563,	2564,	2565,	2566,	2567,	2568,	2569,	2570,	2571,	2572,	2573,	2574,	2575,	2576,	2577,	2578,	2579,	2580,	2581,	2582,	2583,	2584,	2585,	2586,	2587,	2588,	2589,	2590,	2591,	2592,	2593,	2594,	2595,	2596,	2597,	2598,	2599,	2600,	2601,	2602,	2603,	2604,	2605,	2606,	2607,	2608,	2609,	2610,	2611,	2612,	2613,	2614,	2615,	2616,	2617,	2618,	2619,	2620,	2621,	2622,	2623,	2624,	2625,	2626,	2627,	2628,	2629,	2630,	2631,	2632,	2633,	2634,	2635,	2636,	2637,	2638,	2639,	2640,	2641,	2642,	2643,	2644,	2645,	2646,	2647,	2648,	2649,	2650,	2651,	2652,	2653,	2654,	2655,	2656,	2657,	2658,	2659,	2660,	2661,	2662,	2663,	2664,	2665,	2666,	2667,	2668,	2669,	2670,	2671,	2672,	2673,	2674,	2675,	2676,	2677,	2678,	2679,	2680,	2681,	2682,	2683,	2684,	2685,	2686,	2687,	2688,	2689,	2690,	2691,	2692,	2693,	2694,	2695,	2696,	2697,	2698,	2699,	2700,	2701,2702,	2703,	2704,	2705,	2706,	2707,	2708,	2709,	2710,	2711,	2712,	2713,	2714,	2715,	2716,	2717,	2718,	2719,	2720,	2721,	2722,	2723,	2724,	2725,	2726,	2727,	2728,	2729,	2730,	2731,	2732,	2733,	2734,	2735,	2736,	2737,	2738,	2739,	2740,	2741,	2742,	2743,	2744,	2745,	2746,	2747,	2748,	2749,	2750,	2751,	2752,	2753,	2754,	2755,	2756,	2757,	2758,	2759,	2760,	2761,	2762,	2763,	2764,	2765,	2766,	2767,	2768,	2769,	2770,	2771,	2772,	2773,	2774,	2775,	2776,	2777,	2778,	2779,	2780,	2781,	2782,	2783,	2784,	2785,	2786,	2787,	2788,	2789,	2790,	2791,	2792,	2793,	2794,	2795,	2796,	2797,	2798,	2799,	2800,	2801,	2802,	2803,	2804,	2805,	2806,	2807,	2808,	2809,	2810,	2811,	2812,	2813,	2814,	2815,	2816,	2817,	2818,	2819,	2820,	2821,	2822,	2823,	2824,	2825,	2826,	2827,	2828,	2829,	2830,	2831,	2832,	2833,	2834,	2835,	2836,	2837,	2838,	2839,	2840,	2841,	2842,	2843,	2844,	2845,	2846,	2847,	2848,	2849,	2850,	2851,	2852,	2853,	2854,	2855,	2856,	2857,	2858,	2859,	2860,	2861,	2862,	2863,	2864,	2865,	2866,	2867,	2868,	2869,	2870,	2871,	2872,	2873,	2874,	2875,	2876,	2877,	2878,	2879,	2880,	2881,	2882,	2883,	2884,	2885,	2886,	2887,	2888,	2889,	2890,	2891,	2892,	2893,	2894,	2895,	2896,	2897,	2898,	2899,	2900},
																{81.01,	81.02,	81.02,	81.03,	81.04,	81.05,	81.06,	81.07,	81.08,	81.09,	81.1,	81.11,	81.12,	81.13,	81.14,	81.15,	81.16,	81.17,	81.18,	81.19,	81.2,	81.21,	81.22,	81.23,	81.24,	81.25,	81.26,	81.27,	81.28,	81.29,	81.3,	81.31,	81.32,	81.33,	81.34,	81.35,	81.36,	81.37,	81.38,	81.39,	81.4,	81.41,	81.42,	81.43,	81.44,	81.45,	81.46,	81.47,	81.48,	81.49,	81.5,	81.51,	81.52,	81.53,	81.54,	81.55,	81.56,	81.57,	81.58,	81.59,	81.6,	81.61,	81.62,	81.63,	81.64,	81.65,	81.66,	81.67,	81.68,	81.69,	81.7,	81.71,	81.72,	81.73,	81.74,	81.75,	81.76,	81.77,	81.78,	81.79,	81.8,	81.81,	81.82,	81.83,	81.84,	81.85,	81.86,	81.87,	81.88,	81.89,	81.9,	81.91,	81.92,	81.93,	81.94,	81.95,	81.96,	81.97,	81.98,	81.99,	82,	82.01,	82.02,	82.03,	82.04,	82.05,	82.06,	82.07,	82.08,	82.09,	82.1,	82.11,	82.12,	82.13,	82.14,	82.15,	82.16,	82.17,	82.18,	82.19,	82.2,	82.21,	82.22,	82.23,	82.24,	82.25,	82.26,	82.27,	82.28,	82.29,	82.3,	82.31,	82.32,	82.33,	82.34,	82.35,	82.36,	82.37,	82.38,	82.39,	82.4,	82.41,	82.42,	82.43,	82.44,	82.45,	82.46,	82.47,	82.48,	82.49,	82.5,	82.51,	82.52,	82.53,	82.54,	82.55,	82.56,	82.57,	82.58,	82.59,	82.6,	82.61,	82.62,	82.63,	82.64,	82.65,	82.66,	82.67,	82.68,	82.69,	82.7,	82.71,	82.72,	82.73,	82.74,	82.75,	82.76,	82.77,	82.78,	82.79,	82.8,	82.81,	82.82,	82.83,	82.84,	82.85,	82.86,	82.87,	82.88,	82.89,	82.9,	82.91,	82.92,	82.93,	82.94,	82.95,	82.96,	82.97,	82.98,	82.99,	83,	83.01,	83.02,	83.03,	83.04,	83.05,	83.06,	83.07,	83.08,	83.09,	83.1,	83.11,	83.12,	83.13,	83.14,	83.15,	83.16,	83.17,	83.18,	83.19,	83.2,	83.21,	83.22,	83.23,	83.24,	83.25,	83.26,	83.27,	83.28,	83.29,	83.3,	83.31,	83.32,	83.33,	83.34,	83.35,	83.36,	83.37,	83.38,	83.39,	83.4,	83.41,	83.42,	83.43,	83.44,	83.45,	83.46,	83.47,	83.48,	83.49,	83.5,	83.51,	83.52,	83.53,	83.54,	83.55,83.56,	83.57,	83.58,	83.59,	83.6,	83.61,	83.62,	83.63,	83.64,	83.65,	83.66,	83.67,	83.68,	83.69,	83.7,	83.71,	83.72,	83.73,	83.74,	83.75,	83.76,	83.77,	83.78,	83.79,	83.8,	83.81,	83.82,	83.83,	83.84,	83.85,	83.86,	83.87,	83.88,	83.89,	83.9,	83.91,	83.92,	83.93,	83.94,	83.95,	83.96,	83.97,	83.98,	83.99,	84,	84.01,	84.02,	84.03,	84.04,	84.05,	84.06,	84.07,	84.08,	84.09,	84.1,	84.11,	84.12,	84.13,	84.14,	84.15,	84.16,	84.17,	84.18,	84.19,	84.2,	84.21,	84.22,	84.23,	84.24,	84.25,	84.26,	84.27,	84.28,	84.29,	84.3,	84.31,	84.32,	84.33,	84.34,	84.35,	84.36,	84.37,	84.38,	84.39,	84.4,	84.41,	84.42,	84.43,	84.44,	84.45,	84.46,	84.47,	84.48,	84.49,	84.5,	84.51,	84.52,	84.53,	84.54,	84.55,	84.56,	84.57,	84.58,	84.59,	84.6,	84.61,	84.62,	84.63,	84.64,	84.65,	84.66,	84.67,	84.68,	84.69,	84.7,	84.71,	84.72,	84.73,	84.74,	84.75,	84.76,	84.77,	84.78,	84.79,	84.8,	84.81,	84.82,	84.83,	84.84,	84.85,	84.86,	84.87,	84.88,	84.89,	84.9,	84.91,	84.92,	84.93,	84.94,	84.95,	84.96,	84.97,	84.98,	84.99,	85,	85.01,	85.02,	85.03,	85.04,	85.05,	85.06,	85.07,	85.08,	85.09,	85.1,	85.11,	85.12,	85.13,	85.14,	85.15,	85.16,	85.17,	85.18,	85.19,	85.2,	85.21,	85.22,	85.23,	85.24,	85.25,	85.26,	85.27,	85.28,	85.29,	85.3,	85.31,	85.32,	85.33,	85.34,	85.35,	85.36,	85.37,	85.38,	85.39,	85.4,	85.41,	85.42,	85.43,	85.44,	85.45,	85.46,	85.47,	85.48,	85.49,	85.5,	85.51,	85.52,	85.53,	85.54,	85.55,	85.56,	85.57,	85.58,	85.59,	85.6,	85.61,	85.62,	85.63,	85.64,	85.65,	85.66,	85.67,	85.68,	85.69,	85.7,	85.71,	85.72,	85.73,	85.74,	85.75,	85.76,	85.77,	85.78,	85.79,	85.8,	85.81,	85.82,	85.83,	85.84,	85.85,	85.86,	85.87,	85.88,	85.89,	85.9,	85.91,	85.92,	85.93,	85.94,	85.95,	85.96,	85.97,	85.98,	85.99,	86,	86.01,	86.02,	86.03,	86.04,	86.05,	86.06,	86.07,	86.08,	86.09,	86.1,	86.11,86.13,	86.14,	86.15,	86.16,	86.17,	86.18,	86.19,	86.2,	86.21,	86.22,	86.23,	86.24,	86.25,	86.26,	86.27,	86.28,	86.29,	86.3,	86.31,	86.32,	86.33,	86.34,	86.35,	86.36,	86.37,	86.38,	86.39,	86.4,	86.41,	86.42,	86.43,	86.44,	86.45,	86.46,	86.47,	86.48,	86.49,	86.5,	86.51,	86.52,	86.53,	86.54,	86.55,	86.56,	86.57,	86.58,	86.59,	86.6,	86.61,	86.62,	86.63,	86.64,	86.65,	86.66,	86.67,	86.68,	86.69,	86.7,	86.71,	86.72,	86.73,	86.74,	86.75,	86.76,	86.77,	86.78,	86.79,	86.8,	86.81,	86.82,	86.83,	86.84,	86.85,	86.86,	86.87,	86.88,	86.89,	86.9,	86.91,	86.92,	86.93,	86.94,	86.95,	86.96,	86.97,	86.98,	86.99,	87,	87.01,	87.02,	87.03,	87.04,	87.05,	87.06,	87.07,	87.08,	87.09,	87.1,	87.11,	87.12,	87.13,	87.14,	87.15,	87.16,	87.17,	87.18,	87.19,	87.2,	87.21,	87.22,	87.23,	87.24,	87.25,	87.26,	87.27,	87.28,	87.29,	87.3,	87.31,	87.32,	87.33,	87.34,	87.35,	87.36,	87.37,	87.38,	87.39,	87.4,	87.41,	87.42,	87.43,	87.44,	87.45,	87.46,	87.47,	87.48,	87.49,	87.5,	87.51,	87.52,	87.53,	87.54,	87.55,	87.56,	87.57,	87.58,	87.59,	87.6,	87.61,	87.62,	87.63,	87.64,	87.65,	87.66,	87.67,	87.68,	87.69,	87.7,	87.71,	87.72,	87.73,	87.74,	87.75,	87.76,	87.77,	87.78,	87.79,	87.8,	87.81,	87.82,	87.83,	87.84,	87.85,	87.86,	87.87,	87.88,	87.89,	87.9,	87.91,	87.92,	87.93,	87.94,	87.95,	87.96,	87.97,	87.98,	87.99,	88,	88.01,	88.02,	88.03,	88.04,	88.05,	88.06,	88.07,	88.08,	88.08,	88.09}};													
	
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)

    {
				if(m_adc_channel_enabled == 6)
				{
						err_code = nrf_drv_saadc_channel_uninit(0);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
						APP_ERROR_CHECK(err_code);
						m_adc_channel_enabled = 1;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);	
						for(solandoc = 0; solandoc < 105; solandoc++)//20//12
						{
							value_adc_before += p_event->data.done.p_buffer[solandoc];
						}
										
						value_adc_before /= 105;
						value_adc_before = fabs(value_adc_before);
						flexV = ( 4095 / value_adc_before) - 1;
						flexV = ((R_DIV  / flexV));
						flexV = flexV * 10;
						value_th = (unsigned int)flexV;
//						printf("Channel [%d] value_f1: %d\r\n", m_adc_channel_enabled, value_th);
						for(int n = 0; n<1; n++)
						{
							for(int m = 0; m<691; m++)
							{
								if(value_th == th[n][m])
								{
									n=n+1;
									float th1 = th[n][m];
									printf("Channel [%d] value_thg: %0.2f\r\n", m_adc_channel_enabled, th1);
									message_length = sprintf(buffer,"Thigh[%d]=%0.2fcm\n ", m_adc_channel_enabled, th1);
									ble_nus_string_send(&m_nus,(uint8_t *)&buffer, message_length);
									nrf_delay_ms(650);
								}
							}
						}
						printf("\r\n");
		}				
				else if(m_adc_channel_enabled == 1)
				{
						err_code = nrf_drv_saadc_channel_uninit(1);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
						APP_ERROR_CHECK(err_code); 
						m_adc_channel_enabled = 2;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);
						for(int solan = 0; solan < 100; solan++)//20
						{
							value_w += p_event->data.done.p_buffer[solan];
						}
										
						value_w /= 100;
						value_w = fabs(value_w);
						flexV1 = ( 4095 / value_w) - 1;
						flexV1 = (R_DIV  /  flexV1);
						flexV1 = flexV1 * 10;
						value_waist_s =(unsigned int) flexV1;
//						printf("Channel [%d] value_waist: %d\r\n", m_adc_channel_enabled, value_waist_s);
						for(int n0 = 0; n0<1; n0++)
						{
							for(int m0 = 0; m0<711; m0++)
							{
								if(value_waist_s  == waist[n0][m0])
								{
									n0 = n0+1;
									float w = waist[n0][m0];
									printf("Channel [%d] value_waist: %0.2f\r\n", m_adc_channel_enabled, w);
									message_length = sprintf(buffer,"waist[%d]=%0.2fcm\n ", m_adc_channel_enabled, w);
									ble_nus_string_send(&m_nus,(uint8_t *)&buffer, message_length);
									nrf_delay_ms(650);
								}
							}
						}
						printf("\r\n");
				}
				else if(m_adc_channel_enabled == 2)
				{
						err_code = nrf_drv_saadc_channel_uninit(2);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
						APP_ERROR_CHECK(err_code); 
						m_adc_channel_enabled = 3;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);
						for(int s = 0; s < 100; s++)//20//12
						{
							value_h += p_event->data.done.p_buffer[s];
						}
										
						value_h /= 100;
						value_h = fabs(value_h );
						flexV2 = ( 4095 / value_h) - 1;
						flexV2 = (R_DIV  / flexV2);
						flexV2 = flexV2 * 10;
						value_hip_s =(unsigned int) flexV2;
//						printf("Channel [%d] value_h: %d\r\n", m_adc_channel_enabled, value_hip_s);
						for(int n1 = 0; n1 < 1; n1++)
						{
							for(int m1 = 0; m1 < 211; m1++)
							{
								if(value_hip_s == hip[n1][m1])
								{
									n1 = n1 + 1;
									float h = hip[n1][m1];
									printf("Channel [%d] value_h: %0.2f\r\n", m_adc_channel_enabled, h);
									message_length = sprintf(buffer,"Hip[%d]=%0.2fcm\n ", m_adc_channel_enabled, h);
									ble_nus_string_send(&m_nus,(uint8_t *)&buffer, message_length); 
									nrf_delay_ms(650);
								}
							}
						}
				}
				else if(m_adc_channel_enabled == 3)
				{
						err_code = nrf_drv_saadc_channel_uninit(3);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(4, &channel_4_config);
						APP_ERROR_CHECK(err_code);
						m_adc_channel_enabled = 4;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);	
					for(solandoc = 0; solandoc < 5; solandoc++)
						{
							value_adc_before += p_event->data.done.p_buffer[solandoc];
						}
						value_adc_before /= 5;
						flexV = ( 4095 / value_adc_before) - 1;
						flexV = (R_DIV  / flexV);					
				}
				else if(m_adc_channel_enabled == 4)
				{
						err_code = nrf_drv_saadc_channel_uninit(4);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(5, &channel_5_config);
						APP_ERROR_CHECK(err_code);
						m_adc_channel_enabled = 5;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);	
					for(solandoc = 0; solandoc < 5; solandoc++)
						{
							value_adc_before += p_event->data.done.p_buffer[solandoc];
						}
						value_adc_before /= 5;
						flexV = ( 4095 / value_adc_before) - 1;
						flexV = (R_DIV  / flexV);
//						value_size = (flexV / 92); //110						

				}
				else if(m_adc_channel_enabled == 5)
				{
						err_code = nrf_drv_saadc_channel_uninit(5);
						APP_ERROR_CHECK(err_code);
						err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
						APP_ERROR_CHECK(err_code);
						m_adc_channel_enabled = 6;
						err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
						APP_ERROR_CHECK(err_code);	
					for(solandoc = 0; solandoc < 5; solandoc++)
						{
							value_adc_before += p_event->data.done.p_buffer[solandoc];
						}
						value_adc_before /= 5;
						flexV = ( 4095 / value_adc_before) - 1;
						flexV = (R_DIV  / flexV);

    }
}
}

void saadc_init(void)
{
    static ret_code_t err_code;
//		nrf_drv_saadc_config_t saadc_config;
////    //Configure SAADC
//    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;       //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
//    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;//NRF_SAADC_OVERSAMPLE_2X,4,8,16,32,64,128,256// NRF_SAADC_OVERSAMPLE_DISABLED//Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
//    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;   
//		nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
//		saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
	
	  nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
		//set configuration for saadc channel 0
		channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;// NRF_SAADC_RESISTOR_PULLUP//NRF_SAADC_RESISTOR_DISABLED
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;//NRF_SAADC_RESISTOR_DISABLED //NRF_SAADC_RESISTOR_PULLDOWN
    channel_0_config.gain       = NRF_SAADC_GAIN1_4;
    channel_0_config.reference  = NRF_SAADC_REFERENCE_VDD4;// NRF_SAADC_REFERENCE_VDD4 //NRF_SAADC_REFERENCE_INTERNAL
    channel_0_config.acq_time   = NRF_SAADC_ACQTIME_10US;
    channel_0_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;// NRF_SAADC_MODE_DIFFERENTIAL// NRF_SAADC_MODE_SINGLE_ENDED
    channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN1);
    channel_0_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
		//set configuration for saadc channel 1
		channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.gain       = NRF_SAADC_GAIN1_4;
    channel_1_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_1_config.acq_time   = NRF_SAADC_ACQTIME_10US ;
    channel_1_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_1_config.pin_p      = (nrf_saadc_input_t) NRF_SAADC_INPUT_AIN2;////(nrf_saadc_input_t)
    channel_1_config.pin_n      = NRF_SAADC_INPUT_DISABLED;

		//set configuration for saadc channel 2
		channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.gain       = NRF_SAADC_GAIN1_4 ;
    channel_2_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_2_config.acq_time   = NRF_SAADC_ACQTIME_10US ;
    channel_2_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_2_config.pin_p      = (nrf_saadc_input_t) NRF_SAADC_INPUT_AIN4;
    channel_2_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
			//set configuration for saadc channel 3
		channel_3_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.gain       = NRF_SAADC_GAIN1_4 ;
    channel_3_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_3_config.acq_time   = NRF_SAADC_ACQTIME_10US ;
    channel_3_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_3_config.pin_p      = (nrf_saadc_input_t) NRF_SAADC_INPUT_AIN5;
    channel_3_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
		//set configuration for saadc channel 4
		channel_4_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_4_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_4_config.gain       = NRF_SAADC_GAIN1_4 ;
    channel_4_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_4_config.acq_time   = NRF_SAADC_ACQTIME_10US ;
    channel_4_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_4_config.pin_p      = (nrf_saadc_input_t) NRF_SAADC_INPUT_AIN6;//(nrf_saadc_input_t)
    channel_4_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
		//set configuration for saadc channel 5
		channel_5_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_5_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_5_config.gain       = NRF_SAADC_GAIN1_4 ;//NRF_SAADC_GAIN1_2 
    channel_5_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_5_config.acq_time   = NRF_SAADC_ACQTIME_10US ;
    channel_5_config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
    channel_5_config.pin_p      = (nrf_saadc_input_t) NRF_SAADC_INPUT_AIN7;
    channel_5_config.pin_n      = NRF_SAADC_INPUT_DISABLED;
		
	//	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
		err_code = nrf_drv_saadc_init(&saadc_config , saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
		
//		err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
//    APP_ERROR_CHECK(err_code);
		
//		err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
//    APP_ERROR_CHECK(err_code);
//		
//		err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
//    APP_ERROR_CHECK(err_code);
//		
//		err_code = nrf_drv_saadc_channel_init(4, &channel_4_config);
//    APP_ERROR_CHECK(err_code);
//		
//		err_code = nrf_drv_saadc_channel_init(5, &channel_5_config);
//    APP_ERROR_CHECK(err_code);
		m_adc_channel_enabled = 6;

		NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos);//200kHz
		NRF_SAADC->SAMPLERATE |= (0x50 << SAADC_SAMPLERATE_MODE_Pos);//SAADC_CH_CONFIG_REFSEL_Pos/SAADC_SAMPLERATE_CC_Pos/SAADC_SAMPLERATE_MODE_Msk
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	//	NRF_POWER->DCDCEN = 1;  
    uart_init();
    
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();
    printf("\r\nUART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    // Enter main loop.
    for (;;)
    {
				nrf_delay_ms(1);
				nrf_drv_saadc_sample();
			__WFE();
        power_manage();
//				nrf_delay_ms(1);
    }
}
