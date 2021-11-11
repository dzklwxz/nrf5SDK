/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "app_uart.h"
#include "ble_nus_c.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_queue.h"

#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */

//BLE_LBS_C_ARRAY_DEF(m_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED button client instances. */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_c,NRF_SDH_BLE_CENTRAL_LINK_COUNT);       /**<For NUS multi link>**/
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static char const m_target_periph_name[] = "UART_TEST";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */

//david add start 
typedef struct {
    uint8_t * p_data;
    uint16_t length;
} buffer_t;

NRF_BALLOC_DEF(m_balloc_AT_pool,256,20);

NRF_QUEUE_DEF( uint8_t *, m_buf_queue, 20, NRF_QUEUE_MODE_NO_OVERFLOW );  //定义队列

uint16_t test_data_len=0;
//david add end 


//Peter add start
uint8_t id = 0;  //device ID
uint8_t user_conn_state[8];  //连接上从机标志
uint16_t user_conn_handle[8];  //连接上从机句柄
uint8_t user_req_flag = 0;  //需要发送数据给从机标志
uint8_t phy_update_state = 0;  //2M PHY更新标志
//Peter add end


//Peter add start
//1ms定时器，最长定时时间0xFFFF ms
APP_TIMER_DEF( app_timer_rtc_1ms );
uint32_t rtc_1ms = 0;
void app_timer_rtc_1ms_handler( void * p_context )
{
	rtc_1ms++;
}

//启动定时器
void app_timer_rtc_1ms_start( void )
{
	uint32_t err_code = 0;
	static uint8_t state = 1;
	
	if( state )
	{
		state = 0;
		
		err_code = app_timer_create( &app_timer_rtc_1ms, APP_TIMER_MODE_REPEATED, app_timer_rtc_1ms_handler);
		APP_ERROR_CHECK( err_code );
	}
	
	err_code = app_timer_start( app_timer_rtc_1ms, APP_TIMER_TICKS( 1 ), NULL );
	APP_ERROR_CHECK( err_code );
}

//停止定时器
void app_timer_rtc_1ms_stop( void )
{
	uint32_t err_code = 0;
	err_code = app_timer_stop( app_timer_rtc_1ms );
	APP_ERROR_CHECK( err_code );
	rtc_1ms = 0;
}

//Peter add end

//Peter add start
//打印log到缓存，再打印缓存数据到串口，解决自带log打印函数打印不完整问题
#include "app_fifo.h"
app_fifo_t user_fifo;
//uint8_t fifo_buff[4096] = { 0 };
/*
void user_fifo_init(void)
{
	uint32_t err_code;
	err_code =  app_fifo_init( &user_fifo, fifo_buff, sizeof(fifo_buff) );
	APP_ERROR_CHECK(err_code);
}

void user_fifo_task( void )
{
	uint32_t len = 1;
	static uint8_t data[2] = {0};
	uint32_t err_code = app_fifo_read( &user_fifo, data, &len );
	if( err_code == NRF_SUCCESS )
	{
		app_uart_put( data[0] );
	}
}

void log_printf( uint8_t *data, uint32_t len )
{
	app_fifo_write( &user_fifo, data, &len );
}
*/
//Peter add end

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the LED Button Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
//static void lbs_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}

/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    //APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
//    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            if((data_array[index - 1] == '\n') )
            {
            	NRF_LOG_HEXDUMP_INFO( data_array, index );
            	index = 0;
            }
            break;
        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
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
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        //.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
			  .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_1000000,  //NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("Start scanning for device name %s.", (uint32_t)m_target_periph_name);
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}
static void put_data_into_queue(uint8_t * p_data, uint16_t p_data_len)
{
		ret_code_t ret;
	
		uint8_t *buf = nrf_balloc_alloc(&m_balloc_AT_pool);
		if(buf)
		{
			if(p_data_len<250)
			{
				memcpy(buf,p_data,p_data_len);	
				//memcpy(&buf[252],&p_data_len,2);	
				test_data_len=p_data_len;
				buf[252]=p_data_len;
				buf[251]=p_data_len>>8;
				ret = nrf_queue_push(&m_buf_queue, &buf);
				APP_ERROR_CHECK(ret);
				if(p_data_len==0x00)
				{
					NRF_LOG_ERROR("wrong data");
				}
			}
			else
			{
				NRF_LOG_ERROR("Receive length too long");
			}
		}    
		
}

static void data_print_out()
{
	
	 if(!nrf_queue_is_empty(&m_buf_queue))
	 {
			ret_code_t ret;
		  uint16_t data_len = 0;
			uint8_t data[1024] = {0};
			uint32_t rtc_dev[4] = {0};
			uint8_t pos_x = 0;	
			uint8_t *pbuf =NULL;	
		
			ret = nrf_queue_read(&m_buf_queue,&pbuf,1);
			APP_ERROR_CHECK(ret);						
			uint16_t length=pbuf[252];
			
			uint8_t dev_id = pbuf[9];
			
			if(length==0x1F) 			//length 31  one package 
			{
				pos_x=0;
			}
			else if (length==0x3E)//length 62  two package
			{
				pos_x=1;
			}
			else if (length==0x5D)//length 93  three package
			{
				pos_x=2;
			}
			else if (length==0x7C)//length 124 four package
			{
				pos_x=3;
			}
			else if (length==0x9B)//length 155 five package
			{
				pos_x=4;
			}
			else if (length==0xBA)//length 186 SIX package
			{
				pos_x=5;
			}
			else
			{
				NRF_LOG_ERROR("BLE Receive incorrect ");
				ret=NRF_ERROR_SDK_COMMON_ERROR_BASE;
				APP_ERROR_CHECK(ret);
			}
		
	    //打印设备编号
			sprintf( data, "%d ", pbuf[9] );
			data_len += 2;		
			
			//打印主机mtu本地时间
			sprintf( data + data_len, "%010d ", rtc_1ms );
			data_len += 11;
			
			//取数据包1-4 rtc时间
			for( uint8_t i = 0; i < pos_x+1; i++ )
			{
				pos_x = i*31;
				rtc_dev[i] |= pbuf[5 +pos_x+ 0];
				rtc_dev[i] <<= 8;
				rtc_dev[i] |= pbuf[5 +pos_x+ 1];
				rtc_dev[i] <<= 8;
				rtc_dev[i] |= pbuf[5 +pos_x+ 2];
				rtc_dev[i] <<= 8;
				rtc_dev[i] |= pbuf[5 +pos_x+ 3];
				
				sprintf( data + data_len, "%010d ", rtc_dev[i] );
				data_len += 11;
			}
			
			
			/*
			//打印从机mtu本地时间
			for( uint8_t i = 0; i < 1; i ++ )
			{
				//打印1 rtc时间
				sprintf( data + data_len, "%010d ", rtc_dev[i] );
				data_len += 11;
			}
			*/
				//打印结束符
			sprintf( data + data_len, "\r\n" );
			data_len += 2;
			
			NRF_LOG_INFO("%s",data);
			NRF_LOG_FLUSH();
			
			nrf_balloc_free(&m_balloc_AT_pool, pbuf);
			
	 }

}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */

//static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t p_data_len)
static void ble_nus_chars_received_uart_print()
{

		 if (!nrf_queue_is_empty(&m_buf_queue))
		 {			
					//格式化数据，打印数据
				 uint16_t data_len = 0;
				 uint8_t data[1024] = {0};
				 uint32_t rtc_dev[4] = {0};
				 uint8_t pos_x = 0;				
				 ret_code_t ret;
				 uint8_t *pbuf =NULL;
         ret = nrf_queue_read(&m_buf_queue,&pbuf,1);
			   APP_ERROR_CHECK(ret);
					//	//设置接收到从机1-8数据标志
					uint8_t dev_id = pbuf[9];
					//NRF_LOG_RAW_HEXDUMP_INFO(p_data,69);
					//NRF_LOG_RAW_HEXDUMP_INFO(p_data+69,68);
							
							//取数据包1-4 rtc时间
							for( uint8_t i = 0; i < 1; i++ )
							{
								pos_x = i*38;
								rtc_dev[i] |= pbuf[5 + pos_x + 0];
								rtc_dev[i] <<= 8;
								rtc_dev[i] |= pbuf[5 + pos_x + 1];
								rtc_dev[i] <<= 8;
								rtc_dev[i] |= pbuf[5 + pos_x + 2];
								rtc_dev[i] <<= 8;
								rtc_dev[i] |= pbuf[5 + pos_x + 3];
							}

							//打印设备编号
							sprintf( data, "%d ", pbuf[9] );
							data_len += 2;
							
								/*
							//打印主机mtu本地时间
							sprintf( data + data_len, "%010d ", rtc_1ms );
							data_len += 11;
							
							//打印从机mtu本地时间
							for( uint8_t i = 0; i < 1; i ++ )
							{
								//打印1 rtc时间
								sprintf( data + data_len, "%010d ", rtc_dev[i] );
								data_len += 11;
							}
						*/


							// print metadata grounp1 	
							for(uint8_t i=0;i<25;i++)
							{		
								
								sprintf(data + data_len,"%02X",pbuf[5+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
								
							// print metadata grounp2 
							for(uint8_t i=0;i<21;i++)
							{	
								sprintf(data+data_len,"%02X",pbuf[30+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
							
							
							// print metadata grounp3 
							for(uint8_t i=0;i<21;i++)
							{
								sprintf(data+data_len,"%02X",pbuf[51+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
							
							
								// print metadata grounp4 
							for(uint8_t i=0;i<21;i++)
							{
								sprintf(data+data_len,"%02X",pbuf[72+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
							
								// print metadata grounp5 
							for(uint8_t i=0;i<21;i++)
							{
								sprintf(data+data_len,"%02X",pbuf[93+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
							
								// print metadata grounp6 
							for(uint8_t i=0;i<21;i++)
							{
								sprintf(data+data_len,"%02X",pbuf[114+i]);
								data_len += 2;
							}
							sprintf( data + data_len, " " );
							data_len += 1;
							

							
							//打印结束符
							sprintf( data + data_len, "\r\n" );
							data_len += 2;
						 
								//打印数据到缓存
							//log_printf( data, data_len );
							NRF_LOG_INFO("%s",data);
							NRF_LOG_FLUSH();
							nrf_balloc_free(&m_balloc_AT_pool, pbuf);
		 }

	

}

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            NRF_LOG_INFO("p_ble_nus_evt->conn_handle = %d\r\n", p_ble_nus_evt->conn_handle );
            
            //设置对应从机连接标志
            user_conn_state[p_ble_nus_evt->conn_handle] = 1;
            //设置对应从机连接句柄
            user_conn_handle[p_ble_nus_evt->conn_handle] = p_ble_nus_evt->conn_handle;
            break;
        case BLE_NUS_C_EVT_NUS_TX_EVT:
        //    ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
						if(p_ble_nus_evt->data_len==0x00)
						{
							NRF_LOG_ERROR("wrong data");
						}
						put_data_into_queue(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
						
            break;
        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            //Peter add start
            NRF_LOG_INFO("PHY update request.");
					/*
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,  //BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_2MBPS,  //BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
					*/
            //Peter add end
            
            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",p_gap_evt->conn_handle );
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status and check whether it is needed to look for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("LBS central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            if (ble_conn_state_central_conn_count() == 0)
            {
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);

                // Turn off the LED that indicates the connection.
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }

            // Start scanning.
            scan_start();

            // Turn on the LED for indicating scanning.
            bsp_board_led_on(CENTRAL_SCANNING_LED);
            
            //清空对应从机连接标志，清空对应从机连接句柄
			user_conn_state[p_gap_evt->conn_handle] = 0;
			user_conn_handle[p_gap_evt->conn_handle] = 0;
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only the connection requests can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_2MBPS,  //BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_2MBPS,  //BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
		case BLE_GAP_EVT_PHY_UPDATE:
		{
			phy_update_state = 1;  //更新2M PHY OK
		} break;
        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT client timeout event.
            NRF_LOG_DEBUG("GATT client timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT server timeout event.
            NRF_LOG_DEBUG("GATT server timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            //err_code = led_status_send_to_all(button_action);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Nothing happen here", button_action);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it is saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

	ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details This function handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


void req_data_task(void)
{
	//定时2ms通知从机上报数据
	if( rtc_1ms >= 3 )  //30ms内1个主机处理8个从机数据，主机每3.75ms内发送一个ack给一个从机，并从此从机接收235字节数据，下一个3.75ms处理下一个从机
	{
		rtc_1ms = 0;
		user_req_flag = 1;
	}
	
	if( user_req_flag )
	{
		user_req_flag = 0;
		
		//切换下一个有效从机开始
		//3.75ms时间到，通知下一个设备发数据上来，如果下一个设备为空，则切换下一个设备
		id++;
		if( id >= 8 )
		{
			id = 0;
		}
		
		//判断此设备是否在连接状态，不在连接状态，则切换下一个设备
		for( uint8_t i = 0; i < 8; i++ )
		{
			if( user_conn_state[id] )
			{
				break;
			}
			else
			{
				id++;
				if( id >= 8 )
				{
					id = 0;
				}
			}
		}
		//切换下一个有效从机结束
		
		//通知从机上报数据
		uint32_t ret_val;
		uint8_t p_data[5] = {"ACK\r\n"};
		uint8_t data_len = 5;
		//已经连接上从设备，可以发数据给从设备
		
		//设备update ok才可以发送数据
		if( !phy_update_state )
		{
			//phy_update_state = 0;
			return;
		}
		
		if( user_conn_state[id] )
		{
	        //do
	        {
	            ret_val = ble_nus_c_string_send( &m_ble_nus_c[ user_conn_handle[id] ], p_data, data_len );
	            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
	            {
	                NRF_LOG_INFO("Failed sending NUS message. Error 0x%x. ", ret_val);
	                //APP_ERROR_CHECK(ret_val);
	            }
	            //NRF_LOG_INFO( "ret_val = %d\r\n", ret_val );
	        } 
	        //while(ret_val != NRF_SUCCESS);
		}
	}
}


int main(void)
{
    // Initialize.
    uart_init();
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    nus_c_init();
    ble_conn_state_init();
    scan_init();
	 //init memeory block & queue
    nrf_balloc_init(&m_balloc_AT_pool);
	  nrf_queue_reset(&m_buf_queue);
    // Start execution.
    NRF_LOG_INFO("Multilink example started.");
    uint32_t err_code =  sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT,0,RADIO_TXPOWER_TXPOWER_Pos4dBm);
    APP_ERROR_CHECK(err_code);
    scan_start();  //开始扫描
	app_timer_rtc_1ms_start();  //Peter add
	//user_fifo_init();  //Peter add
	
    for (;;)
    {
        idle_state_handle();
	
			data_print_out();
    }
}
