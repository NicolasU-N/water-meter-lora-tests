#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "sys/time.h"

#include "time.h"
#include "sys/time.h"

#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "radio_lora_e5.h"

// UART
#define TX_PIN (GPIO_NUM_17)
#define RX_PIN (GPIO_NUM_16)
#define BUF_SIZE (1024)

// LORA CONSTANTS
#define APP_KEY "99887766554433221199887766554433"
#define CHANNELS "0-15"
#define BAND "AU915"
// DR 1 SF9BW125
#define DR "0"

// ADC
#define V_BAT_ADC1_CHAN6 ADC_CHANNEL_6 // V_BAT_PIN 34

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP32_BT"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA /*Choose show mode: show data or speed*/

const char *TAG_UART = "UART";
const char *TAG_ADC = "ADC";
const char *TAG_BATTERY = "BAT";
const char *TAG_BT = "BT";
// const char *SPP_TAG = "SPP_ACCEPTOR_DEMO";

TaskHandle_t send_data_to_radio_handle;

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
bool do_calibration1;

static char point;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t writeHandle;

esp_err_t init_pins(void);

float read_voltage(void);
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

// char *data_uart;

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
	if (bda == NULL || str == NULL || size < 18)
	{
		return NULL;
	}

	uint8_t *p = bda;
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
			p[0], p[1], p[2], p[3], p[4], p[5]);
	return str;
}

static void print_speed(void)
{
	float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
	float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
	float time_interval = time_new_s - time_old_s;
	float speed = data_num * 8 / time_interval / 1000.0;
	ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s", time_old_s, time_new_s, speed);
	data_num = 0;
	time_old.tv_sec = time_new.tv_sec;
	time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	char bda_str[18] = {0};

	switch (event)
	{
	case ESP_SPP_INIT_EVT:
		if (param->init.status == ESP_SPP_SUCCESS)
		{
			ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
			esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		}
		else
		{
			ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
		}
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status,
				 param->close.handle, param->close.async);
		break;
	case ESP_SPP_START_EVT:
		if (param->start.status == ESP_SPP_SUCCESS)
		{
			ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
					 param->start.scn);
			esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
			esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
		}
		else
		{
			ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
		}
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
		/*
		 * We only show the data in which the data length is less than 128 here. If you want to print the data and
		 * the data rate is high, it is strongly recommended to process them in other lower priority application task
		 * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
		 * stack and also have a effect on the throughput!
		 */
		ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d",
				 param->data_ind.len, param->data_ind.handle);
		if (param->data_ind.len < 128)
		{
			esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
		}

		printf("Value received: ");
		for (size_t i = 0; i < (param->data_ind.len) - 2; i++)
		{
			char value = param->data_ind.data[i];
			printf("%c", value);

			point = value;

			// resume the task to send data to radio
			vTaskResume(send_data_to_radio_handle);
		}
		printf("\n");

		writeHandle = param->data_ind.handle;

		// printf("hanlde ------> %d", writeHandle);
		// esp_spp_write(param->data_ind.handle, param->data_ind.len, param->data_ind.data);
		esp_spp_write(writeHandle, strlen("\nResuming task..."), (uint8_t *)"\nResuming task...");

#else
		gettimeofday(&time_new, NULL);
		data_num += param->data_ind.len;
		if (time_new.tv_sec - time_old.tv_sec >= 3)
		{
			print_speed();
		}
#endif
		break;
	case ESP_SPP_CONG_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
		break;
	case ESP_SPP_WRITE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status,
				 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
		gettimeofday(&time_old, NULL);
		break;
	case ESP_SPP_SRV_STOP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
		break;
	case ESP_SPP_UNINIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
		break;
	default:
		break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	char bda_str[18] = {0};

	switch (event)
	{
	case ESP_BT_GAP_AUTH_CMPL_EVT:
	{
		if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
		{
			ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
					 bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
		}
		else
		{
			ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
		}
		break;
	}
	case ESP_BT_GAP_PIN_REQ_EVT:
	{
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
		if (param->pin_req.min_16_digit)
		{
			ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
			esp_bt_pin_code_t pin_code = {0};
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
		}
		else
		{
			ESP_LOGI(SPP_TAG, "Input pin code: 1234");
			esp_bt_pin_code_t pin_code;
			pin_code[0] = '1';
			pin_code[1] = '2';
			pin_code[2] = '3';
			pin_code[3] = '4';
			esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
		}
		break;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	case ESP_BT_GAP_CFM_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
		esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
		break;
	case ESP_BT_GAP_KEY_NOTIF_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
		break;
	case ESP_BT_GAP_KEY_REQ_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
		break;
#endif

	case ESP_BT_GAP_MODE_CHG_EVT:
		ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
				 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
		break;

	default:
	{
		ESP_LOGI(SPP_TAG, "event: %d", event);
		break;
	}
	}
	return;
}

/**
 * @brief Task to send data to the base station
 *
 * @param pvParameters
 */
void send_data_to_radio(void *pvParameters)
{
	test_uart_connection();

	get_eui_from_radio();

	set_app_key(APP_KEY);

	configure_regional_settings(BAND, DR, CHANNELS);
	esp_spp_write(writeHandle, strlen("\nConfiguring regional settings..."), (uint8_t *)"\nconfiguring regional settings...");
	// Send  data
	if (join_the_things_network())
	{
		while (1)
		{
			// Read voltage from battery
			float v_bat = read_voltage(); // voltage in mV
			ESP_LOGI(TAG_BATTERY, "Voltage read from battery mV: %f\n", v_bat);
			// esp_spp_write(writeHandle, strlen("\nVoltage read from battery mV: "), (uint8_t *)"\nVoltage read from battery mV: ");
			// esp_spp_write(writeHandle, sizeof(v_bat), (uint8_t *)&v_bat);

			ESP_LOGI(TAG_UART, "Sending messages...");
			esp_spp_write(writeHandle, strlen("\nSending messages..."), (uint8_t *)"\nSending messages...");

			for (int count = 1; count <= 600; count++)
			{
				char msg[64];
				snprintf(msg, sizeof(msg), "{'v':%.0f,'p':'%c','c':%d}", v_bat, point, count);
				ESP_LOGI(TAG_UART, "Message: %s", msg);
				// send_message(msg, 1);
				// Send the message once
				if (send_message(msg, 1) == 1)
				{
					ESP_LOGI(TAG_UART, "Message sent successfully");
				}
				else
				{
					ESP_LOGI(TAG_UART, "Message not sent");
				}
				esp_spp_write(writeHandle, strlen("\n"), (uint8_t *)"\n");
				esp_spp_write(writeHandle, strlen(msg), (uint8_t *)msg);

				vTaskDelay(pdMS_TO_TICKS(50));
			}

			set_radio_low_power();

			// Delete the current task
			//	vTaskDelete(NULL);
			ESP_LOGI(TAG_UART, "OK, suspending task!!!!!");
			esp_spp_write(writeHandle, strlen("\nOK, suspending task!!!!!"), (uint8_t *)"\nOK, suspending task!!!!!");
			esp_restart();
			// vTaskSuspend(NULL);
		}
	}
	else
	{
		ESP_LOGE(TAG_UART, "Failed to join the network");
		esp_spp_write(writeHandle, strlen("\nFailed to join the network"), (uint8_t *)"\nFailed to join the network");
		esp_restart();
		// vTaskSuspend(NULL);
	}
}

void app_main(void)
{
	char bda_str[18] = {0};
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_init()) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_enable()) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	esp_spp_cfg_t bt_spp_cfg = {
		.mode = esp_spp_mode,
		.enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
		.tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
	};
	if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK)
	{
		ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	/* Set default parameters for Secure Simple Pairing */
	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

	/*
	 * Set default parameters for Legacy Pairing
	 * Use variable pin, input pin code when pairing
	 */
	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	esp_bt_pin_code_t pin_code;
	esp_bt_gap_set_pin(pin_type, 0, pin_code);

	ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
	//---------------------------------------------------------------------------------------------------------------------
	xTaskCreate(send_data_to_radio, "send_data_to_radio", configMINIMAL_STACK_SIZE * 5, NULL, configMAX_PRIORITIES - 1, &send_data_to_radio_handle);
	vTaskSuspend(send_data_to_radio_handle);

	// init buffer
	// data_uart = (char *)malloc(RX_BUF_SIZE + 1);

	//? Inicialización de pines
	init_pins();

	// resume task
	// vTaskResume(send_data_to_radio_handle);
}

/**
 * @brief Inicializa los pines a utilizar
 *
 * @return esp_err_t
 */
esp_err_t init_pins()
{
	// Configurar los pines de la interfaz serial
	uart_config_t uart_config =
		{.baud_rate = 9600,
		 .data_bits = UART_DATA_8_BITS,
		 .parity = UART_PARITY_DISABLE,
		 .stop_bits = UART_STOP_BITS_1,
		 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		 .source_clk = UART_SCLK_DEFAULT};
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
	ESP_ERROR_CHECK(
		uart_set_pin(UART_NUM_2,
					 TX_PIN, RX_PIN,
					 UART_PIN_NO_CHANGE,
					 UART_PIN_NO_CHANGE));
	//  esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
	ESP_ERROR_CHECK(
		uart_driver_install(UART_NUM_2, BUF_SIZE, 0, 0, NULL, 0));

	// Configurar el pin V_BAT como entrada analógica
	//-------------ADC1 Init---------------//
	adc_oneshot_unit_init_cfg_t init_config1 = {
		.unit_id = ADC_UNIT_1,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

	//-------------ADC1 Config---------------//
	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = ADC_ATTEN_DB_11,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, V_BAT_ADC1_CHAN6, &config));

	//-------------ADC1 Calibration Init---------------//
	do_calibration1 = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);
	return ESP_OK;
}

/**
 * @brief Leer el voltaje de la batería
 *
 * @return float
 */
float read_voltage()
{
	// Leer el valor del pin analógico
	int adc_reading = 0;
	int adc_volt = 0;
	int sum = 0;

	if (do_calibration1)
	{
		for (int i = 0; i < 10; i++)
		{
			adc_oneshot_read(adc1_handle, V_BAT_ADC1_CHAN6, &adc_reading);
			// ESP_LOGI(TAG_ADC, "ADC -> %d", adc_reading);
			// vTaskDelay(pdMS_TO_TICKS(5)); // esperar un poco antes de tomar la siguiente lectura
			adc_cali_raw_to_voltage(adc1_cali_handle, adc_reading, &adc_volt);
			vTaskDelay(pdMS_TO_TICKS(10)); // esperar un poco antes de tomar la siguiente lectura
			sum += adc_volt;
		}
		adc_volt = sum / 10;
	}
	else
	{
		ESP_LOGE(TAG_BATTERY, "Error in calibration");
	}

	ESP_LOGI(TAG_ADC, "ADC BATT VOLT -> %d", adc_volt);
	return (float)adc_volt;
}

/**
 * @brief Inicializa el ADC para realizar la calibración
 *
 * @param unit
 * @param atten
 * @param out_handle
 * @return true
 * @return false
 */
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
	adc_cali_handle_t handle = NULL;
	esp_err_t ret = ESP_FAIL;
	bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	if (!calibrated)
	{
		ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Curve Fitting");
		adc_cali_curve_fitting_config_t cali_config = {
			.unit_id = unit,
			.atten = atten,
			.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
		if (ret == ESP_OK)
		{
			calibrated = true;
		}
	}
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	if (!calibrated)
	{
		ESP_LOGI(TAG_ADC, "calibration scheme version is %s", "Line Fitting");
		adc_cali_line_fitting_config_t cali_config = {
			.unit_id = unit,
			.atten = atten,
			.bitwidth = ADC_BITWIDTH_DEFAULT,
		};
		ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
		if (ret == ESP_OK)
		{
			calibrated = true;
		}
	}
#endif

	*out_handle = handle;
	if (ret == ESP_OK)
	{
		ESP_LOGI(TAG_ADC, "Calibration Success");
	}
	else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
	{
		ESP_LOGW(TAG_ADC, "eFuse not burnt, skip software calibration");
	}
	else
	{
		ESP_LOGE(TAG_ADC, "Invalid arg or no memory");
	}

	return calibrated;
}
