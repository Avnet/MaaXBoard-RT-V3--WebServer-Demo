/*
 * Copyright 2019-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "task.h"

#include "mcmgr.h"

#include "lwip/tcpip.h"
#include "wpl.h"
#include "httpsrv.h"
#include "http_server.h"
#include "webconfig.h"
#include "cred_flash_storage.h"

#include "cJSON.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int cgi_led(HTTPSRV_CGI_REQ_STRUCT *param);
static int CGI_HandleGet(HTTPSRV_CGI_REQ_STRUCT *param);
static int CGI_HandleGetSensor(HTTPSRV_CGI_REQ_STRUCT *param);
static int CGI_HandlePost(HTTPSRV_CGI_REQ_STRUCT *param);
static int CGI_HandleReset(HTTPSRV_CGI_REQ_STRUCT *param);
static int CGI_HandleStatus(HTTPSRV_CGI_REQ_STRUCT *param);

static uint32_t SetBoardToClient();
static uint32_t SetBoardToAP();
static uint32_t CleanUpAP();
static uint32_t CleanUpClient();

typedef enum board_wifi_states
{
    WIFI_STATE_CLIENT,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CLIENT_SCAN,
    WIFI_STATE_AP,
    WIFI_STATE_AP_SCAN,
} board_wifi_states;

struct board_state_variables
{
    board_wifi_states wifiState;
    char ssid[WPL_WIFI_SSID_LENGTH];
    char password[WPL_WIFI_PASSWORD_LENGTH];
    bool connected;
    TaskHandle_t wifiTask;
};

/*******************************************************************************
 * Definitions
 ******************************************************************************/


const HTTPSRV_CGI_LINK_STRUCT cgi_lnk_tbl[] = {
	{"led", cgi_led},
    {"reset", CGI_HandleReset},
    {"get", CGI_HandleGet},
	{"imu", CGI_HandleGetSensor},
    {"post", CGI_HandlePost},
    {"status", CGI_HandleStatus},
    {0, 0} // DO NOT REMOVE - last item - end of table
};

struct board_state_variables g_BoardState;

/*******************************************************************************
 * Code
 ******************************************************************************/
struct _wsclient
{
	uint32_t handleId;
	struct _wsclient *next;
};

static struct _wsclient *wsClients = NULL;
static int16_t fxos_pitch;
static int16_t fxos_roll;
static int16_t fxos_yaw;
static int16_t light_distance;

enum USER_LED
{
	RED = 0,
	GREEN = 1,
	BLUE = 2
};

typedef struct{
	GPIO_Type *base;
	uint32_t pin;
}led_struct;

led_struct led_map[3] =
{
	{BOARD_USER_LED_RED_GPIO, BOARD_USER_LED_RED_GPIO_PIN},
	{BOARD_USER_LED_GREEN_GPIO, BOARD_USER_LED_GREEN_GPIO_PIN},
	{BOARD_USER_LED_BLUE_GPIO, BOARD_USER_LED_BLUE_GPIO_PIN}
};

static uint32_t get_user_led(enum USER_LED _usr_led)
{
	return GPIO_PinRead(led_map[_usr_led].base, led_map[_usr_led].pin);
}

static void set_user_led(enum USER_LED _usr_led, uint8_t state)
{
	GPIO_PinWrite(led_map[_usr_led].base, led_map[_usr_led].pin, state);
}

static bool check_cmd_json(cJSON *json_obj, char *cmd)
{
	cJSON *jsonItem = NULL;
	if (json_obj != NULL)
	{
		jsonItem = cJSON_GetObjectItem(json_obj, "cmdType");
		if (jsonItem->type == cJSON_String && (jsonItem->valuestring != NULL))
		{
			if (strcmp(jsonItem->valuestring, cmd)==0)
			{
				return true;
			}
		}
	}
	return false;
}

static int32_t get_intVal_json(cJSON *json_obj, char *field)
{
	cJSON *jsonItem = NULL;
	if (json_obj != NULL)
	{
		jsonItem = cJSON_GetObjectItem(json_obj, field);
		if (jsonItem->type == cJSON_Number)
		{
			return jsonItem->valueint;
		}
	}
	return -1;
}

/* LED Common Gateway Interface callback. */
static int cgi_led(HTTPSRV_CGI_REQ_STRUCT *param)
{
	HTTPSRV_CGI_RES_STRUCT response = {0};

	response.ses_handle  = param->ses_handle;
	response.status_code = HTTPSRV_CODE_OK;
	char str[7];
	int length = 0;
	char * json_serialized;
	if (param->request_method == HTTPSRV_REQ_GET)
	{
		cJSON *led = NULL;
		cJSON *cmd = NULL;
		cJSON *jsonObj = cJSON_CreateObject();
		for (int i=0; i<3; i++)
		{
			length += sprintf(str+length, "%d", get_user_led(i));
		}
		str[length] = 0;

		cmd = cJSON_CreateString("getLed");
		cJSON_AddItemToObject(jsonObj, "cmdType", cmd);

		led = cJSON_CreateString(str);
		cJSON_AddItemToObject(jsonObj, "led", led);

		json_serialized = cJSON_Print(jsonObj);
		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
	}
	else if (param->request_method == HTTPSRV_REQ_POST)
	{
		uint32_t length = 0;
		uint32_t read;
		char buffer[100] = {0};
		int32_t led_num = -1;
		int32_t led_state = 0;
		length = param->content_length;
		read   = HTTPSRV_cgi_read(param->ses_handle, buffer, (length > sizeof(buffer)) ? sizeof(buffer) : length);

		if (read > 0)
		{
			cJSON *led_json = cJSON_Parse(buffer);

			if (check_cmd_json(led_json, "setLed"))
			{
				led_num = get_intVal_json(led_json, "ledNum");
				led_state = get_intVal_json(led_json, "ledState");

			}
			if (led_json != NULL)
			{
				cJSON_Delete(led_json);
			}
		}

		if (led_num >=0 && led_num < 3)
		{
			set_user_led((uint8_t)led_num, (uint8_t)led_state);
			length = snprintf(str, sizeof(str), "OK\n");
		}
		else
		{
			length = snprintf(str, sizeof(str), "FAIL\n");
		}

	    response.ses_handle   = param->ses_handle;
	    response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
	    response.status_code  = HTTPSRV_CODE_OK;
	    /*
	    ** When the keep-alive is used we have to calculate a correct content length
	    ** so the receiver knows when to ACK the data and continue with a next request.
	    ** Please see RFC2616 section 4.4 for further details.
	    */

	    /* Calculate content length while saving it to buffer */

	    response.data           = str;
	    response.data_length    = length;
	    response.content_length = response.data_length;
	    /* Send response */
	    HTTPSRV_cgi_write(&response);
	}

	return (response.content_length);
}


static void add_node(uint32_t handleId)
{
	struct _wsclient *currClient = wsClients;
	if (currClient==NULL)
	{
		currClient = pvPortMalloc(sizeof(struct _wsclient));
		currClient->handleId = handleId;
		currClient->next = NULL;
		wsClients = currClient;
	}
	else
	{
		while(currClient->next !=NULL)
		{
			currClient = currClient->next;
		}
		currClient->next = pvPortMalloc(sizeof(struct _wsclient));
		(currClient->next)->handleId = handleId;
		currClient->next->next = NULL;
	}
}

static int32_t delete_node(uint32_t handleId)
{
	struct _wsclient *currClient = wsClients;
	struct _wsclient *prevClient = NULL;
	if (currClient==NULL)
	{
		return -1;
	}
	else
	{
		while(currClient != NULL && currClient->handleId != handleId)
		{
			prevClient = currClient;
			currClient = currClient->next;
		}

		if (currClient == NULL)
		{
			return -1;
		}
		else if (prevClient==NULL) /* first element */
		{
			wsClients = currClient->next;
		}
		else if (currClient != NULL)
		{
			prevClient->next = currClient->next;
		}
		vPortFree(currClient);
		return 1;
	}
}

#if HTTPSRV_CFG_WEBSOCKET_ENABLED
/*
 * Echo plugin code - simple plugin which echoes any message it receives back to
 * client.
 */
uint32_t ws_echo_connect(void *param, WS_USER_CONTEXT_STRUCT context)
{
	//add_node(context.handle);
#if DEBUG_WS
    PRINTF("WebSocket echo client connected.\r\n");
#endif
    return (0);
}

uint32_t ws_echo_disconnect(void *param, WS_USER_CONTEXT_STRUCT context)
{
	//delete_node(context.handle);
#if DEBUG_WS
    PRINTF("WebSocket echo client disconnected.\r\n");
#endif
    return (0);
}

uint32_t ws_echo_message(void *param, WS_USER_CONTEXT_STRUCT context)
{
    WS_send(&context); /* Send back what was received.*/
#if DEBUG_WS
    if (context.data.type == WS_DATA_TEXT)
    {
        /* Print received text message to console. */
        context.data.data_ptr[context.data.length] = 0;
        PRINTF("WebSocket message received:\r\n%s\r\n", context.data.data_ptr);
    }
    else
    {
        /* Inform user about binary message. */
        PRINTF("WebSocket binary data with length of %d bytes received.", context.data.length);
    }
#endif

    return (0);
}

uint32_t ws_echo_error(void *param, WS_USER_CONTEXT_STRUCT context)
{
#if DEBUG_WS
    PRINTF("WebSocket error: 0x%X.\r\n", context.error);
#endif
    return (0);
}

WS_PLUGIN_STRUCT ws_tbl[] = {{"/test", ws_echo_connect, ws_echo_message, ws_echo_error, ws_echo_disconnect, NULL},
                             {0, 0, 0, 0, 0, 0}};
#endif /* HTTPSRV_CFG_WEBSOCKET_ENABLED */



/*CGI*/
/* Example Common Gateway Interface callback. */
/* These callbacks are called from the session tasks according to the Link struct above */
/* The get.cgi request triggers a scan and responds with a list of the SSIDs */
static int CGI_HandleGet(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    /* Buffer for hodling response JSON data */
    char buffer[CGI_BUFFER_LENGTH] = {0};
    char *ssids;

    if (g_BoardState.wifiState == WIFI_STATE_CLIENT || g_BoardState.wifiState == WIFI_STATE_AP)
    {
        /* Initiate Scan */
        int result = WPL_Scan();
        if (result != WPL_SUCCESS)
        {
            PRINTF("[!] Scan Error\r\n");
            ssids = "null"; // Interpreted as error by the website
        }
        else
        {
            /* Get JSON with scanned SSIDs */
            ssids = WPL_getSSIDs();
        }

        // Build the response JSON
        snprintf(buffer, sizeof(buffer), "{\"networks\":%s}", ssids);
    }
    else
    {
        // We can not start a scan if a previous scan is running or if we are connecting
        snprintf(buffer, sizeof(buffer), "{\"networks\":false}");
    }

    // Send the response back to browser
    response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
    response.data           = buffer;
    response.data_length    = strlen(buffer);
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    return (response.content_length);
}

/*CGI*/
/* Example Common Gateway Interface callback. */
/* These callbacks are called from the session tasks according to the Link struct above */
/* The get.cgi request triggers a scan and responds with a list of the SSIDs */
static int CGI_HandleGetSensor(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;
    int length = 0;
    char * json_serialized;
    if (param->request_method == HTTPSRV_REQ_GET)
    {
    	cJSON *jsonObj = cJSON_CreateObject();
		int sensors[4] = {0,0,0};
		sensors[0] = fxos_pitch;
		sensors[1] = fxos_roll;
		sensors[2] = fxos_yaw;
		sensors[3] = light_distance;
		cJSON *sensor = NULL;
		cJSON *cmd = NULL;

		if (param->request_method == HTTPSRV_REQ_GET)
		{
			sensor = cJSON_CreateIntArray(sensors,4);
		}

		cmd = cJSON_CreateString("imu");
		cJSON_AddItemToObject(jsonObj, "cmdType", cmd);
		cJSON_AddItemToObject(jsonObj, "sensor", sensor);
		json_serialized = cJSON_Print(jsonObj);

		length = strlen(json_serialized);

		response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
		response.data           = json_serialized;
		response.data_length    = length;
		response.content_length = response.data_length;
		HTTPSRV_cgi_write(&response);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
    }
    return (response.content_length);
}



/* The post.cgi request is used for triggering a connection to an external AP */
static int CGI_HandlePost(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    char buffer[CGI_BUFFER_LENGTH] = {0};

    uint32_t length = 0;
    uint32_t read;
    char posted_passphrase[WPL_WIFI_PASSWORD_LENGTH + 1];
    char posted_ssid[WPL_WIFI_SSID_LENGTH + 1];

    /* We can not join another AP  if we are already connected to one */
    if (g_BoardState.wifiState == WIFI_STATE_CLIENT)
    {
        response.data           = "{\"status\":\"failed\"}";
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        return 0;
    }

    length = param->content_length;
    read   = HTTPSRV_cgi_read(param->ses_handle, buffer, (length < sizeof(buffer)) ? length : sizeof(buffer));

    if (read > 0)
    {
        cgi_get_varval(buffer, "post_ssid", posted_ssid, sizeof(posted_ssid));
        cgi_get_varval(buffer, "post_passphrase", posted_passphrase, sizeof(posted_passphrase));
        cgi_urldecode(posted_ssid);
        cgi_urldecode(posted_passphrase);
    }

    /* Any post processing of the posted data (sanitation, validation) */
    format_post_data(posted_ssid);
    format_post_data(posted_passphrase);

    WC_DEBUG("[i] Chosen ssid: %s\r\n[i] Chosen passphrase: \"%s\" \r\n", posted_ssid, posted_passphrase);

    response.content_type = HTTPSRV_CONTENT_TYPE_HTML;

    /* Initiate joining process */
    PRINTF("[i] Joining: %s\r\n", posted_ssid);
    int32_t result = WPL_Join(posted_ssid, posted_passphrase);
    if (result != WPL_SUCCESS)
    {
        PRINTF("[!] Cannot connect to wifi\r\n[!]ssid: %s\r\n[!]passphrase: %s\r\n", posted_ssid, posted_passphrase);
        /* Respond with a failure to the browser */
        response.data           = "{\"status\":\"failed\"}";
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        response.data_length    = 0;
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
    }
    else
    {
        /* We have successfully connected however the old AP is still running.
         * This session is still active and will try replying to the browser with a success message.
         * This message will also hold the new IP address under which the board will be reachable */
        PRINTF("[i] Successfully joined: %s\r\n", posted_ssid);
        char ip[32];
        /* Get new client address to be sent back to the old browser session */
        WPL_GetIP(ip, 1);
        PRINTF(" Now join that network on your device and connect to this IP: %s\r\n", ip);

        snprintf(buffer, sizeof(buffer), "{\"status\":\"success\",\"new_ip\":\"%s\"}", ip);

        response.data           = buffer;
        response.data_length    = strlen(response.data);
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);
        response.data_length    = 0;
        response.content_length = response.data_length;
        HTTPSRV_cgi_write(&response);

        g_BoardState.wifiState = WIFI_STATE_CLIENT;
        g_BoardState.connected = true;
        /* Since the Joining was successful, we can save the credentials to the Flash */
        save_wifi_credentials(CONNECTION_INFO_FILENAME, posted_ssid, posted_passphrase);

        /* Resume the main task, this will make sure to clean up and shut down the AP*/
        /* Since g_BoardState.connected == true, the reconnection to AP will be skipped and
         * the main task will be put back to sleep waiting for a reset event */
        vTaskResume(g_BoardState.wifiTask);
    }

    return (response.content_length);
}

/* The reset.cgi is used to clear the Flash memory and reset the board back to AP mode */
static int CGI_HandleReset(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response;

    response.ses_handle   = param->ses_handle;
    response.status_code  = HTTPSRV_CODE_OK;
    response.content_type = HTTPSRV_CONTENT_TYPE_PLAIN;
    char str_buffer[128];

    /* Try to clear the flash memory */
    if (reset_saved_wifi_credentials(CONNECTION_INFO_FILENAME) != 0)
    {
        PRINTF("[!] Error occured during resetting of saved credentials!\r\n");
        response.data        = "{\"status\":\"failed\"}";
        response.data_length = strlen(response.data);
    }
    else
    {
        /* The new ip will be the static ip configured for the local AP */
        snprintf(str_buffer, sizeof(str_buffer), "{\"status\":\"success\",\"new_ip\":\"%s\"}", WIFI_AP_IP_ADDR);

        response.data        = str_buffer;
        response.data_length = strlen(str_buffer);
    }

    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);
    response.data_length    = 0;
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    // If we were client, disconnect from the external AP and start local AP
    if (g_BoardState.wifiState == WIFI_STATE_CLIENT)
    {
        g_BoardState.wifiState = WIFI_STATE_AP;
        g_BoardState.connected = false;

        vTaskResume(g_BoardState.wifiTask);
    }

    return 0;
}

/*CGI*/
/* Example Common Gateway Interface callback. */
/* These callbacks are called from the session tasks according to the Link struct above */
/* The status  status.cgi request returns status */
static int CGI_HandleStatus(HTTPSRV_CGI_REQ_STRUCT *param)
{
    HTTPSRV_CGI_RES_STRUCT response = {0};

    response.ses_handle  = param->ses_handle;
    response.status_code = HTTPSRV_CODE_OK;

    /* Buffer for hodling response JSON data */
    char buffer[CGI_BUFFER_LENGTH] = {0};
    char ip[16];
    char status_str[32] = {'\0'};

    // Get the Board IP address
    switch (g_BoardState.wifiState)
    {
        case WIFI_STATE_CONNECTING:
            strcpy(status_str, "connecting");
            WPL_GetIP(ip, 0);
            break;
        case WIFI_STATE_CLIENT_SCAN:
            strcpy(status_str, "scan_");
        case WIFI_STATE_CLIENT:
            strcat(status_str, "client");
            WPL_GetIP(ip, 1);
            break;
        case WIFI_STATE_AP_SCAN:
            strcpy(status_str, "scan_");
        case WIFI_STATE_AP:
        default:
            strcat(status_str, "ap");
            WPL_GetIP(ip, 0);
    }

    // Build the response JSON
    snprintf(buffer, sizeof(buffer), "{\"info\":{\"name\":\"%s\",\"ip\":\"%s\",\"ap\":\"%s\",\"status\":\"%s\"}}",
             BOARD_NAME, ip, g_BoardState.ssid, status_str);

    // Send the response back to browser
    response.content_type   = HTTPSRV_CONTENT_TYPE_PLAIN;
    response.data           = buffer;
    response.data_length    = strlen(buffer);
    response.content_length = response.data_length;
    HTTPSRV_cgi_write(&response);

    return (response.content_length);
}

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Address of memory, from which the secondary core will boot */
#define CORE1_BOOT_ADDRESS (void *)0x20200000

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif (defined(__GNUC__)) && (!defined(__MCUXPRESSO))
extern const char core1_image_start[];
extern const char *core1_image_end;
extern int core1_image_size;
#define CORE1_IMAGE_START ((void *)core1_image_start)
#define CORE1_IMAGE_SIZE  ((void *)core1_image_size)
#endif
#define APP_TASK_STACK_SIZE                                (configMINIMAL_STACK_SIZE + 100)
#define APP_READY_EVENT_DATA                               (1U)
#define APP_MESSAGE_BUFFER_EVENT_DATA                      (1U)
#define APP_MESSAGE_BUFFER_SIZE                            (32U)
#define APP_SH_MEM_PRIMARY_TO_SECONDARY_MB_OFFSET          (0x0u)
#define APP_SH_MEM_SECONDARY_TO_PRIMARY_MB_OFFSET          (0x4u)
#define APP_SH_MEM_PRIMARY_TO_SECONDARY_MB_STRUCT_OFFSET   (0x50u)
#define APP_SH_MEM_SECONDARY_TO_PRIMARY_MB_STRUCT_OFFSET   (0xA0u)
#define APP_SH_MEM_PRIMARY_TO_SECONDARY_BUF_STORAGE_OFFSET (0x100u)
#define APP_SH_MEM_SECONDARY_TO_PRIMARY_BUF_STORAGE_OFFSET (0x200u)

typedef struct the_message
{
    uint32_t DATA;
} THE_MESSAGE, *THE_MESSAGE_PTR;

volatile THE_MESSAGE msg = {0};
static volatile uint8_t sensor_data[30];

#define SH_MEM_TOTAL_SIZE (6144U)
#if defined(__ICCARM__) /* IAR Workbench */
extern unsigned char rpmsg_sh_mem_start[];
extern unsigned char rpmsg_sh_mem_end[];
#define APP_SH_MEM_BASE rpmsg_sh_mem_start
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION) /* Keil MDK */
extern unsigned char Image$$RPMSG_SH_MEM$$Base;
extern unsigned char Image$$RPMSG_SH_MEM$$Length;
#define APP_SH_MEM_BASE &Image$$RPMSG_SH_MEM$$Base
#elif defined(__GNUC__) /* GCC */
unsigned char rpmsg_sh_mem[SH_MEM_TOTAL_SIZE] __attribute__((section(".noinit.$rpmsg_sh_mem")));
#define APP_SH_MEM_BASE (uint32_t) & rpmsg_sh_mem
#else
#error "Message Buffers are not placed in shared memory!"
#endif

#define xPrimaryToSecondaryMessageBuffer \
    (*(MessageBufferHandle_t *)(APP_SH_MEM_BASE + APP_SH_MEM_PRIMARY_TO_SECONDARY_MB_OFFSET))
#define xSecondaryToPrimaryMessageBuffer \
    (*(MessageBufferHandle_t *)(APP_SH_MEM_BASE + APP_SH_MEM_SECONDARY_TO_PRIMARY_MB_OFFSET))
#define xPrimaryToSecondaryMessageBufferStruct \
    (*(StaticStreamBuffer_t *)(APP_SH_MEM_BASE + APP_SH_MEM_PRIMARY_TO_SECONDARY_MB_STRUCT_OFFSET))
#define xSecondaryToPrimaryMessageBufferStruct \
    (*(StaticStreamBuffer_t *)(APP_SH_MEM_BASE + APP_SH_MEM_SECONDARY_TO_PRIMARY_MB_STRUCT_OFFSET))
#define ucPrimaryToSecondaryBufferStorage \
    (*(uint8_t *)(APP_SH_MEM_BASE + APP_SH_MEM_PRIMARY_TO_SECONDARY_BUF_STORAGE_OFFSET))
#define ucSecondaryToPrimaryBufferStorage \
    (*(uint8_t *)(APP_SH_MEM_BASE + APP_SH_MEM_SECONDARY_TO_PRIMARY_BUF_STORAGE_OFFSET))
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif
/*
 * configSUPPORT_STATIC_ALLOCATION is set to 1, requiring this callback to
 * provide statically allocated data for use by the idle task, which is a task
 * created by the scheduler when it starts.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__core1_image"
    image_size = (uint32_t)__section_end("__core1_image") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    image_size = (uint32_t)core1_image_size;
#endif
    return image_size;
}
#endif
static volatile uint16_t RemoteAppReadyEventData = 0U;
static StaticTask_t xTaskBuffer;
static StackType_t xStack[APP_TASK_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide
    an implementation of vApplicationGetIdleTaskMemory() to provide the memory
    that is used by the Idle task.
    www.freertos.org/a00110.html#configSUPPORT_STATIC_ALLOCATION */

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
void vGeneratePrimaryToSecondaryInterrupt(void *xUpdatedMessageBuffer)
{
    /* Trigger the inter-core interrupt using the MCMGR component.
       Pass the APP_MESSAGE_BUFFER_EVENT_DATA as data that accompany
       the kMCMGR_FreeRtosMessageBuffersEvent event. */
    (void)MCMGR_TriggerEventForce(kMCMGR_FreeRtosMessageBuffersEvent, APP_MESSAGE_BUFFER_EVENT_DATA);
}

static void FreeRtosMessageBuffersEventHandler(uint16_t eventData, void *context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Make sure the message has been addressed to us. Using eventData that accompany
       the event of the kMCMGR_FreeRtosMessageBuffersEvent type, we can distinguish
       different consumers. */
    if (APP_MESSAGE_BUFFER_EVENT_DATA == eventData)
    {
        /* Call the API function that sends a notification to any task that is
    blocked on the xUpdatedMessageBuffer message buffer waiting for data to
    arrive. */
        (void)xMessageBufferSendCompletedFromISR(xSecondaryToPrimaryMessageBuffer, &xHigherPriorityTaskWoken);
    }

    /* Normal FreeRTOS "yield from interrupt" semantics, where
    HigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
    pdTRUE if the interrupt unblocks a task that has a priority above that of
    the currently executing task. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    /* No need to clear the interrupt flag here, it is handled by the mcmgr. */
}

static void RemoteAppReadyEventHandler(uint16_t eventData, void *context)
{
    uint16_t *data = (uint16_t *)context;

    *data = eventData;
}

/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    (void)MCMGR_EarlyInit();
}

static void broadcast_wsclients(char * jsonString, uint32_t length)
{
	WS_USER_CONTEXT_STRUCT context;
	context.data.data_ptr = jsonString;
	context.data.length = length;
	context.data.type = WS_DATA_TEXT;
	context.error = WS_ERR_OK;
	context.fin_flag = 1;
	struct _wsclient *currClient = wsClients;
	while(currClient!=NULL)
	{
		context.handle = currClient->handleId;
		WS_send(&context);
		currClient = currClient->next;
	}
}

static void sensor_task(void *pvParameters)
{
	uint8_t arr[10] = {10, 25, 13, 100, 50, 64, 0, 150, 250, 100};
	uint8_t index = 0;
	char * json_serialized;
	uint32_t length = 0;
	while(1)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);
		if (index>=10)
		{
			index=0;
		}
		cJSON *jsonObj = cJSON_CreateObject();
		cJSON *sensor = NULL;
		cJSON *cmd = NULL;

		cmd = cJSON_CreateString("bcSensor");
		cJSON_AddItemToObject(jsonObj, "cmdType", cmd);

		sensor = cJSON_CreateNumber(arr[index++]);
		cJSON_AddItemToObject(jsonObj, "sensor", sensor);

		json_serialized = cJSON_Print(jsonObj);
		length = strlen(json_serialized);

		broadcast_wsclients(json_serialized, length);
		vPortFree(json_serialized);
		cJSON_Delete(jsonObj);
	}
	vTaskSuspend(NULL);
}

typedef struct sensor_t {
	uint8_t data_type;
	int16_t yaw_t;
	int16_t roll_t;
	int16_t pitch_t;
	int16_t distance;
} sensor_t;

static void app_task(void *param)
{
	sensor_t * sensor_ptr;

    size_t xReceivedBytes;
    (void)memset((void *)(char *)APP_SH_MEM_BASE, 0, SH_MEM_TOTAL_SIZE);
    /* Create the Primary-To-Secondary message buffer, statically allocated at a known location
       as both cores need to know where they are. */
    xPrimaryToSecondaryMessageBuffer = xMessageBufferCreateStatic(
        /* The buffer size in bytes. */
        APP_MESSAGE_BUFFER_SIZE,
        /* Statically allocated buffer storage area. */
        &ucPrimaryToSecondaryBufferStorage,
        /* Message buffer handle. */
        &xPrimaryToSecondaryMessageBufferStruct);

#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* This section ensures the secondary core image is copied from flash location to the target RAM memory.
       It consists of several steps: image size calculation and image copying.
       These steps are not required on MCUXpresso IDE which copies the secondary core image to the target memory during
       startup automatically. */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    (void)PRINTF("Copy CORE1 image to address: 0x%x, size: %d\r\n", (void *)(char *)CORE1_BOOT_ADDRESS,
                 core1_image_size);

    /* Copy application from FLASH to RAM */
    (void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif

    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();

    /* Register the application event before starting the secondary core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RemoteAppReadyEventHandler,
                              (void *)&RemoteAppReadyEventData);

    /* Boot Secondary core application */
    (void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, 0, kMCMGR_Start_Synchronous);

    /* Wait until the secondary core application signals it is ready to communicate. */
    while (APP_READY_EVENT_DATA != RemoteAppReadyEventData)
    {
    };

    (void)MCMGR_RegisterEvent(kMCMGR_FreeRtosMessageBuffersEvent, FreeRtosMessageBuffersEventHandler, ((void *)0));

    while (1)
    {
        xReceivedBytes =
            xMessageBufferReceive(xSecondaryToPrimaryMessageBuffer, (void *)&sensor_data[0], APP_MESSAGE_BUFFER_SIZE, portMAX_DELAY);

        if (sensor_data[0] == 0x01)
        {
        	// imu sensor
        	sensor_ptr = &sensor_data[0];
        	fxos_pitch = sensor_ptr->pitch_t;
        	fxos_roll = sensor_ptr->roll_t;
        	fxos_yaw = sensor_ptr->yaw_t;
        	light_distance = sensor_ptr->distance;
//        	(void)PRINTF("Message: Size=%x, DATA = %d, %d, %d, %d\r\n", xReceivedBytes, fxos_pitch, fxos_roll, fxos_yaw, light_distance);
        }
    }

    vMessageBufferDelete(xPrimaryToSecondaryMessageBuffer);
    vTaskDelay(50);
    /* Print the ending banner */
    (void)PRINTF("\r\nFreeRTOS communication task ends here.\r\n");
    for (;;)
    {
    }
}

/*!
 * @brief The main task function
 */
static void wifi_task(void *arg)
{
    uint32_t result = 1;

    PRINTF(
        "\r\n"
        "Starting MaaXBoard Webserver DEMO\r\n");

    /* When the App starts up, it will first read the mflash to check if any
     * credentials have been saved from previous runs.
     * If the mflash is empty, the board starts and AP allowing the user to configure
     * the desired Wi-Fi network.
     * Otherwise the stored credentials will be used to connect to the WiFi network.*/
    WC_DEBUG("[i] Trying to load data from mflash.\r\n");

    char ssid[WPL_WIFI_SSID_LENGTH];
    char password[WPL_WIFI_PASSWORD_LENGTH];

#if defined(SSID_AUTO) && (SSID_AUTO==1)
    init_flash_storage(CONNECTION_INFO_FILENAME);
    result = get_saved_wifi_credentials(CONNECTION_INFO_FILENAME, ssid, password);
#else
    /* temporary input the ssid, password */
    strcpy(ssid, SSID_DEF);
    strcpy(password, PASSWORD_DEF);
    result = 0;
#endif

    if (result == 0 && strcmp(ssid, "") != 0)
    {
        /* Credentials from last time have been found. The board will attempt to
         * connect to this network as a client */
        WC_DEBUG("[i] Saved SSID: %s, Password: %s\r\n", ssid, password);
        g_BoardState.wifiState = WIFI_STATE_CLIENT;
        strcpy(g_BoardState.ssid, ssid);
        strcpy(g_BoardState.password, password);
    }
    else
    {
        /* No credentials are stored, the board will start its own AP */
        WC_DEBUG("[i] Nothing stored yet\r\n");
        strcpy(g_BoardState.ssid, WIFI_SSID);
        strcpy(g_BoardState.password, WIFI_PASSWORD);
        g_BoardState.wifiState = WIFI_STATE_AP;
    }

    g_BoardState.connected = false;

    /* Initialize WiFi board */
    WC_DEBUG("[i] Initializing WiFi connection... \r\n");
    WPL_Init();
    if ((result = WPL_Start()) != WPL_SUCCESS)
    {
        PRINTF("[!] Could not initialize WiFi module %d\r\n", (uint32_t)result);
        __BKPT(0);
    }
    else
    {
        WC_DEBUG("[i] Successfully initialized WiFi module\r\n");
    }

    /* Start WebServer */
    if (xTaskCreate(http_srv_task, "http_srv_task", HTTPD_STACKSIZE, NULL, HTTPD_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("[!] HTTPD Task creation failed.");
        while (1)
            __BKPT(0);
    }

    /* Here other tasks can be created that will run the enduser app.... */

    /* Main Loop */
    while (1)
    {
        /* The SetBoardTo<state> function will configure the board Wifi to that given state.
         * After that, this task will suspend itself. It will remain suspended until it is time
         * to switch the state again. Uppon resuming, it will clean up the current state.
         * Every time the WiFi state changes, this loop will perform an iteration switching back
         * and fourth between the two states as required.
         */
        switch (g_BoardState.wifiState)
        {
            case WIFI_STATE_CLIENT:
                SetBoardToClient();
                /* Suspend here until its time to swtich back to AP */
                vTaskSuspend(NULL);
                CleanUpClient();
                break;
            case WIFI_STATE_AP:
            default:
                SetBoardToAP();
                /* Suspend here until its time to stop the AP */
                vTaskSuspend(NULL);
                CleanUpAP();
        }
        vTaskDelay(5);
    }
}


/* Initialize and start local AP */
static uint32_t SetBoardToAP()
{
    uint32_t result;

    /* Set the global ssid and password to the default AP ssid and password */
    strcpy(g_BoardState.ssid, WIFI_SSID);
    strcpy(g_BoardState.password, WIFI_PASSWORD);

    /* Start the access point */
    PRINTF("Starting Access Point: SSID: %s, Chnl: %d\r\n", g_BoardState.ssid, WIFI_AP_CHANNEL);
    result = WPL_Start_AP(g_BoardState.ssid, g_BoardState.password, WIFI_AP_CHANNEL);

    if (result != WPL_SUCCESS)
    {
        PRINTF("[!] Failed to start access point\r\n");
        while (1)
            __BKPT(0);
    }
    g_BoardState.connected = true;
    /* Start DHCP server */
    WPL_StartDHCPServer(WIFI_AP_IP_ADDR, WIFI_AP_NET_MASK);

    char ip[16];
    WPL_GetIP(ip, 0);
    PRINTF(" Now join that network on your device and connect to this IP: %s\r\n", ip);

    return 0;
}

/* Clean up the local AP after waiting for all tasks to clean up */
static uint32_t CleanUpAP()
{
    /* Give time for reply message to reach the web interface before destorying the conection */
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    WC_DEBUG("[i] Stopping AP!\r\n");
    if (WPL_Stop_AP() != WPL_SUCCESS)
    {
        PRINTF("Error while stopping ap\r\n");
        while (1)
            __BKPT(0);
    }

    WPL_StopDHCPServer();
    return 0;
}

/* Connect to the external AP in g_BoardState.ssid */
static uint32_t SetBoardToClient()
{
    int32_t result;
    // If we are already connected, skip the initialization
    if (!g_BoardState.connected)
    {
        PRINTF("Connecting as client to ssid: %s with password %s\r\n\t", g_BoardState.ssid, g_BoardState.password);

        result = WPL_Join(g_BoardState.ssid, g_BoardState.password);
        if (result != WPL_SUCCESS)
        {
            PRINTF("[!] Cannot connect to Wi-Fi\r\n[!]ssid: %s\r\n[!]passphrase: %s\r\n", g_BoardState.ssid,
                   g_BoardState.password);
            char c;
            do
            {
                PRINTF("[i] To reset the board to AP mode, press 'r'.\r\n");
                PRINTF("[i] In order to try connecting again press 'a'.\r\n");

                do
                {
                    c = GETCHAR();
                    // Skip over \n and \r and don't print the prompt again, just get next char
                } while (c == '\n' || c == '\r');

                switch (c)
                {
                    case 'r':
                    case 'R':
                        if (reset_saved_wifi_credentials(CONNECTION_INFO_FILENAME) != 0)
                        {
                            PRINTF("[!] Error occured during resetting of saved credentials!\r\n");
                            while (1)
                                __BKPT(0);
                        }
                        else
                        {
                            // Reset back to AP mode
                            g_BoardState.wifiState = WIFI_STATE_AP;
                            return 0;
                        }
                        break;
                    case 'a':
                    case 'A':
                        // Try connecting again...
                        return 0;
                    default:
                        PRINTF("Unknown command %c, please try again.\r\n", c);
                }

            } while (1);
        }
        else
        {
            PRINTF("[i] Connected to Wi-Fi\r\nssid: %s\r\n[!]passphrase: %s\r\n", g_BoardState.ssid,
                   g_BoardState.password);
            g_BoardState.connected = true;
            char ip[16];
            WPL_GetIP(ip, 1);
            PRINTF(" Now join that network on your device and connect to this IP: %s\r\n", ip);
        }
    }
    return 0;
}

/* Wait for any transmissions to finish and clean up the Client connection */
static uint32_t CleanUpClient()
{
    /* Give time for reply message to reach the web interface before destorying the conection */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* Leave the external AP */
    if (WPL_Leave() != WPL_SUCCESS)
    {
        PRINTF("[!] Error Leaving from Client network.\r\n");
        __BKPT(0);
    }
    return 0;
}

/*!
 * @brief Main function
 */
int main(void)
{
	BaseType_t stat;
    /* Initialize standard SDK demo application pins */
    BOARD_ConfigMPU();
    BOARD_InitPins();
#if defined(WIFI_BOARD_AW_CM358)
    /* Init SDIO_RST */
    BOARD_InitM2WifiResetPins();
#endif
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
#if defined(WIFI_BOARD_AW_CM358)
    /* Set SDIO_RST to 1 */
    gpio_pin_config_t gpio_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

    GPIO_PinInit(GPIO9, 13, &gpio_config);
    GPIO_WritePinOutput(GPIO9, 13, 1);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
    GPIO_WritePinOutput(GPIO9, 13, 0);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
    GPIO_WritePinOutput(GPIO9, 13, 1);
    SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));
#endif


#ifdef FRTOS_MEM_5
    /* Uncomment below if FreeRTOS memory Scheme 5 is used*/
        const HeapRegion_t xHeapRegions[] =
        {
            { ( uint8_t * ) 0x80000000UL, 0x10000 },
            { NULL, 0 } /* Terminates the array. */
        };

        /* Pass the array into vPortDefineHeapRegions(). */
        vPortDefineHeapRegions( xHeapRegions );
#endif

    /* hyperflash storage couldn't write or erase when following task is enabled. This will enable m4 core.*/
    stat = xTaskCreate(app_task, "APP_TASK", 2048, NULL, configMAX_PRIORITIES - 4, NULL);
    assert(pdPASS == stat);

    stat = xTaskCreate(wifi_task, "wifi_task", 2048, NULL, configMAX_PRIORITIES - 4, &g_BoardState.wifiTask);
    assert(pdPASS == stat);

    vTaskStartScheduler();

    (void)PRINTF("Failed to start FreeRTOS on core0.\r\n");
    for (;;)
    {
    }
}
