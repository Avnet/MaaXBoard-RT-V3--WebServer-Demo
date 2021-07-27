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

#include "fsl_fxos.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_freertos.h"
#include "math.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
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



#define MAX_ACCEL_AVG_COUNT 25U
#define HWTIMER_PERIOD      10000U
/* multiplicative conversion constants */
#define DegToRad 0.017453292
#define RadToDeg 57.295779


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
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

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint16_t SampleEventFlag;
fxos_handle_t g_fxosHandle;
uint8_t g_sensor_address[] = {0x1CU, 0x1EU, 0x1DU, 0x1FU};
uint8_t g_sensorRange      = 0;
uint8_t g_dataScale        = 0;

int16_t g_Ax_Raw = 0;
int16_t g_Ay_Raw = 0;
int16_t g_Az_Raw = 0;

double g_Ax = 0;
double g_Ay = 0;
double g_Az = 0;

int16_t g_Ax_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Ay_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Az_buff[MAX_ACCEL_AVG_COUNT] = {0};

int16_t g_Mx_Raw = 0;
int16_t g_My_Raw = 0;
int16_t g_Mz_Raw = 0;

int16_t g_Mx_Offset = -41;
int16_t g_My_Offset = -163;
int16_t g_Mz_Offset = 325;

double g_Mx = 0;
double g_My = 0;
double g_Mz = 0;

double g_Mx_LP = 0;
double g_My_LP = 0;
double g_Mz_LP = 0;

double g_Yaw    = 0;
double g_Yaw_LP = 0;
double g_Pitch  = 0;
double g_Roll   = 0;

bool g_FirstRun = true;

static lpi2c_rtos_handle_t rtosHandle_i2c_accel;
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
void vGenerateSecondaryToPrimaryInterrupt(void *xUpdatedMessageBuffer)
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
        (void)xMessageBufferSendCompletedFromISR(xPrimaryToSecondaryMessageBuffer, &xHigherPriorityTaskWoken);
    }

    /* Normal FreeRTOS "yield from interrupt" semantics, where
    HigherPriorityTaskWoken is initialzed to pdFALSE and will then get set to
    pdTRUE if the interrupt unblocks a task that has a priority above that of
    the currently executing task. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    /* No need to clear the interrupt flag here, it is handled by the mcmgr. */
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

static void Sensor_ReadData(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Mx, int16_t *My, int16_t *Mz)
{
    fxos_data_t fxos_data;

    if (FXOS_ReadSensorData(&g_fxosHandle, &fxos_data) != kStatus_Success)
    {
        PRINTF("Failed to read acceleration data!\r\n");
    }
    /* Get the accel data from the sensor data structure in 14 bit left format data*/
    *Ax = (int16_t)((uint16_t)((uint16_t)fxos_data.accelXMSB << 8) | (uint16_t)fxos_data.accelXLSB) / 4U;
    *Ay = (int16_t)((uint16_t)((uint16_t)fxos_data.accelYMSB << 8) | (uint16_t)fxos_data.accelYLSB) / 4U;
    *Az = (int16_t)((uint16_t)((uint16_t)fxos_data.accelZMSB << 8) | (uint16_t)fxos_data.accelZLSB) / 4U;
    *Ax *= g_dataScale;
    *Ay *= g_dataScale;
    *Az *= g_dataScale;
    *Mx = (int16_t)((uint16_t)((uint16_t)fxos_data.magXMSB << 8) | (uint16_t)fxos_data.magXLSB);
    *My = (int16_t)((uint16_t)((uint16_t)fxos_data.magYMSB << 8) | (uint16_t)fxos_data.magYLSB);
    *Mz = (int16_t)((uint16_t)((uint16_t)fxos_data.magZMSB << 8) | (uint16_t)fxos_data.magZLSB);
}

static void Magnetometer_Calibrate(void)
{
    int16_t Mx_max = 0;
    int16_t My_max = 0;
    int16_t Mz_max = 0;
    int16_t Mx_min = 0;
    int16_t My_min = 0;
    int16_t Mz_min = 0;

    uint32_t times = 0;
    PRINTF("\r\nCalibrating magnetometer...");
    while (times < 5000)
    {
        Sensor_ReadData(&g_Ax_Raw, &g_Ay_Raw, &g_Az_Raw, &g_Mx_Raw, &g_My_Raw, &g_Mz_Raw);
        if (times == 0)
        {
            Mx_max = Mx_min = g_Mx_Raw;
            My_max = My_min = g_My_Raw;
            Mz_max = Mz_min = g_Mz_Raw;
        }
        if (g_Mx_Raw > Mx_max)
        {
            Mx_max = g_Mx_Raw;
        }
        if (g_My_Raw > My_max)
        {
            My_max = g_My_Raw;
        }
        if (g_Mz_Raw > Mz_max)
        {
            Mz_max = g_Mz_Raw;
        }
        if (g_Mx_Raw < Mx_min)
        {
            Mx_min = g_Mx_Raw;
        }
        if (g_My_Raw < My_min)
        {
            My_min = g_My_Raw;
        }
        if (g_Mz_Raw < Mz_min)
        {
            Mz_min = g_Mz_Raw;
        }
        if (times == 4999)
        {
            if ((Mx_max > (Mx_min + 500)) && (My_max > (My_min + 500)) && (Mz_max > (Mz_min + 500)))
            {
                g_Mx_Offset = (Mx_max + Mx_min) / 2;
                g_My_Offset = (My_max + My_min) / 2;
                g_Mz_Offset = (Mz_max + Mz_min) / 2;
                PRINTF("\r\nCalibrate magnetometer successfully!");
                PRINTF("\r\nMagnetometer offset Mx: %d - My: %d - Mz: %d \r\n", g_Mx_Offset, g_My_Offset, g_Mz_Offset);
            }
            else
            {
                PRINTF("Calibrating magnetometer failed! Press any key to recalibrate...\r\n");
                GETCHAR();
                PRINTF("\r\nCalibrating magnetometer...");
                times = 0;
            }
        }
        times++;
        SDK_DelayAtLeastUs(3000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    }
}

static void app_task(void *param)
{
	int32_t length = 0;
	int size = 0;
    /* Create the Secondary-To-Primary message buffer, statically allocated at a known location
       as both cores need to know where they are. */
    xSecondaryToPrimaryMessageBuffer = xMessageBufferCreateStatic(
        /* The buffer size in bytes. */
        APP_MESSAGE_BUFFER_SIZE,
        /* Statically allocated buffer storage area. */
        &ucSecondaryToPrimaryBufferStorage,
        /* Message buffer handle. */
        &xSecondaryToPrimaryMessageBufferStruct);

    uint32_t startupData;
    mcmgr_status_t status;
    int temp = 0;
    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    (void)MCMGR_RegisterEvent(kMCMGR_FreeRtosMessageBuffersEvent, FreeRtosMessageBuffersEventHandler, ((void *)0));

    /* Signal the other core we are ready by triggering the event and passing the APP_READY_EVENT_DATA */
    (void)MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, APP_READY_EVENT_DATA);

	fxos_config_t config = {0};
    status_t result;
    uint16_t i              = 0;
    uint16_t loopCounter    = 0;
    double sinAngle         = 0;
    double cosAngle         = 0;
    double Bx               = 0;
    double By               = 0;
    uint8_t array_addr_size = 0;

	BOARD_Accel_I2C_Init(&rtosHandle_i2c_accel);
    config.I2C_SendFunc    = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    /* Initialize sensor devices */
    array_addr_size = sizeof(g_sensor_address) / sizeof(g_sensor_address[0]);
    for (i = 0; i < array_addr_size; i++)
    {
        config.slaveAddress = g_sensor_address[i];
        /* Initialize accelerometer sensor */
        result = FXOS_Init(&g_fxosHandle, &config);
        if (result == kStatus_Success)
        {
            break;
        }
    }

    if (result != kStatus_Success)
	{
		PRINTF("\r\nSensor device initialize failed!\r\n");
	}

	/* Get sensor range */
	if (FXOS_ReadReg(&g_fxosHandle, XYZ_DATA_CFG_REG, &g_sensorRange, 1) != kStatus_Success)
	{
		return -1;
	}
	if (g_sensorRange == 0x00)
	{
		g_dataScale = 2U;
	}
	else if (g_sensorRange == 0x01)
	{
		g_dataScale = 4U;
	}
	else if (g_sensorRange == 0x10)
	{
		g_dataScale = 8U;
	}
	else
	{
	}

   /* Infinite loops */
	while(1)
	{
		SampleEventFlag = 0;
		g_Ax_Raw        = 0;
		g_Ay_Raw        = 0;
		g_Az_Raw        = 0;
		g_Ax            = 0;
		g_Ay            = 0;
		g_Az            = 0;
		g_Mx_Raw        = 0;
		g_My_Raw        = 0;
		g_Mz_Raw        = 0;
		g_Mx            = 0;
		g_My            = 0;
		g_Mz            = 0;

		/* Read sensor data */
		Sensor_ReadData(&g_Ax_Raw, &g_Ay_Raw, &g_Az_Raw, &g_Mx_Raw, &g_My_Raw, &g_Mz_Raw);

		/* Average accel value */
		for (i = 1; i < MAX_ACCEL_AVG_COUNT; i++)
		{
			g_Ax_buff[i] = g_Ax_buff[i - 1];
			g_Ay_buff[i] = g_Ay_buff[i - 1];
			g_Az_buff[i] = g_Az_buff[i - 1];
		}

		g_Ax_buff[0] = g_Ax_Raw;
		g_Ay_buff[0] = g_Ay_Raw;
		g_Az_buff[0] = g_Az_Raw;

		for (i = 0; i < MAX_ACCEL_AVG_COUNT; i++)
		{
			g_Ax += (double)g_Ax_buff[i];
			g_Ay += (double)g_Ay_buff[i];
			g_Az += (double)g_Az_buff[i];
		}

		g_Ax /= MAX_ACCEL_AVG_COUNT;
		g_Ay /= MAX_ACCEL_AVG_COUNT;
		g_Az /= MAX_ACCEL_AVG_COUNT;

		if (g_FirstRun)
		{
			g_Mx_LP = g_Mx_Raw;
			g_My_LP = g_My_Raw;
			g_Mz_LP = g_Mz_Raw;
		}

		g_Mx_LP += ((double)g_Mx_Raw - g_Mx_LP) * 0.01;
		g_My_LP += ((double)g_My_Raw - g_My_LP) * 0.01;
		g_Mz_LP += ((double)g_Mz_Raw - g_Mz_LP) * 0.01;

		/* Calculate magnetometer values */
		g_Mx = g_Mx_LP - g_Mx_Offset;
		g_My = g_My_LP - g_My_Offset;
		g_Mz = g_Mz_LP - g_Mz_Offset;

		/* Calculate roll angle g_Roll (-180deg, 180deg) and sin, cos */
		g_Roll   = atan2(g_Ay, g_Az) * RadToDeg;
		sinAngle = sin(g_Roll * DegToRad);
		cosAngle = cos(g_Roll * DegToRad);

		/* De-rotate by roll angle g_Roll */
		By   = g_My * cosAngle - g_Mz * sinAngle;
		g_Mz = g_Mz * cosAngle + g_My * sinAngle;
		g_Az = g_Ay * sinAngle + g_Az * cosAngle;

		/* Calculate pitch angle g_Pitch (-90deg, 90deg) and sin, cos*/
		g_Pitch  = atan2(-g_Ax, g_Az) * RadToDeg;
		sinAngle = sin(g_Pitch * DegToRad);
		cosAngle = cos(g_Pitch * DegToRad);

		/* De-rotate by pitch angle g_Pitch */
		Bx = g_Mx * cosAngle + g_Mz * sinAngle;

		/* Calculate yaw = ecompass angle psi (-180deg, 180deg) */
		g_Yaw = atan2(-By, Bx) * RadToDeg;
		if (g_FirstRun)
		{
			g_Yaw_LP   = g_Yaw;
			g_FirstRun = false;
		}

		g_Yaw_LP += (g_Yaw - g_Yaw_LP) * 0.01;

		if (++loopCounter > 10)
		{
			length = sprintf((char *)sensor_data, "\r\nCompass Angle: %3.1lf", g_Yaw_LP);
			//length = sprintf((char *)sensor_data, "\r\nCompass Angle: %d", temp);
//			temp++;
//			if (temp == 100)
//			{
//				temp = 0;
//			}
			loopCounter = 0;
			size = xMessageBufferSend(xSecondaryToPrimaryMessageBuffer, (void *)&sensor_data[0], (size_t)length, 0);
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	} /* End infinite loops */
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Initialize standard SDK demo application pins */
    BOARD_ConfigMPU();
    BOARD_FXOSInitPins();
    BOARD_BootClockRUN();

    /* Initialize MCMGR before calling its API */
    (void)MCMGR_Init();

    //xTaskCreateStatic(app_task, "APP_TASK", APP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1U, xStack, &xTaskBuffer);

    xTaskCreate(app_task, "APP_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 3, &xTaskBuffer);

    vTaskStartScheduler();

    (void)PRINTF("Failed to start FreeRTOS on core1.\r\n");
    for (;;)
    {
    }
}