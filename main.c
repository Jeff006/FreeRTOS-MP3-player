/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "print.h"

/* ST Std Lib includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"

#define mainFLASH_TASK_PRIORITY					( tskIDLE_PRIORITY + 1UL )
#define LED_STACK_SIZE 							configMINIMAL_STACK_SIZE
#define LCD_STACK_SIZE 							configMINIMAL_STACK_SIZE

/* Structure used to pass parameters to the LED tasks. */
typedef struct LED_PARAMETERS
{
	Led_TypeDef Led;		/*< The output the task should use. */
	TickType_t xFlashRate;	/*< The rate at which the LED should flash. */
} xLEDParameters;

void vToggleLED(Led_TypeDef Led);
static void vLEDFlashTask( void *pvParameters );
void vStartLEDFlashTasks( unsigned portBASE_TYPE uxPriority );
void vStartLCDPrintTasks( unsigned portBASE_TYPE uxPriority );
static void vLCDPrintTask( void *pvParameters );
void vPrintLCD(const char *Message);

/* String to print if USE_STDIO is defined. */
const char * const pcTaskStartMsg = "LED flash task started.";


int main(void)
{
	SystemInit();

	STM_EVAL_LEDInit(LED4);

	STM32f4_Discovery_LCD_Init();
	LCD_DisplayStringLine(LINE(1), (uint8_t *)"FreeRTOS Initialization.");

	vPrintInitialise();
	vStartLCDPrintTasks( mainFLASH_TASK_PRIORITY );
	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1);

}

/*-----------------------------------------------------------*/

void vStartLCDPrintTasks( unsigned portBASE_TYPE uxPriority )
{
	/* Spawn the task. */
	xTaskCreate( vLCDPrintTask, "LCDPrint", LCD_STACK_SIZE, ( void * ) NULL, uxPriority, ( TaskHandle_t * ) NULL );

}

/*-----------------------------------------------------------*/

static void vLCDPrintTask( void *pvParameters )
{
	const char *Message;

	Message = pcPrintGetNextMessage( (TickType_t) 125 );

	for(;;)
	{
		vPrintLCD(Message);
	}
}

/*-----------------------------------------------------------*/

void vPrintLCD(const char *Message)
{
	taskENTER_CRITICAL();
	{
		LCD_DisplayStringLine(LINE(2), (uint8_t *)Message);
	}
	taskEXIT_CRITICAL();

}
/*-----------------------------------------------------------*/


void vStartLEDFlashTasks( unsigned portBASE_TYPE uxPriority )
{
	xLEDParameters *pxLEDParameters;
	const TickType_t xFlashRate = 125;

	/* Create and complete the structure used to pass parameters to the next
			created task. */
	pxLEDParameters = ( xLEDParameters * ) pvPortMalloc( sizeof( xLEDParameters ) );
	pxLEDParameters->Led = LED4;
	pxLEDParameters->xFlashRate = xFlashRate;
	pxLEDParameters->xFlashRate /= portTICK_PERIOD_MS;

	/* Spawn the task. */
	xTaskCreate( vLEDFlashTask, "LED3", LED_STACK_SIZE, ( void * ) pxLEDParameters, uxPriority, ( TaskHandle_t * ) NULL );

}

/*-----------------------------------------------------------*/

static void vLEDFlashTask( void *pvParameters )
{
	xLEDParameters *pxParameters;

	/* Queue a message for printing to say the task has started. */
	vPrintDisplayMessage( &pcTaskStartMsg );

	pxParameters = ( xLEDParameters * ) pvParameters;

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelay( pxParameters->xFlashRate / ( TickType_t ) 2 );
		vToggleLED( pxParameters->Led );

		/* Delay for half the flash period then turn the LED off. */
		vTaskDelay( pxParameters->xFlashRate / ( TickType_t ) 2 );
		vToggleLED( pxParameters->Led );
	}
}

/*-----------------------------------------------------------*/

void vToggleLED(Led_TypeDef Led)
{
	taskENTER_CRITICAL();
	{
		STM_EVAL_LEDToggle(Led);
	}
	taskEXIT_CRITICAL();

}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
