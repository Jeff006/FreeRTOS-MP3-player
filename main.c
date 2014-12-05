/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* ST Std Lib includes. */
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define mainFLASH_TASK_PRIORITY					( tskIDLE_PRIORITY + 1UL )

#define LED_GPIO								GPIOD
#define LED_CLK									RCC_AHB1Periph_GPIOD
#define ledSTACK_SIZE 							configMINIMAL_STACK_SIZE

typedef enum
{
	LED_GREEN_Pin = GPIO_Pin_12,
	LED_ORANGE_Pin = GPIO_Pin_13,
	LED_RED_Pin = GPIO_Pin_14,
	LED_BLUE_Pin = GPIO_Pin_15
} LED_COLOR;

/* Structure used to pass parameters to the LED tasks. */
typedef struct LED_PARAMETERS
{
	LED_COLOR Color;		/*< The output the task should use. */
	TickType_t xFlashRate;	/*< The rate at which the LED should flash. */
} xLEDParameters;

void LED_Init(void);
void LEDToggle(LED_COLOR Color);
void vParTestToggleLED(LED_COLOR Color);
static void vLEDFlashTask( void *pvParameters );
void vStartLEDFlashTasks( unsigned portBASE_TYPE uxPriority );


int main(void)
{
	SystemInit();
	LED_Init();

	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1);

}

/*-----------------------------------------------------------*/

void vStartLEDFlashTasks( unsigned portBASE_TYPE uxPriority )
{
	xLEDParameters *pxLEDParameters;
	const TickType_t xFlashRate = 125;


	/* Create and complete the structure used to pass parameters to the next
		created task. */
	pxLEDParameters = ( xLEDParameters * ) pvPortMalloc( sizeof( xLEDParameters ) );
	pxLEDParameters->Color = LED_GREEN_Pin;
	pxLEDParameters->xFlashRate = ( xFlashRate + ( xFlashRate * ( TickType_t ) 1 ) );
	pxLEDParameters->xFlashRate /= portTICK_PERIOD_MS;

	/* Spawn the task. */
	xTaskCreate( vLEDFlashTask, "LEDx", ledSTACK_SIZE, ( void * ) pxLEDParameters, uxPriority, ( TaskHandle_t * ) NULL );

	/* Create and complete the structure used to pass parameters to the next
	created task. */
	pxLEDParameters = ( xLEDParameters * ) pvPortMalloc( sizeof( xLEDParameters ) );
	pxLEDParameters->Color = LED_ORANGE_Pin;
	pxLEDParameters->xFlashRate = ( xFlashRate + ( xFlashRate * ( TickType_t ) 2 ) );
	pxLEDParameters->xFlashRate /= portTICK_PERIOD_MS;

	xTaskCreate( vLEDFlashTask, "LEDx", ledSTACK_SIZE, ( void * ) pxLEDParameters, uxPriority, ( TaskHandle_t * ) NULL );

	/* Create and complete the structure used to pass parameters to the next
		created task. */
	pxLEDParameters = ( xLEDParameters * ) pvPortMalloc( sizeof( xLEDParameters ) );
	pxLEDParameters->Color = LED_BLUE_Pin;
	pxLEDParameters->xFlashRate = ( xFlashRate + ( xFlashRate * ( TickType_t ) 3 ) );
	pxLEDParameters->xFlashRate /= portTICK_PERIOD_MS;

	xTaskCreate( vLEDFlashTask, "LEDx", ledSTACK_SIZE, ( void * ) pxLEDParameters, uxPriority, ( TaskHandle_t * ) NULL );

	/* Create and complete the structure used to pass parameters to the next
			created task. */
	pxLEDParameters = ( xLEDParameters * ) pvPortMalloc( sizeof( xLEDParameters ) );
	pxLEDParameters->Color = LED_RED_Pin;
	pxLEDParameters->xFlashRate = ( xFlashRate + ( xFlashRate * ( TickType_t ) 4 ) );
	pxLEDParameters->xFlashRate /= portTICK_PERIOD_MS;

	xTaskCreate( vLEDFlashTask, "LEDx", ledSTACK_SIZE, ( void * ) pxLEDParameters, uxPriority, ( TaskHandle_t * ) NULL );

}

/*-----------------------------------------------------------*/

static void vLEDFlashTask( void *pvParameters )
{
	xLEDParameters *pxParameters;

	/* Queue a message for printing to say the task has started. */
	//vPrintDisplayMessage( &pcTaskStartMsg );

	pxParameters = ( xLEDParameters * ) pvParameters;

	for(;;)
	{
		/* Delay for half the flash period then turn the LED on. */
		vTaskDelay( pxParameters->xFlashRate / ( TickType_t ) 2 );
		vParTestToggleLED( pxParameters->Color );

		/* Delay for half the flash period then turn the LED off. */
		vTaskDelay( pxParameters->xFlashRate / ( TickType_t ) 2 );
		vParTestToggleLED( pxParameters->Color );
	}
}

/*-----------------------------------------------------------*/

void vParTestToggleLED(LED_COLOR Color)
{
	taskENTER_CRITICAL();
	{
		LEDToggle(Color);
	}
	taskEXIT_CRITICAL();

}

/*-----------------------------------------------------------*/

void LEDToggle(LED_COLOR Color)
{
	LED_GPIO->ODR ^= Color;
}

/*-----------------------------------------------------------*/

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_GREEN_Pin | LED_ORANGE_Pin | LED_RED_Pin | LED_BLUE_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
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
