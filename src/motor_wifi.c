#include <FreeRTOS.h>
#include <task.h>
#include <platform_stdlib.h>
#include <semphr.h>
#include "motor_wifi.h"
#include "wifi_constants.h"
#include "wifi_structures.h"
#include "lwip_netconf.h"
#include "wifi_conf.h"
#include "netif.h"
#include <lwip/sockets.h>
//#include <osdep_api.h>
#include <osdep_service.h>
//Includes for LED
#include "device.h"
#include "gpio_api.h"
#include "timers.h"

#define GPIO_MOTOR_UP       PC_1
#define GPIO_MOTOR_DOWN       PC_3
#define GPIO_LIMIT PC_4

#define SERVER_PORT     80
#define LISTEN_QLEN     2

#define RBUFSIZE	17

static gpio_t gpio_motor_up;
static gpio_t gpio_motor_down;
static gpio_t gpio_limit;

static SemaphoreHandle_t wifiBufSem = 0;
static char wifiBuf[50];
static char wifiBufSendFlag;

char* ssid;
char* password;


static int tx_exit = 0, rx_exit = 0;

enum windowState {HOLD_WINDOW, OPEN_WINDOW, CLOSE_WINDOW};

typedef enum windowState windowState_e;

static windowState_e window_state = HOLD_WINDOW;
static windowState_e window_target = HOLD_WINDOW;

static void wifi_socket_thread(void *param)
{
	int loops = 0;
	while (1)
	{
		//printf("WiFi loop: %d\n",loops);
		loops++;
		vTaskDelay(10000);
	}
}

void start_sensor_wifi()
{
	gpio_init(&gpio_motor_up, GPIO_MOTOR_UP);
	gpio_dir(&gpio_motor_up, PIN_OUTPUT);    // Direction: Output
	gpio_mode(&gpio_motor_up, PullNone);     // No pull
	gpio_write(&gpio_motor_up, 0);

	gpio_init(&gpio_motor_down, GPIO_MOTOR_DOWN);
	gpio_dir(&gpio_motor_down, PIN_OUTPUT);     // Direction: Output
	gpio_mode(&gpio_motor_down, PullNone);       // No pull
	gpio_write(&gpio_motor_down, 0);

	gpio_init(&gpio_limit, GPIO_LIMIT);
	gpio_dir(&gpio_limit, PIN_INPUT);     // Direction: Input
	gpio_mode(&gpio_limit, PullNone);       // External Pull Down


	vTaskDelay(1000);

	connect_to_network();
	wifiBufSem = xSemaphoreCreateMutex(); // initialize semaphore
	if(xTaskCreate(wifi_socket_thread, ((const char*)"WiFi Socket Thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
			printf("\n\r%s xTaskCreate(wifi_socket_thread) failed", __FUNCTION__);
	if(xTaskCreate(motor_control_thread, ((const char*)"Motor Control Thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
				printf("\n\r%s xTaskCreate(motor_control_thread) failed", __FUNCTION__);
	vTaskDelete(NULL);
}

int connect_to_network()
{
	wifi_on(RTW_MODE_STA); //Resets wifi so we dont get errors
	wifi_off(); //Resets wifi to avoid errors due to warm start
	if(wifi_on(RTW_MODE_STA) < 0){
		printf("\n\r[WIFI] ERROR: wifi_on failed\n");
		return 0;
	}
	// Set the auto-reconnect mode with retry 1 time(limit is 2) and timeout 5 seconds.
	// This command need to be set before invoke wifi_connect() to make reconnection work.
	wifi_config_autoreconnect(1, 525600, 60);
	// Connect to AP with PSK-WPA2-AES.
	ssid = "WLAN_Test";
	password = "testtest";
	if(wifi_connect(ssid, RTW_SECURITY_WPA2_AES_PSK, password, strlen(ssid), strlen(password), -1, NULL) == RTW_SUCCESS)
	{
		printf("WiFi Connected!");
		LwIP_DHCP(0, DHCP_START);
		vTaskDelay(2000);
	}
	else {
		printf("Error connecting to WIFI!");
		return 0;
	}
	return 1;
}

BaseType_t write_wifi_buffer(const char * dataStart){
	BaseType_t retValue = pdFAIL;
	if(wifiBufSem){ // only run with initialized semaphore
		retValue = xSemaphoreTake(wifiBufSem, 0);
		if(retValue == pdPASS){ // copy data and return semaphore if semaphore was taken
			strcpy(wifiBuf, dataStart);
			wifiBufSendFlag = 1;
			xSemaphoreGive(wifiBufSem);
		}
	}
	return retValue;

}


//static _Sema tcp_tx_rx_sema;
static _sema tcp_tx_rx_sema;

static void tx_thread(void *param)
{
	int client_fd = * (int *) param;
	printf("\n%s start\n", __FUNCTION__);
	while(1) {
		int ret = 0;

		//RtlDownSema(&tcp_tx_rx_sema);
		rtw_down_sema(&tcp_tx_rx_sema);
		if(wifiBufSem){ // only run with initialized semaphore
			if(xSemaphoreTake(wifiBufSem, 0) == pdPASS){ // send data and return semaphore if semaphore was taken
				if(wifiBufSendFlag){
					ret = send(client_fd, wifiBuf, strlen(wifiBuf), 0);
				}
				xSemaphoreGive(wifiBufSem);
			}
		}

		//RtlUpSema(&tcp_tx_rx_sema);
		rtw_up_sema(&tcp_tx_rx_sema);
		if(ret <= 0){
			goto exit;
		}
		vTaskDelay(1000);
	}

exit:
	printf("\n%s exit\n", __FUNCTION__);
	tx_exit = 1;
	vTaskDelete(NULL);
}

static void rx_thread(void *param)
{
	int client_fd = * (int *) param;
	unsigned char buffer[RBUFSIZE] = {'\0'};
	printf("\n%s start\n", __FUNCTION__);
	while(1) {
		int ret = 0, sock_err = 0;
		size_t err_len = sizeof(sock_err);
		rtw_down_sema(&tcp_tx_rx_sema);
		ret = recv(client_fd, buffer, 16, MSG_DONTWAIT);
		getsockopt(client_fd, SOL_SOCKET, SO_ERROR, &sock_err, &err_len);
		rtw_up_sema(&tcp_tx_rx_sema);
		if (buffer[0])
		{
			if (buffer[0]== 'c')
			{
				window_target = CLOSE_WINDOW;
			}
			else if (buffer[0] == 'o')
			{
				window_target = OPEN_WINDOW;
			}
			printf("Received: %s\n", buffer);
			memset(&buffer[0], 0, RBUFSIZE);
		}
		vTaskDelay(1000);
		// ret == -1 and socket error == EAGAIN when no data received for nonblocking
		if((ret == -1) && (sock_err == EAGAIN))
			continue;
		else if(ret <= 0)
			goto exit;
	}

exit:
	printf("\n%s exit\n", __FUNCTION__);
	rx_exit = 1;
	vTaskDelete(NULL);
}

static void example_socket_tcp_trx_thread(void *param)
{
	int server_fd = -1, client_fd = -1;
	struct sockaddr_in server_addr, client_addr;
	size_t client_addr_size;

	// Delay to wait for IP by DHCP
	vTaskDelay(10000);
	//printf("\nExample: socket tx/rx 1\n");

	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY;
	//printf("\nExample: socket tx/rx place 2\n");

	if(bind(server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) != 0) {
		printf("ERROR: bind\n");
		goto exit;
	}

	if(listen(server_fd, LISTEN_QLEN) != 0) {
		printf("ERROR: listen\n");
		goto exit;
	}
	while(1) {
		client_addr_size = sizeof(client_addr);
		client_fd = accept(server_fd, (struct sockaddr *) &client_addr, &client_addr_size);
		if(client_fd >= 0) {
			printf("EXAMPLE SOCKET OUTER LOOP");
			tx_exit = 1;
			rtw_init_sema(&tcp_tx_rx_sema, 1);

			if(xTaskCreate(tx_thread, ((const char*)"tx_thread"), 512, &client_fd, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
				printf("\n\r%s xTaskCreate(tx_thread) failed", __FUNCTION__);
			else
				tx_exit = 0;
			vTaskDelay(10);
			rx_exit = 1;
			if(xTaskCreate(rx_thread, ((const char*)"rx_thread"), 512, &client_fd, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
				printf("\n\r%s xTaskCreate(rx_thread) failed", __FUNCTION__);
			else
				rx_exit = 0;
			vTaskDelay(10);
			while(1) {
				if(tx_exit && rx_exit) {
					close(client_fd);
					break;
				}
				else
					vTaskDelay(1000);
			}
			printf("\nExample: socket tx/rx place loop\n");
			rtw_free_sema(&tcp_tx_rx_sema);
		}
	}

exit:
	close(server_fd);
	vTaskDelete(NULL);
}


void example_socket_tcp_trx_1(void)
{
	if(xTaskCreate(example_socket_tcp_trx_thread, ((const char*)"example_socket_tcp_trx_thread"), 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
		printf("\n\r%s xTaskCreate(example_socket_tcp_trx_thread) failed", __FUNCTION__);
}


void stop_motor(TimerHandle_t xTimer){
	do {
		gpio_write(&gpio_motor_down, 0);
		gpio_write(&gpio_motor_up, 0);
		} while (gpio_read(&gpio_motor_down) || gpio_read(&gpio_motor_up));
	window_state = HOLD_WINDOW;
}


void motor_control_thread()
{
	TimerHandle_t stopTimer = xTimerCreate("Stop Timer", pdMS_TO_TICKS(0), pdFALSE, 0, stop_motor);
	while(1){
		while(xTimerIsTimerActive(stopTimer) != pdFALSE){}// wait for timer to stop
		if (window_state!=window_target)
		{
			do {
				gpio_write(&gpio_motor_down, 0);
				gpio_write(&gpio_motor_up, 0);
			} while (gpio_read(&gpio_motor_down) || gpio_read(&gpio_motor_up)); // wait for gpio pins to turn off
			switch(window_target){
				case CLOSE_WINDOW:
					gpio_write(&gpio_motor_down, 1);
					xTimerStart(stopTimer, pdMS_TO_TICKS(30000));
					break;
				case OPEN_WINDOW:
					gpio_write(&gpio_motor_up, 1);
					xTimerStart(stopTimer, pdMS_TO_TICKS(25000));
					break;
				case HOLD_WINDOW:
					break;
			}
			window_state = window_target;

		}
		vTaskDelay(200);
	}

}
