/*
 * tcp_server.c
 *
 *  Created on: Aug 10, 2025
 *      Author: Renato Fernandes
 */

#include "debug_api.h"
//#include "sockets.h"

/* USER CODE BEGIN Header_TCPServerTask */
/**
* @brief Function implementing the TCPServer thread.
* @param argument: Not used
* @retval None
     */
/* USER CODE END Header_TCPServerTask */
void TCPServerTask(void *argument)
{
  /* USER CODE BEGIN TCPServerTask */
	int server_socket, client_socket;
	struct sockaddr_in server_addr, client_addr;
	socklen_t client_length;

	uint8_t buffer[128];

	extern struct netif gnetif;
	while (!netif_is_link_up(&gnetif)) {
	    DBG_LOG(1,("Link is not up yet...\r\n"));
	    osDelay(500);
	}

	char ip_str[16];
	ip4addr_ntoa_r(&gnetif.ip_addr, ip_str, sizeof(ip_str));
	DBG_LOG(1,("Got IP: %s\r\n", ip_str));

	server_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket < 0) {
		DBG_LOG(1,("Socket creation failed\r\n"));
		vTaskDelete(NULL);
	}
	DBG_LOG(1,("Socket created\r\n"));

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;

    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // (10 << 24) | (3 << 16) | (195 << 8) | 225;
	server_addr.sin_port = htons(8080);


	if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
	    DBG_LOG(1,("Bind failed\r\n"));
		closesocket(server_socket);
		vTaskDelete(NULL);
	}
	DBG_LOG(1,("Socket successfully binded\r\n"));

	if (listen(server_socket, 1) < 0) {
		DBG_LOG(1,("Listen failed\r\n"));
	}

	DBG_LOG(1,("Listening to sockets...\r\n"));

	socklen_t server_length = sizeof(server_addr);
	if (getsockname(server_socket, (struct sockaddr *)&server_addr, &server_length) == 0) {
	    printf("TCP server listening on port %d\r\n", ntohs(server_addr.sin_port));
	} else {
	    printf("Getsockname failed\r\n");
	}

  /* Infinite loop */
  for(;;)
  {
	  client_length = sizeof(client_addr);
	  DBG_LOG(1,("Accepting TCP connection to client\r\n"));
	  client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &client_length);
	  if (client_socket >= 0) {
		  DBG_LOG(1,("Client connected\r\n"));

		  while (1) {
			  int len = recv(client_socket, buffer, sizeof(buffer) - 1, 0);

			  if (len <= 0) {
				  break;
			  }

			  buffer[len] = '\0';
			  DBG_LOG(1,("Received from client:\r\n%s\r\n", buffer));
			  send(client_socket, buffer, len, 0);

		  }

		  closesocket(client_socket);
		  DBG_LOG(1,("Client disconnected\r\n"));
	  }

    osDelay(1);
  }
  /* USER CODE END TCPServerTask */
}



