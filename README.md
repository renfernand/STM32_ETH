# STM32_ETH
Author: Renato F. Fernandes
date: 10/08/2025
Basic Program with NUCLEO-H7223ZG using FreeRTOS and Ethernet

This program is based in FREERTOS (CMSIS_V2) an LWIP from the CubeIDE platform.
But The wizard from the cubeIDE cannot answer the ethernet with a wrong way (for this board a basic program does not answer a ping). Then This program create a very simple program:
* create a server conection (port 8080) that reproduce a echo frame (send the same that receive). 
* create a telnet conection (port 23) that has a little menu with some basic commands (user e pass=ufu)
* enable de DBG_LOG in the serial (A way to enable e disable the messages to the UART3)

how can I create this (first program):
1) Using de CubeIDE I created a basic program enabling FRERTOS and LWIP.
2) Then I need to enable de cache from FreeRTOS.
 


