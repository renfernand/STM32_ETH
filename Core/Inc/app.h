/*
 * App.h
 *
 *  Created on: Jan 25, 2025
 *      Author: rffernandes
 */

#ifndef APP_H_
#define APP_H_

#define APP_UART_BUFFER_MAX   32

void app_init(void);
void app_maintask(void);
void app_dnptask(void);
//void app_switch_interrupt (void);
void app_tick_1ms(void);
void hw_uart3_interrupt (void);
void hw_tick_1ms();
void hw_delay_ms(uint32_t delayms);
#endif /* APP_H_ */
