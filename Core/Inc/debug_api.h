/*
 * debug_api.h
 *
 *  Created on: Mar 1, 2025
 *      Author: Maria Beatriz
 */

#ifndef DEBUG_API_H_
#define DEBUG_API_H_

#include "socket.h"

#define ENABLE_DBG_CMD 1
/**
	Send messages via ethernet in printf style.
	@param line parameters
*/
void UdpLogMsg (char *line, ...);
/**
	Send messages via ethernet in printf style.
  THIS FUNCTION WILL BE REMOVED !
	@param pucMsg parameters
*/
//void UdpLogMsgDebug(char *pucMsg, ...);
#define LogPrintf UdpLogMsgDebug

#define DBG_MAX_CMD_SIZE            20

/**
 DBG_LOG Macro.
 It depends only on your own levels.
 @param g   Guard Value. for non-zero values, it will print it's contains.
 @param msg The message. It must be enclosed in parenthesis (), as it was a
            function call.
*/
#define DBG_LOG(g, msg) \
do{\
  if( (g) ){ \
    UdpLogMsg msg; \
  } \
}while(0)


/**
  Pointer to callback functions to handle custom telnet commands.
*/
typedef int (*tDebugCallbackFunc)(int socket, char *cmd, char *arg1, char *arg2, char *arg3);
/**
  Pointer to callback functions to handle custom help on telnet commands.
*/
typedef void (*tHelpCallbackFunc)(int socket);

/**
	This function is used to send replies through the connected socket.
	@param peersock connected socket
	@param line parameters (in printf style)
*/
void DbgAddReply  (int peersock, char *line, ...);


/**
	Get a line from the connected socket and store it in cmd.
	Maximum command line is given by \ref DBG_MAX_CMD_SIZE -2.
	Buffer is always null terminated.
	@param peersock connected socket
	@param cmd Buffer to store the input
	@retval 0 no error
	@retval negative some error happened.
*/
int DbgGetLine  (int peersock, char *cmd);


/*
  Remember that MyFunc must return 0 in case of failure and 1 in case of success.
  All arguments and the command are char *. The socket connection is a int value.
  This function should be registered via function \ref RegisterDebugCallBack.
*/
#define DECLARE_DBG_CALLBACK(f)  int f (int socket, char *dbg_cmd, char *dbg_arg1, char *dbg_arg2, char *dbg_arg3)

/**
  Macros for acessing callback parameters. They are:
  - socket descritor (int)
  - command name (char *)
  - first argument (char *)
  - second argument (char *)
  - third argument (char *)
  @name Callback parameters.
  @{
*/
#define DBG_GET_SOCKET()     (1) //(socket)
#define DBG_GET_CMD_NAME()     (const char *)(dbg_cmd)
#define DBG_GET_FIRST_PARAM()  (const char *)(dbg_arg1)
#define DBG_GET_SECOND_PARAM() (const char *)(dbg_arg2)
#define DBG_GET_THIRD_PARAM()  (const char *)(dbg_arg3)

#define ASSERT(expr) do { if (!(expr)) { \
		DBG_LOG(1,("ASSERT = %s",#expr)); }} while(0)

#endif /* DEBUG_API_H_ */
