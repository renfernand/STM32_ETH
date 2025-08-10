/*
 * debug_api.c
 *
 *  Created on: Mar 5, 2025
 *      Author: Renato Fernandes
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stdbool.h"
#include "debug_api.h"
#include "cmsis_os.h"
#include "sockets.h"
#include "inet.h"

/*
 * Local defines
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define DEBUG_DBG                  1
#define DBG_MAX_REPLY_SIZE       1500   /* max ethernet frame */
#define DBG_MAX_IDLE_TIME        1800   /* seconds */
#define DBG_CONTROL_PORT         23     /* port */
#define DBG_LOG_PORT             6972   /* log port */
#define DBG_DEFAULT_CLUSTER_SIZE	1024
#define DBG_BYTESPERDUMP         16
#define DBG_MAXLINESPERDUMP      500
#define DBG_MAX_CLIENTS          3
#define DBG_CLIENTS_STK_SIZE     10240
#define DBG_DBGTASKNAME_SIZE        20
#define IAC_CMD                     0xFF
#define LOG_DEFAULT true
#define LOG_BROADCAST 0
#define AF_INET         2

#define LOG_MC_ADDR 0xE0000032 /* 224.0.0.50 */
#define LOG_MC_PORT 7580 /* HSE Log */
#define LOG_NUM_ITF 1
/* macros for increasing portability */
#define STRCASECMP(a,b) strcmp((char *) a, (char *) b)
#define VSNPRINTF       vsnprintf
//#define SLEEP           tx_thread_sleep
#define CLOSESOCKET(a)  closesocket(a)  /*NU_Close_Socket(a) */
#define RECV            recv    /*NU_Recv */
//#define INET_NTOA       fns_inet_ntoa
//#define INET_ADDR       fns_inet_addr

/* When set to 1, command will be received and sent by serial. Otherwise,
telnet will be used. Do not use serial modbus at the same time.*/
#define DEBUG_VIA_SERIAL 0
#define USER_AND_PASSWORD_SIZE 8
#define PROMPT_MSG "\r\nLRIA> "
#define WS_DEVICE_NAME  "LRIA "

typedef struct sDbgTaskCtrl
{
  int iState;
  int iSocket;
  void *pMem;
  int arg;
  osThreadFunc_t pThread;
  char pucName[DBG_DBGTASKNAME_SIZE];
} tDbgTaskCtrl;


/*
 * Global vars
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static int iPowerUP; /* From factory init, it will be 0 */
static int iLogEnabled;
static osMutexId_t  sCanPrint;
static int logsock = -1;
static volatile unsigned long _vulLogItf; /* Default interface will be 0 on factory init */
static volatile unsigned long _vulLogType; /* Default type will be multicast */
static struct sockaddr hostlogaddr, peerlogaddr;     /* for logging */

static int iLogStarted;
static char pucUdpLog[DBG_MAX_REPLY_SIZE + 1];
static unsigned char realuser[USER_AND_PASSWORD_SIZE+1]="ufu";  /* user name  */
static unsigned char realpass[USER_AND_PASSWORD_SIZE+1]="ufu";  /* password   */
static tDbgTaskCtrl TaskCtrl[DBG_MAX_CLIENTS];

/* These defines are necessary for supporting dual connection mode (http and telnet) */
static volatile int newline = 1;

#define TURNON_NEWLINE()  newline = 1
#define TURNOFF_NEWLINE() newline = 0
static int prompt = 1;

#define TURNON_PROMPT  prompt = 1
#define TURNOFF_PROMPT prompt = 0
#if (DEBUG_VIA_SERIAL == 1)
  #define DBG_PROMPT()   if(prompt) DbgAddReply(0,"\nDF62>");
#else
#define DBG_PROMPT()    if(prompt) send(peersock,PROMPT_MSG,strlen(PROMPT_MSG),0);
#endif /* (DEBUG_VIA_SERIAL == 1)*/

static void dosetunlockforcemode(int peersock);
static void domonitor(int peersock);
static void dofirmware(int peersock);
static void doreset(int peersock);
static void dohelp(int peersock);
static void dosetip(int peersock, unsigned char *pucVal, unsigned char *pucEth);
static void doifconfig(int peersock, unsigned char *pucVal);
static void parsertask(void *argument);
static int CheckCustCallBacks(int socket, char *cmd, char *arg1, char *arg2, char *arg3);
static int douser(int peersock, char *username);
static int dopass(int peersock, char *passwprovided);
static void dosetmem(int peersock, unsigned char *pucAddr, unsigned char *pucVal);
static void dologging(int peersock, unsigned char *pucOnOff,
                      unsigned char* pucOpt1, unsigned char *pucOpt2);
static void dogetrtc(int peersock);
static void dosetuser(int peersock, unsigned char *pucVal);
static void dosetpass(int peersock, unsigned char *pucVal);
static void doversioninfo(int peersock, unsigned char *pucVal);
//static void dofirmwareinfo(int peersock, int firmware);
int lwipGetIpAddress(unsigned char *pucIpAddress);
int lwipGetDefaultGateway(unsigned char *pucIpGateway);
int lwipGetSubnetMask(unsigned char *pucSubnetMask);

extern ip4_addr_t ipaddr;

//retirei do socket.c
/*
#define IP4ADDR_PORT_TO_SOCKADDR(sin, ipaddr, port) do { \
      (sin)->sin_len = sizeof(struct sockaddr_in); \
      (sin)->sin_family = AF_INET; \
      (sin)->sin_port = lwip_htons((port)); \
      inet_addr_from_ip4addr(&(sin)->sin_addr, ipaddr); \
      memset((sin)->sin_zero, 0, SIN_ZERO_LEN); }while(0)

#define IPADDR_PORT_TO_SOCKADDR(sockaddr, ipaddr, port) IP4ADDR_PORT_TO_SOCKADDR((struct sockaddr_in*)(void*)(sockaddr), ip_2_ip4(ipaddr), port)
*/


enum
{
  DBG_CLIENT_CLEANUP = -2,
  DBG_CLIENT_AVAILABLE = -1,
  DBG_CLIENT_OCCUPIED = 0
};

enum
{
  DBG_NEW_COMMAND_READ = 0,
  DBG_NO_ERROR = 0,
  DBG_END_OF_FILE = -1,
  DBG_READ_ERROR = -2,
  DBG_CONNECTION_TIMEOUT = -3,
  DBG_CANT_CREATE_SOCKET = -4,
  DBG_CONNECTION_ABORTED = -5,
  DBG_LISTEN_FAILURE = -6
};

/**

    Current number of callbacks registered.

*/
volatile static unsigned int uiNumOfCallBacksReg;

#define MAX_NUMBER_OF_DEBUG_CALLBACK 10
tDebugCallbackFunc afCustCallBacks[MAX_NUMBER_OF_DEBUG_CALLBACK];
tHelpCallbackFunc afHelpCallBacks[MAX_NUMBER_OF_DEBUG_CALLBACK];


void UdpLogMsg(char *pucMsg, ...)
{
   //char buf[50];
   //strcpy(buf,pucMsg);
   //strcat(buf,"\n");
   printf(pucMsg);
   printf("\n");
}

/**

  Call all registered callbacks.

  @param socket Socket to send the response
  @param cmd  Command to be executed
  @param arg1 First  string argument (implementation dependent)
  @param arg2 Second string argument (implementation dependent)
  @param arg3 Third  string argument (implementation dependent)

  @retval TRUE  At least one callback was able to treat the command
  @retval FALSE The command was not treated by any callback or there is no
                callbacks registered

*/
static int CheckCustCallBacks(int socket, char *cmd, char *arg1, char *arg2, char *arg3)
{
  int i;
  int j = false;

  ASSERT(uiNumOfCallBacksReg < MAX_NUMBER_OF_DEBUG_CALLBACK);

  for (i = 0; i < uiNumOfCallBacksReg; i++)
  {
    if ((*afCustCallBacks[i]) (socket, cmd, arg1, arg2, arg3) != 0)
      j = true;
  }

  return j;
}


/**
  Initialize the telnet monitor, logger and all necessary structures.
  This function should be called prior any call to

  @param argc    No special meaning. Maybe it will define what interface must be
                 enabled to log, in the future.
 */


void DbgInit(unsigned long argc)
{
  //unsigned long ulIp;
  int iUdpLogSize;

  if( !iPowerUP )
  {
    iLogEnabled = LOG_DEFAULT;
    iPowerUP = true;
  }

  //tx_mutex_create(&sCanPrint, "UDP Log Service", TX_INHERIT);
  sCanPrint = osMutexNew(NULL);
	if (sCanPrint == NULL){
		DBG_LOG(DEBUG_DBG,("erro na criacao no mutex DbgInit!!!"));
	}

  /* tries to create socket file descriptor */
  logsock = socket(AF_INET, SOCK_DGRAM, 0);     /* use UDP */
  if (logsock < 0)
  {
    DBG_LOG(DEBUG_DBG,("DbgInit: File descriptor is invalid !\n"));
  }

  /* Listening */
  DBG_LOG(DEBUG_DBG,("Telnet init ..."));

  //ulIp = _set_log_itf_and_type(_vulLogItf, _vulLogType);
  //peerlogaddr.sa_family = AF_INET;
 //	IPADDR_PORT_TO_SOCKADDR(peerlogaddr.sa_data, 0,DBG_CONTROL_PORT);

//  snprintf(pucUdpLog, DBG_MAX_REPLY_SIZE, "DbgInit: start to logging from %d.%d.%d.%d:%d",
//    (ulIp >> 24) & 255, (ulIp >> 16) & 255, (ulIp >> 8) & 255, ulIp & 255, hostlogaddr.sin_port);

  iUdpLogSize = strlen(pucUdpLog);

  //lwip_sendto(int s, const void *data, size_t size, int flags,const struct sockaddr *to, socklen_t tolen)
  if (sendto(logsock, pucUdpLog, iUdpLogSize, 0, &peerlogaddr, sizeof(struct sockaddr)) < 0)
    iLogStarted = false;
  else
    iLogStarted = true;

  /* default: log is disabled since after download nvram is erased */

}


static void InitCallbacks(void)
{
  int i;

  for (i = 0; i < MAX_NUMBER_OF_DEBUG_CALLBACK; i++)
  {
    afCustCallBacks[i] = (tDebugCallbackFunc) NULL;
    afHelpCallBacks[i] = (tHelpCallbackFunc) NULL;
  }

  uiNumOfCallBacksReg = 0;
}

/*
 * Debug thread entry point
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
void dbgcmd_thread(unsigned long argc)
{
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;
	struct sockaddr_in peeraddr;
	struct sockaddr_in hostaddr;
	int hostsock, peersock;
	socklen_t peeraddrlen = sizeof(struct sockaddr);
	int ret=0;
	volatile int i=0;
    int opt;
    osThreadAttr_t Task_attributes = {
      .name = "DBG_Client",
      .stack_size = 128 * 4,
      .priority = (osPriority_t) osPriorityNormal,
    };

	/* Initialize logging */
	DbgInit(0);

	/* Allow callbacks inclusion */
	InitCallbacks();

	/* initialize control structure */
	for (i = 0; i < DBG_MAX_CLIENTS; i++)
	{
		TaskCtrl[i].iState = DBG_CLIENT_AVAILABLE;
		TaskCtrl[i].pMem = NULL;
		//tx_byte_allocate(&System_Memory, &TaskCtrl[i].pMem, DBG_CLIENTS_STK_SIZE, TX_NO_WAIT);
		TaskCtrl[i].pMem = pvPortMalloc(DBG_CLIENTS_STK_SIZE);
	}

	/* tries to create socket file descriptor */
	hostsock = socket(AF_INET, SOCK_STREAM, 0);   /* uses TCP */
	if (hostsock < 0)
	{
		DBG_LOG(DEBUG_DBG,("File descriptor is invalid "));
	}

	/* host information
	memset((void *) &hostaddr, 0, sizeof(hostaddr));
	*/
	/* does not matter which interface ip.addr = 0.0.0.0 */
	memset(&local_addr, 0, sizeof(struct sockaddr_in));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(DBG_CONTROL_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	/* binding */
    DBG_LOG(DEBUG_DBG, ("Binding listening socket to port %d \r\n", local_addr.sin_port));
    ret = bind(hostsock, (const struct sockaddr *)&local_addr, sizeof(local_addr));
    ASSERT(ret == 0);

	if (ret < 0)
	{
		DBG_LOG(DEBUG_DBG,("bind failure !\r\n"));
		return;
	}
	//ret = hostaddr.sin_addr.s_addr;
	//DBG_LOG(DEBUG_DBG,("Binded to address with %d.%d.%d.%d:%d\n",
	//	(ret >> 24) & 255, (ret >> 16) & 255, (ret >> 8) & 255, ret & 255, ntohs(hostaddr.sin_port)));

	/* Listening */
	DBG_LOG(DEBUG_DBG,("Telnet Listening ...\r\n"));

	/* set socket as blocking */
	opt = 0;
	ret = setsockopt(hostsock, SOL_SOCKET, SO_BROADCAST, (char *) &opt, sizeof(opt));
    //ret = setsockopt(imp->imp_tcp.listen_sock, SOL_SOCKET, SO_KEEPALIVE, (char*)&opt, sizeof(opt));

	if (listen(hostsock, 1) < 0)
	{
		DBG_LOG(DEBUG_DBG,("Listen failure !\r\n"));
	}

	/* main loop */
	while (1)
	{
		/* initialization for each server */

		/* waiting for connections */
		DBG_LOG(DEBUG_DBG,("Telnet Waiting for connections ... \r\n"));

		peersock = accept(hostsock, &peeraddr, &peeraddrlen);

		if (peersock < 0)
		{
			DBG_LOG(DEBUG_DBG,("Accept failure (%d)!\r\n", peersock));
		}
		else
		{
			//DBG_LOG(DEBUG_DBG,("Conected with %s:%d sock %d\n", inet_ntoa(peeraddr.sin_addr),ntohs(peeraddr.sin_port),peersock));
			DBG_LOG(DEBUG_DBG,("Conected with ip %l port %l \r\n", peeraddr.sin_addr,peeraddr.sin_port));

			/* first, execute any possible clean up */

			for (i = 0; i < DBG_MAX_CLIENTS; i++)
			{
				if (TaskCtrl[i].iState == DBG_CLIENT_CLEANUP)
				{
					//tx_thread_delete(&TaskCtrl[i].pThread);
				    osThreadTerminate ((osThreadId_t) TaskCtrl[i].pThread);

					TaskCtrl[i].iState = DBG_CLIENT_AVAILABLE;
					TaskCtrl[i].iSocket = -1;
					DBG_LOG(DEBUG_DBG,("DBG_CLIENT_CLEANUP %d\n", i));
				}
			}

			/* check for available position */

			for (i = 0; i < DBG_MAX_CLIENTS; i++)
			{
				if (TaskCtrl[i].iState == DBG_CLIENT_AVAILABLE)
				{
					TaskCtrl[i].arg = i;
					TaskCtrl[i].iSocket = peersock;

					DBG_LOG(1,("Vou criar parser task [%d] sock=%d \r\n",i,TaskCtrl[i].iSocket));
#if 0
					TaskCtrl[i].pThread = osThreadNew((osThreadFunc_t) parsertask, &TaskCtrl[i].arg, NULL);

					if (TaskCtrl[i].pThread == NULL)
					{
						DBG_LOG(DEBUG_DBG,("Could not create task %d sock \r\n", i, peersock));
						closesocket(peersock);
						break;
					}

					TaskCtrl[i].iState = DBG_CLIENT_OCCUPIED;
					//tx_thread_resume(&TaskCtrl[i].pThread);
					osThreadResume(TaskCtrl[i].pThread);
					DBG_LOG(DEBUG_DBG,("Task created sock %d \r\n", peersock));
					break;
#else
					parsertask(&TaskCtrl[i].arg);

					//TaskCtrl[i].iState = DBG_CLIENT_OCCUPIED;

#endif
				}
			}

			/* not available */
			if (i >= DBG_MAX_CLIENTS)
			{
				closesocket(peersock);
				DBG_LOG(DEBUG_DBG,("No space available. Closing socket %d \r\n",peersock));
			}
		}
	}

}

/*
 * DbgAddReply
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
void DbgAddReply(int peersock, char *line, ...)
{

  char pucBuf[DBG_MAX_REPLY_SIZE + 1];
  va_list ap;
  int iOffs, iLen, iNSent;

  va_start(ap, line);
  VSNPRINTF(pucBuf, DBG_MAX_REPLY_SIZE, line, ap);
  va_end(ap);

  /* standard end of line for telnet protocol */
  if (newline == 1)
  {
    strcat(pucBuf, "\r\n");
  }

  iLen = strlen(pucBuf);

  if (iLen)
  {

    iOffs = 0;
    iNSent = iLen;

    while (iOffs < iLen)
    {
#if (DEBUG_VIA_SERIAL == 1)
      SlipOutput(pucBuf,iLen); /* everything is sent, always */
#else
      iNSent = send(peersock, &pucBuf[iOffs], iLen - iOffs, 0);
#endif /* (DEBUG_VIA_SERIAL == 1)*/
      /* error */
      if (iNSent < 0)
        break;

      iOffs += iNSent;
    }
  }

}



static void parse_args(char *cmd, char **arg1, char **arg2, char **arg3)
{
  int i, n;

  /* 0) remove the backspaces! */
  for (i = 0, n = 0; (n + i) < DBG_MAX_CMD_SIZE; n++)
  {
    if ((cmd[n + i] == '\b'))
    {   /* CRD to use the backspace in commands */
      if (n > 0)
      {
        n = n - 2;
        i += 2;
      }
      else
      {
        n = -1;
        i++;
      }
    }
    else
      cmd[n] = cmd[n + i];
  }

  /* dividing in command and arguments */
  /* 1) find command */
  n = 0;
  while (isalpha(cmd[n]) && n < DBG_MAX_CMD_SIZE)
  {
    cmd[n] = tolower(cmd[n]);
    n++;
  }

  /* 1) remove spaces between command and arguments */
  while (isspace(cmd[n]) && n < DBG_MAX_CMD_SIZE)
    cmd[n++] = '\0';

  /* 2) first argument */
  *arg1 = cmd + n;
  while (isspace(cmd[n]) == 0 && n < DBG_MAX_CMD_SIZE && cmd[n])
    n++;

  while (isspace(cmd[n]) && n < DBG_MAX_CMD_SIZE)
    cmd[n++] = '\0';

  /* 3) second argument */
  *arg2 = cmd + n;
  while (isspace(cmd[n]) == 0 && n < DBG_MAX_CMD_SIZE && cmd[n])
    n++;

  while (isspace(cmd[n]) && n < DBG_MAX_CMD_SIZE)
    cmd[n++] = '\0';

  /* 4) third argument */
  *arg3 = cmd + n;
  while (isspace(cmd[n]) == 0 && n < DBG_MAX_CMD_SIZE && cmd[n])
    n++;

  while (isspace(cmd[n]) && n < DBG_MAX_CMD_SIZE)
    cmd[n++] = '\0';

}


/*
 * parser task
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void parsertask(void *argument)
{
	char *arg1, *arg2, *arg3;
	int i = 0;
	int ncmd;
	char cmd[DBG_MAX_CMD_SIZE+1];
	int passwok = 0;
	int userok = 0;
	int tcb_idx = *((int *)argument);
	int peersock;
	uint8_t buffer[128];

	TURNOFF_NEWLINE();

	if (tcb_idx < DBG_MAX_CLIENTS){
		peersock = TaskCtrl[tcb_idx].iSocket;


		DbgAddReply(peersock, "\r\n LRIA Telnet server.\r\n");

#if 1
		/* logging in */
		for (i = 0; i < 3; i++)
		{
			DbgAddReply(peersock, "\r\nUsername: ");
			DbgGetLine(peersock, cmd);
			DBG_LOG(1,("passe1 \r\n"));
			parse_args(cmd, &arg1, &arg2, &arg3);
			userok = douser(peersock, cmd);
			DBG_LOG(1,("passei user=%d",userok));
			DbgAddReply(peersock, "Password: ");
			DbgGetLine(peersock, cmd);
			parse_args(cmd, &arg1, &arg2, &arg3);
			passwok = dopass(peersock, cmd);

			DBG_LOG(1,("passei pass=%d",userok));

			if ((userok != 1) || (passwok != 1))
			{
				DbgAddReply(peersock, "Login failure.");
				userok = passwok = 0;
			}
			else{
				DbgAddReply(peersock, "Login OK.");
				break;
			}
		}

		TURNON_NEWLINE();

		if (i >= 3)
		{
			DbgAddReply(peersock, "\r\nToo many authentication failures.");
			//SLEEP(MILLISEC_TO_TICKS(3000));
			hw_delay_ms(3000);
		}
		else
		{

			DbgAddReply(peersock, "Type help to see available commands.");
			DBG_PROMPT();

			while (1)
			{

				/* send replies to the client */
				/* get next command */
				ncmd = DbgGetLine(peersock, cmd);
				if (ncmd != DBG_NEW_COMMAND_READ)
				{
					switch (ncmd)
					{
					case DBG_END_OF_FILE:
						DbgAddReply(peersock, "Connection closed by peer.");
						break;
					case DBG_CONNECTION_TIMEOUT:
						DbgAddReply(peersock, "Timeout (%d seconds). Connection closed.", DBG_MAX_IDLE_TIME);
						break;
					case DBG_LISTEN_FAILURE:
					case DBG_READ_ERROR:
					default:
						DbgAddReply(peersock, "Unexpected end of connection.");
						break;
					}
					break;
				}

				parse_args(cmd, &arg1, &arg2, &arg3);

				//DBG_LOG(DEBUG_DBG,("comando = %s, argumento1 = %s, argumento2 = %s\n, argumento3 = %s\n", cmd, arg1, arg2, arg3));

				/* command dissecation */
				if (strlen(cmd) > 13)
				{
					DbgAddReply(peersock, "Unknown command.");
					DBG_PROMPT();
				}
				else if (strlen(arg1) >= (DBG_MAX_CMD_SIZE - strlen(cmd)))
				{ /* ">=" on purpose. */
					DbgAddReply(peersock, "Wrong command");
					DBG_PROMPT();
				}
				else if (!STRCASECMP(cmd, "logging"))
				{
					dologging(peersock, (unsigned char *) arg1, (unsigned char *) arg2, (unsigned char *) arg3);
				}
				else if (!STRCASECMP(cmd, "getrtc"))
				{
					dogetrtc(peersock);
				}
				else if (!STRCASECMP(cmd, "quit") || !STRCASECMP(cmd, "bye") || !STRCASECMP(cmd, "exit"))
				{
					DbgAddReply(peersock, "Have a good day !");
					DBG_PROMPT();
					break;
				}
				else
				{
					/* only if logged in */
					if ((userok == 1) && (passwok == 1))
					{
						/*
						First, check help. The default help and all custom help callback should be
						called. After this check custom commands. In this case, if there is a custom command
						with same name of any standard command, the custom command will overwrite
						the standard command.
						*/
						if (!STRCASECMP(cmd, "?") || !STRCASECMP(cmd, "help"))
						{
							dohelp(peersock);
						}
						else if (CheckCustCallBacks(peersock, cmd, arg1, arg2, arg3) == 0)
						{
							if (!STRCASECMP(cmd, "setuser"))
							{
								dosetuser(peersock, (unsigned char *) arg1);
							}
							else if (!STRCASECMP(cmd, "setpass"))
							{
								dosetpass(peersock, (unsigned char *) arg1);
							}
							else if (!STRCASECMP(cmd, "version"))
							{
								doversioninfo(peersock, (unsigned char *) arg1);
							}
							else if (!STRCASECMP(cmd, "setmem"))
							{
								dosetmem(peersock, (unsigned char *) arg1, (unsigned char *) arg2);
							}
							else if (!STRCASECMP(cmd, "setip") || !STRCASECMP(cmd, "ip"))
							{
								dosetip(peersock, (unsigned char *) arg1, (unsigned char *) arg2);
							}
							else if (!STRCASECMP(cmd, "ifconfig") || !STRCASECMP(cmd, "ipconfig"))
							{
								doifconfig(peersock, (unsigned char *) arg1);
							}
							else
							{
								DbgAddReply(peersock, "Command [%s] unknow or not implemented.", cmd);
								DBG_PROMPT();
							}
						}
						else
						{
							DBG_PROMPT();
						}
					}
					else
					{
						DbgAddReply(peersock, "Not logged in");
						DBG_PROMPT();
					}
				}

			}


		}
		closesocket(peersock);
		TaskCtrl[tcb_idx].iState = DBG_CLIENT_CLEANUP;
		DBG_LOG(DEBUG_DBG,("Cleaning up task %d sock %d", tcb_idx, peersock));
#else
		while (1) {
			int len = recv(peersock, buffer, sizeof(buffer) - 1, 0);

			if (len <= 0) {
			  break;
			}

			buffer[len] = '\0';
			DBG_LOG(1,("Received from client:\r\n%s\r\n", buffer));
			send(peersock, buffer, len, 0);
		}

		closesocket(peersock);
		TaskCtrl[tcb_idx].iState = DBG_CLIENT_AVAILABLE;
		DBG_LOG(DEBUG_DBG,("Cleaning up task %d sock %d", tcb_idx, peersock));
#endif

	}
	else
		DBG_LOG(DEBUG_DBG,("Error... peersock %d doesnt exist!!!! ", peersock));

}

/*
 * DbgGetLine
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
 #define DBG_MAX_SELECT_RETRIES 30
#if 0
int DbgGetLine(int peersock, char *cmd)
{

  int nread, pos;
  int ret;
  unsigned int select_error_retries = DBG_MAX_SELECT_RETRIES;
  fd_set readfds;
  struct timeval tv;

  tv.tv_sec = DBG_MAX_IDLE_TIME;
  tv.tv_usec = 0;
  pos = 0;

  do
  {
    while(select_error_retries)
    {
   	  FD_ZERO(&readfds);
      FD_SET(peersock, &readfds);

      ret = select(FD_SETSIZE, &readfds, NULL, NULL, &tv);
      if(ret > 0)
      {
        select_error_retries = DBG_MAX_SELECT_RETRIES;
        break;
      }
      if(ret == 0)
        return DBG_CONNECTION_TIMEOUT;
      if(ret < 0)
      {
        --select_error_retries;
        //SLEEP(MILLISEC_TO_TICKS(100)); /* max 3s -> max_retries*100ms*/
        hw_delay_ms(1000);
      }
    }
    if(select_error_retries == 0)
      return DBG_LISTEN_FAILURE;

    if (FD_ISSET(peersock, &readfds))
    {
      while (1)
      {
        /* read next valid byte and avoid option negotiation */
        while (1)
        {
          nread = recv(peersock, cmd + pos, 1, 0);

          /* removing IACs - 3 bytes for option negotiation */
          if (cmd[pos] == IAC_CMD)
          {
            nread = recv(peersock, cmd + pos, 1, 0);
            nread = recv(peersock, cmd + pos, 1, 0);
          }
          else
            break;
        }

        if (nread == 0)
        {
          return (DBG_END_OF_FILE);
        }
        else if (nread < 0)
        {
          return (DBG_READ_ERROR);
        }

        if (cmd[pos] == '\n')
        {
          cmd[pos + 1] = '\0';
          return (DBG_NEW_COMMAND_READ);
        }

        if (pos < DBG_MAX_CMD_SIZE - 2)
          pos++;
      }
    }
    else
    {
      return (DBG_CONNECTION_TIMEOUT);
    }
  }
  while (1);

}
#else
int DbgGetLine(int peersock, char *cmd)
{
    int pos = 0;
    char ch;
    int nread;
    fd_set readfds;
    struct timeval tv;
    int ret;

    tv.tv_sec = DBG_MAX_IDLE_TIME;
    tv.tv_usec = 0;

    while (pos < DBG_MAX_CMD_SIZE - 1) {
        FD_ZERO(&readfds);
        FD_SET(peersock, &readfds);
        ret = select(peersock + 1, &readfds, NULL, NULL, &tv);

        if (ret <= 0) {
            return DBG_CONNECTION_TIMEOUT;
        }

        nread = recv(peersock, &ch, 1, 0);

        if (nread <= 0){
            return DBG_READ_ERROR;

        }

        if (ch == IAC_CMD) {
            // Ignora IAC negotiation
            recv(peersock, &ch, 1, 0);
            recv(peersock, &ch, 1, 0);
            continue;
        }

        if (ch == '\r') {
            // opcional: consumir '\n' se vier depois
            recv(peersock, &ch, 1, MSG_DONTWAIT);
            break;
        }

        if (ch == '\n') {
            break;
        }

        cmd[pos++] = ch;
    }

    cmd[pos] = '\0';
    return DBG_NEW_COMMAND_READ;
}
#endif

/*
 * douser
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static int douser(int peersock, char *arg)
{
  int ret;

  if (STRCASECMP(arg, realuser) == 0)
  {
    ret = true;
  }
  else
  {
    ret = false;
  }

  return ret;
}

/*
 * UpScan
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
unsigned UpScan(unsigned ulBegin, unsigned ulEnd)
{
  unsigned *pulTmp, ulFree = 0;
#if 0
  /* scan all task stack looking for free space */
  ulBegin = __next_aligned_addr32(ulBegin);
  ulEnd = __prev_aligned_addr32(ulEnd);

  for (pulTmp = (unsigned *) ulBegin; pulTmp <= (unsigned *) ulEnd; pulTmp++)
    if (*pulTmp == DBG_PERF_PATTERN)
      ulFree += sizeof(unsigned);
#endif
  return ulFree;
}

/*
 * DownScan
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
unsigned DownScan(unsigned ulBegin, unsigned ulEnd)
{
  unsigned char bFound = 0;
  unsigned *pulTmp, ulFree;
#if 0
  /* scan all task stack looking for free space */
  ulBegin = __next_aligned_addr32(ulBegin);
  ulEnd = __prev_aligned_addr32(ulEnd);

  for (pulTmp = (unsigned *) ulEnd; pulTmp > (unsigned *) ulBegin; pulTmp--)
    if (*pulTmp != DBG_PERF_PATTERN)
    {
      bFound = true;
      break;
    }

  if (bFound == TRUE)
    ulFree = (ulEnd + sizeof(unsigned)) - (unsigned) pulTmp;
  else
    ulFree = (ulEnd + sizeof(unsigned)) - ulBegin;
#endif
  return ulFree;
}

/*
 * dopass
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static int dopass(int peersock, char *arg)
{
  int ret;

  if (STRCASECMP(arg, realpass) == 0)
  {
    ret = true;
  }
  else
  {
    ret = false;
  }

  return ret;
}
/*
 * dosetunlockforcemode
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dosetunlockforcemode(int peersock)
{
  //setUnlockForceMode( );
  DbgAddReply(peersock, "Now, your controller will work without battery save.");
  DbgAddReply(peersock, "Replace this controller soon as possible.");
  DbgAddReply(peersock, "REMEMBER: If power down, you will lose your configuration.");
  DBG_PROMPT();
}


/*
 * domonitor
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void domonitor(int peersock)
{
  DbgAddReply(peersock, "Going to monitor mode in 5s. Conection will be lost !");
  DBG_PROMPT();
  //SLEEP(MILLISEC_TO_TICKS(5000));
  hw_delay_ms(5000);
  //HrdwRunMonitor();
}

/*
 * dorun
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dofirmware(int peersock)
{
  DbgAddReply(peersock, "Going to firmware mode in 5s. Conection will be lost !");
  DBG_PROMPT();
  hw_delay_ms(5000);
  //HrdwRunFirmware();
}

/*
 * doreset
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void doreset(int peersock)
{
  DbgAddReply(peersock, "Going to reset in 1s. Connection will be lost !");
  DBG_PROMPT();
  hw_delay_ms(5000);
  //HrdwShutdownNow();
}

/*
 * dosetip
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dosetip(int peersock, unsigned char *pucVal, unsigned char *pucItf)
{
  unsigned char pucAddr[5];
  int iA0 = 0, iA1 = 0, iA2 = 0, iA3 = 0;
  unsigned int uiItf;

  sscanf((char *) pucVal, "%u.%u.%u.%u", &iA0, &iA1, &iA2, &iA3);
  sscanf((char *) pucItf, "%u", &uiItf);

  pucAddr[0] = (unsigned char) iA0;
  pucAddr[1] = (unsigned char) iA1;
  pucAddr[2] = (unsigned char) iA2;
  pucAddr[3] = (unsigned char) iA3;

  //HrdwSetIpAddress(pucAddr, uiItf); /* interface it is used yet */

  DbgAddReply(peersock, "New IP address is %d.%d.%d.%d. Please, reboot.",
    pucAddr[0], pucAddr[1], pucAddr[2], pucAddr[3]);
  DBG_PROMPT();
}

static void dosetmem(int peersock, unsigned char *pucAddr, unsigned char *pucVal)
{

  unsigned long ulAddr;
  unsigned long ulVal;
  char *pucNull;

  ulAddr = strtoul((char *) pucAddr, &pucNull, 16);
  ulVal = strtoul((char *) pucVal, &pucNull, 16);

  *((unsigned long *) ulAddr) = ulVal;

  DbgAddReply(peersock, "Setting contents in address %08X to %08X", ulAddr, ulVal);

  DBG_PROMPT();

}

/*
 * dologging
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

static void dologging(int peersock, unsigned char *pucOnOff,
                      unsigned char* pucOpt1, unsigned char *pucOpt2)
{
  unsigned char* apucOpt[2];
  unsigned long k;
  int bLogTypeChanged = 0;
  int bLogItfChanged = 0;

  if(!iLogStarted)
  {
    DbgAddReply(peersock, "Logging not started due to errors.");
    DBG_PROMPT();
    return;
  }

  if (!STRCASECMP(pucOnOff, "on"))
  {
    iLogEnabled = true;
    DbgAddReply(peersock, "Logging enabled.");
  }
  else if (!STRCASECMP(pucOnOff, "off"))
  {

    iLogEnabled = 0;
    DbgAddReply(peersock, "Logging disabled.");
  }

  apucOpt[0] = pucOpt1;
  apucOpt[1] = pucOpt2;

  for(k = 0; k < 2;k++)
  {
    if(!STRCASECMP(apucOpt[k], "mc") && !bLogTypeChanged)
    {

      _vulLogType = 0;  // LOG_MULTICAST;
      bLogTypeChanged = true;
      DbgAddReply(peersock, "Setting logging to multicast type.");
    }
    else if (!STRCASECMP(apucOpt[k], "bc") && !bLogTypeChanged)
    {
      _vulLogType = LOG_BROADCAST;
      bLogTypeChanged = true;
      DbgAddReply(peersock, "Setting logging to broadcast type.");
    }
    else if(!STRCASECMP(apucOpt[k], "1") && !bLogItfChanged)
    {
      _vulLogItf = 1;
      bLogItfChanged = true;
      DbgAddReply(peersock, "Setting logging to Interface 1.");
    }
    else if (!STRCASECMP(apucOpt[k], "0") && !bLogItfChanged)
    {
      _vulLogItf = 0;
      bLogItfChanged = true;
      DbgAddReply(peersock, "Setting logging to Interface 0.");
    }
    else if (!STRCASECMP(apucOpt[k], "") )
    {
      /* out of loop */
      break;
    }
    else
    {
      DbgAddReply(peersock, "Option %s ignored.", apucOpt[k]);
    }
  }

  DBG_PROMPT();
}
/*
 * dosetuser
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dosetuser(int peersock, unsigned char *pucUsername)
{
#if 0
  if (HrdwSetUsername(pucUsername))
  {
    // get new value from eeprom
    HrdwGetUsername(realuser);
    DbgAddReply(peersock, "New username (%s) set.", realuser);
  }
  else
  {
    DbgAddReply(peersock, "Error when setting the new username.");
  }
#endif

  DBG_PROMPT();
}

/*
 * dosetpass
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dosetpass(int peersock, unsigned char *pucPassword)
{
#if 0
  if (HrdwSetPassword(pucPassword))
  {
    // get new value from eeprom
    HrdwGetPassword(realpass);
    DbgAddReply(peersock, "New password (%s) set.", realpass);
  }
  else
  {
    DbgAddReply(peersock, "Error when setting the new password.");
  }
#endif
  DBG_PROMPT();
}


/**
 * GetCPUFree
 * Generic callback function for getting the current CPUFree
 * @return CPUFree in percentage.
 */
float GetCPUFree (void)
{
#if 0
  float v = 100.0*fPerfAvg/((float)BSP_TICKS_PER_SECOND);
  return v;
#else
  return 0;
#endif
}

/**
 * GetCPULoad
 * Generic callback function for getting the current CPULoad
 * @return CPULoad in percentage.
 */
float GetCPULoad (void)
{
#if 0
  float v = 100 - GetCPUFree();
  return v;
#else
  return 0;
#endif
}

/*
 * doifconfig
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void doifconfig(int peersock, unsigned char *pucVal)
{
  unsigned char pucAddr[5];

  lwipGetIpAddress(pucAddr);
  DbgAddReply(peersock, "IP.....: %03d.%03d.%03d.%03d", pucAddr[0], pucAddr[1], pucAddr[2], pucAddr[3]);

  lwipGetSubnetMask(pucAddr);
  DbgAddReply(peersock, "NETMASK: %03d.%03d.%03d.%03d", pucAddr[0], pucAddr[1], pucAddr[2], pucAddr[3]);

  lwipGetDefaultGateway(pucAddr);
  DbgAddReply(peersock, "GATEWAY: %03d.%03d.%03d.%03d", pucAddr[0], pucAddr[1], pucAddr[2], pucAddr[3]);

  DBG_PROMPT();

}

/*
 * doversioninfo
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

static void doversioninfo(int peersock, unsigned char *pucVal)
{

  DbgAddReply(peersock, "\r\nFirmware version V0.0.1 \r\n================\r\n");

  //dofirmwareinfo(peersock, 1);

  DBG_PROMPT();
}

/*
 * help
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static void dohelp(int peersock)
{
  DbgAddReply(peersock, "--------------------------------------------------------------------");
  DbgAddReply(peersock, "Commands:\n");
  DbgAddReply(peersock, "  reset            : Reset device");
  DbgAddReply(peersock, "  version          : Display version for all modules");
  DbgAddReply(peersock, "  ifconfig         : Show network configuration");
  DbgAddReply(peersock, "  setip ip  n      : Set IP address for interface n. E.g.: setip 10.0.0.1  0");
  DbgAddReply(peersock, "  setnet mask  n   : Set netmask address for interface n");
  DbgAddReply(peersock, "  setgateway [ip]  : Set default gateway address");
  DbgAddReply(peersock, "  setdhcp [on|off] : Configure DHCP");
  DbgAddReply(peersock, "  setuser username : Set a new username for logging in next time");
  DbgAddReply(peersock, "  setpass password : Set a new password for logging in next time");
  DbgAddReply(peersock, "  getrtc           : Get RTC time and date");
  DbgAddReply(peersock, "  quit, bye or exit: Close connection\n");
  DbgAddReply(peersock, "  help or ?        : This command");
  //CheckHelpCallBacks(peersock);
  DbgAddReply(peersock, "--------------------------------------------------------------------");
  DBG_PROMPT();
}
/*
 * dogetrtc_no_prompt
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static void dogetrtc_no_prompt(int peersock)
{
  struct tm tmDate;
  unsigned int ms = 0;

  memset(&tmDate, 0, sizeof(struct tm));

  //GetRtcTmTimeLocalUs(&tmDate,&ms);

  tmDate.tm_mon += 1; /* 0 to 11 according to tm structure */
  tmDate.tm_year += 1900;

  DbgAddReply(peersock, "RTC Data/Time: %04u%02u%02u %02u:%02u:%02u.%d",
    tmDate.tm_year, tmDate.tm_mon, tmDate.tm_mday, tmDate.tm_hour, tmDate.tm_min, tmDate.tm_sec, ms);

}

/*
 * dogetrtc
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static void dogetrtc(int peersock)
{
  dogetrtc_no_prompt(peersock);
  DBG_PROMPT();
}
