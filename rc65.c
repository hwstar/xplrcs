/*
*    rc65 - an RC-65 RS-485 thermostat to xPL bridge
*    Copyright (C) 2012  Stephen A. Rodgers
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
*    xPL bridge to RCS RC65 thermostat
*
*
*/


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctype.h>
#include <getopt.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <xPL.h>
#include "serio.h"
#include "notify.h"

#define SHORT_OPTIONS "a:d:hi:l:np:v"

#define WS_SIZE 256

#define UPDATE_RATE_CFG_NAME	"urate"
#define DEF_UPDATE_RATE		5000
#define DEF_COM_PORT		"/dev/ttyS0"

typedef struct {
	int updateRate;
} config_t;



char *progName;
int debugLvl = 0; 
int noBackground = FALSE;
int rc65Address = 1;

static seriostuff_t *serioStuff = NULL;
static xPL_ServicePtr rc65Service = NULL;
static xPL_MessagePtr rc65StatusMessage = NULL;
static xPL_MessagePtr rc65TriggerMessage = NULL;
static char comPort[WS_SIZE] = DEF_COM_PORT;
static char interface[WS_SIZE] = "";
static char logPath[WS_SIZE] = "";
static char lastLine[WS_SIZE]; 

/* Commandline options. */

static struct option longOptions[] = {
  {"address", 1, 0, 'a'},
  {"com-port", 1, 0, 'p'},
  {"debug", 1, 0, 'd'},
  {"help", 0, 0, 'h'},
  {"interface", 1, 0, 'i'},
  {"log", 1, 0, 'l'},
  {"no-background", 0, 0, 'n'},
  {"version", 0, 0, 'v'},
  {0, 0, 0, 0}
};


static  config_t config = {
	DEF_UPDATE_RATE
};

/*
* Change string to lower case
* Warning: String must be nul terminated.
*/

static String str2Lower(char *q)
{
	char *p;
			
	if(q){
		for (p = q; *p; ++p) *p = tolower(*p);
	}
	return q;
}

/*
* Change string to upper case
* Warning: String must be nul terminated.
*/

static String str2Upper(char *q)
{
	char *p;
			
	if(q){
		for (p = q; *p; ++p) *p = toupper(*p);
	}
	return q;
}





/*
* Set a config integer value
*/

static void setConfigInt(xPL_ServicePtr theService, String theName, int theInt)
{
	char stringRep[18];
	sprintf(stringRep, "%d", theInt);
	xPL_setServiceConfigValue(theService, theName, stringRep);
}

/*
* Get a config integer value
*/

static int getConfigInt(xPL_ServicePtr theService, String theName)
{
	return atoi(xPL_getServiceConfigValue(theService, theName));
}


/*
* Parse config change request 
*/


static void parseConfig(xPL_ServicePtr theService)
{

	int newURate = getConfigInt(theService, UPDATE_RATE_CFG_NAME);


	/* Handle bad configurable (override it) */
	if ((newURate < 250) || newURate > 60000) {
		setConfigInt(theService, UPDATE_RATE_CFG_NAME, config.updateRate );
		return;
	}

	/* Install new update rate */
	config.updateRate = newURate;
}

/*
* Handle config change requests
*/

static void configChangedHandler(xPL_ServicePtr theService, xPL_ObjectPtr userData)
{
  	/* Read config items for service and install */
  	parseConfig(theService);
}

/*
* When the user hits ^C, logically shutdown
* (including telling the network the service is ending)
*/

static void shutdownHandler(int onSignal)
{
	xPL_setServiceEnabled(rc65Service, FALSE);
	xPL_releaseService(rc65Service);
	xPL_shutdown();
	exit(0);
}

/*
* Parse an RC65 status line into its constituant elements
*/

static int parseRC65Status(String ws, String *list, int limit)
{
	int i;
	String strtokrArgSave, arg, argList;
	const String argDelim = " ";

	/* Bail if pointers are NULL or work string has zero length */
	if(!ws  || !list || !strlen(ws))
		return 0;

	for(argList = ws, i = 0; (i < limit) && (arg = strtok_r(argList, argDelim, &strtokrArgSave)); argList = NULL){
		debug(DEBUG_ACTION,"Arg: %s", arg);
		list[i++] = arg;
	}
	list[i] = NULL; /* Terminate end of list */
	return i;
}


/*
* Serial I/O handler (Callback from xPL)
*/

static void serioHandler(int fd, int revents, int userValue)
{
	int curArgc,lastArgc, sendAll = FALSE, i;
	String line;
	String pd,wscur,wslast,arg;
	String curArgList[20];
	String lastArgList[20];

	
	/* Do non-blocking line read */
	if(serio_nb_line_read(serioStuff)){
		/* Got a line */
		line = serio_line(serioStuff);

		/* Compare with last line received */
		if(strcmp(line, lastLine)){
			/* Clear any old name/values */
			xPL_clearMessageNamedValues(rc65TriggerMessage);
			debug(DEBUG_STATUS, "Got %s from serial port", line);

			/* Make working strings from the current and list lines */
			wscur = strdup(line);
			wslast = strdup(lastLine);
			if(!wscur || !wslast){
				fatal("Out of memory in serioHandler");
			}

			/* Parse the current and last lists for comparison */

			curArgc = parseRC65Status(wscur, curArgList, 19);
			lastArgc = parseRC65Status(wslast, lastArgList, 19);

			/* If arg list mismatch, set the sendAll flag */
			if(lastArgc != curArgc){
				sendAll = TRUE;
			}

			/* Iterate through the list and figure out which args to send */
			for(i = 0; i < curArgc; i++){
				arg = curArgList[i];

				/* If sendAll is set or args changed, then add the arg to the list of things to send */
				if(sendAll || strcmp(curArgList[i], lastArgList[i])){
					if(!(pd = strchr(arg, '='))){
						debug(DEBUG_UNEXPECTED, "Parse error in %s", arg);
						sendAll = TRUE;
						continue;
					}
					*pd = 0;
					pd++;

					/* Lower case both key and value */

					str2Lower(arg);
					str2Lower(pd);			
			
					debug(DEBUG_EXPECTED, "Adding: key = %s, value = %s", arg, pd);
					/* Set key and value */
					xPL_setMessageNamedValue(rc65TriggerMessage, arg, pd);
				}

			}
			if(!xPL_sendMessage(rc65TriggerMessage)){
				debug(DEBUG_UNEXPECTED, "Trigger message transmission failed");
			}
			/* Free working strings */
			free(wscur);
			free(wslast);

			/* Copy current string into last string for future comparisons */	
			strncpy(lastLine, line, WS_SIZE);
			lastLine[WS_SIZE - 1] = 0;
		}
	}
}
/*
* Show help
*/

void showHelp(void)
{
	printf("'%s' is a daemon that bridges xPL to rc65 thermostats\n", progName);
	printf("via an RS-232 or RS-485 interface\n");
	printf("\n");
	printf("Usage: %s [OPTION]...\n", progName);
	printf("\n");
	printf("  -a, --address ADDR      Set the address for the RC-65 thermostat\n");
	printf("                          (Valid addresses are 0 - 255, %d is the default)\n", rc65Address); 
	printf("  -d, --debug LEVEL       Set the debug level, 0 is off, the\n");
	printf("                          compiled-in default is %d and the max\n", debugLvl);
	printf("                          level allowed is %d\n", DEBUG_MAX);
	printf("  -h, --help              Shows this\n");
	printf("  -i, --interface NAME    Set the broadcast interface (e.g. eth0)\n");
	printf("  -l, --log  PATH         Path name to log file when daemonized\n");
	printf("  -n, --no-background     Do not fork into the background (useful for debugging)\n");
	printf("  -p, --com-port PORT     Set the communications port (default is %s)\n", comPort);
	printf("  -v, --version           Display program version\n");
	printf("\n");
 	printf("Report bugs to <%s>\n\n", EMAIL);
	return;

}


/*
* main
*/


int main(int argc, char *argv[])
{
	int longindex;
	int optchar;

	/* Set the program name */
	progName=argv[0];

	/* Parse the arguments. */
	while((optchar=getopt_long(argc, argv, SHORT_OPTIONS, longOptions, &longindex)) != EOF) {
		
		/* Handle each argument. */
		switch(optchar) {
			
			/* Was it a long option? */
			case 0:
				
				/* Hrmm, something we don't know about? */
				fatal("Unhandled long getopt option '%s'", longOptions[longindex].name);
			
			/* If it was an error, exit right here. */
			case '?':
				exit(1);
		
			/* Was it a thermostat address? */
			case 'a':
				rc65Address = atoi(optarg);
				if(rc65Address < 0 || rc65Address > 255) {
					fatal("Invalid thermostat address");
				}
				break;

        
			/* Was it a debug level set? */
			case 'd':

				/* Save the value. */
				debugLvl=atoi(optarg);
				if(debugLvl < 0 || debugLvl > DEBUG_MAX) {
					fatal("Invalid debug level");
				}

				break;
			
			/* Was it a help request? */
			case 'h':
				showHelp();
				exit(0);

			/* Specify interface to broadcast on */
			case 'i': 
				strncpy(interface, optarg, WS_SIZE -1);
				interface[WS_SIZE - 1] = 0;
				xPL_setBroadcastInterface(interface);
				break;

			case 'l':
				/* Override log path*/
				strncpy(logPath, optarg, WS_SIZE - 1);
				logPath[WS_SIZE - 1] = 0;
				debug(DEBUG_ACTION,"New log path is: %s",
				logPath);

				break;


			/* Was it a no-backgrounding request? */
			case 'n':

				/* Mark that we shouldn't background. */
				noBackground = TRUE;

				break;
			case 'p':
				/* Override com port*/
				strncpy(comPort, optarg, WS_SIZE - 1);
				comPort[WS_SIZE - 1] = 0;
				debug(DEBUG_ACTION,"New com port is: %s",
				comPort);

				break;


			/* Was it a version request? */
			case 'v':
				printf("Version: %s\n", VERSION);
				exit(0);
	

			
			/* It was something weird.. */
			default:
				fatal("Unhandled getopt return value %d", optchar);
		}
	}

	
	/* If there were any extra arguments, we should complain. */

	if(optind < argc) {
		fatal("Extra argument on commandline, '%s'", argv[optind]);
	}

	/* Turn on library debugging for level 5 */
	if(debugLvl >= 5)
		xPL_setDebugging(TRUE);

  
	/* Fork into the background. */

	if(!noBackground) {
		int retval;
		debug(DEBUG_STATUS, "Forking into background");

    		/* 
		* If debugging is enabled, and we are daemonized, redirect the debug output to a log file if
    		* the path to the logfile is defined
		*/

		if((debugLvl) && (logPath[0]))                          
			notify_logpath(logPath);

		/* Fork and exit the parent */

		if((retval = fork())){
      			if(retval > 0)
				exit(0);  /* Exit parent */
			else
				fatal_with_reason(errno, "parent fork");
    		}



		/*
		* The child creates a new session leader
		* This divorces us from the controlling TTY
		*/

		if(setsid() == -1)
			fatal_with_reason(errno, "creating session leader with setsid");


		/*
		* Fork and exit the session leader, this prohibits
		* reattachment of a controlling TTY.
		*/

		if((retval = fork())){
			if(retval > 0)
        			exit(0); /* exit session leader */
			else
				fatal_with_reason(errno, "session leader fork");
		}

		/* 
		* Change to the root of all file systems to
		* prevent mount/unmount problems.
		*/

		if(chdir("/"))
			fatal_with_reason(errno, "chdir to /");

		/* set the desired umask bits */

		umask(022);
		
		/* Close STDIN, STDOUT, and STDERR */

		close(0);
		close(1);
		close(2);
		} 

	/* Start xPL up */
	if (!xPL_initialize(xPL_getParsedConnectionType())) {
		fatal("Unable to start rc65 xPL lib");
	}

	/* Initialze rc65 service */

	/* Create a configurable service and set our application version */
	rc65Service = xPL_createConfigurableService("hwstar", "rc65", "rc65.xpl");
  	xPL_setServiceVersion(rc65Service, VERSION);

	/* If the configuration was not reloaded, then this is our first time and   */
	/* we need to define what the configurables are and what the default values */
 	/* should be.                                                               */
	if (!xPL_isServiceConfigured(rc65Service)) {
  		/* Define a configurable item and give it a default */
		xPL_addServiceConfigurable(rc65Service, UPDATE_RATE_CFG_NAME, xPL_CONFIG_RECONF, 1);

		setConfigInt(rc65Service, UPDATE_RATE_CFG_NAME, DEF_UPDATE_RATE);
  	}

  	/* Parse the service configurables into a form this program */
  	/* can use (whether we read a config or not)                */
  	parseConfig(rc65Service);

 	/* Add a service change listener we'll use to pick up a new tick rate */
 	xPL_addServiceConfigChangedListener(rc65Service, configChangedHandler, NULL);

	/*
	* Create a status message object
	*/

  	rc65StatusMessage = xPL_createBroadcastMessage(rc65Service, xPL_MESSAGE_STATUS);
  	xPL_setSchema(rc65StatusMessage, "rc65", "status");

	/*
	* Create a trigger message object
	*/

	rc65TriggerMessage = xPL_createBroadcastMessage(rc65Service, xPL_MESSAGE_TRIGGER);
  	xPL_setSchema(rc65TriggerMessage, "rc65", "trigger");


  	/* Install signal traps for proper shutdown */
 	signal(SIGTERM, shutdownHandler);
 	signal(SIGINT, shutdownHandler);

	/* Initialize the COM port */
	
	if(!(serioStuff = serio_open(comPort, 9600)))
		fatal("Could not open com port: %s", comPort);


	/* Flush any partial commands */
	serio_printf(serioStuff, "\r");
	usleep(100000);
	serio_flush_input(serioStuff);


 	/* Enable the service */
  	xPL_setServiceEnabled(rc65Service, TRUE);

	/* Ask xPL to monitor our serial device */
	if(xPL_addIODevice(serioHandler, 1234, serio_fd(serioStuff), TRUE, FALSE, FALSE) == FALSE)
		fatal("Could not register serial I/O fd with xPL");

 	/** Main Loop **/

	for (;;) {
		/* Let XPL run for a while, returning after it hasn't seen any */
		/* activity   */
		xPL_processMessages(config.updateRate);

		/* Process clock tick update checking */
		debug(DEBUG_ACTION, "Polling Status...");
		serio_printf(serioStuff, "A=%d R=1\r", rc65Address);
		
  	}

	exit(1);
}

