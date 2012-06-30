/*
*    xplrcs - an RCS RS-485 thermostat to xPL bridge
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

#define POLL_RATE_CFG_NAME	"prate"
#define DEF_POLL_RATE		5
#define DEF_COM_PORT		"/dev/ttyS0"

/* 
* Command types
*/

typedef enum {CMDTYPE_NONE=0, CMDTYPE_BASIC, CMDTYPE_RQ_SETPOINTS} CmdType_t;


/*
* Command queueing structure
*/

typedef struct cmd_entry CmdEntry_t;

struct cmd_entry {
	String cmd;
	CmdType_t type;
	CmdEntry_t *prev;
	CmdEntry_t *next;
};

	


char *progName;
int debugLvl = 0; 
Bool noBackground = FALSE;
int xplrcsAddress = 1;
int pollRate = 5;
Bool pollPending = FALSE;
CmdType_t lastCmdType = CMDTYPE_NONE;
CmdEntry_t *cmdEntryHead = NULL;
CmdEntry_t *cmdEntryTail = NULL;



static seriostuff_t *serioStuff = NULL;
static xPL_ServicePtr xplrcsService = NULL;
static xPL_MessagePtr xplrcsStatusMessage = NULL;
static xPL_MessagePtr xplrcsTriggerMessage = NULL;
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

/* Basic command list */

static const String const basicCommandList[] = {
	"hvac-mode",
	"fan-mode",
	"setpoint",
	NULL
};


/* Request command list */

static const String const requestCommandList[] = {
	"gateinfo",
	"zonelist",
	"zoneinfo",
	"setpoint",
	NULL
};


/* Heating and cooling modes */

static const String const modeList[] = {
	"off",
	"heat",
	"cool",
	"auto",
	NULL
};

/* Commands for modes */

static const String const modeCommands[] = {
	"M=O",
	"M=H",
	"M=C",
	"M=A",
	NULL
};	

/* Fan modes  */

static const String const fanModeList[] = {
	"auto",
	"on",
	NULL
};

/* Commands for fan modes */

static const String const fanModeCommands[] = {
	"FM=0",
	"FM=1",
	NULL
};

/* List of valid set points */

static const String const setPointList[] = {
	"heating",
	"cooling",
	NULL
};

/* Commands for setpoints */

static const String const setPointCommands[] = {
	"SPH=",
	"SPC=",
	NULL
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

	int newPRate = getConfigInt(theService, POLL_RATE_CFG_NAME);


	/* Handle bad configurable (override it) */
	if ((newPRate < 1) || newPRate > 60) {
		setConfigInt(theService, POLL_RATE_CFG_NAME, pollRate );
		return;
	}

	/* Install new poll rate */
	pollRate = newPRate;
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
	xPL_setServiceEnabled(xplrcsService, FALSE);
	xPL_releaseService(xplrcsService);
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
* Queue a command entry
*/

static void queueCommand( String cmd, CmdType_t type )
{
	CmdEntry_t *newCE = malloc(sizeof(CmdEntry_t));
	/* Did malloc succeed ? */
	if(!newCE)
		fatal("malloc() failed in queueCommand");
	else
		memset(newCE, 0, sizeof(CmdEntry_t)); /* Zero it out */
	/* Dup the command string */
	newCE->cmd = strdup(cmd);

	if(!newCE->cmd) /* Did strdup succeed? */
		fatal("strdup() failed in queueCommand");

	/* Save the type */
	newCE->type = type;

	if(!cmdEntryHead){ /* Empty list */
		cmdEntryHead = cmdEntryTail =  newCE;
	}
	else{ /* List not empty */
		cmdEntryTail->next = newCE;
		newCE->prev = cmdEntryTail;
		cmdEntryTail = newCE;
	}

}

/*
* Dequeue a command entry
*/

static CmdEntry_t *dequeueCommand()
{
	CmdEntry_t *entry;

	if(!cmdEntryHead){
		entry = NULL;
	}
	else if(cmdEntryHead == cmdEntryTail){
		entry = cmdEntryHead;
		entry->prev = entry->next = cmdEntryHead = cmdEntryTail = NULL;
	}
	else{
		entry = cmdEntryHead;
		cmdEntryHead = cmdEntryHead->next;
		entry->prev = entry->next = cmdEntryHead->prev = NULL;	
	}
	return entry;

}

/*
* Free a command entry
*/

static void freeCommand( CmdEntry_t *e)
{
	if(e){
		if(e->cmd)
			free(e->cmd);
		free(e);
	}
}

/*
* Match a command from a NULL-terminated list, return index to list entry
*/

static int matchCommand(const String const *commandList, const String const command)
{
	int i;

	for(i = 0; commandList[i]; i++){
		if(!strcmp(command, commandList[i]))
			break;
	}
	return i;	
}

/*
* Make a comma delimited string from a NULL terminated array of string pointers
*/


static String makeCommaList(String ws, const String const *list)
{
	int i;

	if(!list || !ws)
		return NULL;

	ws[0] = 0;

	for(i = 0; list[i]; i++){
		if(i)
			strcat(ws, ",");
		strcat(ws, list[i]);
	}

	return ws;
}


/*
* Command hander for hvac-mode
*/

static String doHVACMode(String ws, xPL_MessagePtr theMessage, const String const zone)
{
	String res = NULL;
	int i;
	const String const mode = xPL_getMessageNamedValue(theMessage, "mode");

	if(!zone || !ws)
		return res;

	if(mode){
		i = matchCommand(modeList, mode);
		if(modeList[i]){
			strcat(ws, " ");
			res = strcat(ws, modeCommands[i]);
		}
	}
	return res;
}

/*
* Command handler for fan mode
*/

static String doFanMode(String ws, xPL_MessagePtr theMessage, const String const zone)
{
	String res = NULL;
	int i;
	const String const mode = xPL_getMessageNamedValue(theMessage, "mode"); /* Get the mode */

	if(!zone || !ws)
		return res;

	if(mode){
		i = matchCommand(fanModeList, mode);
		if(fanModeList[i]){
			strcat(ws, " ");
			res = strcat(ws, fanModeCommands[i]);
		}
	}
	return res;
}

/*
* Command handler for setpoints
*/

static String doSetSetpoint(String ws, xPL_MessagePtr theMessage, const String const zone)
{
	String res = NULL;
	int i;
	
	if(!zone || !ws)
		return res;


	for(i = 0; setPointList[i] ; i++){

		/* See if setpoint exists in message */
		const String const setpoint = xPL_getMessageNamedValue(theMessage, setPointList[i]);

		if(setpoint){
			/* Setpoint command matched, set the return result */
			res = ws;
			strcat(ws, " ");
			strcat(ws, setPointCommands[i]); /* Cat the command */
			strcat(ws, setpoint);	/* Cat the setpoint */		
		}
	}
	return res;
}




/*
* Return Gateway info 
*/

static void doGateInfo()
{
	xPL_setSchema(xplrcsStatusMessage, "hvac", "gateinfo");

	xPL_clearMessageNamedValues(xplrcsStatusMessage);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "protocol", "RCS");
	xPL_setMessageNamedValue(xplrcsStatusMessage, "description", "xPL to RCS bridge");
	xPL_setMessageNamedValue(xplrcsStatusMessage, "version", VERSION);
	xPL_setMessageNamedValue(xplrcsStatusMessage, "author", "Stephen A. Rodgers");
	xPL_setMessageNamedValue(xplrcsStatusMessage, "info-url", "http://xpl.ohnosec.org");
	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone-count", "1");

	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.gateinfo status transmission failed");
}

/*
* Return Zone List
*/

static void doZoneList()
{
	xPL_setSchema(xplrcsStatusMessage, "hvac", "zonelist");

	xPL_clearMessageNamedValues(xplrcsStatusMessage);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone-count", "1");
	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone-list", "thermostat");

	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.zonelist status transmission failed");
}

/*
* Return Zone Info
*/

static void doZoneInfo(String ws, const String const zone)
{

	/* Bail on NULL pointers */

	if(!zone || !ws)
		return;
	xPL_setSchema(xplrcsStatusMessage, "hvac", "zoneinfo");
	xPL_clearMessageNamedValues(xplrcsStatusMessage);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", "thermostat");

	xPL_setMessageNamedValue(xplrcsStatusMessage, "command-list", makeCommaList(ws, basicCommandList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "hvac-mode-list", makeCommaList(ws, modeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "fan-mode-list", makeCommaList(ws, fanModeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "setpoint-list", makeCommaList(ws, setPointList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "scale", "fahrenheit");

	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.zoneinfo status transmission failed");
}

/*
* Return set point info
*/

static void doGetSetPoint(String ws, const String const zone)
{
	if(!zone || !ws)
		return;

	sprintf(ws+ strlen(ws), " SPH=? SPC=?");

	queueCommand( ws, CMDTYPE_RQ_SETPOINTS);

}


/*
* Our Listener 
*/



static void xPLListener(xPL_MessagePtr theMessage, xPL_ObjectPtr userValue)
{

	String ws, cmd = NULL;


	

	if(!xPL_isBroadcastMessage(theMessage)){ /* If not a broadcast message */
		if(xPL_MESSAGE_COMMAND == xPL_getMessageType(theMessage)){ /* If the message is a command */
			const String const type = xPL_getSchemaType(theMessage);
			const String const class = xPL_getSchemaClass(theMessage);
			const String const command =  xPL_getMessageNamedValue(theMessage, "command");
			const String const zone =  xPL_getMessageNamedValue(theMessage, "zone");
			
			/* Allocate a working string */

			if(!(ws = malloc(WS_SIZE)))
				fatal("Cannot allocate work string in xPLListener");
			ws[0] = 0;
			if(zone){
				// FIXME Zone logic missing
				debug(DEBUG_EXPECTED,"Zone present");
				strcpy(ws, "A=1");
			}


			if(!strcmp(class,"hvac")){
				if(!strcmp(type, "basic")){ /* Basic command schema */
					if(command && zone){
						switch(matchCommand(basicCommandList, command)){
							case 0: /* hvac-mode */
								cmd = doHVACMode(ws, theMessage, zone);
								break;

							case 1: /* fan-mode */
								cmd = doFanMode(ws, theMessage, zone);
								break;

							case 2: /* setpoint */
								cmd = doSetSetpoint(ws, theMessage, zone);
								break;
					
							default:
								break;
						}
					}
					if(cmd){
						queueCommand(cmd, CMDTYPE_BASIC); /* Queue the command */
					}
					else{
						debug(DEBUG_UNEXPECTED, "No command key in message");
					}
				}
				else if(!strcmp(type, "request")){ /* Request command schema */
					if(command){
						switch(matchCommand(requestCommandList, command)){

							case 0: /* gateinfo */
								doGateInfo();
								break;

							case 1: /* zonelist */
								doZoneList();
								break;

							case 2: /* zoneinfo */
								doZoneInfo( ws, zone );
								break;

							case 3: /* setpoint */
								doGetSetPoint(ws, zone);
								break;

							case 4: /* zone */
								break;

							default:
								break;
						}
								
					}

						

					
				


				}
			}
			free(ws);
		}

	}
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
		if(pollPending){
			pollPending = FALSE;
			/* Has to be a response to a poll */
			/* Compare with last line received */
			if(strcmp(line, lastLine)){
				/* Clear any old name/values */
				debug(DEBUG_STATUS, "Got updated poll status: %s", line);

				/* Make working strings from the current and list lines */
				wscur = strdup(line);
				wslast = strdup(lastLine);
				if(!wscur || !wslast){
					fatal("Out of memory in serioHandler(): point 1");
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
							debug(DEBUG_UNEXPECTED, "Parse error in %s point 1", arg);
							sendAll = TRUE;
							continue;
						}
						str2Lower(arg); /* Lower case arg */
						*pd = 0;
						pd++;
						if(strcmp(arg, "a")){ /* Do not send address arg */
							debug(DEBUG_STATUS, "Adding: key = %s, value = %s", arg, pd);
							/* Set key and value */
						}
					}

				}
				/* Free working strings */
				free(wscur);
				free(wslast);

				/* Copy current string into last string for future comparisons */	
				strncpy(lastLine, line, WS_SIZE);
				lastLine[WS_SIZE - 1] = 0;
			}
		} /* End if(pollPending) */
		else{
			/* It's a response not related to a poll */
			if(!(wscur = strdup(line)))
				fatal("Out of memory in serioHandler(): point 2");
			debug(DEBUG_EXPECTED, "Non-poll response: %s", wscur);
			curArgc = parseRC65Status(wscur, curArgList, 19);
			for(i = 0; i < curArgc; i++){
				arg = curArgList[i];
				str2Lower(arg); /* Lower case arg */
				if(!(pd = strchr(arg, '='))){
					debug(DEBUG_UNEXPECTED, "Parse error in %s point 2", arg);
					continue;
				}

				*pd = 0;
				pd++;
				if(strcmp(arg, "a")){  /* Do not send address arg */
					debug(DEBUG_STATUS, "Adding: key = %s, value = %s", arg, pd);
					/* Set key and value */
				}

			}
			free(wscur); /* Free working string */
		}
	} /* End serio_nb_line_read */
}


/*
* Our tick handler. 
* This is used to synchonize the sending of data to the RCS thermostat.
*/

static void tickHandler(int userVal, xPL_ObjectPtr obj)
{
	CmdEntry_t *cmdEntry;
	static short pollCtr = 0;
	static short readySent = FALSE;


	pollCtr++;

	debug(DEBUG_STATUS, "TICK: %d", pollCtr);
	/* Process clock tick update checking */


	if(!readySent){
		readySent = TRUE;
		/* Send trigger message hvac.gateway */
 		xPL_setSchema(xplrcsTriggerMessage, "hvac", "gateway");
		xPL_clearMessageNamedValues(xplrcsTriggerMessage);
		xPL_setMessageNamedValue(xplrcsTriggerMessage, "event", "ready");
		if(!xPL_sendMessage(xplrcsTriggerMessage))
			debug(DEBUG_UNEXPECTED, "Trigger event ready message transmission failed");

	}
	else if((cmdEntry = dequeueCommand())){ /* If command pending */
		/* Uppercase the command string */
		lastCmdType = cmdEntry->type;
		str2Upper(cmdEntry->cmd);
		debug(DEBUG_EXPECTED, "Sending command: %s", cmdEntry->cmd);
		serio_printf(serioStuff, "%s\r", cmdEntry->cmd);
		freeCommand(cmdEntry);
	}	
	else if(pollCtr >= pollRate){ /* Else check poll counter */
		pollCtr = 0;
		debug(DEBUG_ACTION, "Polling Status...");
		serio_printf(serioStuff, "A=%d R=1\r", xplrcsAddress);
		pollPending = TRUE;
	}	
}


/*
* Show help
*/

void showHelp(void)
{
	printf("'%s' is a daemon that bridges xPL to xplrcs thermostats\n", progName);
	printf("via an RS-232 or RS-485 interface\n");
	printf("\n");
	printf("Usage: %s [OPTION]...\n", progName);
	printf("\n");
	printf("  -a, --address ADDR      Set the address for the RC-65 thermostat\n");
	printf("                          (Valid addresses are 0 - 255, %d is the default)\n", xplrcsAddress); 
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
				xplrcsAddress = atoi(optarg);
				if(xplrcsAddress < 0 || xplrcsAddress > 255) {
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
		fatal("Unable to start xPL lib");
	}

	/* Initialze xplrcs service */

	/* Create a configurable service and set our application version */
	xplrcsService = xPL_createConfigurableService("hwstar", "xplrcs", "xplrcs.xpl");
  	xPL_setServiceVersion(xplrcsService, VERSION);

	/* If the configuration was not reloaded, then this is our first time and   */
	/* we need to define what the configurables are and what the default values */
 	/* should be.                                                               */
	if (!xPL_isServiceConfigured(xplrcsService)) {
  		/* Define a configurable item and give it a default */
		xPL_addServiceConfigurable(xplrcsService, POLL_RATE_CFG_NAME, xPL_CONFIG_RECONF, 1);

		setConfigInt(xplrcsService, POLL_RATE_CFG_NAME, DEF_POLL_RATE);
  	}

  	/* Parse the service configurables into a form this program */
  	/* can use (whether we read a config or not)                */
  	parseConfig(xplrcsService);

 	/* Add a service change listener we'll use to pick up a new tick rate */
 	xPL_addServiceConfigChangedListener(xplrcsService, configChangedHandler, NULL);

	/*
	* Create a status message object
	*/

  	xplrcsStatusMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_STATUS);
  	xPL_setSchema(xplrcsStatusMessage, "xplrcs", "status");

	/*
	* Create a trigger message object
	*/

	xplrcsTriggerMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_TRIGGER);
  	xPL_setSchema(xplrcsTriggerMessage, "xplrcs", "trigger");


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
  	xPL_setServiceEnabled(xplrcsService, TRUE);

	/* Ask xPL to monitor our serial device */
	if(xPL_addIODevice(serioHandler, 1234, serio_fd(serioStuff), TRUE, FALSE, FALSE) == FALSE)
		fatal("Could not register serial I/O fd with xPL");

	/* Add 1 second tick service */
	xPL_addTimeoutHandler(tickHandler, 1, NULL);

  	/* And a listener for all xPL messages */
  	xPL_addMessageListener(xPLListener, NULL);



 	/** Main Loop **/

	for (;;) {
		/* Let XPL run forever */
		xPL_processMessages(-1);
  	}

	exit(1);
}

