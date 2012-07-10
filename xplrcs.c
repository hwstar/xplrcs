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
/* Define these if not defined */

#ifndef VERSION
	#define VERSION "X.X.X"
#endif

#ifndef EMAIL
	#define EMAIL "hwstar@rodgers.sdcoxmail.com"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <ctype.h>
#include <getopt.h>
#include <limits.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <xPL.h>
#include "serio.h"
#include "notify.h"

#define MALLOC_ERROR	malloc_error(__FILE__,__LINE__)

#define SHORT_OPTIONS "a:d:f:hi:l:np:s:v"

#define WS_SIZE 256

#define DEF_COM_PORT		"/dev/ttyS0"
#define DEF_PID_FILE		"/var/run/xplrcs.pid"
#define DEF_ZONE		"thermostat"
#define DEF_INSTANCE_ID		"hvac"

/* 
* Command types
*/

typedef enum {CMDTYPE_NONE=0, CMDTYPE_BASIC, CMDTYPE_RQ_SETPOINT_HEAT, CMDTYPE_RQ_SETPOINT_COOL, 
CMDTYPE_RQ_ZONE, CMDTYPE_DATETIME} CmdType_t;


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
static xPL_MessagePtr xplrcsZoneTriggerMessage = NULL;
static xPL_MessagePtr xplrcsHeatSetPointTriggerMessage = NULL;
static xPL_MessagePtr xplrcsCoolSetPointTriggerMessage = NULL;

static char comPort[WS_SIZE] = DEF_COM_PORT;
static char interface[WS_SIZE] = "";
static char logPath[WS_SIZE] = "";
static char lastLine[WS_SIZE]; 
static char instanceID[WS_SIZE] = DEF_INSTANCE_ID;
static char pidFile[WS_SIZE] = DEF_PID_FILE;

/* Commandline options. */

static struct option longOptions[] = {
  {"address", 1, 0, 'a'},
  {"com-port", 1, 0, 'p'},
  {"config",1, 0, 'c'},
  {"debug", 1, 0, 'd'},
  {"help", 0, 0, 'h'},
  {"interface", 1, 0, 'i'},
  {"log", 1, 0, 'l'},
  {"no-background", 0, 0, 'n'},
  {"pid-file", 0, 0, 'f'},
  {"version", 0, 0, 'v'},
  {0, 0, 0, 0}
};

/* Basic command list */

static const String const basicCommandList[] = {
	"hvac-mode",
	"fan-mode",
	"setpoint",
	"display",
	NULL
};


/* Request command list */

static const String const requestCommandList[] = {
	"gateinfo",
	"zonelist",
	"zoneinfo",
	"setpoint",
	"zone",
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
	"F=0",
	"F=1",
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
	"SPH",
	"SPC",
	NULL
};

/* List of valid display keywords */

static const String const displayList[] = {
	"outside-temp",
	"lock",
	NULL
};



/* 
 * Allocate a memory block and zero it out
 */

static void *mallocz(size_t size)
{
	void *m = malloc(size);
	if(m)
		memset(m, 0, size);
	return m;
}
 
/*
 * Malloc error handler
 */
 
static void malloc_error(String file, int line)
{
	fatal("Out of memory in file %s, at line %d");
}


/* 
 * Get the pid from a pidfile.  Returns the pid or -1 if it couldn't get the
 * pid (either not there, stale, or not accesible).
 */
static pid_t pid_read(char *filename) {
	FILE *file;
	pid_t pid;
	
	/* Get the pid from the file. */
	file=fopen(filename, "r");
	if(!file) {
		return(-1);
	}
	if(fscanf(file, "%d", &pid) != 1) {
		fclose(file);
		return(-1);
	}
	if(fclose(file) != 0) {
		return(-1);
	}
	
	/* Check that a process is running on this pid. */
	if(kill(pid, 0) != 0) {
		
		/* It might just be bad permissions, check to be sure. */
		if(errno == ESRCH) {
			return(-1);
		}
	}
	
	/* Return this pid. */
	return(pid);
}


/* 
 * Write the pid into a pid file.  Returns zero if it worked, non-zero
 * otherwise.
 */
static int pid_write(char *filename, pid_t pid) {
	FILE *file;
	
	/* Create the file. */
	file=fopen(filename, "w");
	if(!file) {
		return -1;
	}
	
	/* Write the pid into the file. */
	(void) fprintf(file, "%d\n", pid);
	if(ferror(file) != 0) {
		(void) fclose(file);
		return -1;
	}
	
	/* Close the file. */
	if(fclose(file) != 0) {
		return -1;
	}
	
	/* We finished ok. */
	return 0;
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
* When the user hits ^C, logically shutdown
* (including telling the network the service is ending)
*/

static void shutdownHandler(int onSignal)
{
	xPL_setServiceEnabled(xplrcsService, FALSE);
	xPL_releaseService(xplrcsService);
	xPL_shutdown();
	/* Unlink the pid file if we can. */
	(void) unlink(pidFile);
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
* Iterate through an arg list and return a val on a key match or else return NULL
*/

static String getVal(String ws, int wslimit, String *argList, String key)
{
	int i;
	String res = NULL;
	String v;

	for(i = 0; argList[i]; i++){
		strncpy(ws, argList[i], wslimit); /* Make a local copy we can modify */
		ws[wslimit - 1] = 0;

		if(!(v = strchr(ws, '='))) /* If there is no =, then bail */
			break;
		*v++ = 0;
		if(!strncmp(ws, key, wslimit)){ /* if there is a match, set res to value and bail */
			res = v;
			break;
		}
	}

	return res;
}



/*
* Queue a command entry
*/

static void queueCommand( String cmd, CmdType_t type )
{
	CmdEntry_t *newCE = mallocz(sizeof(CmdEntry_t));
	/* Did malloc succeed ? */
	if(!newCE)
		MALLOC_ERROR;
	
	/* Dup the command string */
	newCE->cmd = strdup(cmd);

	if(!newCE->cmd) /* Did strdup succeed? */
		MALLOC_ERROR;

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
	
	if((!command) || (!commandList))
		return -1;

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
		if((i >= 0) && (modeList[i])){
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
		if((i >= 0) && (fanModeList[i])){
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
	String setpoint, temperature;
	int cmd;

	
	if(!zone || !ws)
		return res;

	setpoint = xPL_getMessageNamedValue(theMessage, "setpoint");
	temperature = xPL_getMessageNamedValue(theMessage, "temperature");

	if(setpoint && temperature){
		if(!strcmp(setpoint, setPointList[0])){
			res = ws;
			cmd = 0;
		}
		else if(!strcmp(setpoint, setPointList[1])){
			res = ws;
			cmd = 1;
		}
		if(res)
			sprintf(ws + strlen(ws)," %s=%s", setPointCommands[cmd], temperature);
	}
	return res;
}



/*
* Send a display update command
*/

static String doDisplay(String ws, xPL_MessagePtr theMessage, const String const zone)
{
	String val, state, res = NULL;

	if(!ws || !zone || !theMessage)
		return res;

	/* Outside Temperature */
	val = xPL_getMessageNamedValue(theMessage, displayList[0]);
	if(val){
		res = ws;
		sprintf(ws+strlen(ws), " OT=%s", val);
	}

	/* Display lock */
	val = xPL_getMessageNamedValue(theMessage, displayList[1]);
	if(val){
		if((!strcmp(val, "on"))||(!strcmp(val, "yes"))||(!strcmp(val, "1")))
			state = "1";
		else
			state = "0";
		res = ws;
		sprintf(ws+strlen(ws), " DL=%s", state);
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
	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone-list", DEF_ZONE);

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

	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", DEF_ZONE);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "command-list", makeCommaList(ws, basicCommandList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "hvac-mode-list", makeCommaList(ws, modeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "fan-mode-list", makeCommaList(ws, fanModeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "setpoint-list", makeCommaList(ws, setPointList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "display-list", makeCommaList(ws, displayList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "scale", "fahrenheit");

	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.zoneinfo status transmission failed");
}

/*
* Return set point info
*/

static void doGetSetPoint(String ws, xPL_MessagePtr theMessage, const String const zone)
{
	String setpoint;

	if(!zone || !ws || !theMessage)
		return;

	setpoint = xPL_getMessageNamedValue(theMessage, "setpoint");



	if(setpoint){
		sprintf(ws + strlen(ws), " R=4");

		if(!strcmp(setpoint, setPointList[0])){
			queueCommand( ws, CMDTYPE_RQ_SETPOINT_HEAT);
		}
		else if(!strcmp(setpoint, setPointList[1])){
			queueCommand( ws, CMDTYPE_RQ_SETPOINT_COOL);
		}
	}
}

/*
* Send a request for zone information
*/

static void doZoneResponse(String ws, const String const zone)
{

	if(!zone  || !ws)
		return;
	
	sprintf(ws + strlen(ws), " R=1");
	queueCommand( ws, CMDTYPE_RQ_ZONE);
}

/*
* Send the time and date to the thermostat
*/


static void doSetDateTime()
{
	time_t now;
	struct tm ltime;
	char ws[WS_SIZE];
	static int count = 0;

	time(&now);
	localtime_r(&now, &ltime);

	if(count >= 3600){
		count = 0;
		sprintf(ws, "TIME=%02d:%02d:%02d DATE=%02d/%02d/%02d DOW=%d", ltime.tm_hour, ltime.tm_min,
		ltime.tm_sec, ltime.tm_mon + 1, ltime.tm_mday, ltime.tm_year % 100, (ltime.tm_wday + 1));
		debug(DEBUG_ACTION, "Time update command: %s", ws);
		queueCommand( ws, CMDTYPE_DATETIME);
	}
	else
		count++;

}


/*
* Our Listener 
*/



static void xPLListener(xPL_MessagePtr theMessage, xPL_ObjectPtr userValue)
{

	String ws, cmd = NULL;


	

	if(!xPL_isBroadcastMessage(theMessage)){ /* If not a broadcast message */
		if(xPL_MESSAGE_COMMAND == xPL_getMessageType(theMessage)){ /* If the message is a command */
			const String type = xPL_getSchemaType(theMessage);
			const String class = xPL_getSchemaClass(theMessage);
			const String command =  xPL_getMessageNamedValue(theMessage, "command");
			const String request =  xPL_getMessageNamedValue(theMessage, "request");
			const String  zone =  xPL_getMessageNamedValue(theMessage, "zone");
			
			/* Allocate a working string */

			if(!(ws = mallocz(WS_SIZE)))
				MALLOC_ERROR;
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

							case 3: /* display */
								cmd = doDisplay(ws, theMessage, zone);
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
					if(request){
						switch(matchCommand(requestCommandList, request)){

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
								doGetSetPoint(ws, theMessage, zone);
								break;

							case 4: /* zone */
								doZoneResponse(ws, zone);
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
	Bool sendZoneTrigger = FALSE;
	Bool sendHeatSetPointTrigger = FALSE;
	Bool sendCoolSetPointTrigger = FALSE;
	static Bool firstTime = TRUE;
	int curArgc,lastArgc, sendAll = FALSE, i;
	String line;
	String pd,wscur,wslast,arg;
	String queryZone = NULL;
	String val = NULL;
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
			if(!firstTime && strcmp(line, lastLine)){
				/* Clear any old name/values */
				debug(DEBUG_STATUS, "Got updated poll status: %s", line);
				queryZone = DEF_ZONE;

				/* Prepare Zone trigger message */
				xPL_clearMessageNamedValues(xplrcsZoneTriggerMessage);
				xPL_setMessageNamedValue(xplrcsZoneTriggerMessage, "zone", queryZone);

				/* Prepare Setpoint trigger messages */
				xPL_clearMessageNamedValues(xplrcsHeatSetPointTriggerMessage);
				xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "zone", queryZone);
				xPL_clearMessageNamedValues(xplrcsCoolSetPointTriggerMessage);
				xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "zone", queryZone);



				/* Make working strings from the current and list lines */
				wscur = strdup(line);
				wslast = strdup(lastLine);
				if(!wscur || !wslast){
					MALLOC_ERROR;
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
						*pd = 0;
						pd++;
						if(!strcmp(arg, "SPH")){
							sendHeatSetPointTrigger = TRUE;
							xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "setpoint", setPointList[0]);
							xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "temperature", pd );
						}

						else if(!strcmp(arg, "SPC")){
							sendCoolSetPointTrigger = TRUE;
							xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "setpoint", setPointList[1]);
							xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "temperature", pd );
						}
						else if(!strcmp(arg, "FM")){
							sendZoneTrigger = TRUE;
							if(!strcmp(pd, "0"))
								val = fanModeList[0];
							else
								val = fanModeList[1];
							xPL_setMessageNamedValue(xplrcsZoneTriggerMessage, "fan-mode", val);
						}
						else if(!strcmp(arg, "M")){
							sendZoneTrigger = TRUE;
							if(!strcmp(pd, "O"))
								val = modeList[0];
							else if(!strcmp(pd, "H"))
								val = modeList[1];
							else if(!strcmp(pd, "C"))
								val = modeList[2];
							else if(!strcmp(pd, "A"))
								val = modeList[3];
							else
								val = "?";
							xPL_setMessageNamedValue(xplrcsZoneTriggerMessage,"hvac-mode", val);
						}
						else if(!strcmp(arg, "T")){
							sendZoneTrigger = TRUE;
							xPL_setMessageNamedValue(xplrcsZoneTriggerMessage,"temperature", pd);
						}
					} /* End if */


				} /* End for */
				if(sendCoolSetPointTrigger){
					if(!xPL_sendMessage(xplrcsCoolSetPointTriggerMessage))
						debug(DEBUG_UNEXPECTED, "Cool Set point trigger message transmission failed");
				}
				if(sendHeatSetPointTrigger){
					if(!xPL_sendMessage(xplrcsHeatSetPointTriggerMessage))
						debug(DEBUG_UNEXPECTED, "Heat Set point trigger message transmission failed");

				}
				if(sendZoneTrigger){
					if(!xPL_sendMessage(xplrcsZoneTriggerMessage))
						debug(DEBUG_UNEXPECTED, "Zone trigger message transmission failed");
				}


				/* Free working strings */
				free(wscur);
				free(wslast);

				
			}
			/* Copy current string into last string for future comparisons */	
			strncpy(lastLine, line, WS_SIZE);
			lastLine[WS_SIZE - 1] = 0;
			firstTime = FALSE;
		} /* End if(pollPending) */
		else{
			/* It's a response not related to a poll */
			queryZone = DEF_ZONE;

			if(!(wscur = strdup(line)))
				MALLOC_ERROR;

			debug(DEBUG_EXPECTED, "Non-poll response: %s", wscur);
			curArgc = parseRC65Status(wscur, curArgList, 19);
			/* If it was a set point request */
			if((lastCmdType == CMDTYPE_RQ_SETPOINT_HEAT)||(lastCmdType == CMDTYPE_RQ_SETPOINT_COOL)){
				char wc[20];
				debug(DEBUG_EXPECTED,"Setpoint Status requested"); 
				xPL_setSchema(xplrcsStatusMessage, "hvac", "setpoint");
				xPL_clearMessageNamedValues(xplrcsStatusMessage);
				xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", queryZone);
				if(lastCmdType == CMDTYPE_RQ_SETPOINT_HEAT){
					val = getVal(wc, sizeof(wc), curArgList, "SPH");
					if(val)
						xPL_setMessageNamedValue(xplrcsStatusMessage, setPointList[0], val );
				}
				else{
					val = getVal(wc, sizeof(wc), curArgList, "SPC");
					if(val)
						xPL_setMessageNamedValue(xplrcsStatusMessage, setPointList[1], val );
				}
				if(val && !xPL_sendMessage(xplrcsStatusMessage))
					debug(DEBUG_UNEXPECTED, "Setpoint status transmission failed");
			}
			/* If it was a zone info request */
			else if(lastCmdType == CMDTYPE_RQ_ZONE){
				char wc[20];
				debug(DEBUG_EXPECTED,"Zone Status requested"); 
				xPL_setSchema(xplrcsStatusMessage, "hvac", "zone");
				xPL_clearMessageNamedValues(xplrcsStatusMessage);
				for(i = 0 ; curArgList[i]; i++){ /* Iterate through arg list */
					debug(DEBUG_ACTION, "Arg: %s", curArgList[i]);
					if(!strncmp(curArgList[i], "O=", 2))
						xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", DEF_ZONE); /* FIXME */
					else if(!strncmp(curArgList[i], "FM=", 3)){
						val = getVal(wc, sizeof(wc), curArgList, "FM");
						if(val){
							if(!strcmp(val, "0"))
								val = fanModeList[0];
							else
								val = fanModeList[1];
							xPL_setMessageNamedValue(xplrcsStatusMessage,"fan-mode", val);
						}
					}
					else if(!strncmp(curArgList[i], "M=", 2)){
						val = getVal(wc, sizeof(wc), curArgList, "M");
						if(val){
							if(!strcmp(val, "O"))
								val = modeList[0];
							else if(!strcmp(val, "H"))
								val = modeList[1];
							else if(!strcmp(val, "C"))
								val = modeList[2];
							else if(!strcmp(val, "A"))
								val = modeList[3];
							else
								val = "?";
							xPL_setMessageNamedValue(xplrcsStatusMessage,"hvac-mode", val);
						}
					}
					else if(!strncmp(curArgList[i], "T=", 2)){
						val = getVal(wc, sizeof(wc), curArgList, "T");
						if(val){
							xPL_setMessageNamedValue(xplrcsStatusMessage,"temperature", val);
						}

					}

				}
				if(!xPL_sendMessage(xplrcsStatusMessage))
					debug(DEBUG_UNEXPECTED, "Zone info transmission failed");
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

	doSetDateTime();

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
	printf("'%s' is a daemon that bridges the xPL protocol to single zone addressable RCS thermostats\n", progName);
	printf("via an RS-232 or RS-485 interface\n");
	printf("\n");
	printf("Usage: %s [OPTION]...\n", progName);
	printf("\n");
	printf("  -a, --address ADDR      Set the address for the RCS thermostat\n");
	printf("                          (Valid addresses are 0 - 255, %d is the default)\n", xplrcsAddress); 
	printf("  -d, --debug LEVEL       Set the debug level, 0 is off, the\n");
	printf("                          compiled-in default is %d and the max\n", debugLvl);
	printf("                          level allowed is %d\n", DEBUG_MAX);
	printf("  -f, --pid-file PATH     Set new pid file path, default is: %s\n", pidFile);
	printf("  -h, --help              Shows this\n");
	printf("  -i, --interface NAME    Set the broadcast interface (e.g. eth0)\n");
	printf("  -l, --log  PATH         Path name to debug log file when daemonized\n");
	printf("  -n, --no-background     Do not fork into the background (useful for debugging)\n");
	printf("  -p, --com-port PORT     Set the communications port (default is %s)\n", comPort);
	printf("  -s, --instance ID       Set instance id. Default is %s", instanceID);
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

			/* Was it a pid file switch? */
			case 'f':
				strncpy(pidFile, optarg, WS_SIZE - 1);
				logPath[WS_SIZE - 1] = 0;
				debug(DEBUG_ACTION,"New pid file path is: %s", pidFile);
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

			/* Was it an instance ID ? */
			case 's':
				strncpy(instanceID, optarg, WS_SIZE);
				instanceID[WS_SIZE -1] = 0;
				debug(DEBUG_ACTION,"New instance ID is: %s", instanceID);
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

  	/* Make sure we are not already running (.pid file check). */
	if(pid_read(pidFile) != -1) {
		fatal("%s is already running", progName);
	}

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

	/* Create a service and set our application version */
	xplrcsService = xPL_createService("hwstar", "xplrcs", instanceID);
  	xPL_setServiceVersion(xplrcsService, VERSION);

	/*
	* Create a status message object
	*/

  	xplrcsStatusMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_STATUS);
  
	/*
	* Create trigger message objects
	*/

	xplrcsTriggerMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_TRIGGER);

	xplrcsZoneTriggerMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_TRIGGER);
	xPL_setSchema(xplrcsZoneTriggerMessage, "hvac", "zone");

	xplrcsHeatSetPointTriggerMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_TRIGGER);
 	xPL_setSchema(xplrcsHeatSetPointTriggerMessage, "hvac", "setpoint");
	xplrcsCoolSetPointTriggerMessage = xPL_createBroadcastMessage(xplrcsService, xPL_MESSAGE_TRIGGER);
 	xPL_setSchema(xplrcsCoolSetPointTriggerMessage, "hvac", "setpoint");




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

	/* Ask xPL to monitor our serial fd */
	if(!xPL_addIODevice(serioHandler, 1234, serio_fd(serioStuff), TRUE, FALSE, FALSE))
		fatal("Could not register serial I/O fd with xPL");

	/* Add 1 second tick service */
	xPL_addTimeoutHandler(tickHandler, 1, NULL);

  	/* And a listener for all xPL messages */
  	xPL_addMessageListener(xPLListener, NULL);


 	/* Enable the service */
  	xPL_setServiceEnabled(xplrcsService, TRUE);

	if(pid_write(pidFile, getpid()) != 0) {
		debug(DEBUG_UNEXPECTED, "Could not write pid file '%s'.", pidFile);
	}




 	/** Main Loop **/

	for (;;) {
		/* Let XPL run forever */
		xPL_processMessages(-1);
  	}

	exit(1);
}

