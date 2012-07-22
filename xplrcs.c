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
#include "types.h"
#include "serio.h"
#include "notify.h"
#include "confread.h"

#define MALLOC_ERROR	malloc_error(__FILE__,__LINE__)

#define SHORT_OPTIONS "c:d:f:hi:l:np:r:s:v"

#define WS_SIZE 256
#define MAX_ZONES 10
#define	POLL_RATE_MIN 2
#define	POLL_RATE_MAX 180
#define SERIAL_RETRY_TIME 5

#define DEF_COM_PORT		"/dev/ttyS0"
#define DEF_PID_FILE		"/var/run/xplrcs.pid"
#define DEF_CONFIG_FILE		"/etc/xplrcs.conf"
#define DEF_INSTANCE_ID		"hvac"
#define	DEF_UNITS			"celsius"

/* 
* Command types
*/

typedef enum {CMDTYPE_NONE=0, CMDTYPE_BASIC, CMDTYPE_RQ_SETPOINT_HEAT, CMDTYPE_RQ_SETPOINT_COOL, 
CMDTYPE_RQ_ZONE, CMDTYPE_DATETIME, CMDTYPE_RQ_HEATTIME, CMDTYPE_RQ_COOLTIME} CmdType_t;


/*
 * Zone entry structure
 */
 
typedef struct zone_entry ZoneEntry_t;
typedef ZoneEntry_t * ZoneEntryPtr_t;

 
struct zone_entry {
	String name;
	unsigned address;
	Bool first_time;
	String last_poll;
	ZoneEntryPtr_t prev;
	ZoneEntryPtr_t next;
}; 

/*
* Command queueing structure
*/

typedef struct cmd_entry CmdEntry_t;
typedef CmdEntry_t * CmdEntryPtr_t;

struct cmd_entry {
	String cmd;
	CmdType_t type;
	Bool sent;
	ZoneEntryPtr_t ze;
	CmdEntryPtr_t prev;
	CmdEntryPtr_t next;
};

/*
 * Command line override bits
 */
 
typedef struct cloverrides {
	unsigned pid_file : 1;
	unsigned com_port : 1;
	unsigned instance_id : 1;
	unsigned log_path : 1;
	unsigned interface : 1;
	unsigned poll_rate : 1;
} clOverride_t;


char *progName;
int debugLvl = 0; 

static Bool noBackground = FALSE;
static unsigned pollRate = 5;
static unsigned numZones = 0;
static unsigned serialRetryTimer = 0;
static clOverride_t clOverride = {0,0,0,0,0,0};
static CmdEntryPtr_t cmdEntryHead = NULL;
static CmdEntryPtr_t cmdEntryTail = NULL;
static ZoneEntryPtr_t zoneEntryHead = NULL;
static ZoneEntryPtr_t zoneEntryTail = NULL;
static ZoneEntryPtr_t pollPending = NULL;

static serioStuffPtr_t serioStuff = NULL;
static xPL_ServicePtr xplrcsService = NULL;
static xPL_MessagePtr xplrcsStatusMessage = NULL;
static xPL_MessagePtr xplrcsTriggerMessage = NULL;
static xPL_MessagePtr xplrcsZoneTriggerMessage = NULL;
static xPL_MessagePtr xplrcsHeatSetPointTriggerMessage = NULL;
static xPL_MessagePtr xplrcsCoolSetPointTriggerMessage = NULL;
static ConfigEntryPtr_t	configEntry = NULL;

static char configFile[WS_SIZE] = DEF_CONFIG_FILE;
static char comPort[WS_SIZE] = DEF_COM_PORT;
static char interface[20] = "";
static char logPath[WS_SIZE] = "";
static char instanceID[128] = DEF_INSTANCE_ID;
static char pidFile[WS_SIZE] = DEF_PID_FILE;
static char units[20] = DEF_UNITS;

/* Commandline options. */

static struct option longOptions[] = {
	{"config-file", 1, 0, 'c'},
	{"com-port", 1, 0, 'p'},
	{"config",1, 0, 'c'},
	{"debug", 1, 0, 'd'},
	{"help", 0, 0, 'h'},
	{"interface", 1, 0, 'i'},	
	{"log", 1, 0, 'l'},
	{"no-background", 0, 0, 'n'},
	{"pid-file", 0, 0, 'f'},
	{"poll-rate", 1, 0, 'r'},
	{"version", 0, 0, 'v'},
	{0, 0, 0, 0}
};

/* Basic command list */

static const String basicCommandList[] = {
	"hvac-mode",
	"fan-mode",
	"setpoint",
	"display",
	NULL
};


/* Request command list */

static const String requestCommandList[] = {
	"gateinfo",
	"zonelist",
	"zoneinfo",
	"setpoint",
	"zone",
	"runtime",
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

static const String modeCommands[] = {
	"M=O",
	"M=H",
	"M=C",
	"M=A",
	NULL
};	

/* Fan modes  */

static const String fanModeList[] = {
	"auto",
	"on",
	NULL
};

/* Commands for fan modes */

static const String fanModeCommands[] = {
	"F=0",
	"F=1",
	NULL
};

/* List of valid set points */

static const String setPointList[] = {
	"heating",
	"cooling",
	NULL
};

/* Commands for setpoints */

static const String setPointCommands[] = {
	"SPH",
	"SPC",
	NULL
};

/* List of valid display keywords */

static const String displayList[] = {
	"outsidetemp",
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
* Convert a string to an unsigned int with bounds checking
*/

static Bool str2uns(String s, unsigned *num, unsigned min, unsigned max)
{
		long val;
		if((!num) || (!s)){
			debug(DEBUG_UNEXPECTED, "NULL pointer passed to str2uns");
			return FALSE;
		}
		val = strtol(s, NULL, 0);
		if((val < min) || (val > max))
			return FALSE;
		*num = (unsigned) val;
		return TRUE;
}



/*
* Duplicate or split a string. 
*
* The string is copied, and the sep characters are replaced with nul's and a list pointers
* is built. 
* 
* If no sep characters are found, the string is just duped and returned.
*
* This function returns the number of arguments found.
*
* When the caller is finished with the list and the return value is non-zero he should free() the first entry.
* 
*
*/

static int dupOrSplitString(const String src, String *list, char sep, int limit)
{
		String p, q, srcCopy;
		int i;
		

		if((!src) || (!list) || (!limit))
			return 0;

		if(!(srcCopy = strdup(src)))
			MALLOC_ERROR;

		for(i = 0, q = srcCopy; (i < limit) && (p = strchr(q, sep)); i++, q = p + 1){
			*p = 0;
			list[i] = q;
		
		}

		list[i] = q;
		i++;

		return i;
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
	
	if(!q)
		return NULL;
			
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
		confreadStringCopy(ws, argList[i], wslimit); /* Make a local copy we can modify */

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

static void queueCommand(ZoneEntryPtr_t ze, String cmd, CmdType_t type )
{
	CmdEntryPtr_t newCE = mallocz(sizeof(CmdEntry_t));
	
	/* Did malloc succeed ? */
	if(!newCE)
		MALLOC_ERROR;
	
	/* Dup the command string */
	newCE->cmd = strdup(cmd);

	if(!newCE->cmd) /* Did strdup succeed? */
		MALLOC_ERROR;
		
	/* Save the optional zone entry in the queued command */
	newCE->ze = ze;

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

static CmdEntryPtr_t dequeueCommand()
{
	CmdEntryPtr_t entry;

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

static void freeCommand( CmdEntryPtr_t e)
{
	if(e){
		if(e->cmd)
			free(e->cmd);
		free(e);
	}
}

/*
 * De-queue and free current command
 */
  
static void dequeueAndFreeCommand()
{
		if(cmdEntryTail){
			CmdEntryPtr_t ctf = dequeueCommand();
			freeCommand(ctf);
		}
}


/*
* Match a command from a NULL-terminated list, return index to list entry
*/

static int matchCommand(const String *commandList, const String command)
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


static String makeCommaList(String ws, const String *list)
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

static String doHVACMode(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	String res = NULL;
	int i;
	const String const mode = xPL_getMessageNamedValue(theMessage, "mode");

	if(!ze || !ws)
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

static String doFanMode(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	String res = NULL;
	int i;
	const String const mode = xPL_getMessageNamedValue(theMessage, "mode"); /* Get the mode */

	if(!ze || !ws)
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

static String doSetSetpoint(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	String res = NULL;
	String setpoint, temperature;
	int cmd;

	
	if(!ze || !ws)
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

static String doDisplay(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	String val, state, res = NULL;

	if(!ws || !ze || !theMessage)
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
 * Build run time command
 */
 
static String buildRTCmd(String ws, char rq, String setQuery)
{
	if(!ws)
		return NULL;
		
	if(!setQuery)
		setQuery = "?";
	
	sprintf(ws+strlen(ws), " RT%c=%s", rq, setQuery);

	return ws;
}

/*
 * Do get runtime command
 */

static void doGetRT(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	char rq;
	
	String state = xPL_getMessageNamedValue(theMessage, "state");
	
	if(!ws || !theMessage || !ze || !state) /* Must have valid pointers */
		return;
		
	if(!strcmp(state, setPointList[0])){ /* heating */
		rq = 'H';
	}
	else if(!strcmp(state, setPointList[1])){ /* cooling */
		rq = 'C';
	}
	else
		return;
	if(buildRTCmd(ws, rq, NULL))	
		queueCommand(ze, ws, (rq == 'H') ? CMDTYPE_RQ_HEATTIME : CMDTYPE_RQ_COOLTIME); /* Queue the command */
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

static void doZoneList(String ws)
{
	int i;
	ZoneEntryPtr_t ze;
	
	if(!ws)
		return;
	
	xPL_setSchema(xplrcsStatusMessage, "hvac", "zonelist");

	xPL_clearMessageNamedValues(xplrcsStatusMessage);
	
	/* Add the zone count */
	snprintf(ws, 20, "%d", numZones);
	xPL_addMessageNamedValue(xplrcsStatusMessage, "zone-count", ws);
	
	/* Add zone names, one per key/value */
	for(ze = zoneEntryHead, i = 0; ze; ze = ze->next, i++)
		xPL_addMessageNamedValue(xplrcsStatusMessage, "zone-list", ze->name);
	
	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.zonelist status transmission failed");
}

/*
* Return Zone Info
*/

static void doZoneInfo(String ws, ZoneEntryPtr_t ze)
{

	/* Bail on NULL pointers */

	if(!ze || !ws)
		return;
		
	xPL_setSchema(xplrcsStatusMessage, "hvac", "zoneinfo");
	xPL_clearMessageNamedValues(xplrcsStatusMessage);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", ze->name);

	xPL_setMessageNamedValue(xplrcsStatusMessage, "command-list", makeCommaList(ws, basicCommandList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "hvac-mode-list", makeCommaList(ws, modeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "fan-mode-list", makeCommaList(ws, fanModeList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "setpoint-list", makeCommaList(ws, setPointList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "display-list", makeCommaList(ws, displayList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "hvac-state-list", makeCommaList(ws, setPointList));
	xPL_setMessageNamedValue(xplrcsStatusMessage, "units", units); 

	if(!xPL_sendMessage(xplrcsStatusMessage))
		debug(DEBUG_UNEXPECTED, "request.zoneinfo status transmission failed");
}

/*
* Return set point info
*/

static void doGetSetPoint(String ws, xPL_MessagePtr theMessage, ZoneEntryPtr_t ze)
{
	String setpoint;

	if(!ze || !ws || !theMessage)
		return;

	setpoint = xPL_getMessageNamedValue(theMessage, "setpoint");



	if(setpoint){
		sprintf(ws + strlen(ws), " R=4");

		if(!strcmp(setpoint, setPointList[0])){
			queueCommand(ze, ws, CMDTYPE_RQ_SETPOINT_HEAT);
		}
		else if(!strcmp(setpoint, setPointList[1])){
			queueCommand(ze, ws, CMDTYPE_RQ_SETPOINT_COOL);
		}
	}
}

/*
* Send a request for zone information
*/

static void doZoneResponse(String ws, ZoneEntryPtr_t ze)
{

	if(!ze  || !ws)
		return;
	
	sprintf(ws + strlen(ws), " R=1");
	queueCommand( ze, ws, CMDTYPE_RQ_ZONE);
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
		queueCommand(NULL, ws, CMDTYPE_DATETIME);
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
	ZoneEntryPtr_t ze = NULL;


	if(!xPL_isBroadcastMessage(theMessage)){ /* If not a broadcast message */
		if(xPL_MESSAGE_COMMAND == xPL_getMessageType(theMessage)){ /* If the message is a command */
			const String iID = xPL_getTargetInstanceID(theMessage);
			const String type = xPL_getSchemaType(theMessage);
			const String class = xPL_getSchemaClass(theMessage);
			const String command =  xPL_getMessageNamedValue(theMessage, "command");
			const String request =  xPL_getMessageNamedValue(theMessage, "request");
			const String  zone =  xPL_getMessageNamedValue(theMessage, "zone");
			
			/* Allocate a working string */

			if(!(ws = mallocz(WS_SIZE)))
				MALLOC_ERROR;
			ws[0] = 0;
			
			/* If a zone was specified, see if it is in the zone list, and get the zone entry */
			if(zone){
				debug(DEBUG_ACTION,"Zone present");
				/* Find zone in list */
				for(ze = zoneEntryHead; ze; ze = ze->next){
					if(!strcmp(ze->name, zone))
						break;
				}
				if(ze){
					/* Copy the address into the working string */
					debug(DEBUG_ACTION,"Zone entry found");
					snprintf(ws, WS_SIZE, "A=%u", ze->address);
				}
			}
			if(command)
				debug(DEBUG_ACTION, "Command = %s", command);
			if(request)
				debug(DEBUG_ACTION, "Request = %s", request);

			if((!strcmp(instanceID, iID)) && (!strcmp(class,"hvac"))){
				if(!strcmp(type, "basic")){ /* Basic command schema */
					if(command && ze){
						switch(matchCommand(basicCommandList, command)){
							case 0: /* hvac-mode */
								cmd = doHVACMode(ws, theMessage, ze);
								break;

							case 1: /* fan-mode */
								cmd = doFanMode(ws, theMessage, ze);
								break;

							case 2: /* setpoint */
								cmd = doSetSetpoint(ws, theMessage, ze);
								break;

							case 3: /* display */
								cmd = doDisplay(ws, theMessage, ze);
								break;
					
							default:
								break;
						}
					}
					if(cmd){
						queueCommand(ze, cmd, CMDTYPE_BASIC); /* Queue the command */
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
								doZoneList(ws);
								break;

							case 2: /* zoneinfo */
								doZoneInfo( ws, ze );
								break;

							case 3: /* setpoint */
								doGetSetPoint(ws, theMessage, ze);
								break;

							case 4: /* zone */
								doZoneResponse(ws, ze);
								break;
								
							case 5: /* runtime */
								doGetRT(ws, theMessage, ze);
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
	char wc[20];
	Bool sendZoneTrigger = FALSE;
	Bool sendHeatSetPointTrigger = FALSE;
	Bool sendCoolSetPointTrigger = FALSE;
	int curArgc,lastArgc, sendAll = FALSE, i;
	String line;
	String pd,wscur,wslast,arg;
	String val = NULL;
	String curArgList[20];
	String lastArgList[20];


	/* Do non-blocking line read */
	if(serio_nb_line_read(serioStuff)){
		/* Got a line or EOF */
		if(serio_ateof(serioStuff)){
			debug(DEBUG_EXPECTED, "EOF detected on serial port, closing port");
			if(!xPL_removeIODevice(serio_fd(serioStuff))) /* Unregister ourself */
				debug(DEBUG_UNEXPECTED,"Could not unregister from poll list");
			serio_close(serioStuff); /* Close serial port */
			serioStuff = NULL;
			serialRetryTimer = SERIAL_RETRY_TIME;
			return; /* Bail */
		}

			
		line = serio_line(serioStuff);
		if(pollPending){ /* If this pointer is non-null, we are expecting a poll response */
			
			/* Has to be a response to a poll */
			/* Compare with last line received */
			if(!pollPending->first_time && strcmp(line, pollPending->last_poll)){
				debug(DEBUG_STATUS, "Got updated poll status: %s", line);

				/* Make working strings from the current and last lines */
				if(!(wscur = strdup(line)))
					MALLOC_ERROR;
				if(!(wslast = strdup(pollPending->last_poll)))
					MALLOC_ERROR;

				/* Parse the current and last lists for comparison */
	
				curArgc = parseRC65Status(wscur, curArgList, 19);
				lastArgc = parseRC65Status(wslast, lastArgList, 19);

				/* If arg list mismatch, set the sendAll flag */
				if(lastArgc != curArgc){
					sendAll = TRUE;
				}
				
				/* Prep a zone trigger just in case something needs to be sent */
				xPL_clearMessageNamedValues(xplrcsZoneTriggerMessage);
				xPL_setMessageNamedValue(xplrcsZoneTriggerMessage, "zone", pollPending->name);
				
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
						if(!strcmp(arg, "SPH")){ /* SPH has a dedicated trigger resource */
							sendHeatSetPointTrigger = TRUE;
							xPL_clearMessageNamedValues(xplrcsHeatSetPointTriggerMessage);
							xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "zone", pollPending->name);
							xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "setpoint", setPointList[0]);
							xPL_setMessageNamedValue(xplrcsHeatSetPointTriggerMessage, "temperature", pd );
						}

						else if(!strcmp(arg, "SPC")){ /* SPC has a dedicated trigger resource */
							sendCoolSetPointTrigger = TRUE;
							xPL_clearMessageNamedValues(xplrcsCoolSetPointTriggerMessage);
							xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "zone", pollPending->name);
							xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "setpoint", setPointList[1]);
							xPL_setMessageNamedValue(xplrcsCoolSetPointTriggerMessage, "temperature", pd );
						}
						else if(!strcmp(arg, "FM")){ /* Zone triggers share a trigger resource */
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
			/* Copy current string into last poll for future comparisons */	
			confreadStringCopy(pollPending->last_poll, line, WS_SIZE);
			
			/* Clear the first time flag */
			
			pollPending->first_time = FALSE;
			
			/* Done with poll, indicate that by setting pollPending to NULL */
			pollPending = NULL;
	
		} /* End if(pollPending) */
		else{  /* It's a response not related to a poll (i.e. a response from a request) */

			if(!(wscur = strdup(line)))
				MALLOC_ERROR;

			debug(DEBUG_EXPECTED, "Non-poll response: %s", wscur);
			
			/* Parse the returned arguments */
			curArgc = parseRC65Status(wscur, curArgList, 19);
			/* If it was a set point request */
			if(cmdEntryTail){
				if((cmdEntryTail->type == CMDTYPE_RQ_SETPOINT_HEAT)||(cmdEntryTail->type == CMDTYPE_RQ_SETPOINT_COOL)){
					/* Setpoint status (heat or cool) requested */
					debug(DEBUG_EXPECTED,"Setpoint Status requested"); 
					xPL_setSchema(xplrcsStatusMessage, "hvac", "setpoint");
					xPL_clearMessageNamedValues(xplrcsStatusMessage);
					xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", 
					(cmdEntryTail->ze && cmdEntryTail->ze->name) ? cmdEntryTail->ze->name : "unknown");

					if(cmdEntryTail->type == CMDTYPE_RQ_SETPOINT_HEAT){
						/* Setpoint heat requested */
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
				else if(cmdEntryTail->type == CMDTYPE_RQ_ZONE){
					char wc[20];
					debug(DEBUG_EXPECTED,"Zone Status requested"); 
					xPL_setSchema(xplrcsStatusMessage, "hvac", "zone");
					xPL_clearMessageNamedValues(xplrcsStatusMessage);
					for(i = 0 ; curArgList[i]; i++){ /* Iterate through arg list */
						debug(DEBUG_ACTION, "Arg: %s", curArgList[i]);
						if(!strncmp(curArgList[i], "O=", 2))
							xPL_setMessageNamedValue(xplrcsStatusMessage, "zone",
							(cmdEntryTail->ze && cmdEntryTail->ze->name) ? cmdEntryTail->ze->name : "unknown");
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
				/* Heat or cool run times */
				else if ((cmdEntryTail->type == CMDTYPE_RQ_HEATTIME)||(cmdEntryTail->type == CMDTYPE_RQ_COOLTIME)){
					debug(DEBUG_EXPECTED,"Run time requested"); 
					xPL_setSchema(xplrcsStatusMessage, "hvac", "runtime");
					xPL_clearMessageNamedValues(xplrcsStatusMessage);
					xPL_setMessageNamedValue(xplrcsStatusMessage, "zone", 
					(cmdEntryTail->ze && cmdEntryTail->ze->name) ? cmdEntryTail->ze->name : "unknown");

					if(cmdEntryTail->type == CMDTYPE_RQ_HEATTIME){
						/* Setpoint heat requested */
						val = getVal(wc, sizeof(wc), curArgList, "RTH");
						if(val)
							xPL_setMessageNamedValue(xplrcsStatusMessage, setPointList[0], val); /* Heating */
					}
					else{
						val = getVal(wc, sizeof(wc), curArgList, "RTC");
						if(val)
							xPL_setMessageNamedValue(xplrcsStatusMessage, setPointList[1], val); /* Cooling */
					}
					if(val){
						
						xPL_setMessageNamedValue(xplrcsStatusMessage, "units", "hours");
						
						if(!xPL_sendMessage(xplrcsStatusMessage))
							debug(DEBUG_UNEXPECTED, "Setpoint status transmission failed");
					
					}
					
				}
			}
			/* Free the command entry */
			dequeueAndFreeCommand();
			 /* Free working string */
			free(wscur);
		}
	} /* End serio_nb_line_read */
}


/*
* Our tick handler. 
* This is used to synchonize the sending of data to the RCS thermostat.
*/

static void tickHandler(int userVal, xPL_ObjectPtr obj)
{
	static short pollCtr = 0;
	static short readySent = FALSE;
	static ZoneEntryPtr_t pollZone = NULL;
	
	

	


	pollCtr++;

	debug(DEBUG_STATUS, "TICK: %d", pollCtr);
	/* Process clock tick update checking */
	
	if(serialRetryTimer){ /* If this is non-zero, we lost the serial connection, wait retry time and try again */
		serialRetryTimer--;
		if(!serialRetryTimer){
			if(!(serioStuff = serio_open(comPort, 9600))){
				debug(DEBUG_UNEXPECTED,"Serial reconnect failed, trying later...");
				serialRetryTimer = SERIAL_RETRY_TIME;
				return;
			}
			else{
				debug(DEBUG_EXPECTED,"Serial reconnect successful");
				if(!xPL_addIODevice(serioHandler, 1234, serio_fd(serioStuff), TRUE, FALSE, FALSE))
					fatal("Could not register serial I/O fd with xPL");
			}
		}
	}
				
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
	else if(cmdEntryTail && (!cmdEntryTail->sent)){ /* If command pending */
		/* Uppercase the command string */
		str2Upper(cmdEntryTail->cmd);
		debug(DEBUG_EXPECTED, "Sending command: %s", cmdEntryTail->cmd);
		serio_printf(serioStuff, "%s\r", cmdEntryTail->cmd);
		cmdEntryTail->sent = TRUE;
		if((cmdEntryTail->type == CMDTYPE_DATETIME)||(cmdEntryTail->type == CMDTYPE_BASIC)||
		(cmdEntryTail->type == CMDTYPE_NONE))
			dequeueAndFreeCommand(); /* These commands do not send back a response */
	}

	else if(pollCtr >= pollRate){ /* Else check poll counter */
		pollCtr = 0;
		/* Ensure pollZone is not NULL */
		if(!pollZone)
			pollZone = zoneEntryHead;
		if(pollZone){
			debug(DEBUG_ACTION, "Polling Status A=%d, R=1...", pollZone->address);
			serio_printf(serioStuff, "A=%d R=1\r", pollZone->address);
			if(pollPending){
				/* Note: This probably warrants a trigger message of some sort */
				debug(DEBUG_UNEXPECTED, "Did not receive a response from zone %s at address %u", 
				pollPending->name, pollPending->address);
			}
			pollPending = pollZone; /* Set to current poll entry */
			pollZone = pollZone->next;
		}
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
	printf("  -c, --config-file PATH  Set the path to the config file\n");
	printf("  -d, --debug LEVEL       Set the debug level, 0 is off, the\n");
	printf("                          compiled-in default is %d and the max\n", debugLvl);
	printf("                          level allowed is %d\n", DEBUG_MAX);
	printf("  -f, --pid-file PATH     Set new pid file path, default is: %s\n", pidFile);
	printf("  -h, --help              Shows this\n");
	printf("  -i, --interface NAME    Set the broadcast interface (e.g. eth0)\n");
	printf("  -l, --log  PATH         Path name to debug log file when daemonized\n");
	printf("  -n, --no-background     Do not fork into the background (useful for debugging)\n");
	printf("  -p, --com-port PORT     Set the communications port (default is %s)\n", comPort);
	printf("  -r, --poll-rate RATE    Set the poll rate in seconds");
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
	int i;
	String p;
	String plist[MAX_ZONES];
	ZoneEntryPtr_t ze;

		

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
		
				/* Was it a config file switch? */
			case 'c':
				confreadStringCopy(configFile, optarg, WS_SIZE - 1);
				debug(DEBUG_ACTION,"New config file path is: %s", configFile);
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
				confreadStringCopy(pidFile, optarg, WS_SIZE - 1);
				clOverride.pid_file = 1;
				debug(DEBUG_ACTION,"New pid file path is: %s", pidFile);
				break;
			
				/* Was it a help request? */
			case 'h':
				showHelp();
				exit(0);

				/* Specify interface to broadcast on */
			case 'i': 
				confreadStringCopy(interface, optarg, WS_SIZE -1);
				clOverride.interface = 1;
				break;

			case 'l':
				/* Override log path*/
				confreadStringCopy(logPath, optarg, WS_SIZE - 1);
				clOverride.log_path = 1;
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
				confreadStringCopy(comPort, optarg, WS_SIZE - 1);
				clOverride.com_port = 1;
				debug(DEBUG_ACTION,"New com port is: %s",
				comPort);

				break;
						
			case 'r':
				/* Was it a poll rate? */
				if(!str2uns(optarg, &pollRate, POLL_RATE_MIN, POLL_RATE_MAX))
					fatal("Command line poll rate must be between %d and  %d seconds", POLL_RATE_MIN, POLL_RATE_MAX);
				clOverride.poll_rate = 1;
				break;
			

				/* Was it an instance ID ? */
			case 's':
				confreadStringCopy(instanceID, optarg, WS_SIZE);
				clOverride.instance_id = 1;
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

	/* Attempt to read a config file */
	
	if(!(configEntry =confreadScan(configFile, NULL)))
		exit(1);

		
	/* Get config file entries in general section */
	
	/* zones (mandatory) */
	if((!(p = confreadValueBySectKey(configEntry, "general", "zones"))) || (!strlen(p)))
		fatal("At least one zone must be defined in %s", configFile);
		
	/* Split the zones */
	numZones = dupOrSplitString(p, plist, ',', MAX_ZONES);
	
	for(i = 0; i < numZones; i++){
		String za;
		if(!confreadFindSection(configEntry, plist[i]))
			fatal("Zone section %s is missing in config file", plist[i]);
		
		/* Initialize zone entry */	
		if(!(ze = mallocz(sizeof(ZoneEntry_t))))
			MALLOC_ERROR;
		if(!(ze->last_poll = mallocz(WS_SIZE)))
			MALLOC_ERROR;
		if(!(ze->name = strdup(plist[i])))
			MALLOC_ERROR;
		if(!(za = confreadValueBySectKey(configEntry, plist[i], "address")))
			fatal("Zone section %s is missing an address key", ze->name);
		if(!str2uns(za, &ze->address, 1, 255))
			fatal("Zone section %s has an out of range address", ze->name);
		ze->first_time = TRUE;
		
		/* Insert into zone list */
		if(!zoneEntryHead)
			zoneEntryHead = zoneEntryTail = ze;
		else{
			zoneEntryTail->next = ze;
			ze->prev = zoneEntryTail;
			zoneEntryTail = ze;
		}		
	}
	free(plist[0]);
	debug(DEBUG_ACTION, "Number of zones defined: %d\n", numZones);
		
	
	/* com port */
	if((!clOverride.com_port) && (p = confreadValueBySectKey(configEntry, "general", "com-port")))
		confreadStringCopy(comPort, p, sizeof(comPort));
			
	/* Instance ID */
	if((!clOverride.instance_id) && (p = confreadValueBySectKey(configEntry, "general", "instance-id")))
		confreadStringCopy(instanceID, p, sizeof(instanceID));
		
	/* Interface */
	if((!clOverride.interface) && (p = confreadValueBySectKey(configEntry, "general", "interface")))
		confreadStringCopy(interface, p, sizeof(interface));
			
	/* pid file */
	if((!clOverride.pid_file) && (p = confreadValueBySectKey(configEntry, "general", "pid-file")))
		confreadStringCopy(pidFile, p, sizeof(pidFile));	
						
	/* log path */
	if((!clOverride.log_path) && (p = confreadValueBySectKey(configEntry, "general", "log-path")))
		confreadStringCopy(logPath, p, sizeof(logPath));
		
	/* poll rate */
	if(((!clOverride.poll_rate) && (p = confreadValueBySectKey(configEntry, "general", "poll-rate")))){
		if(!str2uns(p, &pollRate, POLL_RATE_MIN, POLL_RATE_MAX))
			fatal("Poll Rate must be between %d and %d seconds", POLL_RATE_MIN, POLL_RATE_MAX);
	}
	/* units */
	if(((!clOverride.poll_rate) && (p = confreadValueBySectKey(configEntry, "general", "units")))){
		confreadStringCopy(units, p, sizeof(units));
		if((strcmp(units, "celsius")) && (strcmp(units, "fahrenheit")))
			fatal("Units must be either celsius or fahrenheit");
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
			

		/* Check to see the serial device exists before we fork */
		if(!serio_check_node(comPort))
			fatal("Serial device %s does not exist or its permissions are not allowing it to be used.", comPort);

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

	/* Set the xPL interface */
	xPL_setBroadcastInterface(interface);

	/* Start xPL up */
	if (!xPL_initialize(xPL_getParsedConnectionType())) {
		fatal("Unable to start xPL lib");
	}

	/* Initialize xplrcs service */

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

