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
* serio definitions.
*
*
*/

#ifndef SERIO_H
#define SERIO_H

#define SERIO_MAX_LINE 1024


/* Typedefs. */
typedef struct seriostuff seriostuff_t;

/* Structure to hold serio info. */
struct seriostuff {
	int fd;				/* File descriptor */
	int pos;
	char *line;
};

/* Prototypes. */
seriostuff_t *serio_open(char *tty_name, unsigned baudrate);
void serio_close(seriostuff_t *serio);
int serio_flush_input(seriostuff_t *serio);
int serio_fd(seriostuff_t *s);
int serio_get_baud(unsigned br);
int serio_write(seriostuff_t *serio, const void *buffer, size_t count);
int serio_read(seriostuff_t *serio, void *buffer, size_t count);
int serio_nb_line_read(seriostuff_t *serio);
char *serio_line(seriostuff_t *serio);
int serio_printf(seriostuff_t *serio, const char *format, ...);

#endif
