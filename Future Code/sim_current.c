/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim_current.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "vehicle.h"
#include "ody_defs.h"
#include "ody_types.h"
#include "ody_var.h"
#include "ody_func.h"
#include "odyssey_data_struct.h"

/* forward declaration for function defined in load_mission.c */
void skip_comments( FILE *f );

#define MAX_STRING  100

extern int REAL_CURRENT_NORTH;
extern int REAL_CURRENT_EAST;

float  xstart, xstop, xstep, ystart, ystop, ystep;
char   vxfile[80], vyfile[80];
int    CUR_INIT_FLAG=1, CUR_SIM_ENABLE=1;

void sim_current();
int limit_to_range(float, float, float, float);
float read_from_file(char *, int);
int init_current_params();   /* fixed: was init_currents_params */

void sim_current()
{
  int index, index_x, index_y, y_max;
  float east, north;
  static int init=-1;

  if (read_sensor(READ_CURRENT_FROM_FILE)==1) {
    if (init) {
      init_current_params();   /* fixed: was init_currents_params */
      init=0;
    }
    if (CUR_SIM_ENABLE) {
      east  = read_sensor(REAL_Y);
      north = read_sensor(REAL_X);
      index_x = limit_to_range(xstart,xstop,xstep,east);
      index_y = limit_to_range(ystart,ystop,ystep,north);
      y_max = (int) ((ystop-ystart)/ystep + 1);
      index = y_max*index_x + index_y;
      update_sensor(REAL_CURRENT_EAST,  read_from_file(vxfile, index));
      update_sensor(REAL_CURRENT_NORTH, read_from_file(vyfile, index));
    }
  }
}

int init_current_params()
{
  FILE *infile;
  if (chdir("Current")==-1) {
    fprintf(stderr, "Problem with Current directory. Disabling current sim.\n");
    CUR_SIM_ENABLE = 0;
    return(0);
  }
  if (CUR_SIM_ENABLE) {
    if ((infile = fopen("current_data_params.txt", "r"))==NULL) {
      fprintf(stderr, "sim_current.c: Cannot open input file %s\n",
              "current_data_params.txt");
      fprintf(stderr, "sim_current.c: Disabling current sim.\n");
      CUR_SIM_ENABLE = 0;
      return(0);
    }
    skip_comments(infile);
    if (fscanf(infile," %f %f %f %s",&xstart,&xstop,&xstep,vxfile)==0) {
      fprintf(stderr, "Error reading params in current init file\n");
      exit(-1);
    }
    if (fscanf(infile," %f %f %f %s",&ystart,&ystop,&ystep,vyfile)==0) {
      fprintf(stderr, "sim_current.c: Error reading param vals\n");
      exit(-1);
    }
    fclose(infile);
    chdir("..");
  }
  return(0);
}

int limit_to_range(float start, float stop, float step, float pos)
{
  int index;
  if (pos>start) {
    if (pos<stop)
      index = (int) floor((pos-start)/step);   /* fixed: was ffloor */
    else
      index = (int) (stop-start)/step;
  }
  else
    index = 0;
  return index;
}

float read_from_file(char *file, int index)
{
  FILE *infile;
  float value;
  chdir("Current");
  if ((infile = fopen(file, "r"))==NULL) {
    fprintf(stderr, "sim_current.c: Cannot open input file %s\n", file);
    exit(-1);
  }
  if(fseek(infile,(long) index*4,SEEK_SET)) {
    fprintf(stderr, "sim_current.c: Cannot fseek on velocity file\n");
    exit(-1);
  }
  if (fread(&value,4,1,infile)!=1) {
    fprintf(stderr, "sim_current: Cannot get binary value from Vx file\n");
    exit(-1);
  }
  fclose(infile);
  chdir("..");
  return value;
}
