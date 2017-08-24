#!/usr/bin/python

# =====================================================================
# launch_sphero.py
# =====================================================================
# last updated: August 11, 2017
# author: Martin Herrera mh866
#
# This program serves as an interpreter to the sphero. The scipt continously reads from a 
# fifo in which sphero velocity commands ar being written by another program, it then uses 
# a sphero object to send these commands to the sphero via bluetooth.
# 
import sphero
import time
import sys
import os
import argparse

def findSphero(sphero_color):
	dictNames = {'RPP':"Sphero_RPP",
				 'WPP':"Sphero_WPP",
				 'OBR':"Sphero_OBR",
				 'GGW':"Sphero_GGW", 
				 'RPR':"Sphero_RPR",
				 'PRR':"Sphero_PRR",
				 'WPW':"Sphero_WPW"}

	dictAddr = {'RPP':"68:86:E7:09:2E:FE",
				'WPP':"68:86:E7:08:84:9E",
				'OBR':"68:86:E7:08:01:0B",
				'GGW':"68:86:E7:08:95:5F",
				'RPR':"68:86:E7:07:3A:E1", 
				'PRR':"68:86:E7:09:77:2C",
				'WPW':"68:86:E7:09:0A:9E"}

	return[dictNames[sphero_color],dictAddr[sphero_color]]

def openFifo(spheroName):
	path = "./pipes/"+spheroName + ".fifo"
	file_exists = os.path.exists(path)
	if not file_exists:
		os.mkfifo(path)

	return open(path,"r",0)

if __name__ == '__main__':
	# --------------------------------------------
	# Parse arguments
	# - target: color code of sphero to connect to 
	# --------------------------------------------
	parser = argparse.ArgumentParser(description="Start Sphero")
	parser.add_argument('--target', type=str, help='Specify color of Sphero', nargs='?', \
	                            const='', default='')
	parser.add_argument('--bt_addr', type=str, help='Specify address of bluetooth adapter', nargs='?', const='', default='')
	args, unknown = parser.parse_known_args()

	spheroName = args.target.upper()
	bt_addr = args.bt_addr
	# -----------------------------------------------------------------------
	# Find name and address of correspondoing sphero color code in spheroName
	# -----------------------------------------------------------------------
	target = findSphero(spheroName)
	target_name =  target[0]
	target_addr = target[1]

	print ("Connecting to "+target_name+ " at address: " + target_addr)


	# -----------------------------------------------------------------------
	# Create sphero object that will connect via bluetooth and send commands
	# and open fifo to begin recieving commands
	# -----------------------------------------------------------------------
	sphero = sphero.connectToSphero(target_name,target_addr,bt_addr,1)
	fifo = openFifo(target_name)

	# -----------------------
	# Continously read fifo
	# -----------------------
	while True:
		line = fifo.readline()

		# ------------------------------------------------------------
		# If writer side of fifo is finished, close fifo and reopen it
		# so that a new program can send commands
		# ------------------------------------------------------------
		if len(line) == 0:
			print("Writer closed")
			fifo.close()
			fifo = openFifo(target_name)
		# ----------------------------------------------------------------
		# Else, writer has written a line of commands as "xVel,yVel"
		# parse the commands as numbers and have sphero object send via BT
		# ----------------------------------------------------------------
		else:
			# Check whether command is a velocity or color command
			if (len(command) < 3):
				command = [float(command[0]),float(command[1])]
				sphero.cmd_vel(command)
			else:
				command = [float(command[0]),float(command[1]),float(command[2])]
				sphero.set_color(command)
	fifo.close()