#!/usr/bin/python

# =====================================================================
# sphero_launcher.py
# =====================================================================
# last updated: August 11, 2017
# author: Martin Herrera mh866
#
# This program takes the specified spheros the user wants to connect to and
# instantiates launch_sphero.py for each one. It then waits for a keyboard interrupt
# at which point it terminates all the instances of launch_sphero.py it spawned.
# The purpose of this is to provide the user with a simple way to launch multiple
# spheros without having to write a bash script for eahc set of spheros launched or
# open several terminal windows.
# 

import time
import sys
import os
import argparse
import subprocess

if __name__ == "__main__":
	# ---------------------------------------------
	# Parse through color codes given as arguments
	# i.e.: sphero_launcher.py RPP WPP WPW
	# ---------------------------------------------
	numSpheros = len(sys.argv)
	spheros = sys.argv[1:len(sys.argv)]


	processes = []
	try:
		# -------------------------------------------------------------------------------------
		# Loop through each of the colors specified and launch an instance of launch_sphero.py
		# append process id to process array for termination later.
		# -------------------------------------------------------------------------------------
		for color in spheros:
			p = subprocess.Popen(['./launch_sphero.py','--target',color])
			processes.append(p)
			time.sleep(5)

		# -----------------------------
		# Listen for keyboard interrupt
		# -----------------------------
		while True:
			time.sleep(1)

	# -----------------------------------------------------------------------------
	# Terminate all instances of launch_sphero.py launched upong keyboard intterupt
	# -----------------------------------------------------------------------------
	except KeyboardInterrupt:
		print("Keyboard Interrupt detected, disconnecting Spheros")
		for proc in processes:
			proc.terminate()