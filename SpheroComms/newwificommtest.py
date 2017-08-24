#!/usr/bin/python

import wifiComm

# Create wifiComm object
wifiCommObj = wifiComm.wifiComm()
# Initialize wifiComm object as server
wifiCommObj._init_server()
# Send velocity [X velocity, Y Velocity] command to Sphero RPP
wifiCommObj.sendCmdPacket('RPP','v',[0,90])
# Send color change command [R, G, B] to Sphero RPP
wifiCommObj.sendCmdPacket('RPP','c',[0,255,0])







