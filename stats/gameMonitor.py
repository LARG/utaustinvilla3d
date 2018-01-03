#!/usr/bin/python

import os
import sys
import math
import struct
import socket   #for sockets
import shutil

time = 0
scoreLeft = 0
scoreRight = 0
agentsLeftStart = [False]*11
agentsRightStart = [False]*11
agentsLeftExisted = [False]*11
agentsRightExisted = [False]*11
agentsLeftHere = [False]*11
agentsRightHere = [False]*11

if len(sys.argv) < 2:
  print "Usage: gameMonitor.py <output_file> [host] [port]"
  sys.exit()

outputFile = sys.argv[1]
outputFile = os.path.dirname(outputFile) + "/" + os.path.basename(outputFile)
print outputFile

host = "localhost"
if len(sys.argv) > 2:
  host = sys.argv[2]

port = 3200
if len(sys.argv) > 3:
  port = int(sys.argv[3])

try:
    #create an AF_INET, STREAM socket (TCP)
    sserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error, msg:
    print 'Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1]
    sys.exit();

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    host = socket.gethostbyname( host )

except socket.gaierror:
    #could not resolve
    print 'Hostname could not be resolved. Exiting'
    sys.exit()

#Connect to remote server
sserver.connect((host, port))

reqfullstate = True

while True:
    msgSize = sserver.recv(4)
    msgSize = struct.unpack("!L", msgSize)
    msgFromServer = sserver.recv(msgSize[0])
    msgFromServer = msgFromServer.replace('(', ' ').replace(')', ' ')
    
    timeIndex = msgFromServer.find("time")
    if timeIndex != -1:
      tokens = msgFromServer[timeIndex:].split()
      time = float(tokens[1])
    scoreLeftIndex = msgFromServer.find("score_left")
    if scoreLeftIndex != -1:
      tokens = msgFromServer[scoreLeftIndex:].split()
      scoreLeft = int(tokens[1])
    scoreRightIndex = msgFromServer.find("score_right")
    if scoreRightIndex != -1:
      tokens = msgFromServer[scoreRightIndex:].split()
      scoreRight = int(tokens[1])

    if msgFromServer.find("RSG") != -1:
      agentsLeftHere = [False]*11
      agentsRightHere = [False]*11
    for i in range(1,12):
      if msgFromServer.find("Num" + str(i) + " matLeft") != -1:
        if time == 0:
          agentsLeftStart[i-1] = True
        agentsLeftExisted[i-1] = True
        agentsLeftHere[i-1] = True
      if msgFromServer.find("Num" + str(i) + " matRight") != -1:
        if time == 0:
          agentsRightStart[i-1] = True
        agentsRightExisted[i-1] = True
        agentsRightHere[i-1] = True

    if time >= 300.0:
      #65479 is a message length when a message is truncated in error 
      if scoreLeftIndex == -1 or scoreRightIndex == -1 or reqfullstate or len(msgFromServer) == 65479:
        msg = "(reqfullstate)"
        try :
          #Set the whole string
          sserver.send(struct.pack("!I", len(msg)) + msg)
          reqfullstate = False
        except socket.error:
          #Send failed
          print 'Send failed'

        continue


      allCrashedLeft = True
      allCrashedRight = True
      
      f = open(outputFile, 'w')
      scoreMe = scoreLeft
      scoreOpp = scoreRight
      '''
      if os.path.basename(outputFile).find("_right") != -1:
        scoreMe = scoreRight
        scoreOpp = scoreLeft
      '''
      f.write("score = " + str(scoreMe) + " " + str(scoreOpp) + "\n")

      fMissing = False
      fCrash = False
      for i in range(len(agentsLeftStart)):
        if not agentsLeftExisted[i]:
          f.write("missing_left " + str(i+1) + "\n")
          fMissing = True
        elif not agentsLeftStart[i]:
          f.write("late_left " + str(i+1) + "\n")
          fMissing = True

        if not agentsLeftHere[i] and agentsLeftExisted[i]:
          f.write("crash_left " + str(i+1) + "\n")
          fCrash = True
        elif agentsLeftExisted[i]:
          allCrashedLeft = False

        if not agentsRightExisted[i]:
          f.write("missing_right " + str(i+1) + "\n")
          fMissing = True
        elif not agentsRightStart[i]:
          f.write("late_right " + str(i+1) + "\n")
          fMissing = True

        if not agentsRightHere[i] and agentsRightExisted[i]:
          f.write("crash_right " + str(i+1) + "\n")
          fCrash = True
        elif agentsRightExisted[i]:
          allCrashedRight = False

      if allCrashedLeft:
        f.write("all_crashed_left\n")
      if allCrashedRight:
        f.write("all_crashed_right\n")

      #f.write(str(msgSize) + "\n")
      if fMissing or fCrash:
        f.write("host " + socket.getfqdn(host) + "\n")

      f.close()
        
      break
        
      

sserver.close()
