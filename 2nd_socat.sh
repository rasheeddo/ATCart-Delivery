#!/bin/sh
sleep 2
socat -t 0 -d -d pty,raw,echo=0 pty,raw,echo=0 
