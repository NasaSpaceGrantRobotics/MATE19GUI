#!/bin/bash
xterm -title "Xbox driver" -e -hold "rmmod xpad; sudo xboxdrv --silent"