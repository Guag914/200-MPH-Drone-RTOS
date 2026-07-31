#!/bin/bash
renode simulate.resc &
sleep 2
osascript -e 'tell app "Terminal" to do script "nc -C localhost 3456"'