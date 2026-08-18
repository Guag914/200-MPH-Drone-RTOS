#!/bin/bash

DIR="$(cd "$(dirname "$0")" && pwd)"

renode simulate.resc &
sleep 2
osascript -e "tell app \"Terminal\" to do script \"cd '$DIR' && nc -C localhost 3456\""