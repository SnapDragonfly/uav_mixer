#!/bin/sh

# Path to log file
LOGFILE="/root/uav_mixer.log"
# Path to PID file
PIDFILE="/var/run/uav_mixer.pid"

case "$1" in
    start)
        echo "Starting imu&img mixer service..."
	uav_mixer -f 100 -r 30 -d /dev/ttyS2 > /dev/null 2>&1 &
        #uav_mixer -f 100 -r 30 -d /dev/ttyS2 > "$LOGFILE" 2>&1 &
        echo $! > "$PIDFILE"
        echo "uav_mixer started with PID $(cat $PIDFILE)"
        ;;

    stop)
        echo "Stopping imu&img mixer service..."
        if [ -f "$PIDFILE" ]; then
            kill "$(cat $PIDFILE)" && rm -f "$PIDFILE"
            echo "uav_mixer stopped."
        else
            echo "uav_mixer is not running."
        fi
        ;;

    restart)
	if [ -f "$PIDFILE" ]; then
            kill "$(cat $PIDFILE)" && rm -f "$PIDFILE"
            echo "uav_mixer stopped."
        else
            echo "uav_mixer is not running."
        fi
	sleep 3
	echo "Restarting imu&img mixer service..."
        uav_mixer -f 100 -r 30 -d /dev/ttyS2 > /dev/null 2>&1 &
        #uav_mixer -f 100 -r 30 -d /dev/ttyS2 > "$LOGFILE" 2>&1 &
        echo $! > "$PIDFILE"
        echo "uav_mixer started with PID $(cat $PIDFILE)"
	;;


    status)
	if [ -f /var/run/uav_mixer.pid ]; then
    	    echo "uav_mixer is running with PID $(cat /var/run/uav_mixer.pid)"
	else
    	    echo "uav_mixer is not running."
	fi
	;;

    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;
esac



