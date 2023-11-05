#!/usr/bin/env bash
lock_file="/tmp/rokid.lock"
log_file="/tmp/rokid.log"

default_params="--mode 1920x1200-60hz --vol 1 --brightness 2"
params="${*:-$default_params}"

echo >> $log_file
date >> $log_file
echo "DRIVER = $DRIVER" >> $log_file
echo "ACTION = $ACTION" >> $log_file 

# If we were run manually, thats OK.
# If we were run by udev, we only handle bind actions for usbhid.
if [[ -z "$ACTION" || ($DRIVER == "usbhid" && $ACTION == "bind") ]]; then
    # Check if we updated the lockfile recently, only if triggered by udev
    if [[ -n "$ACTION" ]]; then
        echo "Triggered by udev" >> $log_file
        if [[ -f "$lock_file" && $(find "$lock_file" -newermt '-5 seconds' 2>>$log_file) ]]; then
            echo "Lockfile updated recently. Aborting." >> $log_file
            exit 1
        fi
    else
        echo "Triggered manually : $@" >> $log_file
        env >> $log_file
    fi

    # Make lockfile
    touch "$lock_file"
    # Ensure any user can update it as required.
    chmod ugo+rw "$lock_file"
    /opt/rokid/rokid_max_ctl $params >> $log_file  
    # Ensure log can be updated by any caller as required.  
    chmod ugo+rw "$log_file"

fi