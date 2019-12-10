#! /bin/sh

function module_load
{
    module=aesdchar
    device=aesdchar
    mode="664"

#    if [ $# -ne 1 ]; then
#        echo "Wrong number of arguments"
#        echo "usage: $0 module_name"
#        echo "Will create a corresponding device /dev/module_name associated with module_name.ko"
#        exit 1
#    fi

    set -e
    # Group: since distributions do it differently, look for wheel or use staff
    if grep -q '^staff:' /etc/group; then
        group="staff"
    else
        group="wheel"
    fi

    modprobe $module $* || exit 1
    major=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)
    rm -f /dev/${device}
    mknod /dev/${device} c $major 0
    chgrp $group /dev/${device}
    chmod $mode  /dev/${device}
}

function module_unload
{
    module=aesdchar
    device=aesdchar

#    if [ $# -ne 1 ]; then
#        echo "Wrong number of arguments"
#        echo "usage: $0 module_name"
#        echo "Will unload the module specified by module_name and remove assocaited device"
#        exit 1
#    fi

    # invoke rmmod with all arguments we got
    rmmod $module || exit 1

    # Remove stale nodes

    rm -f /dev/${device}
}

case "$1" in
    start)
        echo "Starting modules"
        module_load
        ;;
    stop)
        echo "Stopping modules"
        module_unload
        ;;
    *)
        echo "Usage: $0 {start|stop}"
    exit 1
esac

exit 0