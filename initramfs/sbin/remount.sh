#!/system/bin/sh
if [ "$1" = "rw" ]; then
  echo "Remounting /system (/dev/stl5) in read/write mode"
  /system/bin/mount -o remount,rw /dev/block/stl5 /system
else
  if [ "$1" = "ro" ]; then
    echo "Remounting /system (/dev/stl5) in read-only mode"
    /system/bin/mount -o remount,ro /dev/block/stl5 /system
  else
    echo "for mounting /system as read/write or read-only"
    echo "usage: remount rw   -or-   remount ro"  
  fi
fi
