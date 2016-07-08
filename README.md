mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=.... ..
make
make install

# Start the binder

```
afb-daemon --token=x --ldpaths=. --port=5555 --rootdir=. --verbose --verbose --verbose --verbose --verbose
```

# Environment variables

AFBGPS_HOST    : hostname to connect to
AFBGPS_SERVICE : service to connect to (tcp port)
AFBGPS_ISNMEA  : 0/1 - does the frames are NMEA or not") ? 0 : 1;


