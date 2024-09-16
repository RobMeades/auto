# Installation
This code builds under ESP-IDF, which can be installed by following the instructions here:

https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html#installation

# Usage
Once you have installed ESP-IDF and cloned this repo, `CD` to this directory and build/flash/monitor it with the single command:

```
idf.py -p <port> flash monitor
```

...where `<port>` is replaced by the serial port on which the ESP32S3 chip inside the Auto can be found.

Press `CTRL ]` to terminate the monitor program.