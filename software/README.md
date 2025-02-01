# Installation
This code builds under ESP-IDF, which can be installed by following the instructions here:

https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html#installation

# Usage
Once you have installed ESP-IDF and cloned this repo, `cd` to this directory and build/flash/monitor it with the single command:

```
idf.py -p <port> flash monitor
```

...where `<port>` is replaced by the serial port on which the ESP32S3 chip inside `auto` can be found.

If you have [ESP-IDF installed inside VSCode](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/get-started/vscode-setup.html) then open this folder and, once you have [configured the serial port](https://github.com/espressif/vscode-esp-idf-extension/?tab=readme-ov-file#using-the-esp-idf-extension-for-vscode), `ctrl-e` followed by `d` will do the same thing.

Press `CTRL ]` to terminate the monitor program.
