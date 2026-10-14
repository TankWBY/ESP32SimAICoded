# Board pictures used by the web UI

| File | Source | License |
|---|---|---|
| `images/esp32-c3-devkitm-1.png` | Cropped from the ESP32-C3-DevKitM-1 pin layout drawing in Espressif's [esp-dev-kits](https://github.com/espressif/esp-dev-kits) documentation | CC BY-SA 4.0, © Espressif Systems |
| `images/esp32-c3-devkitc-02.png` | Cropped from the ESP32-C3-DevKitC-02 pin layout drawing in the same repository | CC BY-SA 4.0, © Espressif Systems |
| `images/seeed_xiao_esp32c3.svg` | Drawn for esp32sim (an illustration, not to exact scale) | same as this project |
| `images/esp32-c6-devkitc-1.png` | Cropped from the ESP32-C6-DevKitC-1 pin layout drawing in esp-dev-kits | CC BY-SA 4.0, © Espressif Systems |
| `images/esp32-c6-devkitm-1.png` | Cropped from the ESP32-C6-DevKitM-1 pin layout drawing in esp-dev-kits | CC BY-SA 4.0, © Espressif Systems |
| `images/seeed_xiao_esp32c6.svg` | Drawn for esp32sim (an illustration, not to exact scale) | same as this project |

The UI also shows Seeed Studio's official XIAO ESP32C3 photo, loaded by the
browser straight from the Seeed wiki (it is not copied into this repository).
The XIAO ESP32C6 layout links the Seeed wiki page only.

`layouts/<board>.json` gives each picture's size and the position of every
pin, button and LED on it. The board id is PlatformIO's.
