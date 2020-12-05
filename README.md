# esp-dmxgw
Simple E1.31 to DMX gateway application targeting the ESP32 platform

## Hardware Targeted
* SparkFun ESP32 Thing Plus: https://www.sparkfun.com/products/15663
* SparkFun ESP32 Thing Plus DMX to LED Shield: https://www.sparkfun.com/products/15110

## Usage
* Build using PlatformIO and program onto ESP32 module
* Connect to "DMXGW" wireless SSID and configure desired SSID+password that you would like the module to connect to.
* A simple Web UI will be available on port 80 of the module when the module is connected to an AP, this allows one to:
    * Manually override values for all DMX channels output to any value between 0 and 255
    * Reset the WiFi settings, causing the module to reboot and serve WifiManager (the configuration UI)
    * Reboot the module, leaving all settings intact
* Additionally, the module will listen for ETSA E1.31 packets sent to its address with universe=1 and output such via serial DMX

## Notes
* This application is not designed to be directly exposed to the internet, as there is no authentication on the web UI, e1.31 interface, or OTA upgrade service.
* This has been tested up to 20 DMX packets/sec using Xlights/FPP as a data source.This likely can handle faster but no guarantees.
* If this app loses the WiFi connection for over one minute, the module will fall back to serving the config portal for 5 minutes.
    * If not reconfigured or the config portal exited in such time, the module will reboot and repeat the process of trying to connect to the last configured AP for one minute and serving WiFimanager for 5 mins.

