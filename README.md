This project is an extension of the project esp32-cam-mjpeg-multiclient originally developped by **arkhipenko** 
and located at https://github.com/arkhipenko/esp32-cam-mjpeg-multiclient.

The following functionalities have been added:
 - status (/status)  : Returns the current Camera settings in json format
 - control (/control): Lets the User modify Camera Settings (for the moment only one at a time)
                       such as framesize, contrast, luminosity, saturation, etc ...
 - reboot (/reboot)  : Let's the user reboot the Camera from a Client App.
                       The code is widely inspired from the project CameraWebServer.

The code of these functionnalities is widely inspired from the example project Camera WebServer from **Espressif**
located at https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer.

TODO: Finalize the documentation ;-)
