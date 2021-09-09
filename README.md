This project is an extension of the project esp32-cam-mjpeg-multiclient originally developped by **arkhipenko** 
and located at https://github.com/arkhipenko/esp32-cam-mjpeg-multiclient.

The following functionalities have been added:
 - settings (/settings): Returns the current Camera settings in json format
 
 - control (/control): Lets the User modify Camera Settings (for the moment only one at a time)
                       such as framesize, contrast, luminosity, saturation, etc ...
 - reboot (/reboot)  : Let's the user reboot the Camera from a Client App.
                       The code is widely inspired from the project CameraWebServer.

The code of these functionnalities is widely inspired from the example project Camera WebServer from **Espressif**
located at https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer.

Assuming that the IP address of the camera is 192.168.1.11, we have:

Video Stream:	http://192.168.1.11/mjpeg/1 
Settings: 		http://192.168.1.11/settings
This command returns the following informations:
	{"framesize":6,"quality":12,"brightness":0,"contrast":0,"saturation":0,"special_effect":0,"wb_mode":0,"awb":1,"awb_gain":1,"aec":1,
	 "aec2":0,"ae_level":0,"aec_value":168,"agc":1,"agc_gain":0,"gainceiling":0,"bpc":0,"wpc":1,"raw_gma":1,"lenc":1,"vflip":0,"hmirror":0,
	 "dcw":1,"colorbar":1,"face_detect":0,"face_enroll":0,"face_recognize":0}

Control: 		http://192.168.1.11/control?cmd=CMD&val=VAL
CMD is the command we want to execute and VAL is the value of the command's argument.

Example: 		http://192.168.1.11/control?cmd=colorbar&val=1
This command set the camera displaying Color Bars.
To turn off Color Bars, simply send the following command:
				http://192.168.1.11/control?cmd=colorbar&val=0


The following commands are available:

framesize:		Set the resolution of the image (e.g: 1024x768)
				http://192.168.1.11/control?cmd=framesize&val=6
				
The following resolution are available:
	0 QQVGA (160x120)
	1 HQVGA (240x176)
	2 QVGA (320x240)
	3 CIF (400x296)
	4 VGA (640x480)
	5 SVGA (800x600)
	6 XGA (1024x768)
	7 SXGA (1280x1024)
	8 UXGA (1600x1200)

				
quality:		Set the video compression factor (e.g 12)
				http://192.168.1.11/control?cmd=quality&val=12
				
The values are between 10 and 63. High values means poor image quality.


brightness:		Set the brightness of the image (e.g. -2)
				http://192.168.1.11/control?cmd=brightness&val=-2

The values are int between -2 and +2. Darker to Lighter.


contrast:		Set the contrast of the image (e.g. 0)
				http://192.168.1.11/control?cmd=contrast&val=0

The values are int between -2 and +2. Minimum to Maximum.


saturation:		Set the saturation of the image (e.g. 2)
				http://192.168.1.11/control?cmd=saturation&val=2

The values are int between -2 and +2. 


special_effect:	Set a special effect for the image (e.g. Green Tint)
				http://192.168.1.11/control?cmd=special_effect&val=4

The following effects are available:
	0 No Effect
    	1 Negative
    	2 Grayscale
    	3 Red Tint
    	4 Green Tint
    	5 Blue Tint
    	6 Sepia

To be continued ...

TODO: Finalize the documentation ;-)
