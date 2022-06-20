#ifndef __HOME_WIFI_MULTI_H__
#define __HOME_WIFI_MULTI_H__
#include <Arduino.h>

class AP {
private:
  String _ssid;
  int _channel;
  int _rssi;

public:  
  AP(String ssid, int channel, int rssi);
  void setSSID(String ssid);
  String getSSID();
  void setChannel(int channel);
  int getChannel();
  void setRSSI(int rssi);
  int getRSSI();
  
};

typedef std::vector<AP> ListAPs;

#define SSID1 "********"
#define PWD1 "**********"
//#define SSID1 "chp-67443"
//#define PWD1 "67a3-7k3u-hahp-d333"

#endif
