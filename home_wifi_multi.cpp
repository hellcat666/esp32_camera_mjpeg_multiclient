#ifndef __HOME_WIFI_MULTI_H__
#include "home_wifi_multi.h"
#endif


AP::AP(String ssid, int channel, int rssi) {
  _ssid = ssid;
  _channel = channel;
  _rssi = rssi;
}

void AP::setSSID(String ssid) {
  _ssid = ssid;
}

String AP::getSSID() {
  return _ssid;
}

void AP::setChannel(int channel) {
  _channel = channel;
}

int AP::getChannel() {
  return _channel;
}

void AP::setRSSI(int rssi) {
  _rssi = rssi;
}
int AP::getRSSI() {
  return _rssi;
}
