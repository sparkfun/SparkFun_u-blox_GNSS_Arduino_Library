/*
  Arduino library for the u-blox SARA-R5 LTE-M / NB-IoT modules with secure cloud, as used on the SparkFun MicroMod Asset Tracker
  By: Paul Clark
  October 19th 2020

  Based extensively on the:
  Arduino Library for the SparkFun LTE CAT M1/NB-IoT Shield - SARA-R4
  Written by Jim Lindblom @ SparkFun Electronics, September 5, 2018

  This Arduino library provides mechanisms to initialize and use
  the SARA-R5 module over either a SoftwareSerial or hardware serial port.

  Please see LICENSE.md for the license information

*/

#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>

SARA_R5::SARA_R5(int powerPin, int resetPin, uint8_t maxInitTries)
{
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  _softSerial = NULL;
#endif
  _hardSerial = NULL;
  _baud = 0;
  _resetPin = resetPin;
  _powerPin = powerPin;
  _invertPowerPin = false;
  _maxInitTries = maxInitTries;
  _socketListenCallback = NULL;
  _socketReadCallback = NULL;
  _socketReadCallbackPlus = NULL;
  _socketCloseCallback = NULL;
  _gpsRequestCallback = NULL;
  _simStateReportCallback = NULL;
  _psdActionRequestCallback = NULL;
  _pingRequestCallback = NULL;
  _httpCommandRequestCallback = NULL;
  _mqttCommandRequestCallback = NULL;
  _registrationCallback = NULL;
  _epsRegistrationCallback = NULL;
  _debugAtPort = NULL;
  _debugPort = NULL;
  _printDebug = false;
  _lastRemoteIP = {0, 0, 0, 0};
  _lastLocalIP = {0, 0, 0, 0};
  for (int i = 0; i < SARA_R5_NUM_SOCKETS; i++)
    _lastSocketProtocol[i] = 0; // Set to zero initially. Will be set to TCP/UDP by socketOpen etc.
  _autoTimeZoneForBegin = true;
  _bufferedPollReentrant = false;
  _pollReentrant = false;
  _saraResponseBacklogLength = 0;
  _saraRXBuffer = NULL;
  _pruneBuffer = NULL;
  _saraResponseBacklog = NULL;
}

SARA_R5::~SARA_R5(void) {
  if (NULL != _saraRXBuffer) {
    delete[] _saraRXBuffer;
    _saraRXBuffer = NULL;
  }
  if (NULL != _pruneBuffer) {
    delete[] _pruneBuffer;
    _pruneBuffer = NULL;
  }
  if (NULL != _saraResponseBacklog) {
    delete[] _saraResponseBacklog;
    _saraResponseBacklog = NULL;
  }
}

#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
bool SARA_R5::begin(SoftwareSerial &softSerial, unsigned long baud)
{
  if (NULL == _saraRXBuffer)
  {
    _saraRXBuffer = new char[_RXBuffSize];
    if (NULL == _saraRXBuffer)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _saraRXBuffer!"));
      return false;
    }
  }
  memset(_saraRXBuffer, 0, _RXBuffSize);

  if (NULL == _pruneBuffer)
  {
    _pruneBuffer = new char[_RXBuffSize];
    if (NULL == _pruneBuffer)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _pruneBuffer!"));
      return false;
    }
  }
  memset(_pruneBuffer, 0, _RXBuffSize);
 
  if (NULL == _saraResponseBacklog)
  {
    _saraResponseBacklog = new char[_RXBuffSize];
    if (NULL == _saraResponseBacklog)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _saraResponseBacklog!"));
      return false;
    }
  }
  memset(_saraResponseBacklog, 0, _RXBuffSize);

  SARA_R5_error_t err;

  _softSerial = &softSerial;

  err = init(baud);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    return true;
  }
  return false;
}
#endif

bool SARA_R5::begin(HardwareSerial &hardSerial, unsigned long baud, bool doBegin)
{
  if (NULL == _saraRXBuffer)
  {
    _saraRXBuffer = new char[_RXBuffSize];
    if (NULL == _saraRXBuffer)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _saraRXBuffer!"));
      return false;
    }
  }
  memset(_saraRXBuffer, 0, _RXBuffSize);

  if (NULL == _pruneBuffer)
  {
    _pruneBuffer = new char[_RXBuffSize];
    if (NULL == _pruneBuffer)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _pruneBuffer!"));
      return false;
    }
  }
  memset(_pruneBuffer, 0, _RXBuffSize);
 
  if (NULL == _saraResponseBacklog)
  {
    _saraResponseBacklog = new char[_RXBuffSize];
    if (NULL == _saraResponseBacklog)
    {
      if (_printDebug == true)
        _debugPort->println(F("begin: not enough memory for _saraResponseBacklog!"));
      return false;
    }
  }
  memset(_saraResponseBacklog, 0, _RXBuffSize);

  SARA_R5_error_t err;

  _hardSerial = &hardSerial;
  if (doBegin) {
    _hardSerial->begin(baud);
  }
  err = init(baud);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    return true;
  }
  return false;
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void SARA_R5::enableDebugging(Print &debugPort)
{
  _debugPort = &debugPort;
  _printDebug = true;
}

//Calling this function with nothing sets the debug port to Serial
//You can also call it with other streams like Serial1, SerialUSB, etc.
void SARA_R5::enableAtDebugging(Print &debugPort)
{
  _debugAtPort = &debugPort;
  _printAtDebug = true;
}

// This function was originally written by Matthew Menze for the LTE Shield (SARA-R4) library
// See: https://github.com/sparkfun/SparkFun_LTE_Shield_Arduino_Library/pull/8
// It does the same job as ::poll but also processed any 'old' data stored in the backlog first
// It also has a built-in timeout - which ::poll does not
bool SARA_R5::bufferedPoll(void)
{
  if (_bufferedPollReentrant == true) // Check for reentry (i.e. bufferedPoll has been called from inside a callback)
    return false;

  _bufferedPollReentrant = true;

  int avail = 0;
  char c = 0;
  bool handled = false;
  unsigned long timeIn = millis();
  char *event;
  int backlogLen = _saraResponseBacklogLength;

  memset(_saraRXBuffer, 0, _RXBuffSize); // Clear _saraRXBuffer

  // Does the backlog contain any data? If it does, copy it into _saraRXBuffer and then clear the backlog
  if (_saraResponseBacklogLength > 0)
  {
    //The backlog also logs reads from other tasks like transmitting.
    if (_printDebug == true)
    {
      _debugPort->print(F("bufferedPoll: backlog found! backlogLen is "));
      _debugPort->println(_saraResponseBacklogLength);
    }
    memcpy(_saraRXBuffer + avail, _saraResponseBacklog, _saraResponseBacklogLength);
    avail += _saraResponseBacklogLength;
    memset(_saraResponseBacklog, 0, _RXBuffSize); // Clear the backlog making sure it is NULL-terminated
    _saraResponseBacklogLength = 0;
  }

  if ((hwAvailable() > 0) || (backlogLen > 0)) // If either new data is available, or backlog had data.
  {
    //Check for incoming serial data. Copy it into the backlog

    // Important note:
    // On ESP32, Serial.available only provides an update every ~120 bytes during the reception of long messages:
    // https://gitter.im/espressif/arduino-esp32?at=5e25d6370a1cf54144909c85
    // Be aware that if a long message is being received, the code below will timeout after _rxWindowMillis = 2 millis.
    // At 115200 baud, hwAvailable takes ~120 * 10 / 115200 = 10.4 millis before it indicates that data is being received.

    while (((millis() - timeIn) < _rxWindowMillis) && (avail < _RXBuffSize))
    {
      if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        c = readChar();
        // bufferedPoll is only interested in the URCs.
        // The URCs are all readable.
        // strtok does not like NULL characters.
        // So we need to make sure no NULL characters are added to _saraRXBuffer
        if (c == '\0')
          c = '0'; // Convert any NULLs to ASCII Zeros
        _saraRXBuffer[avail++] = c;
        timeIn = millis();
      } else {
        yield();
      }
    }

    // _saraRXBuffer now contains the backlog (if any) and the new serial data (if any)

    // A health warning about strtok:
    // strtok will convert any delimiters it finds ("\r\n" in our case) into NULL characters.
    // Also, be very careful that you do not use strtok within an strtok while loop.
    // The next call of strtok(NULL, ...) in the outer loop will use the pointer saved from the inner loop!
    // In our case, strtok is also used in pruneBacklog, which is called by waitForRespone or sendCommandWithResponse,
    // which is called by the parse functions called by processURCEvent...
    // The solution is to use strtok_r - the reentrant version of strtok

    char *preservedEvent;
    event = strtok_r(_saraRXBuffer, "\r\n", &preservedEvent); // Look for an 'event' (_saraRXBuffer contains something ending in \r\n)

    if (event != NULL)
      if (_printDebug == true)
        _debugPort->println(F("bufferedPoll: event(s) found! ===>"));

    while (event != NULL) // Keep going until all events have been processed
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("bufferedPoll: start of event: "));
        _debugPort->println(event);
      }

      //Process the event
      bool latestHandled = processURCEvent((const char *)event);
      if (latestHandled) {
        if ((true == _printAtDebug) && (NULL != event)) {
          _debugAtPort->print(event);
        }
        handled = true; // handled will be true if latestHandled has ever been true
      }
      if ((_saraResponseBacklogLength > 0) && ((avail + _saraResponseBacklogLength) < _RXBuffSize)) // Has any new data been added to the backlog?
      {
        if (_printDebug == true)
        {
          _debugPort->println(F("bufferedPoll: new backlog added!"));
        }
        memcpy(_saraRXBuffer + avail, _saraResponseBacklog, _saraResponseBacklogLength);
        avail += _saraResponseBacklogLength;
        memset(_saraResponseBacklog, 0, _RXBuffSize); //Clear out the backlog buffer again.
        _saraResponseBacklogLength = 0;
      }

      //Walk through any remaining events
      event = strtok_r(NULL, "\r\n", &preservedEvent);

      if (_printDebug == true)
        _debugPort->println(F("bufferedPoll: end of event")); //Just to denote end of processing event.

      if (event == NULL)
        if (_printDebug == true)
          _debugPort->println(F("bufferedPoll: <=== end of event(s)!"));
    }
  }

  free(event);

  _bufferedPollReentrant = false;

  return handled;
} // /bufferedPoll

// Parse incoming URC's - the associated parse functions pass the data to the user via the callbacks (if defined)
bool SARA_R5::processURCEvent(const char *event)
{
  { // URC: +UUSORD (Read Socket Data)
    int socket, length;
    int ret = sscanf(event, "+UUSORD: %d,%d", &socket, &length);
    if (ret == 2)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: read socket data"));
      // From the SARA_R5 AT Commands Manual:
      // "For the UDP socket type the URC +UUSORD: <socket>,<length> notifies that a UDP packet has been received,
      //  either when buffer is empty or after a UDP packet has been read and one or more packets are stored in the
      //  buffer."
      // So we need to check if this is a TCP socket or a UDP socket:
      //  If UDP, we call parseSocketReadIndicationUDP.
      //  Otherwise, we call parseSocketReadIndication.
      if (_lastSocketProtocol[socket] == SARA_R5_UDP)
      {
        if (_printDebug == true)
          _debugPort->println(F("processReadEvent: received +UUSORD but socket is UDP. Calling parseSocketReadIndicationUDP"));
        parseSocketReadIndicationUDP(socket, length);
      }
      else
        parseSocketReadIndication(socket, length);
      return true;
    }
  }
  { // URC: +UUSORF (Receive From command (UDP only))
    int socket, length;
    int ret = sscanf(event, "+UUSORF: %d,%d", &socket, &length);
    if (ret == 2)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: UDP receive"));
      parseSocketReadIndicationUDP(socket, length);
      return true;
    }
  }
  { // URC: +UUSOLI (Set Listening Socket)
    int socket = 0;
    int listenSocket = 0;
    unsigned int port = 0;
    unsigned int listenPort = 0;
    IPAddress remoteIP = {0,0,0,0};
    IPAddress localIP = {0,0,0,0};
    int remoteIPstore[4]  = {0,0,0,0};
    int localIPstore[4] = {0,0,0,0};

    int ret = sscanf(event,
                     "+UUSOLI: %d,\"%d.%d.%d.%d\",%u,%d,\"%d.%d.%d.%d\",%u",
                     &socket,
                     &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3],
                     &port, &listenSocket,
                     &localIPstore[0], &localIPstore[1], &localIPstore[2], &localIPstore[3],
                     &listenPort);
    for (int i = 0; i <= 3; i++)
    {
      if (ret >= 5)
        remoteIP[i] = (uint8_t)remoteIPstore[i];
      if (ret >= 11)
        localIP[i] = (uint8_t)localIPstore[i];
    }
    if (ret >= 5)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: socket listen"));
      parseSocketListenIndication(listenSocket, localIP, listenPort, socket, remoteIP, port);
      return true;
    }
  }
  { // URC: +UUSOCL (Close Socket)
    int socket;
    int ret = sscanf(event, "+UUSOCL: %d", &socket);
    if (ret == 1)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: socket close"));
      if ((socket >= 0) && (socket <= 6))
      {
        if (_socketCloseCallback != NULL)
        {
          _socketCloseCallback(socket);
        }
      }
      return true;
    }
  }
  { // URC: +UULOC (Localization information - CellLocate and hybrid positioning)
    ClockData clck;
    PositionData gps;
    SpeedData spd;
    unsigned long uncertainty;
    int scanNum;
    int latH, lonH, alt;
    unsigned int speedU, cogU;
    char latL[10], lonL[10];
    int dateStore[5];

    // Maybe we should also scan for +UUGIND and extract the activated gnss system?

    // This assumes the ULOC response type is "0" or "1" - as selected by gpsRequest detailed
    scanNum = sscanf(event,
                      "+UULOC: %d/%d/%d,%d:%d:%d.%d,%d.%[^,],%d.%[^,],%d,%lu,%u,%u,%*s",
                      &dateStore[0], &dateStore[1], &clck.date.year,
                      &dateStore[2], &dateStore[3], &dateStore[4], &clck.time.ms,
                      &latH, latL, &lonH, lonL, &alt, &uncertainty,
                      &speedU, &cogU);
    clck.date.day = dateStore[0];
    clck.date.month = dateStore[1];
    clck.time.hour = dateStore[2];
    clck.time.minute = dateStore[3];
    clck.time.second = dateStore[4];

    if (scanNum >= 13)
    {
      // Found a Location string!
      if (_printDebug == true)
      {
        _debugPort->println(F("processReadEvent: location"));
      }

      if (latH >= 0)
        gps.lat = (float)latH + ((float)atol(latL) / pow(10, strlen(latL)));
      else
        gps.lat = (float)latH - ((float)atol(latL) / pow(10, strlen(latL)));
      if (lonH >= 0)
        gps.lon = (float)lonH + ((float)atol(lonL) / pow(10, strlen(lonL)));
      else
        gps.lon = (float)lonH - ((float)atol(lonL) / pow(10, strlen(lonL)));
      gps.alt = (float)alt;
      if (scanNum >= 15) // If detailed response, get speed data
      {
        spd.speed = (float)speedU;
        spd.cog = (float)cogU;
      }

      // if (_printDebug == true)
      // {
      //   _debugPort->print(F("processReadEvent: location:  lat: "));
      //   _debugPort->print(gps.lat, 7);
      //   _debugPort->print(F(" lon: "));
      //   _debugPort->print(gps.lon, 7);
      //   _debugPort->print(F(" alt: "));
      //   _debugPort->print(gps.alt, 2);
      //   _debugPort->print(F(" speed: "));
      //   _debugPort->print(spd.speed, 2);
      //   _debugPort->print(F(" cog: "));
      //   _debugPort->println(spd.cog, 2);
      // }

      if (_gpsRequestCallback != NULL)
      {
        _gpsRequestCallback(clck, gps, spd, uncertainty);
      }

      return true;
    }
  }
  { // URC: +UUSIMSTAT (SIM Status)
    SARA_R5_sim_states_t state;
    int scanNum;
    int stateStore;

    scanNum = sscanf(event, "+UUSIMSTAT:%d", &stateStore); // Note: no space after the colon!

    if (scanNum == 1)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: SIM status"));

      state = (SARA_R5_sim_states_t)stateStore;

      if (_simStateReportCallback != NULL)
      {
        _simStateReportCallback(state);
      }

      return true;
    }
  }
  { // URC: +UUPSDA (Packet Switched Data Action)
    int result;
    IPAddress remoteIP = {0, 0, 0, 0};
    int scanNum;
    int remoteIPstore[4];

    scanNum = sscanf(event, "+UUPSDA: %d,\"%d.%d.%d.%d\"",
                      &result, &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3]);

    if (scanNum == 5)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: packet switched data action"));

      for (int i = 0; i <= 3; i++)
      {
        remoteIP[i] = (uint8_t)remoteIPstore[i];
      }

      if (_psdActionRequestCallback != NULL)
      {
        _psdActionRequestCallback(result, remoteIP);
      }

      return true;
    }
  }
  { // URC: +UUHTTPCR (HTTP Command Result)
    int profile, command, result;
    int scanNum;

    scanNum = sscanf(event, "+UUHTTPCR: %d,%d,%d", &profile, &command, &result);

    if (scanNum == 3)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: HTTP command result"));

      if ((profile >= 0) && (profile < SARA_R5_NUM_HTTP_PROFILES))
      {
        if (_httpCommandRequestCallback != NULL)
        {
          _httpCommandRequestCallback(profile, command, result);
        }
      }

      return true;
    }
  }
  { // URC: +UUMQTTC (HTTP Command Result)
      int command, result;
      int scanNum;
      int qos = -1;
      String topic;
      scanNum = sscanf(event, "+UUMQTTC: %d,%d", &command, &result);
      if ((scanNum == 2) && (command == SARA_R5_MQTT_COMMAND_SUBSCRIBE)) {
        char topicC[100] = "";
        scanNum = sscanf(event, "+UUMQTTC: %*d,%*d,%d,\"%[^\"]\"", &qos, topicC);
        topic = topicC;
      }
      if ((scanNum == 2) || (scanNum == 4))
      {
        if (_printDebug == true)
          _debugPort->println(F("processReadEvent: MQTT command result"));

        if (_mqttCommandRequestCallback != NULL)
        {
          _mqttCommandRequestCallback(command, result);
        }
        
        return true;
      }
  }
  { // URC: +UUPING (Ping Result)
    int retry = 0;
    int p_size = 0;
    int ttl = 0;
    String remote_host = "";
    IPAddress remoteIP = {0, 0, 0, 0};
    long rtt = 0;
    int scanNum;
    const char *searchPtr = event;

    // Try to extract the UUPING retries and payload size
    scanNum = sscanf(searchPtr, "+UUPING: %d,%d,", &retry, &p_size);

    if (scanNum == 2)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("processReadEvent: ping"));
      }

      searchPtr = strchr(++searchPtr, '\"'); // Search to the first quote

      // Extract the remote host name, stop at the next quote
      while ((*(++searchPtr) != '\"') && (*searchPtr != '\0'))
      {
        remote_host.concat(*(searchPtr));
      }

      if (*searchPtr != '\0') // Make sure we found a quote
      {
        int remoteIPstore[4];
        scanNum = sscanf(searchPtr, "\",\"%d.%d.%d.%d\",%d,%ld",
                          &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3], &ttl, &rtt);
        for (int i = 0; i <= 3; i++)
        {
          remoteIP[i] = (uint8_t)remoteIPstore[i];
        }

        if (scanNum == 6) // Make sure we extracted enough data
        {
          if (_pingRequestCallback != NULL)
          {
            _pingRequestCallback(retry, p_size, remote_host, remoteIP, ttl, rtt);
          }
        }
      }
      return true;
    }
  }
  { // URC: +A
    int status = 0;
    unsigned int lac = 0, ci = 0, Act = 0;
    int scanNum = sscanf(event, "+CREG:  %d,\"%4x\",\"%4x\",%d", &status, &lac, &ci, &Act);
    if (scanNum == 4)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: CREG"));

      if (_registrationCallback != NULL)
      {
        _registrationCallback((SARA_R5_registration_status_t)status, lac, ci, Act);
      }
      
      return true;
    }
  }
  { // URC: +CEREG
    int status = 0;
    unsigned int tac = 0, ci = 0, Act = 0;
    int scanNum = sscanf(event, "+CEREG: %d,\"%4x\",\"%4x\",%d", &status, &tac, &ci, &Act);
    if (scanNum == 4)
    {
      if (_printDebug == true)
        _debugPort->println(F("processReadEvent: CEREG"));

      if (_epsRegistrationCallback != NULL)
      {
        _epsRegistrationCallback((SARA_R5_registration_status_t)status, tac, ci, Act);
      }
      
      return true;
    }
  }
  
  return false;
}

// This is the original poll function.
// It is 'blocking' - it does not return when serial data is available until it receives a `\n`.
// ::bufferedPoll is the new improved version. It processes any data in the backlog and includes a timeout.
bool SARA_R5::poll(void)
{
  if (_pollReentrant == true) // Check for reentry (i.e. poll has been called from inside a callback)
    return false;

  _pollReentrant = true;

  int avail = 0;
  char c = 0;
  bool handled = false;

  memset(_saraRXBuffer, 0, _RXBuffSize); // Clear _saraRXBuffer

  if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
  {
    while (c != '\n') // Copy characters into _saraRXBuffer. Stop at the first new line
    {
      if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        c = readChar();
        _saraRXBuffer[avail++] = c;
      } else {
        yield();
      }
    }

    // Now search for all supported URC's
    handled = processURCEvent(_saraRXBuffer);
    if (handled && (true == _printAtDebug)) {
      _debugAtPort->write(_saraRXBuffer, avail);
    }
    if ((handled == false) && (strlen(_saraRXBuffer) > 2))
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("poll: "));
        _debugPort->println(_saraRXBuffer);
      }
    }
    else
    {
    }
  }

  _pollReentrant = false;

  return handled;
}

void SARA_R5::setSocketListenCallback(void (*socketListenCallback)(int, IPAddress, unsigned int, int, IPAddress, unsigned int))
{
  _socketListenCallback = socketListenCallback;
}

void SARA_R5::setSocketReadCallback(void (*socketReadCallback)(int, String))
{
  _socketReadCallback = socketReadCallback;
}

void SARA_R5::setSocketReadCallbackPlus(void (*socketReadCallbackPlus)(int, const char *, int, IPAddress, int)) // socket, data, length, remoteAddress, remotePort
{
  _socketReadCallbackPlus = socketReadCallbackPlus;
}

void SARA_R5::setSocketCloseCallback(void (*socketCloseCallback)(int))
{
  _socketCloseCallback = socketCloseCallback;
}

void SARA_R5::setGpsReadCallback(void (*gpsRequestCallback)(ClockData time,
                                                            PositionData gps, SpeedData spd, unsigned long uncertainty))
{
  _gpsRequestCallback = gpsRequestCallback;
}

void SARA_R5::setSIMstateReportCallback(void (*simStateReportCallback)(SARA_R5_sim_states_t state))
{
  _simStateReportCallback = simStateReportCallback;
}

void SARA_R5::setPSDActionCallback(void (*psdActionRequestCallback)(int result, IPAddress ip))
{
  _psdActionRequestCallback = psdActionRequestCallback;
}

void SARA_R5::setPingCallback(void (*pingRequestCallback)(int retry, int p_size, String remote_hostname, IPAddress ip, int ttl, long rtt))
{
  _pingRequestCallback = pingRequestCallback;
}

void SARA_R5::setHTTPCommandCallback(void (*httpCommandRequestCallback)(int profile, int command, int result))
{
  _httpCommandRequestCallback = httpCommandRequestCallback;
}

void SARA_R5::setMQTTCommandCallback(void (*mqttCommandRequestCallback)(int command, int result))
{
  _mqttCommandRequestCallback = mqttCommandRequestCallback;
}

SARA_R5_error_t SARA_R5::setRegistrationCallback(void (*registrationCallback)(SARA_R5_registration_status_t status, unsigned int lac, unsigned int ci, int Act))
{
  _registrationCallback = registrationCallback;
  
  char *command = sara_r5_calloc_char(strlen(SARA_R5_REGISTRATION_STATUS) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_REGISTRATION_STATUS, 2/*enable URC with location*/);
  SARA_R5_error_t err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setEpsRegistrationCallback(void (*registrationCallback)(SARA_R5_registration_status_t status, unsigned int tac, unsigned int ci, int Act))
{
  _epsRegistrationCallback = registrationCallback;

  char *command = sara_r5_calloc_char(strlen(SARA_R5_EPSREGISTRATION_STATUS) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_EPSREGISTRATION_STATUS, 2/*enable URC with location*/);
  SARA_R5_error_t err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

size_t SARA_R5::write(uint8_t c)
{
  return hwWrite(c);
}

size_t SARA_R5::write(const char *str)
{
  return hwPrint(str);
}

size_t SARA_R5::write(const char *buffer, size_t size)
{
  return hwWriteData(buffer, size);
}

SARA_R5_error_t SARA_R5::at(void)
{
  SARA_R5_error_t err;
  
  err = sendCommandWithResponse(NULL, SARA_R5_RESPONSE_OK, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  return err;
}

SARA_R5_error_t SARA_R5::enableEcho(bool enable)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_ECHO) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s%d", SARA_R5_COMMAND_ECHO, enable ? 1 : 0);
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

String SARA_R5::getManufacturerID(void)
{
  char *response;
  char idResponse[16] = {0x00}; // E.g. u-blox
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_MANU_ID,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", idResponse) != 1)
    {
      memset(idResponse, 0, 16);
    }
  }
  free(response);
  return String(idResponse);
}

String SARA_R5::getModelID(void)
{
  char *response;
  char idResponse[16] = {0x00}; // E.g. SARA-R510M8Q
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_MODEL_ID,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", idResponse) != 1)
    {
      memset(idResponse, 0, 16);
    }
  }
  free(response);
  return String(idResponse);
}

String SARA_R5::getFirmwareVersion(void)
{
  char *response;
  char idResponse[16] = {0x00}; // E.g. 11.40
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_FW_VER_ID,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", idResponse) != 1)
    {
      memset(idResponse, 0, 16);
    }
  }
  free(response);
  return String(idResponse);
}

String SARA_R5::getSerialNo(void)
{
  char *response;
  char idResponse[16] = {0x00}; // E.g. 357520070120767
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_SERIAL_NO,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", idResponse) != 1)
    {
      memset(idResponse, 0, 16);
    }
  }
  free(response);
  return String(idResponse);
}

String SARA_R5::getIMEI(void)
{
  char *response;
  char imeiResponse[16] = {0x00}; // E.g. 004999010640000
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_IMEI,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", imeiResponse) != 1)
    {
      memset(imeiResponse, 0, 16);
    }
  }
  free(response);
  return String(imeiResponse);
}

String SARA_R5::getIMSI(void)
{
  char *response;
  char imsiResponse[16] = {0x00}; // E.g. 222107701772423
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_IMSI,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n%s\r\n", imsiResponse) != 1)
    {
      memset(imsiResponse, 0, 16);
    }
  }
  free(response);
  return String(imsiResponse);
}

String SARA_R5::getCCID(void)
{
  char *response;
  char ccidResponse[21] = {0x00}; // E.g. +CCID: 8939107900010087330
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_CCID,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n+CCID: %s", ccidResponse) != 1)
    {
      memset(ccidResponse, 0, 21);
    }
  }
  free(response);
  return String(ccidResponse);
}

String SARA_R5::getSubscriberNo(void)
{
  char *response;
  char idResponse[128] = {0x00}; // E.g. +CNUM: "ABCD . AAA","123456789012",129
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_CNUM,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_10_SEC_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n+CNUM: %s", idResponse) != 1)
    {
      memset(idResponse, 0, 128);
    }
  }
  free(response);
  return String(idResponse);
}

String SARA_R5::getCapabilities(void)
{
  char *response;
  char idResponse[128] = {0x00}; // E.g. +GCAP: +FCLASS, +CGSM
  SARA_R5_error_t err;

  response = sara_r5_calloc_char(minimumResponseAllocation);

  err = sendCommandWithResponse(SARA_R5_COMMAND_REQ_CAP,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (sscanf(response, "\r\n+GCAP: %s", idResponse) != 1)
    {
      memset(idResponse, 0, 128);
    }
  }
  free(response);
  return String(idResponse);
}

SARA_R5_error_t SARA_R5::reset(void)
{
  SARA_R5_error_t err;

  err = functionality(SILENT_RESET_WITH_SIM);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    // Reset will set the baud rate back to 115200
    //beginSerial(9600);
    err = SARA_R5_ERROR_INVALID;
    while (err != SARA_R5_ERROR_SUCCESS)
    {
      beginSerial(SARA_R5_DEFAULT_BAUD_RATE);
      setBaud(_baud);
      beginSerial(_baud);
      err = at();
    }
    return init(_baud);
  }
  return err;
}

String SARA_R5::clock(void)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  char *clockBegin;
  char *clockEnd;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_CLOCK) + 2);
  if (command == NULL)
    return "";
  sprintf(command, "%s?", SARA_R5_COMMAND_CLOCK);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return "";
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return "";
  }

  // Response format: \r\n+CCLK: "YY/MM/DD,HH:MM:SS-TZ"\r\n\r\nOK\r\n
  clockBegin = strchr(response, '\"'); // Find first quote
  if (clockBegin == NULL)
  {
    free(command);
    free(response);
    return "";
  }
  clockBegin += 1;                     // Increment pointer to begin at first number
  clockEnd = strchr(clockBegin, '\"'); // Find last quote
  if (clockEnd == NULL)
  {
    free(command);
    free(response);
    return "";
  }
  *(clockEnd) = '\0'; // Set last quote to null char -- end string

  String clock = String(clockBegin); // Extract the clock as a String _before_ freeing response

  free(command);
  free(response);

  return (clock);
}

SARA_R5_error_t SARA_R5::clock(uint8_t *y, uint8_t *mo, uint8_t *d,
                               uint8_t *h, uint8_t *min, uint8_t *s, int8_t *tz)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  char tzPlusMinus;
  int scanNum = 0;

  int iy, imo, id, ih, imin, is, itz;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_CLOCK) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_COMMAND_CLOCK);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  // Response format (if TZ is negative): \r\n+CCLK: "YY/MM/DD,HH:MM:SS-TZ"\r\n\r\nOK\r\n
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+CCLK: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+CCLK: \"%d/%d/%d,%d:%d:%d%c%d\"\r\n",
               &iy, &imo, &id, &ih, &imin, &is, &tzPlusMinus, &itz);
    if (scanNum == 8)
    {
      *y = iy;
      *mo = imo;
      *d = id;
      *h = ih;
      *min = imin;
      *s = is;
      if (tzPlusMinus == '-')
        *tz = 0 - itz;
      else
        *tz = itz;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::setClock(uint8_t y, uint8_t mo, uint8_t d,
                                  uint8_t h, uint8_t min, uint8_t s, int8_t tz)
{
  //Convert y,mo,d,h,min,s,tz into a String
  //Some platforms don't support sprintf correctly (for %02d or %+02d) so we need to build the String manually
  //Format is "yy/MM/dd,hh:mm:ss+TZ"
  //TZ can be +/- and is in increments of 15 minutes (not hours)

  String theTime = "";

  theTime.concat(y / 10);
  theTime.concat(y % 10);
  theTime.concat('/');
  theTime.concat(mo / 10);
  theTime.concat(mo % 10);
  theTime.concat('/');
  theTime.concat(d / 10);
  theTime.concat(d % 10);
  theTime.concat(',');
  theTime.concat(h / 10);
  theTime.concat(h % 10);
  theTime.concat(':');
  theTime.concat(min / 10);
  theTime.concat(min % 10);
  theTime.concat(':');
  theTime.concat(s / 10);
  theTime.concat(s % 10);
  if (tz < 0)
  {
    theTime.concat('-');
    tz = 0 - tz;
  }
  else
    theTime.concat('+');
  theTime.concat(tz / 10);
  theTime.concat(tz % 10);

  return (setClock(theTime));
}

SARA_R5_error_t SARA_R5::setClock(String theTime)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_CLOCK) + theTime.length() + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_COMMAND_CLOCK, theTime.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

void SARA_R5::autoTimeZoneForBegin(bool tz)
{
  _autoTimeZoneForBegin = tz;
}

SARA_R5_error_t SARA_R5::setUtimeMode(SARA_R5_utime_mode_t mode, SARA_R5_utime_sensor_t sensor)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_REQUEST_TIME) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (mode == SARA_R5_UTIME_MODE_STOP) // stop UTIME does not require a sensor
    sprintf(command, "%s=%d", SARA_R5_GNSS_REQUEST_TIME, mode);
  else
    sprintf(command, "%s=%d,%d", SARA_R5_GNSS_REQUEST_TIME, mode, sensor);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_10_SEC_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getUtimeMode(SARA_R5_utime_mode_t *mode, SARA_R5_utime_sensor_t *sensor)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  SARA_R5_utime_mode_t m;
  SARA_R5_utime_sensor_t s;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_REQUEST_TIME) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_GNSS_REQUEST_TIME);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_10_SEC_TIMEOUT);

  // Response format: \r\n+UTIME: <mode>[,<sensor>]\r\n\r\nOK\r\n
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int mStore, sStore, scanned = 0;
    char *searchPtr = strstr(response, "+UTIME: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+UTIME: %d,%d\r\n", &mStore, &sStore);
    m = (SARA_R5_utime_mode_t)mStore;
    s = (SARA_R5_utime_sensor_t)sStore;
    if (scanned == 2)
    {
      *mode = m;
      *sensor = s;
    }
    else if (scanned == 1)
    {
      *mode = m;
      *sensor = SARA_R5_UTIME_SENSOR_NONE;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::setUtimeIndication(SARA_R5_utime_urc_configuration_t config)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_TIME_INDICATION) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_GNSS_TIME_INDICATION, config);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getUtimeIndication(SARA_R5_utime_urc_configuration_t *config)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  SARA_R5_utime_urc_configuration_t c;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_TIME_INDICATION) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_GNSS_TIME_INDICATION);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  // Response format: \r\n+UTIMEIND: <mode>\r\n\r\nOK\r\n
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int cStore, scanned = 0;
    char *searchPtr = strstr(response, "+UTIMEIND: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+UTIMEIND: %d\r\n", &cStore);
    c = (SARA_R5_utime_urc_configuration_t)cStore;
    if (scanned == 1)
    {
      *config = c;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::setUtimeConfiguration(int32_t offsetNanoseconds, int32_t offsetSeconds)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_TIME_CONFIGURATION) + 48);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s=%d,%d", SARA_R5_GNSS_TIME_CONFIGURATION, offsetNanoseconds, offsetSeconds);
#else
  sprintf(command, "%s=%ld,%ld", SARA_R5_GNSS_TIME_CONFIGURATION, offsetNanoseconds, offsetSeconds);
#endif

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getUtimeConfiguration(int32_t *offsetNanoseconds, int32_t *offsetSeconds)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  int32_t ons;
  int32_t os;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_TIME_CONFIGURATION) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_GNSS_TIME_CONFIGURATION);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  // Response format: \r\n+UTIMECFG: <offset_nano>,<offset_sec>\r\n\r\nOK\r\n
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int scanned = 0;
    char *searchPtr = strstr(response, "+UTIMECFG: ");
    if (searchPtr != NULL)
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
      scanned = sscanf(searchPtr, "+UTIMECFG: %d,%d\r\n", &ons, &os);
#else
      scanned = sscanf(searchPtr, "+UTIMECFG: %ld,%ld\r\n", &ons, &os);
#endif
    if (scanned == 2)
    {
      *offsetNanoseconds = ons;
      *offsetSeconds = os;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::autoTimeZone(bool enable)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_AUTO_TZ) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_COMMAND_AUTO_TZ, enable ? 1 : 0);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

int8_t SARA_R5::rssi(void)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int rssi;

  command = sara_r5_calloc_char(strlen(SARA_R5_SIGNAL_QUALITY) + 1);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s", SARA_R5_SIGNAL_QUALITY);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command,
                                SARA_R5_RESPONSE_OK_OR_ERROR, response, 10000,
                                minimumResponseAllocation, AT_COMMAND);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return -1;
  }

  int scanned = 0;
  char *searchPtr = strstr(response, "+CSQ: ");
  if (searchPtr != NULL)
    scanned = sscanf(searchPtr, "+CSQ: %d,%*d", &rssi);
  if (scanned != 1)
  {
    rssi = -1;
  }

  free(command);
  free(response);
  return rssi;
}

SARA_R5_registration_status_t SARA_R5::registration(bool eps)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int status;
  const char* tag = eps ? SARA_R5_EPSREGISTRATION_STATUS : SARA_R5_REGISTRATION_STATUS;
  command = sara_r5_calloc_char(strlen(tag) + 3);
  if (command == NULL)
    return SARA_R5_REGISTRATION_INVALID;
  sprintf(command, "%s?", tag);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_REGISTRATION_INVALID;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT,
                                minimumResponseAllocation, AT_COMMAND);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return SARA_R5_REGISTRATION_INVALID;
  }
  
  int scanned = 0;
  const char *startTag = eps ? "+CEREG: " : "+CREG: ";
  char *searchPtr = strstr(response, startTag);
  if (searchPtr != NULL) {
	  const char *format = eps ? "+CEREG: %*d,%d" : "+CREG: %*d,%d";
	  scanned = sscanf(searchPtr, format, &status);
  }
  if (scanned != 1)
    status = SARA_R5_REGISTRATION_INVALID;

  free(command);
  free(response);
  return (SARA_R5_registration_status_t)status;
}

bool SARA_R5::setNetworkProfile(mobile_network_operator_t mno, bool autoReset, bool urcNotification)
{
  mobile_network_operator_t currentMno;

  // Check currently set MNO profile
  if (getMNOprofile(&currentMno) != SARA_R5_ERROR_SUCCESS)
  {
    return false;
  }

  if (currentMno == mno)
  {
    return true;
  }

  // Disable transmit and receive so we can change operator
  if (functionality(MINIMUM_FUNCTIONALITY) != SARA_R5_ERROR_SUCCESS)
  {
    return false;
  }

  if (setMNOprofile(mno, autoReset, urcNotification) != SARA_R5_ERROR_SUCCESS)
  {
    return false;
  }

  if (reset() != SARA_R5_ERROR_SUCCESS)
  {
    return false;
  }

  return true;
}

mobile_network_operator_t SARA_R5::getNetworkProfile(void)
{
  mobile_network_operator_t mno;
  SARA_R5_error_t err;

  err = getMNOprofile(&mno);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    return MNO_INVALID;
  }
  return mno;
}

SARA_R5_error_t SARA_R5::setAPN(String apn, uint8_t cid, SARA_R5_pdp_type pdpType)
{
  SARA_R5_error_t err;
  char *command;
  char pdpStr[8];

  memset(pdpStr, 0, 8);

  if (cid >= 8)
    return SARA_R5_ERROR_UNEXPECTED_PARAM;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_DEF) + strlen(apn.c_str()) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  switch (pdpType)
  {
  case PDP_TYPE_INVALID:
    free(command);
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
    break;
  case PDP_TYPE_IP:
    memcpy(pdpStr, "IP", 2);
    break;
  case PDP_TYPE_NONIP:
    memcpy(pdpStr, "NONIP", 5);
    break;
  case PDP_TYPE_IPV4V6:
    memcpy(pdpStr, "IPV4V6", 6);
    break;
  case PDP_TYPE_IPV6:
    memcpy(pdpStr, "IPV6", 4);
    break;
  default:
    free(command);
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
    break;
  }
  if (apn == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("setAPN: NULL"));
    sprintf(command, "%s=%d,\"%s\",\"\"", SARA_R5_MESSAGE_PDP_DEF,
            cid, pdpStr);
  }
  else
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("setAPN: "));
      _debugPort->println(apn);
    }
    sprintf(command, "%s=%d,\"%s\",\"%s\"", SARA_R5_MESSAGE_PDP_DEF,
            cid, pdpStr, apn.c_str());
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);

  return err;
}

// Return the Access Point Name and IP address for the chosen context identifier
SARA_R5_error_t SARA_R5::getAPN(int cid, String *apn, IPAddress *ip, SARA_R5_pdp_type* pdpType)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  
  if (cid > SARA_R5_NUM_PDP_CONTEXT_IDENTIFIERS)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_DEF) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_MESSAGE_PDP_DEF);

  response = sara_r5_calloc_char(1024);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT, 1024);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    // Example:
    // +CGDCONT: 0,"IP","payandgo.o2.co.uk","0.0.0.0",0,0,0,0,0,0,0,0,0,0
    // +CGDCONT: 1,"IP","payandgo.o2.co.uk.mnc010.mcc234.gprs","10.160.182.234",0,0,0,2,0,0,0,0,0,0
	int rcid = -1;
    char *searchPtr = response;

    bool keepGoing = true;
    while (keepGoing == true)
    {
      int scanned = 0;
      // Find the first/next occurrence of +CGDCONT:
      searchPtr = strstr(searchPtr, "+CGDCONT: ");
      if (searchPtr != NULL)
      {
        char strPdpType[10];
        char strApn[128];
        int ipOct[4];
		
        searchPtr += strlen("+CGDCONT: "); // Point to the cid
        scanned = sscanf(searchPtr, "%d,\"%[^\"]\",\"%[^\"]\",\"%d.%d.%d.%d",
        &rcid, strPdpType, strApn,
				&ipOct[0], &ipOct[1], &ipOct[2], &ipOct[3]);
        if ((scanned == 7) && (rcid == cid)) {
          if (apn) *apn = strApn;
          for (int o = 0; ip && (o < 4); o++)
          {
            (*ip)[o] = (uint8_t)ipOct[o];
          }
          if (pdpType) {
            *pdpType = (0 == strcmp(strPdpType, "IPV4V6"))  ? PDP_TYPE_IPV4V6 :
                       (0 == strcmp(strPdpType, "IPV6"))    ? PDP_TYPE_IPV6 :
                       (0 == strcmp(strPdpType, "IP"))	    ? PDP_TYPE_IP :
                                                              PDP_TYPE_INVALID;
          }
          keepGoing = false;
        }
      }
	  else // We don't have a match so let's clear the APN and IP address
      {
	    if (apn) *apn = "";
		if (pdpType) *pdpType = PDP_TYPE_INVALID;
		if (ip) *ip = {0, 0, 0, 0};
		keepGoing = false;
	  }  
    }
  }
  else
  {
    err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::getSimStatus(String* code)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_SIMPIN) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_COMMAND_SIMPIN);
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int scanned = 0;
    char c[16];
    char *searchPtr = strstr(response, "+CPIN: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+CPIN: %s\r\n", c);
    if (scanned == 1)
    {
      if(code)
        *code = c;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }
  
  free(command);
  free(response);
  
  return err;
}
                                
SARA_R5_error_t SARA_R5::setSimPin(String pin)
{
  SARA_R5_error_t err;
  char *command;
  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_SIMPIN) + 4 + pin.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_COMMAND_SIMPIN, pin.c_str());
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setSIMstateReportingMode(int mode)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_SIM_STATE) + 4);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_SIM_STATE, mode);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getSIMstateReportingMode(int *mode)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  int m;

  command = sara_r5_calloc_char(strlen(SARA_R5_SIM_STATE) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_SIM_STATE);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int scanned = 0;
    char *searchPtr = strstr(response, "+USIMSTAT: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+USIMSTAT: %d\r\n", &m);
    if (scanned == 1)
    {
      *mode = m;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

const char *PPP_L2P[5] = {
    "",
    "PPP",
    "M-HEX",
    "M-RAW_IP",
    "M-OPT-PPP",
};

SARA_R5_error_t SARA_R5::enterPPP(uint8_t cid, char dialing_type_char,
                                  unsigned long dialNumber, SARA_R5::SARA_R5_l2p_t l2p)
{
  SARA_R5_error_t err;
  char *command;

  if ((dialing_type_char != 0) && (dialing_type_char != 'T') &&
      (dialing_type_char != 'P'))
  {
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
  }

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_ENTER_PPP) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (dialing_type_char != 0)
  {
    sprintf(command, "%s%c*%lu**%s*%u#", SARA_R5_MESSAGE_ENTER_PPP, dialing_type_char,
            dialNumber, PPP_L2P[l2p], (unsigned int)cid);
  }
  else
  {
    sprintf(command, "%s*%lu**%s*%u#", SARA_R5_MESSAGE_ENTER_PPP,
            dialNumber, PPP_L2P[l2p], (unsigned int)cid);
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_CONNECT, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

uint8_t SARA_R5::getOperators(struct operator_stats *opRet, int maxOps)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  uint8_t opsSeen = 0;

  command = sara_r5_calloc_char(strlen(SARA_R5_OPERATOR_SELECTION) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=?", SARA_R5_OPERATOR_SELECTION);

  int responseSize = (maxOps + 1) * 48;
  response = sara_r5_calloc_char(responseSize);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // AT+COPS maximum response time is 3 minutes (180000 ms)
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_3_MIN_TIMEOUT, responseSize);

  // Sample responses:
  // +COPS: (3,"Verizon Wireless","VzW","311480",8),,(0,1,2,3,4),(0,1,2)
  // +COPS: (1,"313 100","313 100","313100",8),(2,"AT&T","AT&T","310410",8),(3,"311 480","311 480","311480",8),,(0,1,2,3,4),(0,1,2)

  if (_printDebug == true)
  {
    _debugPort->print(F("getOperators: Response: {"));
    _debugPort->print(response);
    _debugPort->println(F("}"));
  }

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *opBegin;
    char *opEnd;
    int op = 0;
    int stat;
    char longOp[26];
    char shortOp[11];
    int act;
    unsigned long numOp;

    opBegin = response;

    for (; op < maxOps; op++)
    {
      opBegin = strchr(opBegin, '(');
      if (opBegin == NULL)
        break;
      opEnd = strchr(opBegin, ')');
      if (opEnd == NULL)
        break;

      int sscanRead = sscanf(opBegin, "(%d,\"%[^\"]\",\"%[^\"]\",\"%lu\",%d)%*s",
                             &stat, longOp, shortOp, &numOp, &act);
      if (sscanRead == 5)
      {
        opRet[op].stat = stat;
        opRet[op].longOp = (String)(longOp);
        opRet[op].shortOp = (String)(shortOp);
        opRet[op].numOp = numOp;
        opRet[op].act = act;
        opsSeen += 1;
      }
      // TODO: Search for other possible patterns here
      else
      {
        break; // Break out if pattern doesn't match.
      }
      opBegin = opEnd + 1; // Move opBegin to beginning of next value
    }
  }

  free(command);
  free(response);

  return opsSeen;
}

SARA_R5_error_t SARA_R5::registerOperator(struct operator_stats oper)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_OPERATOR_SELECTION) + 24);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=1,2,\"%lu\"", SARA_R5_OPERATOR_SELECTION, oper.numOp);

  // AT+COPS maximum response time is 3 minutes (180000 ms)
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_3_MIN_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::automaticOperatorSelection()
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_OPERATOR_SELECTION) + 6);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=0,0", SARA_R5_OPERATOR_SELECTION);

  // AT+COPS maximum response time is 3 minutes (180000 ms)
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_3_MIN_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getOperator(String *oper)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  char *searchPtr;
  char mode;

  command = sara_r5_calloc_char(strlen(SARA_R5_OPERATOR_SELECTION) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_OPERATOR_SELECTION);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // AT+COPS maximum response time is 3 minutes (180000 ms)
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_3_MIN_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    searchPtr = strstr(response, "+COPS: ");
    if (searchPtr != NULL)
    {
      searchPtr += strlen("+COPS: "); //  Move searchPtr to first char
      mode = *searchPtr;              // Read first char -- should be mode
      if (mode == '2')                // Check for de-register
      {
        err = SARA_R5_ERROR_DEREGISTERED;
      }
      // Otherwise if it's default, manual, set-only, or automatic
      else if ((mode == '0') || (mode == '1') || (mode == '3') || (mode == '4'))
      {
        *oper = "";
        searchPtr = strchr(searchPtr, '\"'); // Move to first quote
        if (searchPtr == NULL)
        {
          err = SARA_R5_ERROR_DEREGISTERED;
        }
        else
        {
          while ((*(++searchPtr) != '\"') && (*searchPtr != '\0'))
          {
            oper->concat(*(searchPtr));
          }
        }
        if (_printDebug == true)
        {
          _debugPort->print(F("getOperator: "));
          _debugPort->println(*oper);
        }
      }
    }
  }

  free(response);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::deregisterOperator(void)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_OPERATOR_SELECTION) + 4);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=2", SARA_R5_OPERATOR_SELECTION);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_3_MIN_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setSMSMessageFormat(SARA_R5_message_format_t textMode)
{
  char *command;
  SARA_R5_error_t err;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_FORMAT) + 4);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_MESSAGE_FORMAT,
          (textMode == SARA_R5_MESSAGE_FORMAT_TEXT) ? 1 : 0);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::sendSMS(String number, String message)
{
  char *command;
  char *messageCStr;
  char *numberCStr;
  SARA_R5_error_t err;

  numberCStr = sara_r5_calloc_char(number.length() + 2);
  if (numberCStr == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  number.toCharArray(numberCStr, number.length() + 1);

  command = sara_r5_calloc_char(strlen(SARA_R5_SEND_TEXT) + strlen(numberCStr) + 8);
  if (command != NULL)
  {
    sprintf(command, "%s=\"%s\"", SARA_R5_SEND_TEXT, numberCStr);

    err = sendCommandWithResponse(command, ">", NULL,
                                  SARA_R5_3_MIN_TIMEOUT);
    free(command);
    free(numberCStr);
    if (err != SARA_R5_ERROR_SUCCESS)
      return err;

    messageCStr = sara_r5_calloc_char(message.length() + 1);
    if (messageCStr == NULL)
    {
      hwWrite(ASCII_CTRL_Z);
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    }
    message.toCharArray(messageCStr, message.length() + 1);
    messageCStr[message.length()] = ASCII_CTRL_Z;

    err = sendCommandWithResponse(messageCStr, SARA_R5_RESPONSE_OK_OR_ERROR,
                                  NULL, SARA_R5_3_MIN_TIMEOUT, minimumResponseAllocation, NOT_AT_COMMAND);

    free(messageCStr);
  }
  else
  {
    free(numberCStr);
    err = SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  return err;
}

SARA_R5_error_t SARA_R5::getPreferredMessageStorage(int *used, int *total, String memory)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  int u;
  int t;

  command = sara_r5_calloc_char(strlen(SARA_R5_PREF_MESSAGE_STORE) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_PREF_MESSAGE_STORE, memory.c_str());

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_3_MIN_TIMEOUT);

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return err;
  }

  int scanned = 0;
  char *searchPtr = strstr(response, "+CPMS: ");
  if (searchPtr != NULL)
    scanned = sscanf(searchPtr, "+CPMS: %d,%d", &u, &t);
  if (scanned == 2)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getPreferredMessageStorage: memory1 (read and delete): "));
      _debugPort->print(memory);
      _debugPort->print(F(" used: "));
      _debugPort->print(u);
      _debugPort->print(F(" total: "));
      _debugPort->println(t);
    }
    *used = u;
    *total = t;
  }
  else
  {
    err = SARA_R5_ERROR_INVALID;
  }

  free(response);
  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::readSMSmessage(int location, String *unread, String *from, String *dateTime, String *message)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  command = sara_r5_calloc_char(strlen(SARA_R5_READ_TEXT_MESSAGE) + 5);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_READ_TEXT_MESSAGE, location);

  response = sara_r5_calloc_char(1024);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_10_SEC_TIMEOUT, 1024);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = response;

    // Find the first occurrence of +CGDCONT:
    searchPtr = strstr(searchPtr, "+CMGR: ");
    if (searchPtr != NULL)
    {
      searchPtr += strlen("+CMGR: "); // Point to the originator address
      int pointer = 0;
      while ((*(++searchPtr) != '\"') && (*searchPtr != '\0') && (pointer < 12))
      {
        unread->concat(*(searchPtr));
        pointer++;
      }
      if ((*searchPtr == '\0') || (pointer == 12))
      {
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }
      // Search to the next quote
      searchPtr = strchr(++searchPtr, '\"');
      pointer = 0;
      while ((*(++searchPtr) != '\"') && (*searchPtr != '\0') && (pointer < 24))
      {
        from->concat(*(searchPtr));
        pointer++;
      }
      if ((*searchPtr == '\0') || (pointer == 24))
      {
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }
      // Skip two commas
      searchPtr = strchr(++searchPtr, ',');
      searchPtr = strchr(++searchPtr, ',');
      // Search to the next quote
      searchPtr = strchr(++searchPtr, '\"');
      pointer = 0;
      while ((*(++searchPtr) != '\"') && (*searchPtr != '\0') && (pointer < 24))
      {
        dateTime->concat(*(searchPtr));
        pointer++;
      }
      if ((*searchPtr == '\0') || (pointer == 24))
      {
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }
      // Search to the next new line
      searchPtr = strchr(++searchPtr, '\n');
      pointer = 0;
      while ((*(++searchPtr) != '\r') && (*searchPtr != '\n') && (*searchPtr != '\0') && (pointer < 512))
      {
        message->concat(*(searchPtr));
        pointer++;
      }
      if ((*searchPtr == '\0') || (pointer == 512))
      {
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }
    }
    else
    {
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }
  }
  else
  {
    err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::deleteSMSmessage(int location, int deleteFlag)
{
  char *command;
  SARA_R5_error_t err;

  command = sara_r5_calloc_char(strlen(SARA_R5_DELETE_MESSAGE) + 12);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (deleteFlag == 0)
    sprintf(command, "%s=%d", SARA_R5_DELETE_MESSAGE, location);
  else
    sprintf(command, "%s=%d,%d", SARA_R5_DELETE_MESSAGE, location, deleteFlag);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, SARA_R5_55_SECS_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setBaud(unsigned long baud)
{
  SARA_R5_error_t err;
  char *command;
  int b = 0;

  // Error check -- ensure supported baud
  for (; b < NUM_SUPPORTED_BAUD; b++)
  {
    if (SARA_R5_SUPPORTED_BAUD[b] == baud)
    {
      break;
    }
  }
  if (b >= NUM_SUPPORTED_BAUD)
  {
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
  }

  // Construct command
  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_BAUD) + 7 + 12);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%lu", SARA_R5_COMMAND_BAUD, baud);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_SET_BAUD_TIMEOUT);

  free(command);

  return err;
}

SARA_R5_error_t SARA_R5::setFlowControl(SARA_R5_flow_control_t value)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_FLOW_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s%d", SARA_R5_FLOW_CONTROL, value);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);

  return err;
}

SARA_R5_error_t SARA_R5::setGpioMode(SARA_R5_gpio_t gpio,
                                     SARA_R5_gpio_mode_t mode, int value)
{
  SARA_R5_error_t err;
  char *command;

  // Example command: AT+UGPIOC=16,2
  // Example command: AT+UGPIOC=23,0,1
  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_GPIO) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (mode == GPIO_OUTPUT)
    sprintf(command, "%s=%d,%d,%d", SARA_R5_COMMAND_GPIO, gpio, mode, value);
  else
    sprintf(command, "%s=%d,%d", SARA_R5_COMMAND_GPIO, gpio, mode);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_10_SEC_TIMEOUT);

  free(command);

  return err;
}

SARA_R5::SARA_R5_gpio_mode_t SARA_R5::getGpioMode(SARA_R5_gpio_t gpio)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  char gpioChar[4];
  char *gpioStart;
  int gpioMode;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_GPIO) + 2);
  if (command == NULL)
    return GPIO_MODE_INVALID;
  sprintf(command, "%s?", SARA_R5_COMMAND_GPIO);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return GPIO_MODE_INVALID;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return GPIO_MODE_INVALID;
  }

  sprintf(gpioChar, "%d", gpio);          // Convert GPIO to char array
  gpioStart = strstr(response, gpioChar); // Find first occurence of GPIO in response

  free(command);
  free(response);

  if (gpioStart == NULL)
    return GPIO_MODE_INVALID; // If not found return invalid
  sscanf(gpioStart, "%*d,%d\r\n", &gpioMode);

  return (SARA_R5_gpio_mode_t)gpioMode;
}

int SARA_R5::socketOpen(SARA_R5_socket_protocol_t protocol, unsigned int localPort)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  int sockId = -1;
  char *responseStart;

  command = sara_r5_calloc_char(strlen(SARA_R5_CREATE_SOCKET) + 10);
  if (command == NULL)
    return -1;
  if (localPort == 0)
    sprintf(command, "%s=%d", SARA_R5_CREATE_SOCKET, (int)protocol);
  else
    sprintf(command, "%s=%d,%d", SARA_R5_CREATE_SOCKET, (int)protocol, localPort);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    if (_printDebug == true)
      _debugPort->println(F("socketOpen: Fail: NULL response"));
    free(command);
    return -1;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("socketOpen: Fail: Error: "));
      _debugPort->print(err);
      _debugPort->print(F("  Response: {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
    free(command);
    free(response);
    return -1;
  }

  responseStart = strstr(response, "+USOCR");
  if (responseStart == NULL)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("socketOpen: Failure: {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
    free(command);
    free(response);
    return -1;
  }

  sscanf(responseStart, "+USOCR: %d", &sockId);
  _lastSocketProtocol[sockId] = (int)protocol;

  free(command);
  free(response);

  return sockId;
}

SARA_R5_error_t SARA_R5::socketClose(int socket, unsigned long timeout)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  command = sara_r5_calloc_char(strlen(SARA_R5_CLOSE_SOCKET) + 10);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  sprintf(command, "%s=%d", SARA_R5_CLOSE_SOCKET, socket);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response, timeout);

  if ((err != SARA_R5_ERROR_SUCCESS) && (_printDebug == true))
  {
    _debugPort->print(F("socketClose: Error: "));
    _debugPort->println(socketGetLastError());
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::socketConnect(int socket, const char *address,
                                       unsigned int port)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_CONNECT_SOCKET) + strlen(address) + 11);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,\"%s\",%d", SARA_R5_CONNECT_SOCKET, socket, address, port);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, SARA_R5_IP_CONNECT_TIMEOUT);

  free(command);

  return err;
}

SARA_R5_error_t SARA_R5::socketConnect(int socket, IPAddress address,
                                       unsigned int port)
{
  char *charAddress = sara_r5_calloc_char(16);
  if (charAddress == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  memset(charAddress, 0, 16);
  sprintf(charAddress, "%d.%d.%d.%d", address[0], address[1], address[2], address[3]);

  return (socketConnect(socket, (const char *)charAddress, port));
}

SARA_R5_error_t SARA_R5::socketWrite(int socket, const char *str, int len)
{
  char *command;
  char *response;
  SARA_R5_error_t err;

  command = sara_r5_calloc_char(strlen(SARA_R5_WRITE_SOCKET) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  int dataLen = len == -1 ? strlen(str) : len;
  sprintf(command, "%s=%d,%d", SARA_R5_WRITE_SOCKET, socket, dataLen);

  err = sendCommandWithResponse(command, "@", response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT * 5);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    unsigned long writeDelay = millis();
    while (millis() < (writeDelay + 50))
      delay(1); //u-blox specification says to wait 50ms after receiving "@" to write data.

    if (len == -1)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketWrite: writing: "));
        _debugPort->println(str);
      }
      hwPrint(str);
    }
    else
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketWrite: writing "));
        _debugPort->print(len);
        _debugPort->println(F(" bytes"));
      }
      hwWriteData(str, len);
    }

    err = waitForResponse(SARA_R5_RESPONSE_OK, SARA_R5_RESPONSE_ERROR, SARA_R5_SOCKET_WRITE_TIMEOUT);
  }

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("socketWrite: Error: "));
      _debugPort->print(err);
      _debugPort->print(F(" => {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::socketWrite(int socket, String str)
{
  return socketWrite(socket, str.c_str(), str.length());
}

SARA_R5_error_t SARA_R5::socketWriteUDP(int socket, const char *address, int port, const char *str, int len)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int dataLen = len == -1 ? strlen(str) : len;

  command = sara_r5_calloc_char(64);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  sprintf(command, "%s=%d,\"%s\",%d,%d", SARA_R5_WRITE_UDP_SOCKET,
          socket, address, port, dataLen);
  err = sendCommandWithResponse(command, "@", response, SARA_R5_STANDARD_RESPONSE_TIMEOUT * 5);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (len == -1) //If binary data we need to send a length.
    {
      hwPrint(str);
    }
    else
    {
      hwWriteData(str, len);
    }
    err = waitForResponse(SARA_R5_RESPONSE_OK, SARA_R5_RESPONSE_ERROR, SARA_R5_SOCKET_WRITE_TIMEOUT);
  }
  else
  {
    if (_printDebug == true)
      _debugPort->print(F("socketWriteUDP: Error: "));
    if (_printDebug == true)
      _debugPort->println(socketGetLastError());
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::socketWriteUDP(int socket, IPAddress address, int port, const char *str, int len)
{
  char *charAddress = sara_r5_calloc_char(16);
  if (charAddress == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  memset(charAddress, 0, 16);
  sprintf(charAddress, "%d.%d.%d.%d", address[0], address[1], address[2], address[3]);

  return (socketWriteUDP(socket, (const char *)charAddress, port, str, len));
}

SARA_R5_error_t SARA_R5::socketWriteUDP(int socket, String address, int port, String str)
{
  return socketWriteUDP(socket, address.c_str(), port, str.c_str(), str.length());
}

SARA_R5_error_t SARA_R5::socketRead(int socket, int length, char *readDest, int *bytesRead)
{
  char *command;
  char *response;
  char *strBegin;
  int readIndexTotal = 0;
  int readIndexThisRead = 0;
  SARA_R5_error_t err;
  int scanNum = 0;
  int readLength = 0;
  int socketStore = 0;
  int bytesLeftToRead = length;
  int bytesToRead;

  // Set *bytesRead to zero
  if (bytesRead != NULL)
    *bytesRead = 0;

  // Check if length is zero
  if (length == 0)
  {
    if (_printDebug == true)
      _debugPort->print(F("socketRead: length is 0! Call socketReadAvailable?"));
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
  }

  // Allocate memory for the command
  command = sara_r5_calloc_char(strlen(SARA_R5_READ_SOCKET) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  // Allocate memory for the response
  // We only need enough to read _saraR5maxSocketRead bytes - not the whole thing
  int responseLength = _saraR5maxSocketRead + strlen(SARA_R5_READ_SOCKET) + minimumResponseAllocation;
  response = sara_r5_calloc_char(responseLength);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // If there are more than _saraR5maxSocketRead (1024) bytes to be read,
  // we need to do multiple reads to get all the data

  while (bytesLeftToRead > 0)
  {
    if (bytesLeftToRead > _saraR5maxSocketRead) // Limit a single read to _saraR5maxSocketRead
      bytesToRead = _saraR5maxSocketRead;
    else
      bytesToRead = bytesLeftToRead;

    sprintf(command, "%s=%d,%d", SARA_R5_READ_SOCKET, socket, bytesToRead);

    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT, responseLength);

    if (err != SARA_R5_ERROR_SUCCESS)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketRead: sendCommandWithResponse err "));
        _debugPort->println(err);
      }
      free(command);
      free(response);
      return err;
    }

    // Extract the data
    char *searchPtr = strstr(response, "+USORD: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USORD: %d,%d",
                        &socketStore, &readLength);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketRead: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    // Check that readLength == bytesToRead
    if (readLength != bytesToRead)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketRead: length mismatch! bytesToRead="));
        _debugPort->print(bytesToRead);
        _debugPort->print(F(" readLength="));
        _debugPort->println(readLength);
      }
    }

    // Check that readLength > 0
    if (readLength == 0)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("socketRead: zero length!"));
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_ZERO_READ_LENGTH;
    }

    // Find the first double-quote:
    strBegin = strchr(searchPtr, '\"');

    if (strBegin == NULL)
    {
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    // Now copy the data into readDest
    readIndexThisRead = 1; // Start after the quote
    while (readIndexThisRead < (readLength + 1))
    {
      readDest[readIndexTotal] = strBegin[readIndexThisRead];
      readIndexTotal++;
      readIndexThisRead++;
    }

    if (_printDebug == true)
      _debugPort->println(F("socketRead: success"));

    // Update *bytesRead
    if (bytesRead != NULL)
      *bytesRead = readIndexTotal;

    // How many bytes are left to read?
    // This would have been bytesLeftToRead -= bytesToRead
    // Except the SARA can potentially return less data than requested...
    // So we need to subtract readLength instead.
    bytesLeftToRead -= readLength;
    if (_printDebug == true)
    {
      if (bytesLeftToRead > 0)
      {
        _debugPort->print(F("socketRead: multiple read. bytesLeftToRead: "));
        _debugPort->println(bytesLeftToRead);
      }
    }
  } // /while (bytesLeftToRead > 0)

  free(command);
  free(response);

  return SARA_R5_ERROR_SUCCESS;
}

SARA_R5_error_t SARA_R5::socketReadAvailable(int socket, int *length)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int readLength = 0;
  int socketStore = 0;

  command = sara_r5_calloc_char(strlen(SARA_R5_READ_SOCKET) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,0", SARA_R5_READ_SOCKET, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USORD: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USORD: %d,%d",
                        &socketStore, &readLength);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketReadAvailable: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *length = readLength;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::socketReadUDP(int socket, int length, char *readDest,
                                      IPAddress *remoteIPAddress, int *remotePort, int *bytesRead)
{
  char *command;
  char *response;
  char *strBegin;
  int readIndexTotal = 0;
  int readIndexThisRead = 0;
  SARA_R5_error_t err;
  int scanNum = 0;
  int remoteIPstore[4] = { 0, 0, 0, 0 };
  int portStore = 0;
  int readLength = 0;
  int socketStore = 0;
  int bytesLeftToRead = length;
  int bytesToRead;

  // Set *bytesRead to zero
  if (bytesRead != NULL)
    *bytesRead = 0;

  // Check if length is zero
  if (length == 0)
  {
    if (_printDebug == true)
      _debugPort->print(F("socketReadUDP: length is 0! Call socketReadAvailableUDP?"));
    return SARA_R5_ERROR_UNEXPECTED_PARAM;
  }

  // Allocate memory for the command
  command = sara_r5_calloc_char(strlen(SARA_R5_READ_UDP_SOCKET) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  // Allocate memory for the response
  // We only need enough to read _saraR5maxSocketRead bytes - not the whole thing
  int responseLength = _saraR5maxSocketRead + strlen(SARA_R5_READ_UDP_SOCKET) + minimumResponseAllocation;
  response = sara_r5_calloc_char(responseLength);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // If there are more than _saraR5maxSocketRead (1024) bytes to be read,
  // we need to do multiple reads to get all the data

  while (bytesLeftToRead > 0)
  {
    if (bytesLeftToRead > _saraR5maxSocketRead) // Limit a single read to _saraR5maxSocketRead
      bytesToRead = _saraR5maxSocketRead;
    else
      bytesToRead = bytesLeftToRead;

    sprintf(command, "%s=%d,%d", SARA_R5_READ_UDP_SOCKET, socket, bytesToRead);

    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT, responseLength);

    if (err != SARA_R5_ERROR_SUCCESS)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketReadUDP: sendCommandWithResponse err "));
        _debugPort->println(err);
      }
      free(command);
      free(response);
      return err;
    }

    // Extract the data
    char *searchPtr = strstr(response, "+USORF: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USORF: %d,\"%d.%d.%d.%d\",%d,%d",
                        &socketStore, &remoteIPstore[0], &remoteIPstore[1], &remoteIPstore[2], &remoteIPstore[3],
                        &portStore, &readLength);
    if (scanNum != 7)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketReadUDP: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    // Check that readLength == bytesToRead
    if (readLength != bytesToRead)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketReadUDP: length mismatch! bytesToRead="));
        _debugPort->print(bytesToRead);
        _debugPort->print(F(" readLength="));
        _debugPort->println(readLength);
      }
    }

    // Check that readLength > 0
    if (readLength == 0)
    {
      if (_printDebug == true)
      {
        _debugPort->println(F("socketRead: zero length!"));
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_ZERO_READ_LENGTH;
    }

    // Find the third double-quote
    strBegin = strchr(searchPtr, '\"');
    strBegin = strchr(strBegin + 1, '\"');
    strBegin = strchr(strBegin + 1, '\"');

    if (strBegin == NULL)
    {
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    // Now copy the data into readDest
    readIndexThisRead = 1; // Start after the quote
    while (readIndexThisRead < (readLength + 1))
    {
      readDest[readIndexTotal] = strBegin[readIndexThisRead];
      readIndexTotal++;
      readIndexThisRead++;
    }

    // If remoteIPaddress is not NULL, copy the remote IP address
    if (remoteIPAddress != NULL)
    {
      IPAddress tempAddress;
      for (int i = 0; i <= 3; i++)
      {
        tempAddress[i] = (uint8_t)remoteIPstore[i];
      }
      *remoteIPAddress = tempAddress;
    }

    // If remotePort is not NULL, copy the remote port
    if (remotePort != NULL)
    {
      *remotePort = portStore;
    }

    if (_printDebug == true)
      _debugPort->println(F("socketReadUDP: success"));

    // Update *bytesRead
    if (bytesRead != NULL)
      *bytesRead = readIndexTotal;

    // How many bytes are left to read?
    // This would have been bytesLeftToRead -= bytesToRead
    // Except the SARA can potentially return less data than requested...
    // So we need to subtract readLength instead.
    bytesLeftToRead -= readLength;
    if (_printDebug == true)
    {
      if (bytesLeftToRead > 0)
      {
        _debugPort->print(F("socketReadUDP: multiple read. bytesLeftToRead: "));
        _debugPort->println(bytesLeftToRead);
      }
    }
  } // /while (bytesLeftToRead > 0)

  free(command);
  free(response);

  return SARA_R5_ERROR_SUCCESS;
}

SARA_R5_error_t SARA_R5::socketReadAvailableUDP(int socket, int *length)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int readLength = 0;
  int socketStore = 0;

  command = sara_r5_calloc_char(strlen(SARA_R5_READ_UDP_SOCKET) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,0", SARA_R5_READ_UDP_SOCKET, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USORF: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USORF: %d,%d",
                        &socketStore, &readLength);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("socketReadAvailableUDP: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *length = readLength;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::socketListen(int socket, unsigned int port)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_LISTEN_SOCKET) + 9);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d", SARA_R5_LISTEN_SOCKET, socket, port);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::socketDirectLinkMode(int socket)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_DIRECT_LINK) + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_SOCKET_DIRECT_LINK, socket);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_CONNECT, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::socketDirectLinkTimeTrigger(int socket, unsigned long timerTrigger)
{
  // valid range is 0 (trigger disabled), 100-120000
  if (!((timerTrigger == 0) || ((timerTrigger >= 100) && (timerTrigger <= 120000))))
    return SARA_R5_ERROR_ERROR;

  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_UD_CONFIGURATION) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=5,%d,%ld", SARA_R5_UD_CONFIGURATION, socket, timerTrigger);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::socketDirectLinkDataLengthTrigger(int socket, int dataLengthTrigger)
{
  // valid range is 0, 3-1472 for UDP
  if (!((dataLengthTrigger == 0) || ((dataLengthTrigger >= 3) && (dataLengthTrigger <= 1472))))
    return SARA_R5_ERROR_ERROR;

  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_UD_CONFIGURATION) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=6,%d,%d", SARA_R5_UD_CONFIGURATION, socket, dataLengthTrigger);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::socketDirectLinkCharacterTrigger(int socket, int characterTrigger)
{
  // The allowed range is -1, 0-255, the factory-programmed value is -1; -1 means trigger disabled.
  if (!((characterTrigger >= -1) && (characterTrigger <= 255)))
    return SARA_R5_ERROR_ERROR;

  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_UD_CONFIGURATION) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=7,%d,%d", SARA_R5_UD_CONFIGURATION, socket, characterTrigger);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::socketDirectLinkCongestionTimer(int socket, unsigned long congestionTimer)
{
  // valid range is 0, 1000-72000
  if (!((congestionTimer == 0) || ((congestionTimer >= 1000) && (congestionTimer <= 72000))))
    return SARA_R5_ERROR_ERROR;

  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_UD_CONFIGURATION) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=8,%d,%ld", SARA_R5_UD_CONFIGURATION, socket, congestionTimer);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::querySocketType(int socket, SARA_R5_socket_protocol_t *protocol)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,0", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,0,%d",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketType: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *protocol = (SARA_R5_socket_protocol_t)paramVal;
    _lastSocketProtocol[socketStore] = paramVal;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketLastError(int socket, int *error)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,1", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,1,%d",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketLastError: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *error = paramVal;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketTotalBytesSent(int socket, uint32_t *total)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  long unsigned int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,2", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,2,%lu",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketTotalBytesSent: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *total = (uint32_t)paramVal;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketTotalBytesReceived(int socket, uint32_t *total)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  long unsigned int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,3", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,3,%lu",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketTotalBytesReceived: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *total = (uint32_t)paramVal;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketRemoteIPAddress(int socket, IPAddress *address, int *port)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  int paramVals[5];

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,4", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,4,\"%d.%d.%d.%d\",%d",
                        &socketStore,
                        &paramVals[0], &paramVals[1], &paramVals[2], &paramVals[3],
                        &paramVals[4]);
    if (scanNum != 6)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketRemoteIPAddress: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    IPAddress tempAddress = { (uint8_t)paramVals[0], (uint8_t)paramVals[1],
                              (uint8_t)paramVals[2], (uint8_t)paramVals[3] };
    *address = tempAddress;
    *port = paramVals[4];
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketStatusTCP(int socket, SARA_R5_tcp_socket_status_t *status)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,10", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,10,%d",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketStatusTCP: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *status = (SARA_R5_tcp_socket_status_t)paramVal;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::querySocketOutUnackData(int socket, uint32_t *total)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int socketStore = 0;
  long unsigned int paramVal;

  command = sara_r5_calloc_char(strlen(SARA_R5_SOCKET_CONTROL) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,11", SARA_R5_SOCKET_CONTROL, socket);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOCTL: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+USOCTL: %d,11,%lu",
                        &socketStore, &paramVal);
    if (scanNum != 2)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("querySocketOutUnackData: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    *total = (uint32_t)paramVal;
  }

  free(command);
  free(response);

  return err;
}

//Issues command to get last socket error, then prints to serial. Also updates rx/backlog buffers.
int SARA_R5::socketGetLastError()
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  int errorCode = -1;

  command = sara_r5_calloc_char(64);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  sprintf(command, "%s", SARA_R5_GET_ERROR);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+USOER: ");
    if (searchPtr != NULL)
      sscanf(searchPtr, "+USOER: %d", &errorCode);
  }

  free(command);
  free(response);

  return errorCode;
}

IPAddress SARA_R5::lastRemoteIP(void)
{
  return _lastRemoteIP;
}

SARA_R5_error_t SARA_R5::resetHTTPprofile(int profile)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_HTTP_PROFILE, profile);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPserverIPaddress(int profile, IPAddress address)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 64);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%d.%d.%d.%d\"", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_SERVER_IP,
          address[0], address[1], address[2], address[3]);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPserverName(int profile, String server)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 12 + server.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_SERVER_NAME,
          server.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPusername(int profile, String username)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 12 + username.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_USERNAME,
          username.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPpassword(int profile, String password)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 12 + password.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_PASSWORD,
          password.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPauthentication(int profile, bool authenticate)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,%d", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_AUTHENTICATION,
          authenticate);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPserverPort(int profile, int port)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,%d", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_SERVER_PORT,
          port);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPcustomHeader(int profile, String header)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 12 + header.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_ADD_CUSTOM_HEADERS,
          header.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setHTTPsecure(int profile, bool secure, int secprofile)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROFILE) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (secprofile == -1)
      sprintf(command, "%s=%d,%d,%d", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_SECURE,
          secure);
  else sprintf(command, "%s=%d,%d,%d,%d", SARA_R5_HTTP_PROFILE, profile, SARA_R5_HTTP_OP_CODE_SECURE,
        secure, secprofile);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::ping(String remote_host, int retry, int p_size,
                              unsigned long timeout, int ttl)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_PING_COMMAND) + 48 +
                                remote_host.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\",%d,%d,%ld,%d", SARA_R5_PING_COMMAND,
          remote_host.c_str(), retry, p_size, timeout, ttl);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::sendHTTPGET(int profile, String path, String responseFilename)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_COMMAND) + 24 +
                                path.length() + responseFilename.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\",\"%s\"", SARA_R5_HTTP_COMMAND, profile, SARA_R5_HTTP_COMMAND_GET,
          path.c_str(), responseFilename.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::sendHTTPPOSTdata(int profile, String path, String responseFilename,
                                          String data, SARA_R5_http_content_types_t httpContentType)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_COMMAND) + 24 +
                                path.length() + responseFilename.length() + data.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\",\"%s\",\"%s\",%d", SARA_R5_HTTP_COMMAND, profile, SARA_R5_HTTP_COMMAND_POST_DATA,
          path.c_str(), responseFilename.c_str(), data.c_str(), httpContentType);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::sendHTTPPOSTfile(int profile, String path, String responseFilename,
                                          String requestFile, SARA_R5_http_content_types_t httpContentType)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_HTTP_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_COMMAND) + 24 +
                                path.length() + responseFilename.length() + requestFile.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\",\"%s\",\"%s\",%d", SARA_R5_HTTP_COMMAND, profile, SARA_R5_HTTP_COMMAND_POST_FILE,
          path.c_str(), responseFilename.c_str(), requestFile.c_str(), httpContentType);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getHTTPprotocolError(int profile, int *error_class, int *error_code)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  int rprofile, eclass, ecode;

  command = sara_r5_calloc_char(strlen(SARA_R5_HTTP_PROTOCOL_ERROR) + 4);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_HTTP_PROTOCOL_ERROR, profile);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int scanned = 0;
    char *searchPtr = strstr(response, "+UHTTPER: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+UHTTPER: %d,%d,%d\r\n",
                        &rprofile, &eclass, &ecode);
    if (scanned == 3)
    {
      *error_class = eclass;
      *error_code = ecode;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::nvMQTT(SARA_R5_mqtt_nv_parameter_t parameter)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_NVM) + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d", SARA_R5_MQTT_NVM, parameter);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::setMQTTclientId(String clientId)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_PROFILE) + clientId.length() + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d,\"%s\"", SARA_R5_MQTT_PROFILE, SARA_R5_MQTT_PROFILE_CLIENT_ID, clientId.c_str());
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::setMQTTserver(String serverName, int port)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_PROFILE) + serverName.length() + 16);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d,\"%s\",%d", SARA_R5_MQTT_PROFILE, SARA_R5_MQTT_PROFILE_SERVERNAME, serverName.c_str(), port);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::setMQTTsecure(bool secure, int secprofile)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_PROFILE) + 16);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    if (secprofile == -1) sprintf(command, "%s=%d,%d", SARA_R5_MQTT_PROFILE, SARA_R5_MQTT_PROFILE_SECURE, secure);
    else sprintf(command, "%s=%d,%d,%d", SARA_R5_MQTT_PROFILE, SARA_R5_MQTT_PROFILE_SECURE, secure, secprofile);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::connectMQTT(void)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_COMMAND) + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d", SARA_R5_MQTT_COMMAND, SARA_R5_MQTT_COMMAND_LOGIN);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}
  
SARA_R5_error_t SARA_R5::disconnectMQTT(void)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_COMMAND) + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d", SARA_R5_MQTT_COMMAND, SARA_R5_MQTT_COMMAND_LOGOUT);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}
  
SARA_R5_error_t SARA_R5::subscribeMQTTtopic(int max_Qos, String topic)
{
  SARA_R5_error_t err;
  char *command;
  command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_COMMAND) + 16 + topic.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_MQTT_COMMAND, SARA_R5_MQTT_COMMAND_SUBSCRIBE, max_Qos, topic.c_str());
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}
 
SARA_R5_error_t SARA_R5::unsubscribeMQTTtopic(String topic)
{
  SARA_R5_error_t err;
  char *command;
  command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_COMMAND) + 16 + topic.length());
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,\"%s\"", SARA_R5_MQTT_COMMAND, SARA_R5_MQTT_COMMAND_UNSUBSCRIBE, topic.c_str());
  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  free(command);
  return err;
}
    
SARA_R5_error_t SARA_R5::readMQTT(int* pQos, String* pTopic, uint8_t *readDest, int readLength, int *bytesRead)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int total_length, topic_length, data_length;
  
  // Set *bytesRead to zero
  if (bytesRead != NULL)
    *bytesRead = 0;

  // Allocate memory for the command
  command = sara_r5_calloc_char(strlen(SARA_R5_MQTT_COMMAND) + 10);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  // Allocate memory for the response
  int responseLength = readLength + minimumResponseAllocation;
  response = sara_r5_calloc_char(responseLength);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  
  // Note to self: if the file contents contain "OK\r\n" sendCommandWithResponse will return true too early...
  // To try and avoid this, look for \"\r\n\r\nOK\r\n there is a extra \r\n beetween " and the the standard \r\nOK\r\n
  const char mqttReadTerm[] = "\r\n\r\nOK\r\n";
  sprintf(command, "%s=%d,%d", SARA_R5_MQTT_COMMAND, SARA_R5_MQTT_COMMAND_READ, 1);
  err = sendCommandWithResponse(command, mqttReadTerm, response,
                                (5 * SARA_R5_STANDARD_RESPONSE_TIMEOUT), responseLength);
  
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("readMQTT: sendCommandWithResponse err "));
      _debugPort->println(err);
    }
    free(command);
    free(response);
    return err;
  }

  // Extract the data
  char *searchPtr = strstr(response, "+UMQTTC: 6");
  if (searchPtr != NULL)
    scanNum = sscanf(searchPtr, "+UMQTTC: 6,%d,%d,%d,\"%*[^\"]\",%d,\"", pQos, &total_length, &topic_length, &data_length);
  if (scanNum != 4)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("readMQTT: error: scanNum is "));
      _debugPort->println(scanNum);
    }
    free(command);
    free(response);
    return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }
  
  err = SARA_R5_ERROR_SUCCESS;
  searchPtr = strstr(searchPtr, "\"");
  if (searchPtr!= NULL) {
    if (pTopic) {
      searchPtr[topic_length + 1] = '\0'; // zero terminate
      *pTopic = searchPtr + 1;
      searchPtr[topic_length + 1] = '\"'; // restore
    }
    searchPtr = strstr(searchPtr + topic_length + 2, "\"");
    if (readDest && (searchPtr != NULL) && (searchPtr[data_length + 1] == '"')) {
      if (data_length > readLength) {
        data_length = readLength;
        if (_printDebug == true) {
          _debugPort->print(F("readMQTT: error: trucate message"));
        }
        err = SARA_R5_ERROR_OUT_OF_MEMORY;
      }
      memcpy(readDest, searchPtr+1, data_length);
      *bytesRead = data_length;
    } else {
      if (_printDebug == true) {
        _debugPort->print(F("readMQTT: error: message end "));
      }
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }
  }
  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::getMQTTprotocolError(int *error_code, int *error_code2)
{
  SARA_R5_error_t err;
  char *response;

  int code, code2;

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(SARA_R5_MQTT_PROTOCOL_ERROR, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    int scanned = 0;
    char *searchPtr = strstr(response, "+UMQTTER: ");
    if (searchPtr != NULL)
      scanned = sscanf(searchPtr, "+UMQTTER:%d,%d\r\n",
                        &code, &code2);
    if (scanned == 2)
    {
      *error_code = code;
      *error_code2 = code2;
    }
    else
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::resetSecurityProfile(int secprofile)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_SEC_PROFILE) + 6);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  sprintf(command, "%s=%d", SARA_R5_SEC_PROFILE, secprofile);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::configSecurityProfile(int secprofile, SARA_R5_sec_profile_parameter_t parameter, int value)
{
    SARA_R5_error_t err;
    char *command;

    command = sara_r5_calloc_char(strlen(SARA_R5_SEC_PROFILE) + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d,%d,%d", SARA_R5_SEC_PROFILE, secprofile,parameter,value);
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::configSecurityProfileString(int secprofile, SARA_R5_sec_profile_parameter_t parameter, String value)
{
    SARA_R5_error_t err;
    char *command;
    command = sara_r5_calloc_char(strlen(SARA_R5_SEC_PROFILE) + value.length() + 10);
    if (command == NULL)
      return SARA_R5_ERROR_OUT_OF_MEMORY;
    sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_SEC_PROFILE, secprofile,parameter,value.c_str());
    err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                  SARA_R5_STANDARD_RESPONSE_TIMEOUT);
    free(command);
    return err;
}

SARA_R5_error_t SARA_R5::setSecurityManager(SARA_R5_sec_manager_opcode_t opcode, SARA_R5_sec_manager_parameter_t parameter, String name, String data)
{
  char *command;
  char *response;
  SARA_R5_error_t err;

  command = sara_r5_calloc_char(strlen(SARA_R5_SEC_MANAGER) + name.length() + 20);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  int dataLen = data.length();
  sprintf(command, "%s=%d,%d,\"%s\",%d", SARA_R5_SEC_MANAGER, opcode, parameter, name.c_str(), dataLen);

  err = sendCommandWithResponse(command, ">", response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("dataDownload: writing "));
      _debugPort->print(dataLen);
      _debugPort->println(F(" bytes"));
    }
    hwWriteData(data.c_str(), dataLen);
    err = waitForResponse(SARA_R5_RESPONSE_OK, SARA_R5_RESPONSE_ERROR, SARA_R5_STANDARD_RESPONSE_TIMEOUT*3);
  }
    

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("dataDownload: Error: "));
      _debugPort->print(err);
      _debugPort->print(F(" => {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::setPDPconfiguration(int profile, SARA_R5_pdp_configuration_parameter_t parameter, int value)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_PSD_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_CONFIG) + 24);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,%d", SARA_R5_MESSAGE_PDP_CONFIG, profile, parameter,
          value);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setPDPconfiguration(int profile, SARA_R5_pdp_configuration_parameter_t parameter, SARA_R5_pdp_protocol_type_t value)
{
  return (setPDPconfiguration(profile, parameter, (int)value));
}

SARA_R5_error_t SARA_R5::setPDPconfiguration(int profile, SARA_R5_pdp_configuration_parameter_t parameter, String value)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_PSD_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_CONFIG) + 64);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%s\"", SARA_R5_MESSAGE_PDP_CONFIG, profile, parameter,
          value.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::setPDPconfiguration(int profile, SARA_R5_pdp_configuration_parameter_t parameter, IPAddress value)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_PSD_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_CONFIG) + 64);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d,\"%d.%d.%d.%d\"", SARA_R5_MESSAGE_PDP_CONFIG, profile, parameter,
          value[0], value[1], value[2], value[3]);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::performPDPaction(int profile, SARA_R5_pdp_actions_t action)
{
  SARA_R5_error_t err;
  char *command;

  if (profile >= SARA_R5_NUM_PSD_PROFILES)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_ACTION) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d", SARA_R5_MESSAGE_PDP_ACTION, profile, action);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::activatePDPcontext(bool status, int cid)
{
  SARA_R5_error_t err;
  char *command;

  if (cid >= SARA_R5_NUM_PDP_CONTEXT_IDENTIFIERS)
    return SARA_R5_ERROR_ERROR;

  command = sara_r5_calloc_char(strlen(SARA_R5_MESSAGE_PDP_CONTEXT_ACTIVATE) + 32);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (cid == -1)
    sprintf(command, "%s=%d", SARA_R5_MESSAGE_PDP_CONTEXT_ACTIVATE, status);
  else
    sprintf(command, "%s=%d,%d", SARA_R5_MESSAGE_PDP_CONTEXT_ACTIVATE, status, cid);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::getNetworkAssignedIPAddress(int profile, IPAddress *address)
{
  char *command;
  char *response;
  SARA_R5_error_t err;
  int scanNum = 0;
  int profileStore = 0;
  int paramTag = 0; // 0: IP address: dynamic IP address assigned during PDP context activation
  int paramVals[4];

  command = sara_r5_calloc_char(strlen(SARA_R5_NETWORK_ASSIGNED_DATA) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d,%d", SARA_R5_NETWORK_ASSIGNED_DATA, profile, paramTag);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    char *searchPtr = strstr(response, "+UPSND: ");
    if (searchPtr != NULL)
      scanNum = sscanf(searchPtr, "+UPSND: %d,%d,\"%d.%d.%d.%d\"",
                        &profileStore, &paramTag,
                        &paramVals[0], &paramVals[1], &paramVals[2], &paramVals[3]);
    if (scanNum != 6)
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("getNetworkAssignedIPAddress: error: scanNum is "));
        _debugPort->println(scanNum);
      }
      free(command);
      free(response);
      return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }

    IPAddress tempAddress = { (uint8_t)paramVals[0], (uint8_t)paramVals[1],
                              (uint8_t)paramVals[2], (uint8_t)paramVals[3] };
    *address = tempAddress;
  }

  free(command);
  free(response);

  return err;
}

bool SARA_R5::isGPSon(void)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  bool on = false;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_POWER) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_GNSS_POWER);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response,
                                SARA_R5_10_SEC_TIMEOUT);

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    // Example response: "+UGPS: 0" for off "+UGPS: 1,0,1" for on
    // Search for a ':' followed by a '1' or ' 1'
    char *pch1 = strchr(response, ':');
    if (pch1 != NULL)
    {
      char *pch2 = strchr(response, '1');
      if ((pch2 != NULL) && ((pch2 == pch1 + 1) || (pch2 == pch1 + 2)))
        on = true;
    }
  }

  free(command);
  free(response);

  return on;
}

SARA_R5_error_t SARA_R5::gpsPower(bool enable, gnss_system_t gnss_sys, gnss_aiding_mode_t gnss_aiding)
{
  SARA_R5_error_t err;
  char *command;
  bool gpsState;

  // Don't turn GPS on/off if it's already on/off
  gpsState = isGPSon();
  if ((enable && gpsState) || (!enable && !gpsState))
  {
    return SARA_R5_ERROR_SUCCESS;
  }

  // GPS power management
  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_POWER) + 32); // gnss_sys could be up to three digits
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (enable)
  {
    sprintf(command, "%s=1,%d,%d", SARA_R5_GNSS_POWER, gnss_aiding, gnss_sys);
  }
  else
  {
    sprintf(command, "%s=0", SARA_R5_GNSS_POWER);
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, 10000);

  free(command);
  return err;
}

/*
SARA_R5_error_t SARA_R5::gpsEnableClock(bool enable)
{
    // AT+UGZDA=<0,1>
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsGetClock(struct ClockData *clock)
{
    // AT+UGZDA?
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsEnableFix(bool enable)
{
    // AT+UGGGA=<0,1>
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsGetFix(struct PositionData *pos)
{
    // AT+UGGGA?
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsEnablePos(bool enable)
{
    // AT+UGGLL=<0,1>
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsGetPos(struct PositionData *pos)
{
    // AT+UGGLL?
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsEnableSat(bool enable)
{
    // AT+UGGSV=<0,1>
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsGetSat(uint8_t *sats)
{
    // AT+UGGSV?
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}
*/

SARA_R5_error_t SARA_R5::gpsEnableRmc(bool enable)
{
  // AT+UGRMC=<0,1>
  SARA_R5_error_t err;
  char *command;

  // ** Don't call gpsPower here. It causes problems for +UTIME and the PPS signal **
  // ** Call isGPSon and gpsPower externally if required **
  // if (!isGPSon())
  // {
  //     err = gpsPower(true);
  //     if (err != SARA_R5_ERROR_SUCCESS)
  //     {
  //         return err;
  //     }
  // }

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_GPRMC) + 3);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_GNSS_GPRMC, enable ? 1 : 0);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, SARA_R5_10_SEC_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::gpsGetRmc(struct PositionData *pos, struct SpeedData *spd,
                                   struct ClockData *clk, bool *valid)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  char *rmcBegin;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_GPRMC) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_GNSS_GPRMC);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_10_SEC_TIMEOUT);
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    // Fast-forward response string to $GPRMC starter
    rmcBegin = strstr(response, "$GPRMC");
    if (rmcBegin == NULL)
    {
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }
    else
    {
      *valid = parseGPRMCString(rmcBegin, pos, clk, spd);
    }
  }

  free(command);
  free(response);
  return err;
}

/*
SARA_R5_error_t SARA_R5::gpsEnableSpeed(bool enable)
{
    // AT+UGVTG=<0,1>
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}

SARA_R5_error_t SARA_R5::gpsGetSpeed(struct SpeedData *speed)
{
    // AT+UGVTG?
    SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
    return err;
}
*/

SARA_R5_error_t SARA_R5::gpsRequest(unsigned int timeout, uint32_t accuracy,
                                    bool detailed, unsigned int sensor)
{
  // AT+ULOC=2,<useCellLocate>,<detailed>,<timeout>,<accuracy>
  SARA_R5_error_t err;
  char *command;

  // This function will only work if the GPS module is initially turned off.
  if (isGPSon())
  {
    gpsPower(false);
  }

  if (timeout > 999)
    timeout = 999;
  if (accuracy > 999999)
    accuracy = 999999;

  command = sara_r5_calloc_char(strlen(SARA_R5_GNSS_REQUEST_LOCATION) + 24);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  sprintf(command, "%s=2,%d,%d,%d,%d", SARA_R5_GNSS_REQUEST_LOCATION,
          sensor, detailed ? 1 : 0, timeout, accuracy);
#else
  sprintf(command, "%s=2,%d,%d,%d,%ld", SARA_R5_GNSS_REQUEST_LOCATION,
          sensor, detailed ? 1 : 0, timeout, accuracy);
#endif

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, SARA_R5_10_SEC_TIMEOUT);

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::gpsAidingServerConf(const char *primaryServer, const char *secondaryServer, const char *authToken,
                                             unsigned int days, unsigned int period, unsigned int resolution,
                                             unsigned int gnssTypes, unsigned int mode, unsigned int dataType)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_AIDING_SERVER_CONFIGURATION) + 256);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  sprintf(command, "%s=\"%s\",\"%s\",\"%s\",%d,%d,%d,%d,%d,%d", SARA_R5_AIDING_SERVER_CONFIGURATION,
          primaryServer, secondaryServer, authToken,
          days, period, resolution, gnssTypes, mode, dataType);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);
  return err;
}


// OK for text files. But will fail with binary files (containing \0) on some platforms.
SARA_R5_error_t SARA_R5::appendFileContents(String filename, const char *str, int len)
{
  char *command;
  char *response;
  SARA_R5_error_t err;

  command = sara_r5_calloc_char(strlen(SARA_R5_FILE_SYSTEM_DOWNLOAD_FILE) + filename.length() + 10);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }
  int dataLen = len == -1 ? strlen(str) : len;
  sprintf(command, "%s=\"%s\",%d", SARA_R5_FILE_SYSTEM_DOWNLOAD_FILE, filename.c_str(), dataLen);

  err = sendCommandWithResponse(command, ">", response,
                                SARA_R5_STANDARD_RESPONSE_TIMEOUT*2);
  
  unsigned long writeDelay = millis();
  while (millis() < (writeDelay + 50))
    delay(1); //uBlox specification says to wait 50ms after receiving "@" to write data.

  if (err == SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("fileDownload: writing "));
      _debugPort->print(dataLen);
      _debugPort->println(F(" bytes"));
    }
    hwWriteData(str, dataLen);
    
    err = waitForResponse(SARA_R5_RESPONSE_OK, SARA_R5_RESPONSE_ERROR, SARA_R5_STANDARD_RESPONSE_TIMEOUT*5);
  }
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("fileDownload: Error: "));
      _debugPort->print(err);
      _debugPort->print(F(" => {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::appendFileContents(String filename, String str)
{
    return appendFileContents(filename, str.c_str(), str.length());
}


// OK for text files. But will fail with binary files (containing \0) on some platforms.
SARA_R5_error_t SARA_R5::getFileContents(String filename, String *contents)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  // Start by getting the file size so we know in advance how much data to expect
  int fileSize = 0;
  err = getFileSize(filename, &fileSize);
  if (err != SARA_R5_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: getFileSize returned err "));
      _debugPort->println(err);
    }
    return err;
  }
  
  command = sara_r5_calloc_char(strlen(SARA_R5_FILE_SYSTEM_READ_FILE) + filename.length() + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_FILE_SYSTEM_READ_FILE, filename.c_str());

  response = sara_r5_calloc_char(fileSize + 2*minimumResponseAllocation);
  if (response == NULL)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: response alloc failed: "));
      _debugPort->println(fileSize + 2*minimumResponseAllocation);
    }
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // A large file will completely fill the backlog buffer - but it will be pruned afterwards
  // Note to self: if the file contents contain "OK\r\n" sendCommandWithResponse will return true too early...
  // To try and avoid this, look for \"\r\nOK\r\n
  const char fileReadTerm[] = "\r\nOK\r\n";
  err = sendCommandWithResponse(command, fileReadTerm,
                                response, (10 * SARA_R5_STANDARD_RESPONSE_TIMEOUT),
                                (fileSize + 2*minimumResponseAllocation));

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: sendCommandWithResponse returned err "));
      _debugPort->print(">>>");_debugPort->print(response);_debugPort->print("<<<");
      _debugPort->println(err);
    }
    free(command);
    free(response);
    return err;
  }

  // Response format: \r\n+URDFILE: "filename",36,"these bytes are the data of the file"\r\n\r\nOK\r\n
  int scanned = 0;
  int readFileSize = 0;
  char *searchPtr = strstr(response, "+URDFILE: ");
  if (searchPtr != NULL)
  {
    searchPtr = strchr(searchPtr, '\"'); // Find the first quote
    searchPtr = strchr(++searchPtr, '\"'); // Find the second quote

    scanned = sscanf(searchPtr, "\",%d,", &readFileSize); // Get the file size (again)
    if (scanned == 1)
    {
      searchPtr = strchr(++searchPtr, '\"'); // Find the third quote

      if (searchPtr == NULL)
      {
        if (_printDebug == true)
        {
          _debugPort->println(F("getFileContents: third quote not found!"));
        }
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }

      int bytesRead = 0;

      while (bytesRead < readFileSize)
      {
        searchPtr++; // Increment searchPtr then copy file char into contents
      // Important Note: some implementations of concat, like the one on ESP32, are binary-compatible.
      // But some, like SAMD, are not. They use strlen or strcpy internally - which don't like \0's.
      // The only true binary-compatible solution is to use getFileContents(String filename, char *contents)...
        contents->concat(*(searchPtr)); // Append file char to contents
        bytesRead++;
      }
      if (_printDebug == true)
      {
        _debugPort->print(F("getFileContents: total bytes read: "));
        _debugPort->println(bytesRead);
      }
      err = SARA_R5_ERROR_SUCCESS;
    }
    else
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("getFileContents: sscanf failed! scanned is "));
        _debugPort->println(scanned);
      }
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }
  }
  else
  {
    if (_printDebug == true)
      _debugPort->println(F("getFileContents: strstr failed!"));
    err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

// OK for binary files. Make sure contents can hold the entire file. Get the size first with getFileSize.
SARA_R5_error_t SARA_R5::getFileContents(String filename, char *contents)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  // Start by getting the file size so we know in advance how much data to expect
  int fileSize = 0;
  err = getFileSize(filename, &fileSize);
  if (err != SARA_R5_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: getFileSize returned err "));
      _debugPort->println(err);
    }
    return err;
  }

  command = sara_r5_calloc_char(strlen(SARA_R5_FILE_SYSTEM_READ_FILE) + filename.length() + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_FILE_SYSTEM_READ_FILE, filename.c_str());

  response = sara_r5_calloc_char(fileSize + minimumResponseAllocation);
  if (response == NULL)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: response alloc failed: "));
      _debugPort->println(fileSize + minimumResponseAllocation);
    }
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  // A large file will completely fill the backlog buffer - but it will be pruned afterwards
  // Note to self: if the file contents contain "OK\r\n" sendCommandWithResponse will return true too early...
  // To try and avoid this, look for \"\r\nOK\r\n
  const char fileReadTerm[] = "\"\r\nOK\r\n";
  err = sendCommandWithResponse(command, fileReadTerm,
                                response, (5 * SARA_R5_STANDARD_RESPONSE_TIMEOUT),
                                (fileSize + minimumResponseAllocation));

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileContents: sendCommandWithResponse returned err "));
      _debugPort->println(err);
    }
    free(command);
    free(response);
    return err;
  }

  // Response format: \r\n+URDFILE: "filename",36,"these bytes are the data of the file"\r\n\r\nOK\r\n
  int scanned = 0;
  int readFileSize = 0;
  char *searchPtr = strstr(response, "+URDFILE: ");
  if (searchPtr != NULL)
  {
    searchPtr = strchr(searchPtr, '\"'); // Find the first quote
    searchPtr = strchr(++searchPtr, '\"'); // Find the second quote

    scanned = sscanf(searchPtr, "\",%d,", &readFileSize); // Get the file size (again)
    if (scanned == 1)
    {
      searchPtr = strchr(++searchPtr, '\"'); // Find the third quote

      if (searchPtr == NULL)
      {
        if (_printDebug == true)
        {
          _debugPort->println(F("getFileContents: third quote not found!"));
        }
        free(command);
        free(response);
        return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
      }

      int bytesRead = 0;

      while (bytesRead < readFileSize)
      {
        searchPtr++; // Increment searchPtr then copy file char into contents
        contents[bytesRead] = *searchPtr; // Append file char to contents
        bytesRead++;
      }
      if (_printDebug == true)
      {
        _debugPort->print(F("getFileContents: total bytes read: "));
        _debugPort->println(bytesRead);
      }
      err = SARA_R5_ERROR_SUCCESS;
    }
    else
    {
      if (_printDebug == true)
      {
        _debugPort->print(F("getFileContents: sscanf failed! scanned is "));
        _debugPort->println(scanned);
      }
      err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
    }
  }
  else
  {
    if (_printDebug == true)
      _debugPort->println(F("getFileContents: strstr failed!"));
    err = SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::getFileSize(String filename, int *size)
{
  SARA_R5_error_t err;
  char *command;
  char *response;

  command = sara_r5_calloc_char(strlen(SARA_R5_FILE_SYSTEM_LIST_FILES) + filename.length() + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=2,\"%s\"", SARA_R5_FILE_SYSTEM_LIST_FILES, filename.c_str());

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileSize: Fail: Error: "));
      _debugPort->print(err);
      _debugPort->print(F("  Response: {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
    free(command);
    free(response);
    return err;
  }

  char *responseStart = strstr(response, "+ULSTFILE: ");
  if (responseStart == NULL)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getFileSize: Failure: {"));
      _debugPort->print(response);
      _debugPort->println(F("}"));
    }
    free(command);
    free(response);
    return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  int fileSize;
  sscanf(responseStart, "+ULSTFILE: %d", &fileSize);
  *size = fileSize;

  free(command);
  free(response);
  return err;
}

SARA_R5_error_t SARA_R5::deleteFile(String filename)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_FILE_SYSTEM_DELETE_FILE) + filename.length() + 8);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=\"%s\"", SARA_R5_FILE_SYSTEM_DELETE_FILE, filename.c_str());

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  if (err != SARA_R5_ERROR_SUCCESS)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("deleteFile: Fail: Error: "));
      _debugPort->println(err);
    }
  }

  free(command);
  return err;
}

SARA_R5_error_t SARA_R5::modulePowerOff(void)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_POWER_OFF) + 6);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  sprintf(command, "%s", SARA_R5_COMMAND_POWER_OFF);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR, NULL,
                                SARA_R5_POWER_OFF_TIMEOUT);

  free(command);
  return err;
}

void SARA_R5::modulePowerOn(void)
{
  if (_powerPin >= 0)
  {
    powerOn();
  }
  else
  {
    if (_printDebug == true)
      _debugPort->println(F("modulePowerOn: not supported. _powerPin not defined."));
  }
}

/////////////
// Private //
/////////////

SARA_R5_error_t SARA_R5::init(unsigned long baud,
                              SARA_R5::SARA_R5_init_type_t initType)
{
  int retries = _maxInitTries;
  SARA_R5_error_t err = SARA_R5_ERROR_SUCCESS;
  
  beginSerial(baud);
  
  do
  {
    if (_printDebug == true)
      _debugPort->println(F("init: Begin module init."));

    if (initType == SARA_R5_INIT_AUTOBAUD)
    {
      if (_printDebug == true)
        _debugPort->println(F("init: Attempting autobaud connection to module."));
      
      err = autobaud(baud);
      
      if (err != SARA_R5_ERROR_SUCCESS) {
        initType = SARA_R5_INIT_RESET;
      }
    }
    else if (initType == SARA_R5_INIT_RESET)
    {
      if (_printDebug == true)
        _debugPort->println(F("init: Power cycling module."));
      
      powerOff();
      delay(SARA_R5_POWER_OFF_PULSE_PERIOD);
      powerOn();
      beginSerial(baud);
      delay(2000);
      
      err = at();
      if (err != SARA_R5_ERROR_SUCCESS)
      {
         initType = SARA_R5_INIT_AUTOBAUD;
      }
    }
    if (err == SARA_R5_ERROR_SUCCESS)
    {
      err = enableEcho(false); // = disableEcho
      if (err != SARA_R5_ERROR_SUCCESS)
      {
        if (_printDebug == true)
          _debugPort->println(F("init: Module failed echo test."));
        initType =  SARA_R5_INIT_AUTOBAUD;
      }
    }
  }
  while ((retries --) && (err != SARA_R5_ERROR_SUCCESS));
  
  // we tried but seems failed
  if (err != SARA_R5_ERROR_SUCCESS) {
    if (_printDebug == true)
      _debugPort->println(F("init: Module failed to init. Exiting."));
    return (SARA_R5_ERROR_NO_RESPONSE);
  }

  if (_printDebug == true)
    _debugPort->println(F("init: Module responded successfully."));

  _baud = baud;
  setGpioMode(GPIO1, NETWORK_STATUS);
  //setGpioMode(GPIO2, GNSS_SUPPLY_ENABLE);
  setGpioMode(GPIO6, TIME_PULSE_OUTPUT);
  setSMSMessageFormat(SARA_R5_MESSAGE_FORMAT_TEXT);
  autoTimeZone(_autoTimeZoneForBegin);
  for (int i = 0; i < SARA_R5_NUM_SOCKETS; i++)
  {
    socketClose(i, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  }

  return SARA_R5_ERROR_SUCCESS;
}

void SARA_R5::invertPowerPin(bool invert)
{
  _invertPowerPin = invert;
}

// Do a graceful power off. Hold the PWR_ON pin low for SARA_R5_POWER_OFF_PULSE_PERIOD
// Note: +CPWROFF () is preferred to this.
void SARA_R5::powerOff(void)
{
  if (_powerPin >= 0)
  {
    if (_invertPowerPin) // Set the pin state before making it an output
      digitalWrite(_powerPin, HIGH);
    else
      digitalWrite(_powerPin, LOW);
    pinMode(_powerPin, OUTPUT);
    if (_invertPowerPin) // Set the pin state
      digitalWrite(_powerPin, HIGH);
    else
      digitalWrite(_powerPin, LOW);
    delay(SARA_R5_POWER_OFF_PULSE_PERIOD);
    pinMode(_powerPin, INPUT); // Return to high-impedance, rely on (e.g.) SARA module internal pull-up
    if (_printDebug == true)
      _debugPort->println(F("powerOff: complete"));
  }
}

void SARA_R5::powerOn(void)
{
  if (_powerPin >= 0)
  {
    if (_invertPowerPin) // Set the pin state before making it an output
      digitalWrite(_powerPin, HIGH);
    else
      digitalWrite(_powerPin, LOW);
    pinMode(_powerPin, OUTPUT);
    if (_invertPowerPin) // Set the pin state
      digitalWrite(_powerPin, HIGH);
    else
      digitalWrite(_powerPin, LOW);
    delay(SARA_R5_POWER_ON_PULSE_PERIOD);
    pinMode(_powerPin, INPUT); // Return to high-impedance, rely on (e.g.) SARA module internal pull-up
    //delay(2000);               // Do this in init. Wait before sending AT commands to module. 100 is too short.
    if (_printDebug == true)
      _debugPort->println(F("powerOn: complete"));
  }
}

//This does an abrupt emergency hardware shutdown of the SARA-R5 series modules.
//It only works if you have access to both the RESET_N and PWR_ON pins.
//You cannot use this function on the SparkFun Asset Tracker and RESET_N is tied to the MicroMod processor !RESET!...
void SARA_R5::hwReset(void)
{
  if ((_resetPin >= 0) && (_powerPin >= 0))
  {
    digitalWrite(_resetPin, HIGH); // Start by making sure the RESET_N pin is high
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, HIGH);

    if (_invertPowerPin) // Now pull PWR_ON low - invert as necessary (on the Asset Tracker)
    {
      digitalWrite(_powerPin, HIGH); // Inverted - Asset Tracker
      pinMode(_powerPin, OUTPUT);
      digitalWrite(_powerPin, HIGH);
    }
    else
    {
      digitalWrite(_powerPin, LOW); // Not inverted
      pinMode(_powerPin, OUTPUT);
      digitalWrite(_powerPin, LOW);
    }

    delay(SARA_R5_RESET_PULSE_PERIOD); // Wait 23 seconds... (Yes, really!)

    digitalWrite(_resetPin, LOW); // Now pull RESET_N low

    delay(100); // Wait a little... (The data sheet doesn't say how long for)

    if (_invertPowerPin) // Now pull PWR_ON high - invert as necessary (on the Asset Tracker)
    {
      digitalWrite(_powerPin, LOW); // Inverted - Asset Tracker
    }
    else
    {
      digitalWrite(_powerPin, HIGH); // Not inverted
    }

    delay(1500); // Wait 1.5 seconds

    digitalWrite(_resetPin, HIGH); // Now pull RESET_N high again

    pinMode(_resetPin, INPUT); // Return to high-impedance, rely on SARA module internal pull-up
    pinMode(_powerPin, INPUT); // Return to high-impedance, rely on SARA module internal pull-up
  }
}

SARA_R5_error_t SARA_R5::functionality(SARA_R5_functionality_t function)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_FUNC) + 16);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s=%d", SARA_R5_COMMAND_FUNC, function);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_3_MIN_TIMEOUT);

  free(command);

  return err;
}

SARA_R5_error_t SARA_R5::setMNOprofile(mobile_network_operator_t mno, bool autoReset, bool urcNotification)
{
  SARA_R5_error_t err;
  char *command;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_MNO) + 9);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  if (mno == MNO_SIM_ICCID) // Only add autoReset and urcNotification if mno is MNO_SIM_ICCID
    sprintf(command, "%s=%d,%d,%d", SARA_R5_COMMAND_MNO, (uint8_t)mno, (uint8_t)autoReset, (uint8_t)urcNotification);
  else
    sprintf(command, "%s=%d", SARA_R5_COMMAND_MNO, (uint8_t)mno);

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                NULL, SARA_R5_STANDARD_RESPONSE_TIMEOUT);

  free(command);

  return err;
}

SARA_R5_error_t SARA_R5::getMNOprofile(mobile_network_operator_t *mno)
{
  SARA_R5_error_t err;
  char *command;
  char *response;
  mobile_network_operator_t o;
  int d;
  int r;
  int u;
  int oStore;

  command = sara_r5_calloc_char(strlen(SARA_R5_COMMAND_MNO) + 2);
  if (command == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  sprintf(command, "%s?", SARA_R5_COMMAND_MNO);

  response = sara_r5_calloc_char(minimumResponseAllocation);
  if (response == NULL)
  {
    free(command);
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  err = sendCommandWithResponse(command, SARA_R5_RESPONSE_OK_OR_ERROR,
                                response, SARA_R5_STANDARD_RESPONSE_TIMEOUT);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(command);
    free(response);
    return err;
  }

  int scanned = 0;
  char *searchPtr = strstr(response, "+UMNOPROF: ");
  if (searchPtr != NULL)
    scanned = sscanf(searchPtr, "+UMNOPROF: %d,%d,%d,%d", &oStore, &d, &r, &u);
  o = (mobile_network_operator_t)oStore;

  if (scanned >= 1)
  {
    if (_printDebug == true)
    {
      _debugPort->print(F("getMNOprofile: MNO is: "));
      _debugPort->println(o);
    }
    *mno = o;
  }
  else
  {
    err = SARA_R5_ERROR_INVALID;
  }

  free(command);
  free(response);

  return err;
}

SARA_R5_error_t SARA_R5::waitForResponse(const char *expectedResponse, const char *expectedError, uint16_t timeout)
{
  unsigned long timeIn;
  bool found = false;
  bool error = false;
  int responseIndex = 0, errorIndex = 0;
  // bool printedSomething = false;

  timeIn = millis();

  int responseLen = (int)strlen(expectedResponse);
  int errorLen = (int)strlen(expectedError);
  
  while ((!found) && ((timeIn + timeout) > millis()))
  {
    if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      char c = readChar();
      // if (_printDebug == true)
      // {
      //   if (printedSomething == false)
      //     _debugPort->print(F("waitForResponse: "));
      //   _debugPort->write(c);
      //   printedSomething = true;
      // }
      if ((responseIndex < responseLen) && (c == expectedResponse[responseIndex]))
      {
        if (++responseIndex == responseLen)
        {
          found = true;
        }
      }
      else
      {
        responseIndex = ((responseIndex < responseLen) && (c == expectedResponse[0])) ? 1 : 0;
      }
      if ((errorIndex < errorLen) && (c == expectedError[errorIndex]))
      {
        if (++errorIndex == errorLen)
        {
          error = true;
          found = true;
        }
      }
      else
      {
        errorIndex = ((errorIndex < errorLen) && (c == expectedError[0])) ? 1 : 0;
      }
      //_saraResponseBacklog is a global array that holds the backlog of any events
      //that came in while waiting for response. To be processed later within bufferedPoll().
      //Note: the expectedResponse or expectedError will also be added to the backlog.
      //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
      //bufferedPoll uses strtok - which does not like NULL characters.
      //So let's make sure no NULLs end up in the backlog!
      if (_saraResponseBacklogLength < _RXBuffSize) // Don't overflow the buffer
      {
        if (c == '\0')
          _saraResponseBacklog[_saraResponseBacklogLength++] = '0'; // Change NULLs to ASCII Zeros
        else
          _saraResponseBacklog[_saraResponseBacklogLength++] = c;
      }
    } else {
      yield();
    }
  }

  // if (_printDebug == true)
  //   if (printedSomething)
  //     _debugPort->println();

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  if (found == true)
  {
    if (true == _printAtDebug) {
      _debugAtPort->print((error == true) ? expectedError : expectedResponse);
    }
    
    return (error == true) ? SARA_R5_ERROR_ERROR : SARA_R5_ERROR_SUCCESS;
  }

  return SARA_R5_ERROR_NO_RESPONSE;
}

SARA_R5_error_t SARA_R5::sendCommandWithResponse(
    const char *command, const char *expectedResponse, char *responseDest,
    unsigned long commandTimeout, int destSize, bool at)
{
  bool found = false;
  bool error = false;
  int responseIndex = 0;
  int errorIndex = 0;
  int destIndex = 0;
  unsigned int charsRead = 0;
  int responseLen = 0;
  int errorLen = 0;
  const char* expectedError= NULL;
  //bool printedSomething = false;

  if (_printDebug == true)
  {
    _debugPort->print(F("sendCommandWithResponse: Command: "));
    _debugPort->println(String(command));
  }
  
  sendCommand(command, at); //Sending command needs to dump data to backlog buffer as well.
  unsigned long timeIn = millis();
  if (SARA_R5_RESPONSE_OK_OR_ERROR == expectedResponse) {
    expectedResponse = SARA_R5_RESPONSE_OK;
    expectedError = SARA_R5_RESPONSE_ERROR;
    responseLen = sizeof(SARA_R5_RESPONSE_OK)-1;
    errorLen = sizeof(SARA_R5_RESPONSE_ERROR)-1;
  } else {
    responseLen = (int)strlen(expectedResponse);
  }
  
  while ((!found) && ((timeIn + commandTimeout) > millis()))
  {
    if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
    {
      char c = readChar();
      // if (_printDebug == true)
      // {
      //   if (printedSomething == false)
      //   {
      //     _debugPort->print(F("sendCommandWithResponse: Response: "));
      //     printedSomething = true;
      //   }
      //   _debugPort->write(c);
      // }
      if (responseDest != NULL)
      {
        if (destIndex < destSize) // Only add this char to response if there is room for it
          responseDest[destIndex] = c;
        destIndex++;
        if (destIndex == destSize)
        {
          if (_printDebug == true)
          {
            // if (printedSomething)
            //   _debugPort->println();
            _debugPort->print(F("sendCommandWithResponse: Panic! responseDest is full!"));
            // if (printedSomething)
            //   _debugPort->print(F("sendCommandWithResponse: Ignored response: "));
          }
        }
      }
      charsRead++;
      if ((errorIndex < errorLen) && (c == expectedError[errorIndex]))
      {
        if (++errorIndex == errorLen)
        {
          error = true;
          found = true;
        }
      }
      else
      {
        errorIndex = ((errorIndex < errorLen) && (c == expectedError[0])) ? 1 : 0;
      }
      if ((responseIndex < responseLen) && (c == expectedResponse[responseIndex]))
      {
        if (++responseIndex == responseLen)
        {
          found = true;
        }
      }
      else
      {
        responseIndex = ((responseIndex < responseLen) && (c == expectedResponse[0])) ? 1 : 0;
      }
      //_saraResponseBacklog is a global array that holds the backlog of any events
      //that came in while waiting for response. To be processed later within bufferedPoll().
      //Note: the expectedResponse or expectedError will also be added to the backlog
      //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
      //bufferedPoll uses strtok - which does not like NULL characters.
      //So let's make sure no NULLs end up in the backlog!
      if (_saraResponseBacklogLength < _RXBuffSize) // Don't overflow the buffer
      {
        if (c == '\0')
          _saraResponseBacklog[_saraResponseBacklogLength++] = '0'; // Change NULLs to ASCII Zeros
        else
          _saraResponseBacklog[_saraResponseBacklogLength++] = c;
      }
    } else {
      yield();
    }
  }
  
  // if (_printDebug == true)
  //   if (printedSomething)
  //     _debugPort->println();

  pruneBacklog(); // Prune any incoming non-actionable URC's and responses/errors from the backlog

  if (found)
  {
    if ((true == _printAtDebug) && ((NULL != responseDest) || (NULL != expectedResponse))) {
      _debugAtPort->print((NULL != responseDest) ? responseDest : expectedResponse);
    }
    return error ? SARA_R5_ERROR_ERROR : SARA_R5_ERROR_SUCCESS;
  }
  else if (charsRead == 0)
  {
    return SARA_R5_ERROR_NO_RESPONSE;
  }
  else
  {
    if ((true == _printAtDebug) && (NULL != responseDest)) {
      _debugAtPort->print(responseDest);
    }
    return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }
}

// Send a custom command with an expected (potentially partial) response, store entire response
SARA_R5_error_t SARA_R5::sendCustomCommandWithResponse(const char *command, const char *expectedResponse,
                                                       char *responseDest, unsigned long commandTimeout, bool at)
{
  // Assume the user has allocated enough storage for any response. Set destSize to 32766.
  return sendCommandWithResponse(command, expectedResponse, responseDest, commandTimeout, 32766, at);
}

void SARA_R5::sendCommand(const char *command, bool at)
{
  //Check for incoming serial data. Copy it into the backlog
  
  // Important note:
  // On ESP32, Serial.available only provides an update every ~120 bytes during the reception of long messages:
  // https://gitter.im/espressif/arduino-esp32?at=5e25d6370a1cf54144909c85
  // Be aware that if a long message is being received, the code below will timeout after _rxWindowMillis = 2 millis.
  // At 115200 baud, hwAvailable takes ~120 * 10 / 115200 = 10.4 millis before it indicates that data is being received.

  unsigned long timeIn = millis();
  if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
  {
    while (((millis() - timeIn) < _rxWindowMillis) && (_saraResponseBacklogLength < _RXBuffSize)) //May need to escape on newline?
    {
      if (hwAvailable() > 0) //hwAvailable can return -1 if the serial port is NULL
      {
        //_saraResponseBacklog is a global array that holds the backlog of any events
        //that came in while waiting for response. To be processed later within bufferedPoll().
        //Note: the expectedResponse or expectedError will also be added to the backlog
        //The backlog is only used by bufferedPoll to process the URCs - which are all readable.
        //bufferedPoll uses strtok - which does not like NULL characters.
        //So let's make sure no NULLs end up in the backlog!
        char c = readChar();
        if (c == '\0') // Make sure no NULL characters end up in the backlog! Change them to ASCII Zeros
          c = '0';
        _saraResponseBacklog[_saraResponseBacklogLength++] = c;
        timeIn = millis();
      } else {
        yield();
      }
    }
  }

  //Now send the command
  if (at)
  {
    hwPrint(SARA_R5_COMMAND_AT);
    hwPrint(command);
    hwPrint("\r\n");
  }
  else
  {
    hwPrint(command);
  }
}

SARA_R5_error_t SARA_R5::parseSocketReadIndication(int socket, int length)
{
  SARA_R5_error_t err;
  char *readDest;

  if ((socket < 0) || (length < 0))
  {
    return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  // Return now if both callbacks pointers are NULL - otherwise the data will be read and lost!
  if ((_socketReadCallback == NULL) && (_socketReadCallbackPlus == NULL))
    return SARA_R5_ERROR_INVALID;

  readDest = sara_r5_calloc_char(length + 1);
  if (readDest == NULL)
    return SARA_R5_ERROR_OUT_OF_MEMORY;

  int bytesRead;
  err = socketRead(socket, length, readDest, &bytesRead);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(readDest);
    return err;
  }

  if (_socketReadCallback != NULL)
  {
    String dataAsString = ""; // Create an empty string
    // Copy the data from readDest into the String in a binary-compatible way
    // Important Note: some implementations of concat, like the one on ESP32, are binary-compatible.
    // But some, like SAMD, are not. They use strlen or strcpy internally - which don't like \0's.
    // The only true binary-compatible solution is to use socketReadCallbackPlus...
    for (int i = 0; i < bytesRead; i++)
      dataAsString.concat(readDest[i]);
    _socketReadCallback(socket, dataAsString);
  }

  if (_socketReadCallbackPlus != NULL)
  {
    IPAddress dummyAddress = { 0, 0, 0, 0 };
    int dummyPort = 0;
    _socketReadCallbackPlus(socket, (const char *)readDest, bytesRead, dummyAddress, dummyPort);
  }

  free(readDest);
  return SARA_R5_ERROR_SUCCESS;
}

SARA_R5_error_t SARA_R5::parseSocketReadIndicationUDP(int socket, int length)
{
  SARA_R5_error_t err;
  char *readDest;
  IPAddress remoteAddress = { 0, 0, 0, 0 };
  int remotePort = 0;

  if ((socket < 0) || (length < 0))
  {
    return SARA_R5_ERROR_UNEXPECTED_RESPONSE;
  }

  // Return now if both callbacks pointers are NULL - otherwise the data will be read and lost!
  if ((_socketReadCallback == NULL) && (_socketReadCallbackPlus == NULL))
    return SARA_R5_ERROR_INVALID;

  readDest = sara_r5_calloc_char(length + 1);
  if (readDest == NULL)
  {
    return SARA_R5_ERROR_OUT_OF_MEMORY;
  }

  int bytesRead;
  err = socketReadUDP(socket, length, readDest, &remoteAddress, &remotePort, &bytesRead);
  if (err != SARA_R5_ERROR_SUCCESS)
  {
    free(readDest);
    return err;
  }

  if (_socketReadCallback != NULL)
  {
    String dataAsString = ""; // Create an empty string
    // Important Note: some implementations of concat, like the one on ESP32, are binary-compatible.
    // But some, like SAMD, are not. They use strlen or strcpy internally - which don't like \0's.
    // The only true binary-compatible solution is to use socketReadCallbackPlus...
    for (int i = 0; i < bytesRead; i++) // Copy the data from readDest into the String in a binary-compatible way
      dataAsString.concat(readDest[i]);
    _socketReadCallback(socket, dataAsString);
  }

  if (_socketReadCallbackPlus != NULL)
  {
    _socketReadCallbackPlus(socket, (const char *)readDest, bytesRead, remoteAddress, remotePort);
  }

  free(readDest);
  return SARA_R5_ERROR_SUCCESS;
}

SARA_R5_error_t SARA_R5::parseSocketListenIndication(int listeningSocket, IPAddress localIP, unsigned int listeningPort, int socket, IPAddress remoteIP, unsigned int port)
{
  _lastLocalIP = localIP;
  _lastRemoteIP = remoteIP;

  if (_socketListenCallback != NULL)
  {
    _socketListenCallback(listeningSocket, localIP, listeningPort, socket, remoteIP, port);
  }

  return SARA_R5_ERROR_SUCCESS;
}

SARA_R5_error_t SARA_R5::parseSocketCloseIndication(String *closeIndication)
{
  int search;
  int socket;

  search = closeIndication->indexOf("UUSOCL: ") + strlen("UUSOCL: ");

  // Socket will be first integer, should be single-digit number between 0-6:
  socket = closeIndication->substring(search, search + 1).toInt();

  if (_socketCloseCallback != NULL)
  {
    _socketCloseCallback(socket);
  }

  return SARA_R5_ERROR_SUCCESS;
}

size_t SARA_R5::hwPrint(const char *s)
{
  if ((true == _printAtDebug) && (NULL != s)) {
    _debugAtPort->print(s);
  }
  if (_hardSerial != NULL)
  {
    return _hardSerial->print(s);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->print(s);
  }
#endif

  return (size_t)0;
}

size_t SARA_R5::hwWriteData(const char *buff, int len)
{
  if ((true == _printAtDebug) && (NULL != buff) && (0 < len) ) {
    _debugAtPort->write(buff,len);
  }
  if (_hardSerial != NULL)
  {
    return _hardSerial->write((const uint8_t *)buff, len);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->write((const uint8_t *)buff, len);
  }
#endif
  return (size_t)0;
}

size_t SARA_R5::hwWrite(const char c)
{
  if (true == _printAtDebug) {
    _debugAtPort->write(c);
  }
  if (_hardSerial != NULL)
  {
    return _hardSerial->write(c);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->write(c);
  }
#endif

  return (size_t)0;
}

int SARA_R5::readAvailable(char *inString)
{
  int len = 0;

  if (_hardSerial != NULL)
  {
    while (_hardSerial->available())
    {
      char c = (char)_hardSerial->read();
      if (inString != NULL)
      {
        inString[len++] = c;
      }
    }
    if (inString != NULL)
    {
      inString[len] = 0;
    }
    //if (_printDebug == true)
    //  _debugPort->println(inString);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    while (_softSerial->available())
    {
      char c = (char)_softSerial->read();
      if (inString != NULL)
      {
        inString[len++] = c;
      }
    }
    if (inString != NULL)
    {
      inString[len] = 0;
    }
  }
#endif

  return len;
}

char SARA_R5::readChar(void)
{
  char ret = 0;

  if (_hardSerial != NULL)
  {
    ret = (char)_hardSerial->read();
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    ret = (char)_softSerial->read();
  }
#endif

  return ret;
}

int SARA_R5::hwAvailable(void)
{
  if (_hardSerial != NULL)
  {
    return _hardSerial->available();
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    return _softSerial->available();
  }
#endif

  return -1;
}

void SARA_R5::beginSerial(unsigned long baud)
{
  delay(100);
  if (_hardSerial != NULL)
  {
    _hardSerial->updateBaudRate(baud);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    _softSerial->end();
    _softSerial->begin(baud);
  }
#endif
  delay(100);
}

void SARA_R5::setTimeout(unsigned long timeout)
{
  if (_hardSerial != NULL)
  {
    _hardSerial->setTimeout(timeout);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    _softSerial->setTimeout(timeout);
  }
#endif
}

bool SARA_R5::find(char *target)
{
  bool found = false;
  if (_hardSerial != NULL)
  {
    found = _hardSerial->find(target);
  }
#ifdef SARA_R5_SOFTWARE_SERIAL_ENABLED
  else if (_softSerial != NULL)
  {
    found = _softSerial->find(target);
  }
#endif
  return found;
}

SARA_R5_error_t SARA_R5::autobaud(unsigned long desiredBaud)
{
  SARA_R5_error_t err = SARA_R5_ERROR_INVALID;
  int b = 0;

  while ((err != SARA_R5_ERROR_SUCCESS) && (b < NUM_SUPPORTED_BAUD))
  {
    beginSerial(SARA_R5_SUPPORTED_BAUD[b++]);
    setBaud(desiredBaud);
    beginSerial(desiredBaud);
    err = at();
  }
  if (err == SARA_R5_ERROR_SUCCESS)
  {
    beginSerial(desiredBaud);
  }
  return err;
}

char *SARA_R5::sara_r5_calloc_char(size_t num)
{
  return (char *)calloc(num, sizeof(char));
}

//This prunes the backlog of non-actionable events. If new actionable events are added, you must modify the if statement.
void SARA_R5::pruneBacklog()
{
  char *event;

  // if (_printDebug == true)
  // {
  //   if (_saraResponseBacklogLength > 0) //Handy for debugging new parsing.
  //   {
  //     _debugPort->println(F("pruneBacklog: before pruning, backlog was:"));
  //     _debugPort->println(_saraResponseBacklog);
  //     _debugPort->println(F("pruneBacklog: end of backlog"));
  //   }
  //   else
  //   {
  //     _debugPort->println(F("pruneBacklog: backlog was empty"));
  //   }
  // }

  memset(_pruneBuffer, 0, _RXBuffSize); // Clear the _pruneBuffer

  _saraResponseBacklogLength = 0; // Zero the backlog length

  char *preservedEvent;
  event = strtok_r(_saraResponseBacklog, "\r\n", &preservedEvent); // Look for an 'event' - something ending in \r\n

  while (event != NULL) //If event is actionable, add it to pruneBuffer.
  {
    // These are the events we want to keep so they can be processed by poll / bufferedPoll
    if ((strstr(event, "+UUSORD:") != NULL)
        || (strstr(event, "+UUSORF:") != NULL)
        || (strstr(event, "+UUSOLI:") != NULL)
        || (strstr(event, "+UUSOCL:") != NULL)
        || (strstr(event, "+UULOC:") != NULL)
        || (strstr(event, "+UUSIMSTAT:") != NULL)
        || (strstr(event, "+UUPSDA:") != NULL)
        || (strstr(event, "+UUPING:") != NULL)
        || (strstr(event, "+UUMQTTC:") != NULL)
        || (strstr(event, "+UUCREG:") != NULL)
        || (strstr(event, "+UUCEREG:") != NULL)
        || (strstr(event, "+UUHTTPCR:") != NULL))
    {
      strcat(_pruneBuffer, event); // The URCs are all readable text so using strcat is OK
      strcat(_pruneBuffer, "\r\n"); // strtok blows away delimiter, but we want that for later.
      _saraResponseBacklogLength += strlen(event) + 2; // Add the length of this event to _saraResponseBacklogLength
    }

    event = strtok_r(NULL, "\r\n", &preservedEvent); // Walk though any remaining events
  }

  memset(_saraResponseBacklog, 0, _RXBuffSize); //Clear out backlog buffer.
  memcpy(_saraResponseBacklog, _pruneBuffer, _saraResponseBacklogLength); //Copy the pruned buffer back into the backlog

  // if (_printDebug == true)
  // {
  //   if (_saraResponseBacklogLength > 0) //Handy for debugging new parsing.
  //   {
  //     _debugPort->println(F("pruneBacklog: after pruning, backlog is now:"));
  //     _debugPort->println(_saraResponseBacklog);
  //     _debugPort->println(F("pruneBacklog: end of backlog"));
  //   }
  //   else
  //   {
  //     _debugPort->println(F("pruneBacklog: backlog is now empty"));
  //   }
  // }

  free(event);
}

// GPS Helper Functions:

// Read a source string until a delimiter is hit, store the result in destination
char *SARA_R5::readDataUntil(char *destination, unsigned int destSize,
                             char *source, char delimiter)
{

  char *strEnd;
  size_t len;

  strEnd = strchr(source, delimiter);

  if (strEnd != NULL)
  {
    len = strEnd - source;
    memset(destination, 0, destSize);
    memcpy(destination, source, len);
  }

  return strEnd;
}

bool SARA_R5::parseGPRMCString(char *rmcString, PositionData *pos,
                               ClockData *clk, SpeedData *spd)
{
  char *ptr, *search;
  unsigned long tTemp;
  char tempData[TEMP_NMEA_DATA_SIZE];

  // if (_printDebug == true)
  // {
  //   _debugPort->println(F("parseGPRMCString: rmcString: "));
  //   _debugPort->println(rmcString);
  // }

  // Fast-forward test to first value:
  ptr = strchr(rmcString, ',');
  ptr++; // Move ptr past first comma

  // If the next character is another comma, there's no time data
  // Find time:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  // Next comma should be present and not the next position
  if ((search != NULL) && (search != ptr))
  {
    pos->utc = atof(tempData);                             // Extract hhmmss.ss as float
    tTemp = pos->utc;                                      // Convert to unsigned long (discard the digits beyond the decimal point)
    clk->time.ms = ((unsigned int)(pos->utc * 100)) % 100; // Extract the milliseconds
    clk->time.hour = tTemp / 10000;
    tTemp -= ((unsigned long)clk->time.hour * 10000);
    clk->time.minute = tTemp / 100;
    tTemp -= ((unsigned long)clk->time.minute * 100);
    clk->time.second = tTemp;
  }
  else
  {
    pos->utc = 0.0;
    clk->time.hour = 0;
    clk->time.minute = 0;
    clk->time.second = 0;
  }
  ptr = search + 1; // Move pointer to next value

  // Find status character:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  // Should be a single character: V = Data invalid, A = Data valid
  if ((search != NULL) && (search == ptr + 1))
  {
    pos->status = *ptr; // Assign char at ptr to status
  }
  else
  {
    pos->status = 'X'; // Made up very bad status
  }
  ptr = search + 1;

  // Find latitude:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    pos->lat = atof(tempData);              // Extract ddmm.mmmmm as float
    unsigned long lat_deg = pos->lat / 100; // Extract the degrees
    pos->lat -= (float)lat_deg * 100.0;     // Subtract the degrees leaving only the minutes
    pos->lat /= 60.0;                       // Convert minutes into degrees
    pos->lat += (float)lat_deg;             // Finally add the degrees back on again
  }
  else
  {
    pos->lat = 0.0;
  }
  ptr = search + 1;

  // Find latitude hemishpere
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'S')    // Is the latitude South
      pos->lat *= -1.0; // Make lat negative
  }
  ptr = search + 1;

  // Find longitude:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    pos->lon = atof(tempData);              // Extract dddmm.mmmmm as float
    unsigned long lon_deg = pos->lon / 100; // Extract the degrees
    pos->lon -= (float)lon_deg * 100.0;     // Subtract the degrees leaving only the minutes
    pos->lon /= 60.0;                       // Convert minutes into degrees
    pos->lon += (float)lon_deg;             // Finally add the degrees back on again
  }
  else
  {
    pos->lon = 0.0;
  }
  ptr = search + 1;

  // Find longitude hemishpere
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'W')    // Is the longitude West
      pos->lon *= -1.0; // Make lon negative
  }
  ptr = search + 1;

  // Find speed
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->speed = atof(tempData); // Extract speed over ground in knots
    spd->speed *= 0.514444;      // Convert to m/s
  }
  else
  {
    spd->speed = 0.0;
  }
  ptr = search + 1;

  // Find course over ground
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->cog = atof(tempData);
  }
  else
  {
    spd->cog = 0.0;
  }
  ptr = search + 1;

  // Find date
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    tTemp = atol(tempData);
    clk->date.day = tTemp / 10000;
    tTemp -= ((unsigned long)clk->date.day * 10000);
    clk->date.month = tTemp / 100;
    tTemp -= ((unsigned long)clk->date.month * 100);
    clk->date.year = tTemp;
  }
  else
  {
    clk->date.day = 0;
    clk->date.month = 0;
    clk->date.year = 0;
  }
  ptr = search + 1;

  // Find magnetic variation in degrees:
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search != ptr))
  {
    spd->magVar = atof(tempData);
  }
  else
  {
    spd->magVar = 0.0;
  }
  ptr = search + 1;

  // Find magnetic variation direction
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, ',');
  if ((search != NULL) && (search == ptr + 1))
  {
    if (*ptr == 'W')       // Is the magnetic variation West
      spd->magVar *= -1.0; // Make magnetic variation negative
  }
  ptr = search + 1;

  // Find position system mode
  // Possible values for posMode: N = No fix, E = Estimated/Dead reckoning fix, A = Autonomous GNSS fix,
  //                              D = Differential GNSS fix, F = RTK float, R = RTK fixed
  search = readDataUntil(tempData, TEMP_NMEA_DATA_SIZE, ptr, '*');
  if ((search != NULL) && (search = ptr + 1))
  {
    pos->mode = *ptr;
  }
  else
  {
    pos->mode = 'X';
  }
  ptr = search + 1;

  if (pos->status == 'A')
  {
    return true;
  }
  return false;
}
