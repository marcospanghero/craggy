#ifndef SESSION_H
#define SESSION_H

#include <inttypes.h>

struct sessionGps {
    uint16_t gpsReceiverId;
    uint8_t  gpsReceiverState;  //This should take the current GPS receiver state: connected, not connected etc
    uint8_t  fixState;          //Here we keep is we are in cold start or not
    uint8_t  timeState;         //Keep the time state here: do we have any connection to the timing service?

} sessionGps;

struct sessionRTime {
    uint8_t isHostOnline;
    uint8_t timestamp;
    uint8_t session_id;
} RTsession;

#endif