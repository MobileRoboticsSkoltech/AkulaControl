//-----------------------------//
#include "SerialMonitor.h"
//-----------------------------//
int main() {
    SerialMonitor MonitorSTM("/dev/ttyStmVP", 32);
    MonitorSTM.startSerialLoop();

    return 0;
}
