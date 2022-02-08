#include <iostream>
#include <csignal>
//-----------------------------//
#include "Server.h"
//-----------------------------//
bool Running = true;
std::condition_variable MainCV;
std::mutex MainMutex;
//-----------------------------//
void sigtermHandler(int tSigNum) {
    if (tSigNum == SIGTERM) {
        std::scoped_lock <std::mutex> Lock(MainMutex);
        Running = false;
        MainCV.notify_one();

        std::cout << "Terminating..." << std::endl;
    }
}
//-----------------------------//
int main() {
    std::unique_lock <std::mutex> MainLock(MainMutex);
    signal(SIGTERM, sigtermHandler);

    try {
        auto AkulaServer = new Server;
    } catch (const std::runtime_error& tExcept) {
        std::cerr << tExcept.what() << std::endl;
        return -1;
    }

    MainCV.wait(MainLock, []{ return !Running; });

    return 0;
}
