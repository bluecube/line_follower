#pragma once

//#include "RobotHal/util/motion.h"

#include <cstdint>
#include <string_view>

class BluetoothRemote {
protected:
    BluetoothRemote() {} // Bluetooth remote is only accessible through HAL
public:
    BluetoothRemote(const BluetoothRemote&) = delete;
    BluetoothRemote(BluetoothRemote&&) = delete;
    BluetoothRemote& operator=(const BluetoothRemote&) = delete;
    BluetoothRemote& operator=(BluetoothRemote&&) = delete;

    //auto getJoystickState() { return joystickState; }
    void sendTelemetry(std::string_view data);

protected:
    void setup();

};
