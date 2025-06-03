/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include <app/clusters/door-lock-server/door-lock-server.h>

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h" // provides FreeRTOS timer support

#include <lib/core/CHIPError.h>
#include "driver/gpio.h"

// GPIO pins for actuator control
#define ACTUATOR_PIN_OPEN  GPIO_NUM_2  // GPIO pin to open the door
#define ACTUATOR_PIN_CLOSE GPIO_NUM_3  // GPIO pin to close the door

using namespace chip;
using namespace chip::app::Clusters::DoorLock;

class BoltLockManager
{
public:
    enum Action_t
    {
        LOCK_ACTION = 0,
        UNLOCK_ACTION,

        INVALID_ACTION
    } Action;

    CHIP_ERROR InitLockState();
    CHIP_ERROR Init(chip::app::DataModel::Nullable<DlLockState> state);

    bool Lock(chip::EndpointId endpointId, const Optional<chip::ByteSpan> & pin, OperationErrorEnum & err);
    bool Unlock(chip::EndpointId endpointId, const Optional<chip::ByteSpan> & pin, OperationErrorEnum & err);

    bool setLockState(chip::EndpointId endpointId, DlLockState lockState, const Optional<chip::ByteSpan> & pin,
                      OperationErrorEnum & err);
    const char * lockStateToString(DlLockState lockState) const;

private:
    friend BoltLockManager & BoltLockMgr();
    
    // Initialize GPIO pins for actuator control
    void initActuatorPins();
    
    // Control actuator to open/close the door
    void controlActuator(bool isOpen);

    static BoltLockManager sLock;
};

inline BoltLockManager & BoltLockMgr()
{
    return BoltLockManager::sLock;
}
