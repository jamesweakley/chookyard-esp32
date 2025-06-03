/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "door_lock_manager.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <cstring>
#include <esp_log.h>

static const char *TAG = "doorlock_manager";

BoltLockManager BoltLockManager::sLock;

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::DoorLock;

CHIP_ERROR BoltLockManager::Init(DataModel::Nullable<DlLockState> state)
{
    ESP_LOGI(TAG, "Initializing simplified door lock manager");
    
    // Initialize GPIO pins for actuator control
    initActuatorPins();
    
    return CHIP_NO_ERROR;
}

void BoltLockManager::initActuatorPins()
{
    // Configure GPIO pins for actuator control
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ACTUATOR_PIN_OPEN) | (1ULL << ACTUATOR_PIN_CLOSE);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable pull-up resistors
    gpio_config(&io_conf);
    
    // Initialize both pins to low
    gpio_set_level(ACTUATOR_PIN_OPEN, 0);
    gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
    
    ESP_LOGI(TAG, "Actuator pins initialized with pull-ups: OPEN=%d, CLOSE=%d", ACTUATOR_PIN_OPEN, ACTUATOR_PIN_CLOSE);
}

void BoltLockManager::controlActuator(bool isOpen)
{
    if (isOpen) {
        // Open the door
        gpio_set_level(ACTUATOR_PIN_CLOSE, 0);  // Ensure close pin is off
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(ACTUATOR_PIN_OPEN, 1);   // Activate open pin
        
        
        ESP_LOGI(TAG, "Door actuator: OPENED");
    } else {
        // Close the door
        gpio_set_level(ACTUATOR_PIN_OPEN, 0);   // Ensure open pin is off
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(ACTUATOR_PIN_CLOSE, 1);  // Activate close pin
        
        ESP_LOGI(TAG, "Door actuator: CLOSED");
    }
}

bool BoltLockManager::Lock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Lock command received [endpointId=%d]", endpointId);
    return setLockState(endpointId, DlLockState::kLocked, pin, err);
}

bool BoltLockManager::Unlock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Unlock command received [endpointId=%d]", endpointId);
    return setLockState(endpointId, DlLockState::kUnlocked, pin, err);
}

const char * BoltLockManager::lockStateToString(DlLockState lockState) const
{
    switch (lockState)
    {
    case DlLockState::kNotFullyLocked:
        return "Not Fully Locked";
    case DlLockState::kLocked:
        return "Locked";
    case DlLockState::kUnlocked:
        return "Unlocked";
    case DlLockState::kUnlatched:
        return "Unlatched";
    case DlLockState::kUnknownEnumValue:
        break;
    }

    return "Unknown";
}

bool BoltLockManager::setLockState(EndpointId endpointId, DlLockState lockState, const Optional<ByteSpan> & pin,
                                   OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Door Lock App: Setting door lock state to \"%s\" [endpointId=%d]", lockStateToString(lockState), endpointId);
    
    // Update the lock state in the Matter system
    DoorLockServer::Instance().SetLockState(endpointId, lockState);
    
    // Control the actuator based on the lock state
    if (lockState == DlLockState::kLocked) {
        controlActuator(false); // Close the door
    } else if (lockState == DlLockState::kUnlocked) {
        controlActuator(true);  // Open the door
    }
    
    return true;
}

CHIP_ERROR BoltLockManager::InitLockState()
{
    // Initial lock state
    DataModel::Nullable<DlLockState> state;
    EndpointId endpointId{ 1 };
    DoorLock::Attributes::LockState::Get(endpointId, state);

    // Initialize the simplified door lock manager
    CHIP_ERROR err = BoltLockMgr().Init(state);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "BoltLockMgr().Init() failed");
        return err;
    }

    // Set initial state to locked
    OperationErrorEnum opErr;
    setLockState(endpointId, DlLockState::kLocked, Optional<ByteSpan>(), opErr);
    
    ESP_LOGI(TAG, "Door lock initialized successfully");
    return CHIP_NO_ERROR;
}
