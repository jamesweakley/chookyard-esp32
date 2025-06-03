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
    ESP_LOGI(TAG, "Initializing actuator pins with maximum drive capability");
    
    // Configure GPIO pins for actuator control
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ACTUATOR_PIN_OPEN) | (1ULL << ACTUATOR_PIN_CLOSE);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;  // Disable pull-up resistors
    gpio_config(&io_conf);
    
    // Initialize both pins to low
    gpio_set_level(ACTUATOR_PIN_OPEN, 0);
    gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
    
    // Set maximum GPIO drive capability for both pins
    // GPIO_DRIVE_CAP_3 is the strongest drive capability (40mA)
    ESP_ERROR_CHECK(gpio_set_drive_capability(ACTUATOR_PIN_OPEN, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_drive_capability(ACTUATOR_PIN_CLOSE, GPIO_DRIVE_CAP_3));
    
    // Verify drive capability was set correctly
    gpio_drive_cap_t drive_cap_open, drive_cap_close;
    ESP_ERROR_CHECK(gpio_get_drive_capability(ACTUATOR_PIN_OPEN, &drive_cap_open));
    ESP_ERROR_CHECK(gpio_get_drive_capability(ACTUATOR_PIN_CLOSE, &drive_cap_close));
    
    ESP_LOGI(TAG, "Actuator pins initialized: OPEN=%d (drive=%d), CLOSE=%d (drive=%d)",
             ACTUATOR_PIN_OPEN, drive_cap_open, ACTUATOR_PIN_CLOSE, drive_cap_close);
}

void BoltLockManager::controlActuator(bool isOpen)
{
    const int startup_delay = 100;        // 100ms startup delay
    const int direction_change_delay = 300; // 300ms when changing direction
    const int operation_time = 5000;      // 5 seconds of continuous operation
    
    ESP_LOGI(TAG, "Door actuator: Starting %s operation", isOpen ? "OPEN" : "CLOSE");
    
    // First ensure both pins are off to avoid any conflicts
    gpio_set_level(ACTUATOR_PIN_OPEN, 0);
    gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
    vTaskDelay(pdMS_TO_TICKS(direction_change_delay));
    
    if (isOpen) {
        // Open the door
        ESP_LOGI(TAG, "Setting CLOSE pin to LOW");
        gpio_set_level(ACTUATOR_PIN_CLOSE, 0);  // Ensure close pin is off
        vTaskDelay(pdMS_TO_TICKS(startup_delay));
        
        ESP_LOGI(TAG, "Setting OPEN pin to HIGH");
        gpio_set_level(ACTUATOR_PIN_OPEN, 1);   // Activate open pin
        
        // Keep the actuator running for the specified time
        vTaskDelay(pdMS_TO_TICKS(operation_time));
        
        // Turn off the pin when done
        gpio_set_level(ACTUATOR_PIN_OPEN, 0);
        ESP_LOGI(TAG, "Door actuator: OPENED");
    } else {
        // Close the door
        ESP_LOGI(TAG, "Setting OPEN pin to LOW");
        gpio_set_level(ACTUATOR_PIN_OPEN, 0);   // Ensure open pin is off
        vTaskDelay(pdMS_TO_TICKS(startup_delay));
        
        ESP_LOGI(TAG, "Setting CLOSE pin to HIGH");
        gpio_set_level(ACTUATOR_PIN_CLOSE, 1);  // Activate close pin
        
        // Keep the actuator running for the specified time
        vTaskDelay(pdMS_TO_TICKS(operation_time));
        
        // Turn off the pin when done
        gpio_set_level(ACTUATOR_PIN_CLOSE, 0);
        ESP_LOGI(TAG, "Door actuator: CLOSED");
    }
    
    ESP_LOGI(TAG, "Door actuator: Operation completed");
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
