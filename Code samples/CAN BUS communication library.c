#include "can_communication.h"

// CAN handle and message structures
CAN_HandleTypeDef hcan;
CanTxMsgTypeDef canTxMessage;
CanRxMsgTypeDef canRxMessage;

// Timeout flag for communication checks
uint8_t can_timeout_flag = 0;

// Function to initialize CAN communication
void CAN_Init(void) {
    // Initialize CAN peripherals (HAL CAN init should be done here)
    // Configure CAN filter, baud rate, etc.
    CAN_FilterConfTypeDef canFilterConfig;

    // Configure the CAN filter for this device's address (this can be dynamically updated for hot-plugging)
    canFilterConfig.FilterNumber = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    canFilterConfig.FilterIdHigh = CAN_DEVICE_ADDRESS << 5;  // Set device's address
    canFilterConfig.FilterIdLow  = CAN_DEVICE_ADDRESS << 5;  // Set device's address
    canFilterConfig.FilterMaskIdHigh = 0x0000;                // Accept all messages for this address
    canFilterConfig.FilterMaskIdLow = 0x0000;                 // Accept all messages for this address
    canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;         // Use FIFO0 for receiving
    canFilterConfig.FilterActivation = ENABLE;                // Enable the filter
    canFilterConfig.BankNumber = 0;

    // Apply the CAN filter configuration
    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

    // Start receiving CAN messages with interrupt mode
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

    // Set up the CAN to transmit data
    hcan.pTxMsg = &canTxMessage;
    hcan.pRxMsg = &canRxMessage;
}

// Function to transmit CAN message
void CAN_Transmit(uint32_t id, uint8_t* data, uint8_t length) {
    canTxMessage.StdId = id;   // Set the standard ID of the message
    canTxMessage.DLC = length; // Set the length of the data
    for (uint8_t i = 0; i < length; i++) {
        canTxMessage.Data[i] = data[i]; // Set the message data
    }

    // Transmit the message using CAN interrupt mode
    HAL_CAN_Transmit_IT(&hcan);
}

// Function to receive CAN message
void CAN_Receive(uint32_t* id, uint8_t* data, uint8_t* length) {
    // Retrieve the message ID, data, and length
    *id = canRxMessage.StdId;
    *length = canRxMessage.DLC;
    for (uint8_t i = 0; i < *length; i++) {
        data[i] = canRxMessage.Data[i]; // Copy the received data to the provided buffer
    }

    // Re-enable the receive interrupt
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}

// Error handler function for CAN
void CAN_Error_Handler(void) {
    // Handle CAN errors (e.g., bus off, overrun, etc.)
    // For example, we could reset the CAN bus or attempt to recover
    HAL_CAN_Stop(&hcan);  // Stop the CAN peripheral
    HAL_CAN_Start(&hcan); // Restart the CAN peripheral
}

// Function to update the CAN filter when a device joins the bus (hot-plug)
void CAN_UpdateFilter(uint32_t address) {
    CAN_FilterConfTypeDef canFilterConfig;

    // Update the filter for the new device address
    canFilterConfig.FilterIdHigh = address << 5;
    canFilterConfig.FilterIdLow = address << 5;
    canFilterConfig.FilterMaskIdHigh = 0x0000; // Accept all messages for this address
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;

    // Apply the filter
    HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);
}

// Function to recover from CAN bus errors or timeout conditions
void CAN_Recover(void) {
    if (can_timeout_flag) {
        // Reset the CAN peripheral or re-initialize it
        HAL_CAN_Stop(&hcan);
        HAL_CAN_Start(&hcan);

        // Clear timeout flag
        can_timeout_flag = 0;
    }
}

// Function to send a discovery message when a device is first powered on or hot-plugged
void CAN_DiscoveryMessage(void) {
    uint8_t data[] = {0x01, 0x00, 0x00}; // Example discovery message format (could be a unique ID, etc.)
    CAN_Transmit(0x01, data, sizeof(data)); // Send discovery message with ID 0x01
}

// Function to check for communication timeouts
void CAN_TimeoutCheck(void) {
    // Check if no message has been received for a certain period (this function can be called periodically via timer)
    if (can_timeout_flag) {
        CAN_Recover(); // Attempt to recover from timeout
    }
}

// CAN interrupt callback function (called when a CAN message is received)
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle) {
    // Handle received CAN message
    uint32_t id;
    uint8_t data[8];
    uint8_t length;

    CAN_Receive(&id, data, &length);

    // Process the received message (add your application-specific logic here)
    // For example, check if the message is for this device
    if (id == CAN_DEVICE_ADDRESS) {
        // Handle the data as needed
    }

    // Re-enable the CAN receive interrupt for future messages
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}
