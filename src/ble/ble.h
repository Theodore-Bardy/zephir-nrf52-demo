

/**
 * @file ble.h
 * @brief API of the BLE agent
 */

#ifndef _BLE_H_
#define _BLE_H_

#include <stdbool.h>

bool ble_agent_init(void);

bool ble_agent_resume(void);

bool ble_agent_pause(void);

#endif // _BLE_H_
