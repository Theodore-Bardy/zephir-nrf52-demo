#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

#include "ble_task.h"
#include "mpu6050.h"
#include "serialization.h"

LOG_MODULE_REGISTER(ble);

ZBUS_CHAN_DECLARE(sensor_data);

// BLE task configuration
#define TASK_BLE_STACK_SIZE (1024)
#define TASK_BLE_PRIORITITY (2)
static void pvrTaskBle(void *sub);
ZBUS_SUBSCRIBER_DEFINE(ble_com_sub, 1);
ZBUS_CHAN_ADD_OBS(sensor_data, ble_com_sub, 1);
K_THREAD_DEFINE(task_ble_id, TASK_BLE_STACK_SIZE, pvrTaskBle, &ble_com_sub,
                NULL, NULL, TASK_BLE_PRIORITITY, K_ESSENTIAL, 0);

typedef enum { BLE_IDLE, BLE_ADV, BLE_CONNECT, BLE_CONNECTED } BleTaskState;
static BleTaskState current_state = BLE_IDLE;

// Synchronization variables
K_SEM_DEFINE(sem_task_ble_start, 0, 1);

// BLE device name
#ifdef CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#else
#define DEVICE_NAME "Zephyr demo"
#endif
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct bt_conn *phone_conn = NULL;
volatile bool temp_notify_enable = false;
volatile bool acce_notify_enable = false;
volatile bool gyro_notify_enable = false;

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                 uint16_t value) {
  ARG_UNUSED(attr);
  temp_notify_enable = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Temp notification %s", temp_notify_enable ? "enabled" : "disabled");
}

static void acce_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                 uint16_t value) {
  ARG_UNUSED(attr);
  acce_notify_enable = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Acce notification %s", acce_notify_enable ? "enabled" : "disabled");
}

static void gyro_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                 uint16_t value) {
  ARG_UNUSED(attr);
  gyro_notify_enable = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Gyro notification %s", gyro_notify_enable ? "enabled" : "disabled");
}

// MPU (accelero - gyro) BLE service UUID:
// 0x2F4A3B1E-0584-4865-9BF5-A7A99C9A0000
#define BT_UUID_MPU_SERVICE_VAL                                                \
  BT_UUID_128_ENCODE(0x2F4A3B1E, 0x0584, 0x4865, 0x9BF5, 0xA7A99C9A0000)

static const struct bt_uuid_128 mpu_uuid =
    BT_UUID_INIT_128(BT_UUID_MPU_SERVICE_VAL);
// Define custom characteristic UUIDs (accelero and gyro)
static const struct bt_uuid_128 accel_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x2F4A3B1E, 0x0584, 0x4865, 0x9BF5, 0xA7A99C9A0001));
static const struct bt_uuid_128 gyro_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x2F4A3B1E, 0x0584, 0x4865, 0x9BF5, 0xA7A99C9A0002));

static struct mpu6050_data mpu_srv_data = {0};

BT_GATT_SERVICE_DEFINE(
    ess_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
    BT_GATT_CHARACTERISTIC(BT_UUID_ES_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, &mpu_srv_data.temp),
    BT_GATT_CCC(temp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

BT_GATT_SERVICE_DEFINE(
    mpu_service, BT_GATT_PRIMARY_SERVICE(&mpu_uuid),
    BT_GATT_CHARACTERISTIC(&accel_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, &mpu_srv_data.acce),
    BT_GATT_CCC(acce_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&gyro_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, &mpu_srv_data.gyro),
    BT_GATT_CCC(gyro_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

// Advertisment data
static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

void prvMtuUpdated(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
  LOG_INF("Updated MTU: Tx: %d - Rx: %d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = prvMtuUpdated,
};

static void prvConnectedCb(struct bt_conn *conn, uint8_t err) {
  if (err) {
    LOG_ERR("Connection failed: %d", err);
  } else {
    LOG_INF("Connected");
    current_state = BLE_CONNECT;
    if (!phone_conn) {
      phone_conn = bt_conn_ref(conn);
    }
  }
}

static void prvDisconnectedCb(struct bt_conn *conn, uint8_t reason) {
  if (phone_conn) {
    bt_conn_unref(conn);
    phone_conn = NULL;
  }
  LOG_INF("Disconnected: reason 0x%02x %s", reason, bt_hci_err_to_str(reason));
  current_state = BLE_ADV;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = prvConnectedCb,
    .disconnected = prvDisconnectedCb,
};

void prvBtReady(int err) {
  if (err) {
    LOG_ERR("Bluetooth stack init failed: %d", err);
    return;
  }

  LOG_INF("Bluetooth initialized");
}

bool task_ble_init(void) {
  // Initialize BLE stack
  int ret = bt_enable(&prvBtReady);
  if (ret != 0) {
    LOG_ERR("Unable to initialize the BLE stack: %d", ret);
    return false;
  }

  // Set BLE name - appearance (sensor) and callbacks
  bt_set_name(DEVICE_NAME);
  bt_set_appearance(0x015);
  bt_gatt_cb_register(&gatt_callbacks);

  k_sem_give(&sem_task_ble_start);
  current_state = BLE_ADV;
  return true;
}

int prvNotifyData(const struct bt_gatt_attr *attr, void *data, uint16_t len) {
  int ret;
  uint16_t mtu_size = bt_gatt_get_mtu(phone_conn) - 3u;
  uint16_t index = 0;

  do {
    ret = bt_gatt_notify(phone_conn, attr, (char *)data + index,
                         (mtu_size > (len - index)) ? (len - index) : mtu_size);
    if (ret != 0) {
      LOG_ERR("BLE notification failed: %d", ret);
      return ret;
    }
    index += mtu_size;
  } while (index < len);

  return ret;
}

static void pvrTaskBle(void *sub) {
  const struct zbus_channel *channel;
  const struct zbus_observer *data_sub = sub;
  int ret;

  k_sem_take(&sem_task_ble_start, K_FOREVER);

  LOG_INF("BLE task start...");

  // BLE state machine
  while (true) {
    switch (current_state) {
    case BLE_IDLE:
      // Nothing to do
      k_sleep(K_SECONDS(1));
      break;

    case BLE_ADV:
      ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, adv_data,
                            ARRAY_SIZE(adv_data), NULL, 0);
      if (ret != 0) {
        LOG_ERR("Advertising failed to start: %d", ret);
      }
      current_state = BLE_IDLE;
      break;

    case BLE_CONNECT:
      ret = bt_le_adv_stop();
      if (ret != 0) {
        LOG_ERR("Advertising failed to stop: %d", ret);
        current_state = BLE_IDLE;
      }
      current_state = BLE_CONNECTED;
      break;

    case BLE_CONNECTED:
      // Wait for data from the sensor task - catch new event every 500ms
      while (!zbus_sub_wait(data_sub, &channel, K_MSEC(500))) {
        if (&sensor_data != channel) {
          LOG_ERR("Wrong channel triggered: %s (expected: %s)", channel->name,
                  sensor_data.name);
          continue;
        }
        ret = zbus_chan_read(channel, &mpu_srv_data, K_NO_WAIT);
        if (ret != 0) {
          LOG_ERR("Unable to read data over channel %s", channel->name);
          continue;
        }

        if (temp_notify_enable) {
          uint8_t data[sizeof(mpu_srv_data.temp)];
          size_t index = 0;
          ser_encoder_data(data, &index, &mpu_srv_data.temp,
                           sizeof(mpu_srv_data.temp));
          memcpy(data, &mpu_srv_data.temp, sizeof(mpu_srv_data.temp));
          ret = prvNotifyData(&ess_service.attrs[2], data,
                              sizeof(mpu_srv_data.temp));
        } else {
          LOG_WRN("Temp notification not enabled");
        }
        if (acce_notify_enable) {
          uint8_t data[sizeof(mpu_srv_data.acce)];
          size_t index = 0;
          ser_encoder_data(data, &index, mpu_srv_data.acce,
                           sizeof(mpu_srv_data.acce));
          ret = prvNotifyData(&mpu_service.attrs[2], data,
                              sizeof(mpu_srv_data.acce));
        } else {
          LOG_WRN("Acce notification not enabled");
        }
        if (gyro_notify_enable) {
          uint8_t data[sizeof(mpu_srv_data.gyro)];
          size_t index = 0;
          ser_encoder_data(data, &index, mpu_srv_data.gyro,
                           sizeof(mpu_srv_data.gyro));
          ret = prvNotifyData(&mpu_service.attrs[4], &mpu_srv_data.gyro,
                              sizeof(mpu_srv_data.gyro));
        } else {
          LOG_WRN("Gyro notification not enabled");
        }
      }
      break;
    }
  }
}
