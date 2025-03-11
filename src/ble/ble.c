/**
 * @file ble.c
 * @brief Implementation of the BLE agent
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include "stream_tx.h"
#include "zephyr/bluetooth/att.h"
#include "zephyr/fatal.h"

LOG_MODULE_REGISTER(ble);

static struct bt_conn *default_conn;
static struct bt_bap_unicast_group *unicast_group;
static struct audio_sink {
  struct bt_bap_ep *ep;
  uint16_t seq_num;
} sinks[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT];
static struct bt_bap_ep *sources[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];
static struct bt_bap_stream streams[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT +
                                    CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];
static size_t configured_sink_stream_count;
static size_t configured_source_stream_count;

#define CONFIGURED_STREAM_COUNT                                                \
  (configured_sink_stream_count + configured_source_stream_count)

/* Select a codec configuration to apply that is mandatory to support by both
 * client and server. Allows this sample application to work without logic to
 * parse the codec capabilities of the server and selection of an appropriate
 * codec configuration.
 */
static struct bt_bap_lc3_preset codec_configuration =
    BT_BAP_LC3_UNICAST_PRESET_16_2_1(BT_AUDIO_LOCATION_FRONT_LEFT,
                                     BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

/**
 * @brief BLE thread definition
 */
typedef enum {
  BLE_THREAD_IDLE,
  BLE_THREAD_SCAN,
  BLE_THREAD_CONNECT,
  BLE_THREAD_CONNECTED,
  BLE_THREAD_PLAY,
  BLE_THREAD_PAUSE,
  BLE_THREAD_DISCONNECT
} BleThread_State;
static BleThread_State current_state;

#define BLE_THREAD_STACKSIZE (1024)
static void prvBleThread(void *arg0, void *arg1, void *arg2);
K_THREAD_DEFINE(ble_thread_id, BLE_THREAD_STACKSIZE, prvBleThread, NULL, NULL,
                NULL, 1, K_ESSENTIAL, 0);

/**
 * @brief Synchronization variables
 */
static K_SEM_DEFINE(ble_thread_start_or_resume, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);
static K_SEM_DEFINE(sem_mtu_updated, 0, 1);
static K_SEM_DEFINE(sem_security_updated, 0, 1);
static K_SEM_DEFINE(sem_sinks_discovered, 0, 1);
static K_SEM_DEFINE(sem_sources_discovered, 0, 1);
static K_SEM_DEFINE(sem_stream_configured, 0, 1);
static K_SEM_DEFINE(sem_stream_qos, 0, 1);
static K_SEM_DEFINE(sem_stream_enabled, 0, 1);
static K_SEM_DEFINE(sem_stream_started, 0, 1);
static K_SEM_DEFINE(sem_stream_connected, 0, 1);

static void prvStreamConfigured(struct bt_bap_stream *stream,
                                const struct bt_bap_qos_cfg_pref *pref) {
  LOG_INF("Audio stream %p configured", stream);
  k_sem_give(&sem_stream_configured);
}

static void prvStreamQosSet(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p QoS set", stream);
  k_sem_give(&sem_stream_qos);
}

static void prvStreamEanbled(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p enabled", stream);
  k_sem_give(&sem_stream_enabled);
}

static bool prvStreamTxCanSend(const struct bt_bap_stream *stream) {
  struct bt_bap_ep_info info;
  int err;

  if (stream == NULL || stream->ep == NULL) {
    return false;
  }

  err = bt_bap_ep_get_info(stream->ep, &info);
  if (err != 0) {
    return false;
  }

  return info.can_send;
}

static void prvStreamStarted(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p started", stream);

  // Register the stream for Tx if it can send
  if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && prvStreamTxCanSend(stream)) {
    if (stream_tx_register(stream)) {
      LOG_ERR("Failed to register stram %p for Tx", stream);
    }
  }

  k_sem_give(&sem_stream_started);
}

static void prvStreamMetadataUpdated(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p metadata updated", stream);
}

static void prvStreamDisable(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p disabled", stream);
}

static void prvStreamStopped(struct bt_bap_stream *stream, uint8_t reason) {
  LOG_INF("Audio stream %p stropped with reason 0x%02X", stream, reason);

  // Unregister the stream for Tx if it can send
  if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && prvStreamTxCanSend(stream)) {
    if (stream_tx_unregister(stream)) {
      LOG_ERR("Failed to unregister stream %p for Tx", stream);
    }
  }
}

static void prvStreamReleased(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p released", stream);
}

static void prvStreamRecv(struct bt_bap_stream *stream,
                          const struct bt_iso_recv_info *info,
                          struct net_buf *buf) {
  if (info->flags & BT_ISO_FLAGS_VALID) {
    LOG_INF("Incoming audio on stream &p len &u", stream, buf->len);
  }
}

static void prvStreamConnected(struct bt_bap_stream *stream) {
  LOG_INF("Audio stream %p connected", stream);

  // Reset sequence number for sinks
  for (size_t i = 0U; i < configured_sink_stream_count; i++) {
    if (stream->ep == sinks[i].ep) {
      sinks[i].seq_num = 0U;
      break;
    }
  }

  k_sem_give(&sem_stream_connected);
}

static struct bt_bap_stream_ops stream_ops = {
    .configured = prvStreamConfigured,
    .qos_set = prvStreamQosSet,
    .enabled = prvStreamEanbled,
    .started = prvStreamStarted,
    .metadata_updated = prvStreamMetadataUpdated,
    .disabled = prvStreamDisable,
    .stopped = prvStreamStopped,
    .released = prvStreamReleased,
    .recv = prvStreamRecv,
    .connected = prvStreamConnected,
};

static void prvAttMtuUpdated(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
  LOG_INF("MTU updated: %u / %u", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = prvAttMtuUpdated,
};

static void prvUnicastClientLocation(struct bt_conn *conn,
                                     enum bt_audio_dir dir,
                                     enum bt_audio_location loc) {
  LOG_INF("dir %u loc %X", dir, loc);
}

static void prvAvailableContexts(struct bt_conn *conn,
                                 enum bt_audio_context snk_ctx,
                                 enum bt_audio_context src_ctx) {
  LOG_INF("snk ctx %u src ctx %u", snk_ctx, src_ctx);
}

static void prvAddRemoteSrc(struct bt_bap_ep *ep) {
  for (size_t i = 0U; i < ARRAY_SIZE(sources); i++) {
    if (sources[i] == NULL) {
      LOG_INF("Source #%zu", i);
      sources[i] = ep;
      return;
    }
  }

  LOG_ERR("Could not add source ep");
}

static void prvAddRemoteSnk(struct bt_bap_ep *ep) {
  for (size_t i = 0U; i < ARRAY_SIZE(sinks); i++) {
    if (sinks[i].ep == NULL) {
      LOG_INF("Sink #%zu", i);
      sinks[i].ep = ep;
      return;
    }
  }

  LOG_ERR("Could not add sink ep\n");
}

static void prvEndpoint(struct bt_conn *conn, enum bt_audio_dir dir,
                        struct bt_bap_ep *ep) {
  if (dir == BT_AUDIO_DIR_SOURCE) {
    prvAddRemoteSrc(ep);
  } else if (dir == BT_AUDIO_DIR_SINK) {
    prvAddRemoteSnk(ep);
  }
}

static struct bt_bap_unicast_client_cb unicast_client_cbs = {
    .location = prvUnicastClientLocation,
    .available_contexts = prvAvailableContexts,
    .pac_record = NULL,
    .endpoint = prvEndpoint,
};

bool ble_agent_init(void) {
  // Initialize bluetooth stack
  int err = bt_enable(NULL);
  if (err) {
    LOG_ERR("Bluetooth init failed: %d", err);
    return false;
  }

  for (size_t i = 0; i < ARRAY_SIZE(streams); i++) {
    streams[i].ops = &stream_ops;
  }

  bt_gatt_cb_register(&gatt_callbacks);

  if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
    stream_tx_init();
  }

  LOG_INF("Bluetooth initialized");

  err = bt_bap_unicast_client_register_cb(&unicast_client_cbs);
  if (err) {
    LOG_ERR("Failed to register client callbacks: %d", err);
    return false;
  }

  // Give signal to start the agent's thread
  k_sem_give(&ble_thread_start_or_resume);
  return true;
}

bool ble_agent_resume(void) {
  // Give signal to resume the agent's thread
  k_sem_give(&ble_thread_start_or_resume);
  return true;
}

bool ble_agent_pause(void) {
  // Give signal to stop the agent's thread
  k_sem_reset(&ble_thread_start_or_resume);
  return true;
}

static bool prvCheckAudioSupportAndConnect(struct bt_data *data,
                                           void *user_data) {
  struct net_buf_simple ascs_svc_data;
  bt_addr_le_t *addr = user_data;
  uint8_t announcement_type;
  uint32_t audio_contexts;
  const struct bt_uuid *uuid;
  uint16_t uuid_val;
  uint8_t meta_len;
  size_t min_size;
  int err;

  LOG_INF("[AD]: %u data_len %u", data->type, data->data_len);

  if (data->type != BT_DATA_SVC_DATA16) {
    // Continue parsing to next AD data type
    return true;
  }

  if (data->data_len < sizeof(uuid_val)) {
    LOG_INF("AD invalid size %u", data->data_len);
    // Continue parsing to next AD data type
    return true;
  }

  net_buf_simple_init_with_data(&ascs_svc_data, (void *)data->data,
                                data->data_len);

  uuid_val = net_buf_simple_pull_le16(&ascs_svc_data);
  uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(uuid_val));
  if (bt_uuid_cmp(uuid, BT_UUID_ASCS) != 0) {
    // We are looking for the ASCS service data
    // Continue parsing to next AD data type
    return true;
  }

  min_size =
      sizeof(announcement_type) + sizeof(audio_contexts) + sizeof(meta_len);
  if (ascs_svc_data.len < min_size) {
    LOG_INF("AD invalid size %u", data->data_len);
    // Stop parsing
    return false;
  }

  announcement_type = net_buf_simple_pull_u8(&ascs_svc_data);
  audio_contexts = net_buf_simple_pull_le32(&ascs_svc_data);
  meta_len = net_buf_simple_pull_u8(&ascs_svc_data);

  err = bt_le_scan_stop();
  if (err != 0) {
    LOG_ERR("Failed to stop scan: %d", err);
    return false; /* Stop parsing */
  }

  LOG_INF("Audio server found with type %u, contexts 0x%08x and meta_len %u; "
          "connecting",
          announcement_type, audio_contexts, meta_len);

  err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                          BT_LE_CONN_PARAM_DEFAULT, &default_conn);
  if (err != 0) {
    LOG_ERR("Create connection failed (%u)", err);
    current_state = BLE_THREAD_SCAN;
  }

  // Stop parsing
  return false;
}

static void prvDeviceFound(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                           struct net_buf_simple *ad) {
  char addr_str[BT_ADDR_LE_STR_LEN];

  if (default_conn != NULL) {
    // Already connected
    return;
  }

  // We're only interested in connecteable events
  if (type != BT_GAP_ADV_TYPE_ADV_IND &&
      type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
      type != BT_GAP_ADV_TYPE_EXT_ADV) {
    return;
  }

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
  LOG_INF("Device found: %s (RSSI %d)", addr_str, rssi);

  // Connect only if device is close
  if (rssi < -50) {
    return;
  }
  // Connect only if device name is 'Bose Color SoundLink'
  if (!strcmp(addr_str, "Bose Color SoundLink")) {
    return;
  }

  bt_data_parse(ad, prvCheckAudioSupportAndConnect, (void *)addr);
}

static void prvDiscoverSinksCb(struct bt_conn *conn, int err,
                               enum bt_audio_dir dir) {
  if (err != 0 && err != BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
    LOG_ERR("Discovery failed: %d", err);
    return;
  }

  if (err == BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
    LOG_ERR("Discover sinks completed without finding any sink ASEs");
  } else {
    LOG_INF("Discover sinks completed: %d", err);
  }

  k_sem_give(&sem_sinks_discovered);
}

static void prvDiscoverSrcCb(struct bt_conn *conn, int err,
                             enum bt_audio_dir dir) {
  if (err != 0 && err != BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
    LOG_ERR("Discovery failed: %d", err);
    return;
  }

  if (err == BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
    LOG_ERR("Discover sinks completed without finding any source ASEs");
  } else {
    LOG_INF("Discover sources complete: err %d", err);
  }

  k_sem_give(&sem_sources_discovered);
}

static int configure_stream(struct bt_bap_stream *stream,
                            struct bt_bap_ep *ep) {
  int err;

  err = bt_bap_stream_config(default_conn, stream, ep,
                             &codec_configuration.codec_cfg);
  if (err != 0) {
    return err;
  }

  err = k_sem_take(&sem_stream_configured, K_FOREVER);
  if (err != 0) {
    printk("failed to take sem_stream_configured (err %d)\n", err);
    return err;
  }

  return 0;
}

static int configure_streams(void) {
  int err;

  for (size_t i = 0; i < ARRAY_SIZE(sinks); i++) {
    struct bt_bap_ep *ep = sinks[i].ep;
    struct bt_bap_stream *stream = &streams[i];

    if (ep == NULL) {
      continue;
    }

    err = configure_stream(stream, ep);
    if (err != 0) {
      printk("Could not configure sink stream[%zu]: %d\n", i, err);
      return err;
    }

    printk("Configured sink stream[%zu]\n", i);
    configured_sink_stream_count++;
  }

  for (size_t i = 0; i < ARRAY_SIZE(sources); i++) {
    struct bt_bap_ep *ep = sources[i];
    struct bt_bap_stream *stream = &streams[i + configured_sink_stream_count];

    if (ep == NULL) {
      continue;
    }

    err = configure_stream(stream, ep);
    if (err != 0) {
      printk("Could not configure source stream[%zu]: %d\n", i, err);
      return err;
    }

    printk("Configured source stream[%zu]\n", i);
    configured_source_stream_count++;
  }

  return 0;
}

static int create_group(void) {
  const size_t params_count =
      MAX(configured_sink_stream_count, configured_source_stream_count);
  struct bt_bap_unicast_group_stream_pair_param pair_params[params_count];
  struct bt_bap_unicast_group_stream_param
      stream_params[CONFIGURED_STREAM_COUNT];
  struct bt_bap_unicast_group_param param;
  int err;

  for (size_t i = 0U; i < CONFIGURED_STREAM_COUNT; i++) {
    stream_params[i].stream = &streams[i];
    stream_params[i].qos = &codec_configuration.qos;
  }

  for (size_t i = 0U; i < params_count; i++) {
    if (i < configured_sink_stream_count) {
      pair_params[i].tx_param = &stream_params[i];
    } else {
      pair_params[i].tx_param = NULL;
    }

    if (i < configured_source_stream_count) {
      pair_params[i].rx_param =
          &stream_params[i + configured_sink_stream_count];
    } else {
      pair_params[i].rx_param = NULL;
    }
  }

  param.params = pair_params;
  param.params_count = params_count;
  param.packing = BT_ISO_PACKING_SEQUENTIAL;

  err = bt_bap_unicast_group_create(&param, &unicast_group);
  if (err != 0) {
    printk("Could not create unicast group (err %d)\n", err);
    return err;
  }

  return 0;
}

static int set_stream_qos(void) {
  int err;

  err = bt_bap_stream_qos(default_conn, unicast_group);
  if (err != 0) {
    printk("Unable to setup QoS: %d\n", err);
    return err;
  }

  for (size_t i = 0U; i < CONFIGURED_STREAM_COUNT; i++) {
    printk("QoS: waiting for %zu streams\n", CONFIGURED_STREAM_COUNT);
    err = k_sem_take(&sem_stream_qos, K_FOREVER);
    if (err != 0) {
      printk("failed to take sem_stream_qos (err %d)\n", err);
      return err;
    }
  }

  return 0;
}

static int enable_streams(void) {
  for (size_t i = 0U; i < CONFIGURED_STREAM_COUNT; i++) {
    int err;

    err = bt_bap_stream_enable(&streams[i], codec_configuration.codec_cfg.meta,
                               codec_configuration.codec_cfg.meta_len);
    if (err != 0) {
      printk("Unable to enable stream: %d\n", err);
      return err;
    }

    err = k_sem_take(&sem_stream_enabled, K_FOREVER);
    if (err != 0) {
      printk("failed to take sem_stream_enabled (err %d)\n", err);
      return err;
    }
  }

  return 0;
}

static int connect_streams(void) {
  for (size_t i = 0U; i < CONFIGURED_STREAM_COUNT; i++) {
    int err;

    k_sem_reset(&sem_stream_connected);

    err = bt_bap_stream_connect(&streams[i]);
    if (err == -EALREADY) {
      /* We have already connected a paired stream */
      continue;
    } else if (err != 0) {
      printk("Unable to start stream: %d\n", err);
      return err;
    }

    err = k_sem_take(&sem_stream_connected, K_FOREVER);
    if (err != 0) {
      printk("failed to take sem_stream_connected (err %d)\n", err);
      return err;
    }
  }

  return 0;
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream) {
  struct bt_bap_ep_info ep_info;
  int err;

  err = bt_bap_ep_get_info(stream->ep, &ep_info);
  if (err != 0) {
    printk("Failed to get ep info for %p: %d\n", stream, err);
    __ASSERT_NO_MSG(false);

    return 0;
  }

  return ep_info.dir;
}

static int start_streams(void) {
  for (size_t i = 0U; i < CONFIGURED_STREAM_COUNT; i++) {
    struct bt_bap_stream *stream = &streams[i];
    int err;

    if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
      err = bt_bap_stream_start(&streams[i]);
      if (err != 0) {
        printk("Unable to start stream: %d\n", err);
        return err;
      }
    } /* Sink streams are started by the unicast server */

    err = k_sem_take(&sem_stream_started, K_FOREVER);
    if (err != 0) {
      printk("failed to take sem_stream_started (err %d)\n", err);
      return err;
    }
  }

  return 0;
}

static void prvBleThread(void *arg0, void *arg1, void *arg2) {
  int err;

  while (1 == k_sem_count_get(&ble_thread_start_or_resume)) {
    switch (current_state) {
    case BLE_THREAD_IDLE:
      current_state = BLE_THREAD_SCAN;
      break;

    case BLE_THREAD_SCAN:
      LOG_INF("Waiting for connection...");
      err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, prvDeviceFound);
      if (err != 0) {
        LOG_ERR("Scanning failed to start: %d", err);
      }
      break;

    case BLE_THREAD_CONNECT:
      // Wait 2s for connection event, otherwise start scanning
      err = k_sem_take(&sem_stream_connected, K_MSEC(2000));
      if (err != 0) {
        LOG_ERR("Connected event not received: %d", err);
        current_state = BLE_THREAD_SCAN;
        break;
      }

      // Wait 2s for MTU exchanged event, otherwise start scanning
      err = k_sem_take(&sem_mtu_updated, K_MSEC(2000));
      if (err != 0) {
        LOG_ERR("MTU exchanged event not received: %d", err);
        current_state = BLE_THREAD_SCAN;
        break;
      }

      // Set security level over the connection
      err = bt_conn_set_security(default_conn, BT_SECURITY_L2);
      if (err != 0) {
        LOG_ERR("Failed to update security: %d", err);
        current_state = BLE_THREAD_SCAN;
        break;
      }

      // Wait 2s for security updated event, otherwise start scanning
      err = k_sem_take(&sem_security_updated, K_MSEC(2000));
      if (err != 0) {
        LOG_ERR("Security updated event not received: %d", err);
        current_state = BLE_THREAD_SCAN;
        break;
      }

      LOG_INF("Connected");
      current_state = BLE_THREAD_CONNECTED;
      break;

    case BLE_THREAD_CONNECTED:
      unicast_client_cbs.discover = prvDiscoverSinksCb;
      err = bt_bap_unicast_client_discover(default_conn, BT_AUDIO_DIR_SINK);
      if (err != 0) {
        LOG_ERR("Failed to discover sinks: %d", err);
        k_fatal_halt(err);
      }

      err = k_sem_take(&sem_sinks_discovered, K_FOREVER);
      if (err != 0) {
        LOG_ERR("Failed to take sem sinks");
        k_fatal_halt(err);
      }

      unicast_client_cbs.discover = prvDiscoverSrcCb;
      err = bt_bap_unicast_client_discover(default_conn, BT_AUDIO_DIR_SOURCE);
      if (err != 0) {
        LOG_ERR("Failed to discover sources: %d", err);
        k_fatal_halt(err);
      }

      err = k_sem_take(&sem_sources_discovered, K_FOREVER);
      if (err != 0) {
        LOG_ERR("Failed to take sem sources");
        k_fatal_halt(err);
      }

      err = configure_streams();
      if (err != 0) {
        LOG_ERR("Failed to take sem sources");
        k_fatal_halt(err);
      }

      if (CONFIGURED_STREAM_COUNT == 0U) {
        LOG_ERR("No streams were configured");
        k_fatal_halt(err);
      }

      LOG_INF("Creating unicast group");
      err = create_group();
      if (err != 0) {
        k_fatal_halt(err);
      }
      LOG_INF("Unicast group created");

      LOG_INF("Setting stream QoS");
      err = set_stream_qos();
      if (err != 0) {
        k_fatal_halt(err);
      }
      LOG_INF("Stream QoS Set");

      LOG_INF("Enabling streams");
      err = enable_streams();
      if (err != 0) {
        k_fatal_halt(err);
      }
      LOG_INF("Streams enabled");

      LOG_INF("Connecting streams");
      err = connect_streams();
      if (err != 0) {
        k_fatal_halt(err);
      }
      LOG_INF("Streams connected");

      LOG_INF("Starting streams");
      err = start_streams();
      if (err != 0) {
        k_fatal_halt(err);
      }
      LOG_INF("Streams started");

      /* Wait for disconnect */
      err = k_sem_take(&sem_disconnected, K_FOREVER);

      break;
    }
    // Do nothing
    k_sleep(K_FOREVER);
  }
}
