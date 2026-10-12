#include "ble-lib.h"
#include <string.h>
LOG_MODULE_REGISTER(blelib, LOG_LEVEL_DBG);

// semaphore ensures bt_init_ok() completes before proceeding
static K_SEM_DEFINE(bt_init_ok, 1, 1);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)
#define NOMINAL_BATTERY_VOLT_MV 3000

static int32_t ble_temperature_degC = 0;
static uint32_t ble_error_flags = 0;

enum bt_data_notifications_enabled notifications_enabled;



/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_REMOTE_SERV_VAL), 
};

/* UUID of the Remote Service */
// Project ID: 065 (3rd entry)
// BLE MFG ID = 0x02DF (4th entry)
/*  UUID of Data Characteristic  */
struct bt_uuid_128 remote_temperature_data_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));
/*  UUID of Errors Characteristic  */
struct bt_uuid_128 remote_err_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));
/*  UUID of Message Characteristic   */
struct bt_uuid_128 remote_msg_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0xe9ea0003, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48));

static const char temp_cud[] = "Temperature (C)";
static const char err_cud[]  = "Error Code";
static const char msg_cud[]  = "Message Input";

static ssize_t read_cud(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const char *s = (const char *)attr->user_data;
    if (!s) {
        return 0;
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, s, strlen(s));
}

/* Setup services */
BT_GATT_SERVICE_DEFINE(remote_srv,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),

    /* Temperature characteristic (value, then CUD, then CCC) */
    BT_GATT_CHARACTERISTIC(&remote_temperature_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_temperature_data_cb, NULL, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CUD,
                       BT_GATT_PERM_READ,
                       read_cud, NULL,
                       (void *)temp_cud),
    BT_GATT_CCC(ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Error Code characteristic */
    BT_GATT_CHARACTERISTIC(&remote_err_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_error_cb, NULL, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CUD,
                       BT_GATT_PERM_READ,
                       read_cud, NULL,
                       (void *)err_cud),
    BT_GATT_CCC(ccc_cfg_changed_cb, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Message (write) characteristic */
    BT_GATT_CHARACTERISTIC(&remote_msg_uuid.uuid,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, on_write, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CUD,
                       BT_GATT_PERM_READ,
                       read_cud, NULL,
                       (void *)msg_cud)
);

static const struct bt_gatt_attr *temp_attr;
static const struct bt_gatt_attr *err_attr;

struct bt_conn *current_conn;
struct bt_conn_cb bluetooth_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

struct bt_remote_srv_cb remote_service_callbacks = {
    .notif_changed = on_notif_changed,
    .data_rx = on_data_rx,
};



/* Callbacks */
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len) {
    // copy the data to a temporary buffer and append a NULL character
    uint8_t temp_str[len+1];
    memcpy(temp_str, data, len);
    temp_str[len] = 0x00;  // append NULL character at the end
    LOG_INF("BT received data on conn %p. Len: %d", (void *)conn, len);
    LOG_INF("Data: %s", temp_str);
}

void on_connected(struct bt_conn *conn, uint8_t ret) {
    if (ret) {
        LOG_ERR("Connection error: %d", ret);
        return;
    }
    LOG_INF("BT connected");
    current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("BT disconnected (reason: %d)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
}

void on_notif_changed(enum bt_data_notifications_enabled status) {
    if (status == BT_DATA_NOTIFICATIONS_ENABLED) {
        LOG_INF("BT notifications enabled");
    } else {
        LOG_INF("BT notifications disabled");
    }
}

void ccc_cfg_changed_cb(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications: %s", notif_enabled? "enabled":"disabled");
    notifications_enabled = notif_enabled? BT_DATA_NOTIFICATIONS_ENABLED:BT_DATA_NOTIFICATIONS_DISABLED;
    if (remote_service_callbacks.notif_changed) {
        remote_service_callbacks.notif_changed(notifications_enabled);
    }
}

/* These callbacks are used when a read request is made to the remote service */
ssize_t read_temperature_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &ble_temperature_degC, sizeof(ble_temperature_degC));
}

ssize_t read_error_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &ble_error_flags, sizeof(ble_error_flags));
}

ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_INF("Received data, handle %d, conn %p", attr->handle, (void *)conn);

    if (remote_service_callbacks.data_rx) {
        remote_service_callbacks.data_rx(conn, buf, len);
    }
    return len;
}


void on_sent(struct bt_conn *conn, void *user_data) {
    ARG_UNUSED(user_data);
    LOG_INF("Notification sent on connection %p", (void *)conn);
}

void bt_ready(int ret) {
    if (ret) {
        LOG_ERR("bt_enable returned %d", ret);
    }
    // release the thread once initialized
    k_sem_give(&bt_init_ok);
}

int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_cb) {
    LOG_INF("Initializing Bluetooth");
    if ((bt_cb == NULL) | (remote_cb == NULL)) {
        LOG_ERR("Invalid callback(s)");
        return -NRFX_ERROR_NULL;
    }
    bt_conn_cb_register(bt_cb);
    remote_service_callbacks.notif_changed = remote_cb->notif_changed;
    remote_service_callbacks.data_rx = remote_cb->data_rx;
    
    temp_attr = &remote_srv.attrs[2];
    err_attr  = &remote_srv.attrs[5];
    
    int ret = bt_enable(bt_ready);
    if (ret) {
        LOG_ERR("bt_enable returned %d", ret);
        return ret;
    }

    // hold the thread until Bluetooth is initialized
    k_sem_take(&bt_init_ok, K_FOREVER);
    if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
        settings_load();
    }
    ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Could not start advertising (ret = %d)", ret);
        return ret;
    }
    return ret;
}

/* BAS Functions */
uint8_t bluetooth_get_battery_level(void) {
    uint8_t battery_level;
    battery_level =  bt_bas_get_battery_level();
    LOG_INF("Battery Level: %d pct", battery_level);
    return battery_level;
}

void bluetooth_set_battery_level(int32_t raw_mV) {
    LOG_DBG("Raw Battery: %d mV", raw_mV);
    
    int32_t pct = (int32_t)((100 * raw_mV) / NOMINAL_BATTERY_VOLT_MV);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    
    LOG_INF("Normalized Battery Level: %d%%", pct);
    int err = bt_bas_set_battery_level((uint8_t)pct);
    if (err) {
        LOG_ERR("BAS set error (err = %d)", err);
    }
}

//helpers for temp and errrs

int bluetooth_notify_temperature(int32_t temperature_degC)
{
    int rc;
    ble_temperature_degC = temperature_degC;

    /* if we have a connected client, prefer notifying it; otherwise skip */
    if (!current_conn) {
        LOG_DBG("No current connection - skipping temperature notify");
        return -ENOTCONN;
    }

    /* check subscription for this conn and attribute */
    if (!bt_gatt_is_subscribed(current_conn, temp_attr, BT_GATT_CCC_NOTIFY)) {
        LOG_DBG("Client not subscribed to temperature notifications - skipping");
        return -EACCES;
    }

    /* temperature_degC must be an integer degrees Celsius (not centi-deg) */
    rc = bt_gatt_notify(current_conn, temp_attr, &ble_temperature_degC, sizeof(ble_temperature_degC));
    if (rc < 0) {
        LOG_ERR("bt_gatt_notify(temp) returned %d", rc);
    } else {
        LOG_INF("Temperature notification sent (degC=%d)", ble_temperature_degC);
    }
    return rc;
}

int bluetooth_notify_error(uint32_t flags)
{
    int rc;
    ble_error_flags = flags;

    if (!current_conn) {
        LOG_DBG("No current connection - skipping error notify");
        return -ENOTCONN;
    }

    if (!bt_gatt_is_subscribed(current_conn, err_attr, BT_GATT_CCC_NOTIFY)) {
        LOG_DBG("Client not subscribed to error notifications - skipping");
        return -EACCES;
    }

    rc = bt_gatt_notify(current_conn, err_attr, &ble_error_flags, sizeof(ble_error_flags));
    if (rc < 0) {
        LOG_ERR("bt_gatt_notify(error) returned %d", rc);
    } else {
        LOG_INF("Error notification sent (events=0x%08X)", ble_error_flags);
    }
    return rc;
}
