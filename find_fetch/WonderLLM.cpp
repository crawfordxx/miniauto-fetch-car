#include "WonderLLM.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define WONDERLLM_FRAME_HEADER_H (0xAA)
#define WONDERLLM_FRAME_HEADER_H_I2C (0x54)
#define WONDERLLM_FRAME_HEADER_L (0x55)
#define WONDERLLM_FRAME_HEADER_SIZE (8U)
#define WONDERLLM_FRAME_MAX_SIZE (32U)
#define WONDERLLM_MAX_FRAGMENTS (32U)
#define WONDERLLM_MAX_FRAME_DATA_SIZE (WONDERLLM_FRAME_MAX_SIZE - WONDERLLM_FRAME_HEADER_SIZE - 1U)
#define WONDERLLM_PACKET_BUFFER_SIZE (512U)
#define WONDERLLM_COLOR_BLOCK_SIZE (9U)
#define WONDERLLM_LINE_RESULT_SIZE (17U)
#define WONDERLLM_HAND_GESTURE_RECORD_SIZE (9U)
#define WONDERLLM_OBJECT_DETECTION_RECORD_SIZE (9U)
#define WONDERLLM_POSE_RECORD_HEADER_SIZE (10U)
#define WONDERLLM_POSE_KEYPOINT_RECORD_SIZE (5U)
#define WONDERLLM_APRILTAG_RECORD_SIZE (38U)
#define WONDERLLM_QRCODE_RECORD_HEADER_SIZE (11U)

static const char WONDERLLM_COLOR_CONFIG_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"color_config\"}";
static const char WONDERLLM_COLOR_RESULT_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"color_result\"}";
static const char WONDERLLM_HAND_GESTURE_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"hand_gesture\"}";
static const char WONDERLLM_CAT_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"cat\"}";
static const char WONDERLLM_DOG_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"dog\"}";
static const char WONDERLLM_PEDESTRIAN_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"pedestrian\"}";
static const char WONDERLLM_YOLO11N_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"coco_detect\"}";
static const char WONDERLLM_FACE_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"human\"}";
static const char WONDERLLM_POSE_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"coco_pose\"}";
static const char WONDERLLM_APRILTAG_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"apriltag_recognition\"}";
static const char WONDERLLM_QRCODE_COMMAND[] = "{\"command\":\"get_info\",\"params\":\"qrcode\"}";
static const char WONDERLLM_APRILTAG_SETTING_PREFIX[] = "{\"command\":\"vision_setting\",\"params\":{\"tag\":\"";
static const char WONDERLLM_APRILTAG_SETTING_SUFFIX[] = "\"}}";

static void wonderllm_wait_response_ready(void)
{
    delay(10);
}

static uint8_t wonderllm_checksum_update(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;

    for(size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }

    return (uint8_t)(checksum & 0xff);
}

static uint16_t wonderllm_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static float wonderllm_read_float_le(const uint8_t *data)
{
    float value = 0.0f;
    uint8_t bytes[sizeof(float)];

    bytes[0] = data[0];
    bytes[1] = data[1];
    bytes[2] = data[2];
    bytes[3] = data[3];
    memcpy(&value, bytes, sizeof(value));

    return value;
}

static void wonderllm_print_failed_frame(wonderllm_err_t err,
                                         const uint8_t *header_buf,
                                         size_t header_len,
                                         const uint8_t *payload_buf,
                                         size_t payload_len)
{
    Serial.print("parse failed err=");
    Serial.print((int)err);
    Serial.print(" frame:");

    for (size_t i = 0; i < header_len; i++) {
        Serial.print(' ');
        if (header_buf[i] < 0x10) {
            Serial.print('0');
        }
        Serial.print(header_buf[i], HEX);
    }

    for (size_t i = 0; i < payload_len; i++) {
        Serial.print(' ');
        if (payload_buf[i] < 0x10) {
            Serial.print('0');
        }
        Serial.print(payload_buf[i], HEX);
    }

    Serial.println();
}

static wonderllm_err_t wonderllm_parse_frame_header(const uint8_t *header_buf,
                                                    size_t data_capacity,
                                                    wonderllm_frame_header_t *header)
{
    if (header_buf == NULL || header == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    if ((header_buf[0] != WONDERLLM_FRAME_HEADER_H &&
         header_buf[0] != WONDERLLM_FRAME_HEADER_H_I2C) ||
        header_buf[1] != WONDERLLM_FRAME_HEADER_L) {
        return WONDERLLM_ERR_INVALID_RESPONSE;
    }

    header->data_len = (uint16_t)header_buf[2] | ((uint16_t)header_buf[3] << 8);
    header->fragment_id = (uint16_t)header_buf[4] | ((uint16_t)header_buf[5] << 8);
    header->total_fragments = (uint16_t)header_buf[6] | ((uint16_t)header_buf[7] << 8);

    if (header->data_len == 0 ||
        header->data_len > data_capacity ||
        header->data_len > WONDERLLM_MAX_FRAME_DATA_SIZE) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    if (header->fragment_id == 0 ||
        header->total_fragments == 0 ||
        header->fragment_id > WONDERLLM_MAX_FRAGMENTS ||
        header->total_fragments > WONDERLLM_MAX_FRAGMENTS ||
        header->fragment_id > header->total_fragments) {
        return WONDERLLM_ERR_INVALID_RESPONSE;
    }

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_voice_broadcast(wonderllm_handle_t *handle,
                                          uint8_t num,
                                          uint32_t timeout_ms)
{
    char command[48] = {0};
    int written = snprintf(command,
                           sizeof(command),
                           "{\"command\":\"voice_broadcast\",\"params\":%u}",
                           (unsigned int)num);

    if (written < 0 || (size_t)written >= sizeof(command)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    return wonderllm_send_json(handle, command, timeout_ms);
}

wonderllm_err_t wonderllm_receive_frame(wonderllm_handle_t *handle,
                                        uint8_t *data,
                                        size_t data_capacity,
                                        wonderllm_frame_header_t *header,
                                        uint32_t timeout_ms)
{
    uint8_t frame_buf[WONDERLLM_FRAME_MAX_SIZE] = {0};
    const uint8_t *payload_buf = &frame_buf[WONDERLLM_FRAME_HEADER_SIZE];
    uint8_t cal_checksum = 0;

    if (handle == NULL ||
        handle->callbacks.read == NULL ||
        data == NULL ||
        header == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = handle->callbacks.read(frame_buf, sizeof(frame_buf), timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    ret = wonderllm_parse_frame_header(frame_buf, data_capacity, header);
    if (ret != WONDERLLM_OK) {
        wonderllm_print_failed_frame(ret, frame_buf, sizeof(frame_buf), NULL, 0);
        return ret;
    }

    cal_checksum = wonderllm_checksum_update(payload_buf, header->data_len);

    if(cal_checksum != payload_buf[header->data_len]) {
        Serial.print("checksum mismatch calculated=0x");
        if (cal_checksum < 0x10) {
            Serial.print('0');
        }
        Serial.print(cal_checksum, HEX);
        Serial.print(" received=0x");
        if (payload_buf[header->data_len] < 0x10) {
            Serial.print('0');
        }
        Serial.println(payload_buf[header->data_len], HEX);

        wonderllm_print_failed_frame(WONDERLLM_ERR_INVALID_CHECKSUM,
                                     frame_buf,
                                     WONDERLLM_FRAME_HEADER_SIZE,
                                     payload_buf,
                                     (size_t)header->data_len + 1U);
        return WONDERLLM_ERR_INVALID_CHECKSUM;
    }
    memcpy(data, payload_buf, header->data_len);
    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_receive_packet(wonderllm_handle_t *handle,
                                         uint8_t *data,
                                         size_t data_capacity,
                                         size_t *packet_len,
                                         uint32_t timeout_ms)
{
    wonderllm_frame_header_t header;
    size_t offset = 0;
    uint16_t expected_fragment_id = 1;
    uint16_t total_fragments = 0;

    if (handle == NULL ||
        data == NULL ||
        packet_len == NULL ||
        data_capacity == 0) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    *packet_len = 0;

    do {
        wonderllm_err_t ret = wonderllm_receive_frame(handle,
                                                      &data[offset],
                                                      data_capacity - offset,
                                                      &header,
                                                      timeout_ms);
        delay(1);
        if (ret != WONDERLLM_OK) {
            return ret;
        }

        if (header.fragment_id != expected_fragment_id) {
            return WONDERLLM_ERR_INVALID_RESPONSE;
        }

        if (total_fragments == 0) {
            total_fragments = header.total_fragments;
        } else if (header.total_fragments != total_fragments) {
            return WONDERLLM_ERR_INVALID_RESPONSE;
        }

        if (header.data_len > data_capacity - offset) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        offset += header.data_len;
        *packet_len = offset;

        if (header.fragment_id < total_fragments && offset >= data_capacity) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        expected_fragment_id++;
    } while ((uint16_t)(expected_fragment_id - 1) < total_fragments);

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_init(wonderllm_handle_t *handle)
{
    if (handle == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(handle, 0, sizeof(*handle));

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_register_i2c_callbacks(wonderllm_handle_t *handle,
                                                 wonderllm_i2c_write_cb_t write_callback,
                                                 wonderllm_i2c_read_cb_t read_callback)
{
    if (handle == NULL || write_callback == NULL || read_callback == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    handle->callbacks.write = write_callback;
    handle->callbacks.read = read_callback;

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_send_json(wonderllm_handle_t *handle,
                                    const char *json,
                                    uint32_t timeout_ms)
{
    if (handle == NULL ||
        handle->callbacks.write == NULL ||
        json == NULL ||
        json[0] == '\0') {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    size_t json_len = strlen(json);
    return handle->callbacks.write((const uint8_t *)json, json_len, timeout_ms);
}

wonderllm_err_t wonderllm_get_color_config(wonderllm_handle_t *handle,
                                           wonderllm_color_config_t *config,
                                           uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || config == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_COLOR_CONFIG_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    if (packet_len < 2U) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    config->mode = packet_data[0];
    config->registered_color_count = packet_data[1];

    if (config->mode > WONDERLLM_VISION_MODE_LINE ||
        config->registered_color_count > WONDERLLM_MAX_TARGET_COLORS) {
        return WONDERLLM_ERR_INVALID_RESPONSE;
    }

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_set_vision_mode(wonderllm_handle_t *handle,
                                          uint8_t mode,
                                          const uint8_t *targets,
                                          size_t target_count,
                                          bool *success,
                                          uint32_t timeout_ms)
{
    char command[128] = {0};
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;
    size_t offset = 0;

    if (handle == NULL ||
        targets == NULL ||
        success == NULL ||
        mode > WONDERLLM_VISION_MODE_LINE ||
        target_count == 0U ||
        target_count > WONDERLLM_MAX_TARGET_COLORS ||
        (mode == WONDERLLM_VISION_MODE_LINE && target_count != 1U)) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    offset = (size_t)snprintf(command,
                             sizeof(command),
                             "{\"command\":\"vision_setting\",\"params\":{\"mode\":%u,\"target\":[",
                             (unsigned int)mode);
    if (offset >= sizeof(command)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < target_count; i++) {
        int written = snprintf(&command[offset],
                               sizeof(command) - offset,
                               "%s%u",
                               (i == 0U) ? "" : ",",
                               (unsigned int)targets[i]);
        if (written < 0 || (size_t)written >= sizeof(command) - offset) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }
        offset += (size_t)written;
    }

    if (snprintf(&command[offset], sizeof(command) - offset, "]}}") >= (int)(sizeof(command) - offset)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, command, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    if (packet_len < 1U) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    if (packet_data[0] > 1U) {
        return WONDERLLM_ERR_INVALID_RESPONSE;
    }

    *success = (packet_data[0] == 1U);

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_get_color_result(wonderllm_handle_t *handle,
                                           wonderllm_color_result_t *result,
                                           uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_COLOR_RESULT_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    if (packet_len < 1U) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    result->count = packet_data[0];
    if (result->count == 0U) {
        return WONDERLLM_OK;
    }

    if (result->count > WONDERLLM_MAX_COLOR_BLOCKS ||
        packet_len < 1U + ((size_t)result->count * WONDERLLM_COLOR_BLOCK_SIZE)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    for (uint8_t i = 0; i < result->count; i++) {
        const uint8_t *block_data = &packet_data[1U + ((size_t)i * WONDERLLM_COLOR_BLOCK_SIZE)];
        result->blocks[i].index = block_data[0];
        result->blocks[i].x = wonderllm_read_u16_le(&block_data[1]);
        result->blocks[i].y = wonderllm_read_u16_le(&block_data[3]);
        result->blocks[i].width = wonderllm_read_u16_le(&block_data[5]);
        result->blocks[i].height = wonderllm_read_u16_le(&block_data[7]);
    }

    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_get_line_result(wonderllm_handle_t *handle,
                                          wonderllm_line_result_t *result,
                                          uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_COLOR_RESULT_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    if (packet_len < 1U) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    if (packet_data[0] > 1U) {
        return WONDERLLM_ERR_INVALID_RESPONSE;
    }

    result->detected = (packet_data[0] == 1U);
    if (!result->detected) {
        return WONDERLLM_OK;
    }

    if (packet_len < WONDERLLM_LINE_RESULT_SIZE) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    result->start_x = wonderllm_read_u16_le(&packet_data[1]);
    result->start_y = wonderllm_read_u16_le(&packet_data[3]);
    result->end_x = wonderllm_read_u16_le(&packet_data[5]);
    result->end_y = wonderllm_read_u16_le(&packet_data[7]);
    result->angle_deg = wonderllm_read_float_le(&packet_data[9]);
    result->offset = wonderllm_read_float_le(&packet_data[13]);

    return WONDERLLM_OK;
}

const char *wonderllm_get_hand_gesture_request(void)
{
    return WONDERLLM_HAND_GESTURE_COMMAND;
}

static wonderllm_err_t wonderllm_parse_hand_gesture_data(const uint8_t *data,
                                                  size_t data_len,
                                                  wonderllm_hand_gesture_result_t *result)
{
    uint8_t parse_count = 0;

    if (data == NULL || result == NULL || data_len < 1U) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));
    result->total_count = data[0];

    if (result->total_count == 0U) {
        return WONDERLLM_OK;
    }

    if (data_len < 1U + ((size_t)result->total_count * WONDERLLM_HAND_GESTURE_RECORD_SIZE)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    parse_count = result->total_count;
    if (parse_count > WONDERLLM_HAND_GESTURE_MAX_COUNT) {
        parse_count = WONDERLLM_HAND_GESTURE_MAX_COUNT;
    }

    for (uint8_t i = 0; i < parse_count; i++) {
        const uint8_t *item_data = &data[1U + ((size_t)i * WONDERLLM_HAND_GESTURE_RECORD_SIZE)];
        if (item_data[0] > WONDERLLM_HAND_GESTURE_NO_HAND) {
            return WONDERLLM_ERR_INVALID_RESPONSE;
        }

        result->items[i].gesture = item_data[0];
        result->items[i].x = wonderllm_read_u16_le(&item_data[1]);
        result->items[i].y = wonderllm_read_u16_le(&item_data[3]);
        result->items[i].width = wonderllm_read_u16_le(&item_data[5]);
        result->items[i].height = wonderllm_read_u16_le(&item_data[7]);
    }

    result->parsed_count = parse_count;
    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_get_hand_gesture_result(wonderllm_handle_t *handle,
                                                  wonderllm_hand_gesture_result_t *result,
                                                  uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_HAND_GESTURE_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_hand_gesture_data(packet_data, packet_len, result);
}

const char *wonderllm_get_cat_request(void)
{
    return WONDERLLM_CAT_COMMAND;
}

const char *wonderllm_get_dog_request(void)
{
    return WONDERLLM_DOG_COMMAND;
}

const char *wonderllm_get_pedestrian_request(void)
{
    return WONDERLLM_PEDESTRIAN_COMMAND;
}

const char *wonderllm_get_yolo11n_request(void)
{
    return WONDERLLM_YOLO11N_COMMAND;
}

const char *wonderllm_get_face_request(void)
{
    return WONDERLLM_FACE_COMMAND;
}

const char *wonderllm_get_pose_request(void)
{
    return WONDERLLM_POSE_COMMAND;
}

const char *wonderllm_get_coco_pose_request(void)
{
    return WONDERLLM_POSE_COMMAND;
}

const char *wonderllm_get_apriltag_request(void)
{
    return WONDERLLM_APRILTAG_COMMAND;
}

const char *wonderllm_get_qrcode_request(void)
{
    return WONDERLLM_QRCODE_COMMAND;
}

wonderllm_err_t wonderllm_set_apriltag_family(wonderllm_handle_t *handle,
                                              const char *tag_family,
                                              uint32_t timeout_ms)
{
    char command[96] = {0};
    int written = 0;

    if (handle == NULL || tag_family == NULL || tag_family[0] == '\0') {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    for (const char *p = tag_family; *p != '\0'; p++) {
        if (!((*p >= '0' && *p <= '9') ||
              (*p >= 'A' && *p <= 'Z') ||
              (*p >= 'a' && *p <= 'z') ||
              *p == '_' ||
              *p == '-')) {
            return WONDERLLM_ERR_INVALID_ARG;
        }
    }

    written = snprintf(command,
                       sizeof(command),
                       "%s%s%s",
                       WONDERLLM_APRILTAG_SETTING_PREFIX,
                       tag_family,
                       WONDERLLM_APRILTAG_SETTING_SUFFIX);
    if (written < 0 || (size_t)written >= sizeof(command)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    return wonderllm_send_json(handle, command, timeout_ms);
}

wonderllm_err_t wonderllm_parse_object_detection_data(const uint8_t *data,
                                                      size_t data_len,
                                                      wonderllm_object_detection_result_t *result)
{
    uint8_t parse_count = 0;

    if (data == NULL || result == NULL || data_len < 1U) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));
    result->total_count = data[0];

    if (result->total_count == 0U) {
        return WONDERLLM_OK;
    }

    if (data_len < 1U + ((size_t)result->total_count * WONDERLLM_OBJECT_DETECTION_RECORD_SIZE)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    parse_count = result->total_count;
    if (parse_count > WONDERLLM_OBJECT_DETECTION_MAX_COUNT) {
        parse_count = WONDERLLM_OBJECT_DETECTION_MAX_COUNT;
    }

    for (uint8_t i = 0; i < parse_count; i++) {
        const uint8_t *item_data = &data[1U + ((size_t)i * WONDERLLM_OBJECT_DETECTION_RECORD_SIZE)];
        result->items[i].index = item_data[0];
        result->items[i].x = wonderllm_read_u16_le(&item_data[1]);
        result->items[i].y = wonderllm_read_u16_le(&item_data[3]);
        result->items[i].width = wonderllm_read_u16_le(&item_data[5]);
        result->items[i].height = wonderllm_read_u16_le(&item_data[7]);
    }

    result->parsed_count = parse_count;
    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_parse_yolo11n_detection_data(const uint8_t *data,
                                                       size_t data_len,
                                                       wonderllm_object_detection_result_t *result)
{
    wonderllm_err_t ret = wonderllm_parse_object_detection_data(data, data_len, result);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    for (uint8_t i = 0; i < result->parsed_count; i++) {
        if (result->items[i].index > WONDERLLM_YOLO11N_CLASS_MAX_ID) {
            return WONDERLLM_ERR_INVALID_RESPONSE;
        }
    }

    return WONDERLLM_OK;
}

static wonderllm_err_t wonderllm_get_object_detection_result(wonderllm_handle_t *handle,
                                                             const char *command,
                                                             wonderllm_object_detection_result_t *result,
                                                             uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || command == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, command, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_object_detection_data(packet_data, packet_len, result);
}

wonderllm_err_t wonderllm_get_cat_result(wonderllm_handle_t *handle,
                                         wonderllm_object_detection_result_t *result,
                                         uint32_t timeout_ms)
{
    return wonderllm_get_object_detection_result(handle, WONDERLLM_CAT_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_get_dog_result(wonderllm_handle_t *handle,
                                         wonderllm_object_detection_result_t *result,
                                         uint32_t timeout_ms)
{
    return wonderllm_get_object_detection_result(handle, WONDERLLM_DOG_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_get_pedestrian_result(wonderllm_handle_t *handle,
                                                wonderllm_object_detection_result_t *result,
                                                uint32_t timeout_ms)
{
    return wonderllm_get_object_detection_result(handle, WONDERLLM_PEDESTRIAN_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_get_yolo11n_result(wonderllm_handle_t *handle,
                                             wonderllm_object_detection_result_t *result,
                                             uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_YOLO11N_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_yolo11n_detection_data(packet_data, packet_len, result);
}

wonderllm_err_t wonderllm_get_face_result(wonderllm_handle_t *handle,
                                          wonderllm_object_detection_result_t *result,
                                          uint32_t timeout_ms)
{
    return wonderllm_get_object_detection_result(handle, WONDERLLM_FACE_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_parse_pose_data(const uint8_t *data,
                                          size_t data_len,
                                          wonderllm_pose_result_t *result)
{
    uint8_t parse_count = 0;
    size_t offset = 1U;

    if (data == NULL || result == NULL || data_len < 1U) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));
    result->total_count = data[0];

    if (result->total_count == 0U) {
        return WONDERLLM_OK;
    }

    parse_count = result->total_count;
    if (parse_count > WONDERLLM_POSE_MAX_COUNT) {
        parse_count = WONDERLLM_POSE_MAX_COUNT;
    }

    for (uint8_t i = 0; i < result->total_count; i++) {
        if (offset > data_len || data_len - offset < WONDERLLM_POSE_RECORD_HEADER_SIZE) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        const uint8_t *item_data = &data[offset];
        uint8_t keypoint_count = item_data[9];
        size_t record_size = WONDERLLM_POSE_RECORD_HEADER_SIZE +
                             ((size_t)keypoint_count * WONDERLLM_POSE_KEYPOINT_RECORD_SIZE);

        if (keypoint_count > WONDERLLM_POSE_KEYPOINT_COUNT ||
            data_len - offset < record_size) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        if (i < parse_count) {
            result->items[i].index = item_data[0];
            result->items[i].x = wonderllm_read_u16_le(&item_data[1]);
            result->items[i].y = wonderllm_read_u16_le(&item_data[3]);
            result->items[i].width = wonderllm_read_u16_le(&item_data[5]);
            result->items[i].height = wonderllm_read_u16_le(&item_data[7]);
            result->items[i].keypoint_count = keypoint_count;

            for (uint8_t k = 0; k < keypoint_count; k++) {
                const uint8_t *keypoint_data = &item_data[10U + ((size_t)k * WONDERLLM_POSE_KEYPOINT_RECORD_SIZE)];
                uint8_t id = keypoint_data[0];
                if (id >= WONDERLLM_POSE_KEYPOINT_COUNT) {
                    return WONDERLLM_ERR_INVALID_RESPONSE;
                }

                result->items[i].keypoint_valid[id] = true;
                result->items[i].keypoints[id].index = id;
                result->items[i].keypoints[id].x = wonderllm_read_u16_le(&keypoint_data[1]);
                result->items[i].keypoints[id].y = wonderllm_read_u16_le(&keypoint_data[3]);
            }
        }

        offset += record_size;
    }

    result->parsed_count = parse_count;
    return WONDERLLM_OK;
}

static wonderllm_err_t wonderllm_get_pose_like_result(wonderllm_handle_t *handle,
                                                      const char *command,
                                                      wonderllm_pose_result_t *result,
                                                      uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || command == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, command, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();
    wonderllm_wait_response_ready();
    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_pose_data(packet_data, packet_len, result);
}

wonderllm_err_t wonderllm_get_pose_result(wonderllm_handle_t *handle,
                                          wonderllm_pose_result_t *result,
                                          uint32_t timeout_ms)
{
    return wonderllm_get_pose_like_result(handle, WONDERLLM_POSE_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_get_coco_pose_result(wonderllm_handle_t *handle,
                                               wonderllm_pose_result_t *result,
                                               uint32_t timeout_ms)
{
    return wonderllm_get_pose_like_result(handle, WONDERLLM_POSE_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_parse_apriltag_data(const uint8_t *data,
                                              size_t data_len,
                                              wonderllm_apriltag_result_t *result)
{
    uint8_t parse_count = 0;

    if (data == NULL || result == NULL || data_len < 1U) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));
    result->total_count = data[0];

    if (result->total_count == 0U) {
        return WONDERLLM_OK;
    }

    if (data_len < 1U + ((size_t)result->total_count * WONDERLLM_APRILTAG_RECORD_SIZE)) {
        return WONDERLLM_ERR_INVALID_SIZE;
    }

    parse_count = result->total_count;
    if (parse_count > WONDERLLM_APRILTAG_MAX_COUNT) {
        parse_count = WONDERLLM_APRILTAG_MAX_COUNT;
    }

    for (uint8_t i = 0; i < parse_count; i++) {
        const uint8_t *item_data = &data[1U + ((size_t)i * WONDERLLM_APRILTAG_RECORD_SIZE)];
        result->items[i].id = wonderllm_read_u16_le(&item_data[0]);
        result->items[i].x = wonderllm_read_u16_le(&item_data[2]);
        result->items[i].y = wonderllm_read_u16_le(&item_data[4]);
        result->items[i].width = wonderllm_read_u16_le(&item_data[6]);
        result->items[i].height = wonderllm_read_u16_le(&item_data[8]);
        result->items[i].rotation = wonderllm_read_float_le(&item_data[10]);
        result->items[i].x_translation = wonderllm_read_float_le(&item_data[14]);
        result->items[i].y_translation = wonderllm_read_float_le(&item_data[18]);
        result->items[i].z_translation = wonderllm_read_float_le(&item_data[22]);
        result->items[i].x_rotation = wonderllm_read_float_le(&item_data[26]);
        result->items[i].y_rotation = wonderllm_read_float_le(&item_data[30]);
        result->items[i].z_rotation = wonderllm_read_float_le(&item_data[34]);
    }

    result->parsed_count = parse_count;
    return WONDERLLM_OK;
}

static wonderllm_err_t wonderllm_get_apriltag_like_result(wonderllm_handle_t *handle,
                                                          const char *command,
                                                          wonderllm_apriltag_result_t *result,
                                                          uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || command == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, command, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_apriltag_data(packet_data, packet_len, result);
}

wonderllm_err_t wonderllm_get_apriltag_result(wonderllm_handle_t *handle,
                                              wonderllm_apriltag_result_t *result,
                                              uint32_t timeout_ms)
{
    return wonderllm_get_apriltag_like_result(handle, WONDERLLM_APRILTAG_COMMAND, result, timeout_ms);
}

wonderllm_err_t wonderllm_parse_qrcode_data(const uint8_t *data,
                                            size_t data_len,
                                            wonderllm_qrcode_result_t *result)
{
    uint8_t parse_count = 0;
    size_t offset = 1U;

    if (data == NULL || result == NULL || data_len < 1U) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    memset(result, 0, sizeof(*result));
    result->total_count = data[0];

    if (result->total_count == 0U) {
        return WONDERLLM_OK;
    }

    parse_count = result->total_count;
    if (parse_count > WONDERLLM_QRCODE_MAX_COUNT) {
        parse_count = WONDERLLM_QRCODE_MAX_COUNT;
    }

    for (uint8_t i = 0; i < result->total_count; i++) {
        if (offset > data_len || data_len - offset < WONDERLLM_QRCODE_RECORD_HEADER_SIZE) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        const uint8_t *item_data = &data[offset];
        uint16_t text_len = wonderllm_read_u16_le(&item_data[9]);
        if (data_len - offset - WONDERLLM_QRCODE_RECORD_HEADER_SIZE < text_len) {
            return WONDERLLM_ERR_INVALID_SIZE;
        }

        if (i < parse_count) {
            size_t copy_len = text_len;
            if (copy_len > WONDERLLM_QRCODE_MAX_TEXT_LEN) {
                copy_len = WONDERLLM_QRCODE_MAX_TEXT_LEN;
            }

            result->items[i].index = item_data[0];
            result->items[i].x = wonderllm_read_u16_le(&item_data[1]);
            result->items[i].y = wonderllm_read_u16_le(&item_data[3]);
            result->items[i].width = wonderllm_read_u16_le(&item_data[5]);
            result->items[i].height = wonderllm_read_u16_le(&item_data[7]);
            result->items[i].text_len = text_len;
            result->items[i].content_len = text_len;
            memcpy(result->items[i].text, &item_data[11], copy_len);
            result->items[i].text[copy_len] = '\0';
            memcpy(result->items[i].content, &item_data[11], copy_len);
            result->items[i].content[copy_len] = '\0';
        }

        offset += WONDERLLM_QRCODE_RECORD_HEADER_SIZE + (size_t)text_len;
    }

    result->parsed_count = parse_count;
    return WONDERLLM_OK;
}

wonderllm_err_t wonderllm_get_qrcode_result(wonderllm_handle_t *handle,
                                            wonderllm_qrcode_result_t *result,
                                            uint32_t timeout_ms)
{
    uint8_t packet_data[WONDERLLM_PACKET_BUFFER_SIZE] = {0};
    size_t packet_len = 0;

    if (handle == NULL || result == NULL) {
        return WONDERLLM_ERR_INVALID_ARG;
    }

    wonderllm_err_t ret = wonderllm_send_json(handle, WONDERLLM_QRCODE_COMMAND, timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    wonderllm_wait_response_ready();

    ret = wonderllm_receive_packet(handle,
                                   packet_data,
                                   sizeof(packet_data),
                                   &packet_len,
                                   timeout_ms);
    if (ret != WONDERLLM_OK) {
        return ret;
    }

    return wonderllm_parse_qrcode_data(packet_data, packet_len, result);
}
