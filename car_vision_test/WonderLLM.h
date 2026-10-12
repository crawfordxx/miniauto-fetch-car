#ifndef WONDERLLM_H_
#define WONDERLLM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WonderLLM I2C driver error code.
 */
typedef enum {
    WONDERLLM_OK = 0,                    /*!< Operation succeeded. */
    WONDERLLM_ERR_FAIL = -1,             /*!< Generic failure. */
    WONDERLLM_ERR_INVALID_ARG = -3,      /*!< Invalid argument. */
    WONDERLLM_ERR_INVALID_SIZE = -4,     /*!< Invalid frame or buffer size. */
    WONDERLLM_ERR_TIMEOUT = -5,          /*!< Receive timeout. */
    WONDERLLM_ERR_INVALID_RESPONSE = -6, /*!< Invalid response fields. */
    WONDERLLM_ERR_INVALID_CHECKSUM = -7, /*!< Frame checksum mismatch. */
} wonderllm_err_t;

/**
 * @brief Low-level I2C write callback.
 *
 * @param data Data buffer to write.
 * @param len Data length in bytes.
 * @param timeout_ms Write timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
typedef wonderllm_err_t (*wonderllm_i2c_write_cb_t)(const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Low-level I2C read callback.
 *
 * @param data Destination buffer.
 * @param len Number of bytes to read.
 * @param timeout_ms Read timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
typedef wonderllm_err_t (*wonderllm_i2c_read_cb_t)(uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief MCU-specific callbacks.
 */
typedef struct {
    wonderllm_i2c_write_cb_t write;                /*!< I2C write callback. */
    wonderllm_i2c_read_cb_t read;                  /*!< I2C read callback. */
} wonderllm_callbacks_t;

/**
 * @brief Protocol frame metadata.
 */
typedef struct {
    uint16_t data_len;            /*!< DATA field length. */
    uint16_t fragment_id;         /*!< Fragment id, starting from 1. */
    uint16_t total_fragments;     /*!< Total fragment count. */
} wonderllm_frame_header_t;

/**
 * @brief WonderLLM client object.
 */
typedef struct {
    wonderllm_callbacks_t callbacks;          /*!< I2C callbacks. */
} wonderllm_handle_t;

#define WONDERLLM_MAX_COLOR_BLOCKS (6U)
#define WONDERLLM_MAX_TARGET_COLORS (6U)
#define WONDERLLM_HAND_GESTURE_MAX_COUNT (3U)
#define WONDERLLM_OBJECT_DETECTION_MAX_COUNT (6U)
#define WONDERLLM_YOLO11N_CLASS_MAX_ID (79U)
#define WONDERLLM_POSE_MAX_COUNT (3U)
#define WONDERLLM_POSE_KEYPOINT_COUNT (17U)
#define WONDERLLM_POSE_KEYPOINT_MAX_COUNT WONDERLLM_POSE_KEYPOINT_COUNT
#define WONDERLLM_APRILTAG_MAX_COUNT (6U)
#define WONDERLLM_QRCODE_MAX_COUNT (6U)
#define WONDERLLM_QRCODE_MAX_TEXT_LEN (96U)
#define WONDERLLM_QRCODE_MAX_TEXT_SIZE WONDERLLM_QRCODE_MAX_TEXT_LEN

typedef enum {
    WONDERLLM_VISION_MODE_COLOR = 0,
    WONDERLLM_VISION_MODE_LINE = 1,
} wonderllm_vision_mode_t;

typedef struct {
    uint8_t mode;                    /*!< 0: color recognition, 1: line tracking. */
    uint8_t registered_color_count;  /*!< Number of registered color classes. */
} wonderllm_color_config_t;

typedef struct {
    uint8_t index;       /*!< Color block index. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
} wonderllm_color_block_t;

typedef struct {
    uint8_t count;                                   /*!< Detected color block count. */
    wonderllm_color_block_t blocks[WONDERLLM_MAX_COLOR_BLOCKS];
} wonderllm_color_result_t;

typedef struct {
    bool detected;       /*!< true if a line is detected. */
    uint16_t start_x;    /*!< Line start X. */
    uint16_t start_y;    /*!< Line start Y. */
    uint16_t end_x;      /*!< Line end X. */
    uint16_t end_y;      /*!< Line end Y. */
    float angle_deg;     /*!< Angle relative to the vertical screen axis, in degrees. */
    float offset;        /*!< Horizontal offset from the screen center, usually -1.0 to 1.0. */
} wonderllm_line_result_t;

typedef enum {
    WONDERLLM_HAND_GESTURE_ONE = 0,
    WONDERLLM_HAND_GESTURE_TWO = 1,
    WONDERLLM_HAND_GESTURE_THREE = 2,
    WONDERLLM_HAND_GESTURE_FOUR = 3,
    WONDERLLM_HAND_GESTURE_FIVE = 4,
    WONDERLLM_HAND_GESTURE_LIKE = 5,
    WONDERLLM_HAND_GESTURE_OK = 6,
    WONDERLLM_HAND_GESTURE_NO_GESTURE = 7,
    WONDERLLM_HAND_GESTURE_CALL = 8,
    WONDERLLM_HAND_GESTURE_DISLIKE = 9,
    WONDERLLM_HAND_GESTURE_NO_HAND = 10,
} wonderllm_hand_gesture_type_t;

typedef struct {
    uint8_t gesture;     /*!< Gesture class id. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
} wonderllm_hand_gesture_item_t;

typedef struct {
    uint8_t total_count;       /*!< Total detected gestures in DATA. */
    uint8_t parsed_count;      /*!< Parsed item count stored in items. */
    wonderllm_hand_gesture_item_t items[WONDERLLM_HAND_GESTURE_MAX_COUNT];
} wonderllm_hand_gesture_result_t;

typedef struct {
    uint8_t index;       /*!< Detection index. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
} wonderllm_object_detection_item_t;

typedef struct {
    uint8_t total_count;       /*!< Total detected objects in DATA. */
    uint8_t parsed_count;      /*!< Parsed item count stored in items. */
    wonderllm_object_detection_item_t items[WONDERLLM_OBJECT_DETECTION_MAX_COUNT];
} wonderllm_object_detection_result_t;

typedef enum {
    WONDERLLM_POSE_KEYPOINT_NOSE = 0,
    WONDERLLM_POSE_KEYPOINT_LEFT_EYE = 1,
    WONDERLLM_POSE_KEYPOINT_RIGHT_EYE = 2,
    WONDERLLM_POSE_KEYPOINT_LEFT_EAR = 3,
    WONDERLLM_POSE_KEYPOINT_RIGHT_EAR = 4,
    WONDERLLM_POSE_KEYPOINT_LEFT_SHOULDER = 5,
    WONDERLLM_POSE_KEYPOINT_RIGHT_SHOULDER = 6,
    WONDERLLM_POSE_KEYPOINT_LEFT_ELBOW = 7,
    WONDERLLM_POSE_KEYPOINT_RIGHT_ELBOW = 8,
    WONDERLLM_POSE_KEYPOINT_LEFT_WRIST = 9,
    WONDERLLM_POSE_KEYPOINT_RIGHT_WRIST = 10,
    WONDERLLM_POSE_KEYPOINT_LEFT_HIP = 11,
    WONDERLLM_POSE_KEYPOINT_RIGHT_HIP = 12,
    WONDERLLM_POSE_KEYPOINT_LEFT_KNEE = 13,
    WONDERLLM_POSE_KEYPOINT_RIGHT_KNEE = 14,
    WONDERLLM_POSE_KEYPOINT_LEFT_ANKLE = 15,
    WONDERLLM_POSE_KEYPOINT_RIGHT_ANKLE = 16,
} wonderllm_pose_keypoint_id_t;

typedef struct {
    uint8_t index;       /*!< Keypoint index. */
    uint16_t x;          /*!< Keypoint X. */
    uint16_t y;          /*!< Keypoint Y. */
} wonderllm_pose_keypoint_t;

typedef struct {
    uint8_t index;       /*!< Pose index. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
    uint8_t keypoint_count; /*!< Detected keypoint count in DATA. */
    bool keypoint_valid[WONDERLLM_POSE_KEYPOINT_COUNT];
    wonderllm_pose_keypoint_t keypoints[WONDERLLM_POSE_KEYPOINT_COUNT];
} wonderllm_pose_item_t;

typedef struct {
    uint8_t total_count;       /*!< Total detected poses in DATA. */
    uint8_t parsed_count;      /*!< Parsed item count stored in items. */
    wonderllm_pose_item_t items[WONDERLLM_POSE_MAX_COUNT];
} wonderllm_pose_result_t;

typedef struct {
    uint16_t id;         /*!< AprilTag ID. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
    float rotation;      /*!< In-plane rotation angle. */
    float x_translation; /*!< X translation. */
    float y_translation; /*!< Y translation. */
    float z_translation; /*!< Z translation. */
    float x_rotation;    /*!< Rotation around X axis. */
    float y_rotation;    /*!< Rotation around Y axis. */
    float z_rotation;    /*!< Rotation around Z axis. */
} wonderllm_apriltag_item_t;

typedef struct {
    uint8_t total_count;       /*!< Total recognized AprilTags in DATA. */
    uint8_t parsed_count;      /*!< Parsed item count stored in items. */
    wonderllm_apriltag_item_t items[WONDERLLM_APRILTAG_MAX_COUNT];
} wonderllm_apriltag_result_t;

typedef struct {
    uint8_t index;       /*!< QR code index. */
    uint16_t x;          /*!< Bounding box X. */
    uint16_t y;          /*!< Bounding box Y. */
    uint16_t width;      /*!< Bounding box width. */
    uint16_t height;     /*!< Bounding box height. */
    uint16_t text_len;   /*!< Original QR code text byte length. */
    uint16_t content_len; /*!< Compatibility alias for original QR code text byte length. */
    char text[WONDERLLM_QRCODE_MAX_TEXT_LEN + 1U]; /*!< Null-terminated text buffer. */
    char content[WONDERLLM_QRCODE_MAX_TEXT_LEN + 1U]; /*!< Compatibility alias for text buffer. */
} wonderllm_qrcode_item_t;

typedef struct {
    uint8_t total_count;       /*!< Total recognized QR codes in DATA. */
    uint8_t parsed_count;      /*!< Parsed item count stored in items. */
    wonderllm_qrcode_item_t items[WONDERLLM_QRCODE_MAX_COUNT];
} wonderllm_qrcode_result_t;

/**
 * @brief Initialize a WonderLLM handle.
 *
 * @param handle Client object.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_init(wonderllm_handle_t *handle);

/**
 * @brief Register I2C write and read callbacks.
 *
 * @param handle Client object.
 * @param write_callback I2C write callback function.
 * @param read_callback I2C read callback function.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_register_i2c_callbacks(wonderllm_handle_t *handle,
                                                 wonderllm_i2c_write_cb_t write_callback,
                                                 wonderllm_i2c_read_cb_t read_callback);

/**
 * @brief Send a JSON command.
 *
 * @param handle Client object.
 * @param json Null-terminated UTF-8 JSON text.
 * @param timeout_ms Write timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_send_json(wonderllm_handle_t *handle,
                                    const char *json,
                                    uint32_t timeout_ms);

/**
 * @brief Receive and parse one fixed-size 32-byte protocol frame.
 *
 * @param handle Client object.
 * @param data Destination buffer for the DATA field.
 * @param data_capacity Destination buffer size in bytes.
 * @param header Output frame header.
 * @param timeout_ms Read timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_receive_frame(wonderllm_handle_t *handle,
                                        uint8_t *data,
                                        size_t data_capacity,
                                        wonderllm_frame_header_t *header,
                                        uint32_t timeout_ms);

/**
 * @brief Receive and assemble one complete protocol packet.
 *
 * @param handle Client object.
 * @param data Destination buffer for the assembled DATA fields.
 * @param data_capacity Destination buffer size in bytes.
 * @param packet_len Output assembled DATA length.
 * @param timeout_ms Read timeout for each frame in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_receive_packet(wonderllm_handle_t *handle,
                                         uint8_t *data,
                                         size_t data_capacity,
                                         size_t *packet_len,
                                         uint32_t timeout_ms);

/**
 * @brief Get the current color recognition or line tracking configuration.
 *
 * @param handle Client object.
 * @param config Output color/line vision configuration.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_color_config(wonderllm_handle_t *handle,
                                           wonderllm_color_config_t *config,
                                           uint32_t timeout_ms);

/**
 * @brief Set color recognition or line tracking mode.
 *
 * @param handle Client object.
 * @param mode Vision mode, see wonderllm_vision_mode_t.
 * @param targets Target color indexes. Line tracking mode accepts exactly one target.
 * @param target_count Number of entries in targets.
 * @param success Output command result reported by the device.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_set_vision_mode(wonderllm_handle_t *handle,
                                          uint8_t mode,
                                          const uint8_t *targets,
                                          size_t target_count,
                                          bool *success,
                                          uint32_t timeout_ms);

/**
 * @brief Get color recognition result.
 *
 * @param handle Client object.
 * @param result Output detected color blocks.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_color_result(wonderllm_handle_t *handle,
                                           wonderllm_color_result_t *result,
                                           uint32_t timeout_ms);

/**
 * @brief Get line tracking result.
 *
 * @param handle Client object.
 * @param result Output detected line data.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_line_result(wonderllm_handle_t *handle,
                                          wonderllm_line_result_t *result,
                                          uint32_t timeout_ms);

/**
 * @brief Get the JSON request string for hand gesture recognition.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_hand_gesture_request(void);

/**
 * @brief Get hand gesture recognition result.
 *
 * @param handle Client object.
 * @param result Output hand gesture detections.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_hand_gesture_result(wonderllm_handle_t *handle,
                                                  wonderllm_hand_gesture_result_t *result,
                                                  uint32_t timeout_ms);

/**
 * @brief Get the JSON request string for cat detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_cat_request(void);

/**
 * @brief Get the JSON request string for dog detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_dog_request(void);

/**
 * @brief Get the JSON request string for pedestrian detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_pedestrian_request(void);

/**
 * @brief Get the JSON request string for YOLO11n COCO object detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_yolo11n_request(void);

/**
 * @brief Get the JSON request string for face detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_face_request(void);

/**
 * @brief Get the JSON request string for COCO pose detection.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_pose_request(void);

const char *wonderllm_get_coco_pose_request(void);

/**
 * @brief Get the JSON request string for AprilTag recognition.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_apriltag_request(void);

/**
 * @brief Get the JSON request string for QR code recognition.
 *
 * @return Null-terminated JSON request string.
 */
const char *wonderllm_get_qrcode_request(void);

/**
 * @brief Set the AprilTag family used by the recognition module.
 *
 * @param handle Client object.
 * @param tag_family Null-terminated tag family name, for example "36H11".
 * @param timeout_ms Write timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_set_apriltag_family(wonderllm_handle_t *handle,
                                              const char *tag_family,
                                              uint32_t timeout_ms);

/**
 * @brief Parse generic fixed-size object detection DATA bytes.
 *
 * @param data DATA field buffer returned by the device.
 * @param data_len DATA field length in bytes.
 * @param result Output parsed object detections.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_parse_object_detection_data(const uint8_t *data,
                                                      size_t data_len,
                                                      wonderllm_object_detection_result_t *result);

/**
 * @brief Parse YOLO11n object detection DATA bytes.
 *
 * @param data DATA field buffer returned by the device.
 * @param data_len DATA field length in bytes.
 * @param result Output parsed object detections.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_parse_yolo11n_detection_data(const uint8_t *data,
                                                       size_t data_len,
                                                       wonderllm_object_detection_result_t *result);

/**
 * @brief Get cat detection result.
 *
 * @param handle Client object.
 * @param result Output detected cats.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_cat_result(wonderllm_handle_t *handle,
                                         wonderllm_object_detection_result_t *result,
                                         uint32_t timeout_ms);

/**
 * @brief Get dog detection result.
 *
 * @param handle Client object.
 * @param result Output detected dogs.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_dog_result(wonderllm_handle_t *handle,
                                         wonderllm_object_detection_result_t *result,
                                         uint32_t timeout_ms);

/**
 * @brief Get pedestrian detection result.
 *
 * @param handle Client object.
 * @param result Output detected pedestrians.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_pedestrian_result(wonderllm_handle_t *handle,
                                                wonderllm_object_detection_result_t *result,
                                                uint32_t timeout_ms);

/**
 * @brief Get YOLO11n COCO object detection result.
 *
 * @param handle Client object.
 * @param result Output detected objects.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_yolo11n_result(wonderllm_handle_t *handle,
                                             wonderllm_object_detection_result_t *result,
                                             uint32_t timeout_ms);

/**
 * @brief Get face detection result.
 *
 * @param handle Client object.
 * @param result Output detected faces.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_face_result(wonderllm_handle_t *handle,
                                          wonderllm_object_detection_result_t *result,
                                          uint32_t timeout_ms);

/**
 * @brief Parse COCO pose detection DATA bytes.
 *
 * @param data DATA field buffer returned by the device.
 * @param data_len DATA field length in bytes.
 * @param result Output parsed pose detections.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_parse_pose_data(const uint8_t *data,
                                          size_t data_len,
                                          wonderllm_pose_result_t *result);

/**
 * @brief Get COCO pose detection result.
 *
 * @param handle Client object.
 * @param result Output pose detections.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_pose_result(wonderllm_handle_t *handle,
                                          wonderllm_pose_result_t *result,
                                          uint32_t timeout_ms);

wonderllm_err_t wonderllm_get_coco_pose_result(wonderllm_handle_t *handle,
                                               wonderllm_pose_result_t *result,
                                               uint32_t timeout_ms);

/**
 * @brief Parse AprilTag recognition DATA bytes.
 *
 * @param data DATA field buffer returned by the device.
 * @param data_len DATA field length in bytes.
 * @param result Output parsed AprilTag detections.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_parse_apriltag_data(const uint8_t *data,
                                              size_t data_len,
                                              wonderllm_apriltag_result_t *result);

/**
 * @brief Get AprilTag recognition result.
 *
 * @param handle Client object.
 * @param result Output recognized AprilTags.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_apriltag_result(wonderllm_handle_t *handle,
                                              wonderllm_apriltag_result_t *result,
                                              uint32_t timeout_ms);

/**
 * @brief Parse QR code recognition DATA bytes.
 *
 * @param data DATA field buffer returned by the device.
 * @param data_len DATA field length in bytes.
 * @param result Output parsed QR code detections.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_parse_qrcode_data(const uint8_t *data,
                                            size_t data_len,
                                            wonderllm_qrcode_result_t *result);

/**
 * @brief Get QR code recognition result.
 *
 * @param handle Client object.
 * @param result Output recognized QR codes.
 * @param timeout_ms I2C operation timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_get_qrcode_result(wonderllm_handle_t *handle,
                                            wonderllm_qrcode_result_t *result,
                                            uint32_t timeout_ms);

/**
 * @brief Forward a voice playback number to the voice module.
 *
 * The protocol does not define an I2C response for this command.
 *
 * @param handle Client object.
 * @param num Voice playback number.
 * @param timeout_ms Write timeout in milliseconds. 0 means wait forever.
 * @return WONDERLLM_OK on success, otherwise an error code.
 */
wonderllm_err_t wonderllm_voice_broadcast(wonderllm_handle_t *handle,
                                          uint8_t num,
                                          uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif

#endif
