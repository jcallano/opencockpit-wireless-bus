/**
 * @file slip.h
 * @brief SLIP (Serial Line Internet Protocol) encoder/decoder
 *
 * Provides simple, reliable framing for binary data over serial.
 * Reference: RFC 1055
 *
 * Part of OpenCockpit Wireless Bus project
 */

#ifndef SLIP_H
#define SLIP_H

#include <stdint.h>
#include <stddef.h>

// SLIP special characters
#define SLIP_END     0xC0  // Frame delimiter
#define SLIP_ESC     0xDB  // Escape character
#define SLIP_ESC_END 0xDC  // Escaped END (0xC0 in data becomes 0xDB 0xDC)
#define SLIP_ESC_ESC 0xDD  // Escaped ESC (0xDB in data becomes 0xDB 0xDD)

// Maximum frame size (ESP-NOW max payload + overhead)
#define SLIP_MAX_FRAME_SIZE 300

// Decoder state
typedef enum {
    SLIP_STATE_NORMAL,
    SLIP_STATE_ESCAPED
} SlipState;

// Decoder context
typedef struct {
    uint8_t buffer[SLIP_MAX_FRAME_SIZE];
    size_t index;
    SlipState state;
} SlipDecoder;

/**
 * @brief Initialize SLIP decoder context
 * @param decoder Pointer to decoder context
 */
static inline void slip_decoder_init(SlipDecoder* decoder) {
    decoder->index = 0;
    decoder->state = SLIP_STATE_NORMAL;
}

/**
 * @brief Reset decoder to initial state (after frame complete or error)
 * @param decoder Pointer to decoder context
 */
static inline void slip_decoder_reset(SlipDecoder* decoder) {
    decoder->index = 0;
    decoder->state = SLIP_STATE_NORMAL;
}

/**
 * @brief Process a single byte through the SLIP decoder
 * @param decoder Pointer to decoder context
 * @param byte Input byte to process
 * @return true if a complete frame is ready in decoder->buffer
 */
static inline bool slip_decode_byte(SlipDecoder* decoder, uint8_t byte) {
    switch (decoder->state) {
        case SLIP_STATE_NORMAL:
            switch (byte) {
                case SLIP_END:
                    if (decoder->index > 0) {
                        return true;  // Frame complete
                    }
                    // Empty frame or leading END - ignore
                    break;
                case SLIP_ESC:
                    decoder->state = SLIP_STATE_ESCAPED;
                    break;
                default:
                    if (decoder->index < SLIP_MAX_FRAME_SIZE) {
                        decoder->buffer[decoder->index++] = byte;
                    }
                    // else: buffer overflow, silently drop
            }
            break;

        case SLIP_STATE_ESCAPED:
            decoder->state = SLIP_STATE_NORMAL;
            switch (byte) {
                case SLIP_ESC_END:
                    if (decoder->index < SLIP_MAX_FRAME_SIZE) {
                        decoder->buffer[decoder->index++] = SLIP_END;
                    }
                    break;
                case SLIP_ESC_ESC:
                    if (decoder->index < SLIP_MAX_FRAME_SIZE) {
                        decoder->buffer[decoder->index++] = SLIP_ESC;
                    }
                    break;
                default:
                    // Protocol error - invalid escape sequence
                    // Store the byte anyway for robustness
                    if (decoder->index < SLIP_MAX_FRAME_SIZE) {
                        decoder->buffer[decoder->index++] = byte;
                    }
            }
            break;
    }
    return false;
}

/**
 * @brief Calculate the encoded size for a given payload
 * @param data Pointer to data to encode
 * @param len Length of data
 * @return Encoded size including delimiters (worst case: 2 + 2*len)
 */
static inline size_t slip_encoded_size(const uint8_t* data, size_t len) {
    size_t size = 2;  // START and END delimiters
    for (size_t i = 0; i < len; i++) {
        if (data[i] == SLIP_END || data[i] == SLIP_ESC) {
            size += 2;  // Escaped byte
        } else {
            size += 1;
        }
    }
    return size;
}

/**
 * @brief Encode data into SLIP frame
 * @param data Input data to encode
 * @param len Length of input data
 * @param output Output buffer (must be at least slip_encoded_size() bytes)
 * @return Number of bytes written to output
 */
static inline size_t slip_encode(const uint8_t* data, size_t len, uint8_t* output) {
    size_t out_idx = 0;

    // Start frame
    output[out_idx++] = SLIP_END;

    // Encode payload
    for (size_t i = 0; i < len; i++) {
        switch (data[i]) {
            case SLIP_END:
                output[out_idx++] = SLIP_ESC;
                output[out_idx++] = SLIP_ESC_END;
                break;
            case SLIP_ESC:
                output[out_idx++] = SLIP_ESC;
                output[out_idx++] = SLIP_ESC_ESC;
                break;
            default:
                output[out_idx++] = data[i];
        }
    }

    // End frame
    output[out_idx++] = SLIP_END;

    return out_idx;
}

#ifdef ARDUINO
#include <Stream.h>

/**
 * @brief Encode and send data via Arduino Stream (Serial, etc.)
 * @param data Input data to encode
 * @param len Length of input data
 * @param stream Arduino Stream to write to
 * @return Number of bytes written
 */
static inline size_t slip_send(const uint8_t* data, size_t len, Stream& stream) {
    size_t written = 0;

    // Start frame
    written += stream.write(SLIP_END);

    // Encode and send payload
    for (size_t i = 0; i < len; i++) {
        switch (data[i]) {
            case SLIP_END:
                written += stream.write(SLIP_ESC);
                written += stream.write(SLIP_ESC_END);
                break;
            case SLIP_ESC:
                written += stream.write(SLIP_ESC);
                written += stream.write(SLIP_ESC_ESC);
                break;
            default:
                written += stream.write(data[i]);
        }
    }

    // End frame
    written += stream.write(SLIP_END);

    return written;
}
#endif // ARDUINO

#endif // SLIP_H
