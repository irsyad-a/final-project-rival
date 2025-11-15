#pragma once
// Minimal ASCII protocol parser for STM32 (HAL)
// Format (newline-terminated, ASCII):
//   F <cm> <speed>    // Forward  distance in cm, speed 0..255
//   B <cm> <speed>    // Backward distance in cm, speed 0..255
//   TL <deg> <speed>  // Turn Left  angle in degrees
//   TR <deg> <speed>  // Turn Right angle in degrees
//   S                 // Stop
//   HB                // Heartbeat (optional)
//
// This parser is independent from UART. Feed each complete line to parse_line().

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_NONE = 0,
    CMD_MOVE,
    CMD_TURN,
    CMD_STOP,
    CMD_HB
} cmd_t;

typedef struct {
    cmd_t type;
    int   forward;    // 1 forward, 0 backward (for MOVE)
    int   left;       // 1 left, 0 right    (for TURN)
    int   dist_cm;    // distance in cm     (MOVE)
    int   angle_deg;  // angle in degree    (TURN)
    int   speed;      // 0..255
} proto_cmd_t;

// Parse a single line (with or without trailing CR/LF). Returns 1 on success, 0 otherwise.
int parse_line(const char *line, proto_cmd_t *out_cmd);

#ifdef __cplusplus
}
#endif


