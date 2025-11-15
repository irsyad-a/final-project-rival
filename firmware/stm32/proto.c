#include "proto.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static size_t trim_len(const char *s)
{
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || isspace((unsigned char)s[n - 1]))) n--;
    return n;
}

int parse_line(const char *line, proto_cmd_t *o)
{
    if (!line || !o) return 0;
    memset(o, 0, sizeof(*o));

    size_t n = trim_len(line);
    if (n == 0) return 0;

    // STOP
    if (line[0] == 'S' && (n == 1 || isspace((unsigned char)line[1])))
    {
        o->type = CMD_STOP;
        return 1;
    }

    // HeartBeat
    if (n >= 2 && line[0] == 'H' && line[1] == 'B')
    {
        o->type = CMD_HB;
        return 1;
    }

    // MOVE (F/B) or TURN (TL/TR)
    if (line[0] == 'F' || line[0] == 'B')
    {
        // Example: F 15 170
        char c0[3] = {0};
        int cm = 0, sp = 0;
        if (sscanf(line, "%2s %d %d", c0, &cm, &sp) == 3)
        {
            o->type = CMD_MOVE;
            o->forward = (c0[0] == 'F') ? 1 : 0;
            o->dist_cm = cm;
            o->speed = sp;
            return 1;
        }
        return 0;
    }

    if ((line[0] == 'T' && line[1] == 'L') || (line[0] == 'T' && line[1] == 'R'))
    {
        // Example: TL 12 150
        char c0[3] = {0};
        int deg = 0, sp = 0;
        if (sscanf(line, "%2s %d %d", c0, &deg, &sp) == 3)
        {
            o->type = CMD_TURN;
            o->left = (c0[1] == 'L') ? 1 : 0;
            o->angle_deg = deg;
            o->speed = sp;
            return 1;
        }
        return 0;
    }

    return 0;
}


