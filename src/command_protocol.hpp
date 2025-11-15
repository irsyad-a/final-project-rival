// Lightweight helpers to format ASCII commands for the rover firmware.
// Each command ends with a single '\n'.
// Examples:
// MOVE FWD D=20 V=150
// TURN LEFT A=30 V=130
// STOP
// HEARTBEAT
#pragma once
#include <string>

namespace rovercmd
{
inline std::string trim(const std::string &s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

inline std::string buildMove(bool forward, int distanceCm, int speed)
{
    if (distanceCm < 0) distanceCm = 0;
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    std::string dir = forward ? "FWD" : "BACK";
    return "MOVE " + dir + " D=" + std::to_string(distanceCm) + " V=" + std::to_string(speed) + "\n";
}

inline std::string buildTurn(bool left, int angleDeg, int speed)
{
    if (angleDeg < 0) angleDeg = 0;
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;
    std::string dir = left ? "LEFT" : "RIGHT";
    return "TURN " + dir + " A=" + std::to_string(angleDeg) + " V=" + std::to_string(speed) + "\n";
}

inline std::string buildStop()
{
    return "STOP\n";
}

inline std::string buildHeartbeat()
{
    return "HEARTBEAT\n";
}

inline std::string buildCalSetMsPerCm(int msPerCm)
{
    if (msPerCm < 0) msPerCm = 0;
    return "CAL SET ms_per_cm=" + std::to_string(msPerCm) + "\n";
}

inline std::string buildCalSetMsPerDeg(int msPerDeg)
{
    if (msPerDeg < 0) msPerDeg = 0;
    return "CAL SET ms_per_deg=" + std::to_string(msPerDeg) + "\n";
}

inline std::string buildGripOpen()
{
    return "GRIP OPEN\n";
}

inline std::string buildGripClose()
{
    return "GRIP CLOSE\n";
}
} // namespace rovercmd


