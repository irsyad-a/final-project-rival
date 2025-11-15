#pragma once
#include <string>
#include <vector>

struct RoverConfig
{
    int speedForward = 150;
    int speedBackward = 150;
    int speedTurn = 130;
    int msPerCm = 80;
    int msPerDeg = 12;
    bool useStm32Protocol = false;
};

// RobotBridge maps human-readable navigation instructions into ASCII protocol commands.
// e.g. "MAJU 20 cm + PUTAR KIRI" -> ["MOVE FWD D=20 V=150", "TURN LEFT A=12 V=130"]
class RobotBridge
{
public:
    explicit RobotBridge(const RoverConfig &cfg);

    // Parse high-level Indonesian instruction and return 0..N protocol lines.
    std::vector<std::string> mapInstruction(const std::string &instruction);

private:
    RoverConfig config;

    static int clamp(int v, int lo, int hi);
    static int parseFirstNumber(const std::string &text, int fallback);
};


