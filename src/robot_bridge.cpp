#include "robot_bridge.hpp"
#include "command_protocol.hpp"
#include <algorithm>
#include <cctype>

using rovercmd::buildMove;
using rovercmd::buildTurn;
using rovercmd::buildStop;
using rovercmd::buildGripOpen;
using rovercmd::buildGripClose;

RobotBridge::RobotBridge(const RoverConfig &cfg) : config(cfg) {}

int RobotBridge::clamp(int v, int lo, int hi)
{
    return std::max(lo, std::min(hi, v));
}

int RobotBridge::parseFirstNumber(const std::string &text, int fallback)
{
    int sign = 1;
    long val = 0;
    bool inNum = false;
    for (size_t i = 0; i < text.size(); ++i)
    {
        char c = text[i];
        if (!inNum)
        {
            if (c == '-') { sign = -1; inNum = true; }
            else if (std::isdigit(static_cast<unsigned char>(c))) { val = c - '0'; inNum = true; }
        }
        else
        {
            if (std::isdigit(static_cast<unsigned char>(c))) { val = val * 10 + (c - '0'); }
            else break;
        }
    }
    if (!inNum) return fallback;
    val *= sign;
    if (val > 10000) val = 10000;
    if (val < -10000) val = -10000;
    return static_cast<int>(val);
}

std::vector<std::string> RobotBridge::mapInstruction(const std::string &instruction)
{
    std::vector<std::string> out;
    std::string s = instruction;
    // uppercase copy for easier contains
    std::string u = s;
    std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c){ return std::toupper(c); });

    // helpers for dual protocol mode
    auto emitStop = [&](){
        if (config.useStm32Protocol) out.push_back(std::string("S\n"));
        else out.push_back(buildStop());
    };
    auto emitMove = [&](bool forward, int cm, int speed){
        if (config.useStm32Protocol) {
            if (forward) out.push_back("F " + std::to_string(cm) + " " + std::to_string(speed) + "\n");
            else         out.push_back("B " + std::to_string(cm) + " " + std::to_string(speed) + "\n");
        } else {
            out.push_back(buildMove(forward, cm, speed));
        }
    };
    auto emitTurn = [&](bool left, int deg, int speed){
        if (config.useStm32Protocol) {
            if (left)  out.push_back("TL " + std::to_string(deg) + " " + std::to_string(speed) + "\n");
            else       out.push_back("TR " + std::to_string(deg) + " " + std::to_string(speed) + "\n");
        } else {
            out.push_back(buildTurn(left, deg, speed));
        }
    };

    // Stop on completion or unknown
    if (u.find("SCAN COMPLETE") != std::string::npos)
    {
        emitStop();
        return out;
    }
    if (u.find("LOCKED") != std::string::npos)
    {
        emitStop();
        return out;
    }
    if (u.find("GRIP OPEN") != std::string::npos)
    {
        out.push_back(buildGripOpen());
        return out;
    }
    if (u.find("GRIP CLOSE") != std::string::npos)
    {
        out.push_back(buildGripClose());
        return out;
    }
    if (u.find("PUTAR KIRI - SCAN LINGKUNGAN") != std::string::npos)
    {
        // Slow spin left to scan environment
        emitTurn(true, 20, clamp(config.speedTurn, 80, 200));
        return out;
    }

    // Compound hint "+ Sedikit KIRI/KANAN"
    bool sedikitKiri = (u.find("SEDIKIT KIRI") != std::string::npos);
    bool sedikitKanan = (u.find("SEDIKIT KANAN") != std::string::npos);

    // MOVE forward/back
    if (u.find("MAJU") != std::string::npos)
    {
        int cm = parseFirstNumber(u, 15);
        emitMove(true, cm, clamp(config.speedForward, 0, 255));
        if (sedikitKiri) emitTurn(true, 8, clamp(config.speedTurn, 0, 255));
        if (sedikitKanan) emitTurn(false, 8, clamp(config.speedTurn, 0, 255));
        return out;
    }
    if (u.find("MUNDUR") != std::string::npos)
    {
        int cm = parseFirstNumber(u, 10);
        emitMove(false, cm, clamp(config.speedBackward, 0, 255));
        return out;
    }

    // TURN left/right with angle
    if (u.find("PUTAR KIRI") != std::string::npos)
    {
        int deg = parseFirstNumber(u, 15);
        emitTurn(true, deg, clamp(config.speedTurn, 0, 255));
        return out;
    }
    if (u.find("PUTAR KANAN") != std::string::npos)
    {
        int deg = parseFirstNumber(u, 15);
        emitTurn(false, deg, clamp(config.speedTurn, 0, 255));
        return out;
    }

    // Fallback: do nothing or stop
    emitStop();
    return out;
}


