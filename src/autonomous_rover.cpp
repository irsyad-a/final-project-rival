#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cctype>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <deque>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <thread>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <atomic>
#include <espeak/speak_lib.h>
#include "serial_transport.hpp"
#include "robot_bridge.hpp"
#include "command_protocol.hpp"
#include "path_planner.hpp"
#include "obstacle_detector.hpp"

// YOLOv5 detector (conditional compilation)
#ifdef USE_YOLO_DETECTOR
#include "yolo_detector.hpp"
#endif

// ==================== Decision logging ====================
struct DecisionEntry
{
    std::string timeStr;
    std::string instruction;
    int numMarkers;
    int targetId;
    double yellowRatio;
};

static bool gEspeakReady = false;
static std::atomic<bool> gTtsEnabled{true};
static bool gShowYellowMaskDebug = false;
static bool gVerboseDistanceLog = false;
static bool gShowRoutinePreview = true;
static bool gShowRoutinePopup = true;

struct ScannedMarkerDetail
{
    int id;
    std::string timestamp;
    float distance;
    float markerSize;
    cv::Point2f position;
    int scanCount;
};

static std::string nowString();
static void showScanSuccessPopup(int markerId, const ScannedMarkerDetail &detail);
static void showDetectionPopup(int markerId, const ScannedMarkerDetail &detail);
static ScannedMarkerDetail buildDetailFromCorners(int markerId, const std::vector<cv::Point2f> &corners);

static void setTtsEnabled(bool enabled)
{
    gTtsEnabled.store(enabled);
}

static void speakText(const std::string &text, bool force = false)
{
    if (text.empty())
        return;

    if (!force && !gTtsEnabled.load())
        return;

    static auto lastSpeak = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSpeak).count() < 2000)
        return;

    lastSpeak = now;

    if (gEspeakReady)
    {
        espeak_SetVoiceByName("id");
        espeak_Synth(text.c_str(), text.size() + 1, 0, POS_CHARACTER, 0, espeakCHARS_UTF8, nullptr, nullptr);
        return;
    }

    if (!force && !gTtsEnabled.load())
        return;

    std::string sanitized = text;
    for (char &ch : sanitized)
    {
        if (ch == '"')
            ch = '\'';
    }
    std::string cmd = "espeak -v id \"" + sanitized + "\" >/dev/null 2>&1";
    int ret = std::system(cmd.c_str());
    static bool fallbackWarned = false;
    if (ret != 0 && !fallbackWarned)
    {
        std::cerr << "WARN: Tidak dapat menjalankan fallback perintah 'espeak'.\n";
        fallbackWarned = true;
    }
}

static void speakNavigationHint(const std::string &instruction)
{
    if (!gTtsEnabled.load())
        return;

    if (instruction.empty())
        return;

    std::string upper = instruction;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c)
                   { return std::toupper(c); });

    if (upper.find("STOP") != std::string::npos)
    {
        speakText("berhenti");
        return;
    }
    if (upper.find("MUNDUR") != std::string::npos)
    {
        speakText("mundur");
        return;
    }
    if (upper.find("MAJU") != std::string::npos)
    {
        speakText("maju");
        return;
    }
    if (upper.find("KIRI") != std::string::npos)
    {
        speakText("ke kiri");
        return;
    }
    if (upper.find("KANAN") != std::string::npos)
    {
        speakText("ke kanan");
    }
}

// ==================== Distance Estimation from Perspective ====================
// Estimasi jarak ke garis kuning berdasarkan posisi pixel dan parameter kamera
static double estimateDistanceToYellowLine(int pixelYPosition, int frameHeight,
                                           double cameraHeightCm, double fovVerticalDeg,
                                           double cameraTiltDeg = 90.0,    // Tambah tilt (derajat dari horizontal)
                                           double calibrationFactor = 1.0) // NEW: Calibration factor untuk fine-tune
{
    /*
     * Fixed perspective projection untuk downward/tilted camera.
     * d = h * tan(alpha), di mana alpha = angle from vertical.
     * Reference: Standard CV distance estimation (tan-based for ground plane).
     *
     * pixelYPosition: Y dari top (0 = far/top, frameHeight = close/bottom)
     * normalizedY kecil = far, alpha besar, d besar.
     * Clamp min 0.01m (1cm) untuk hindari underestimate ekstrem pada proximity.
     *
     * calibrationFactor: Adjust jika estimasi tidak match real (test dengan tape)
     *   - Jika underestimate (estimasi < real), naikkan factor (misal 1.2)
     *   - Jika overestimate (estimasi > real), turunkan factor (misal 0.8)
     */

    if (frameHeight <= 0 || cameraHeightCm <= 0)
        return -1.0;

    double normalizedY = (double)pixelYPosition / (double)frameHeight;

    // FOV to rad
    double fovRad = fovVerticalDeg * M_PI / 180.0;

    // Angle from center (nadir if tilt=90°): flip sign agar top=positive (front/far)
    double pixelAngleFromCenterRad = (0.5 - normalizedY) * fovRad; // Positive untuk top (far), negative untuk bottom (behind)

    // Adjust untuk tilt (tiltDeg from horizontal: 90°=down, <90°=tilted forward)
    double tiltRad = cameraTiltDeg * M_PI / 180.0;
    double angleFromVerticalRad = pixelAngleFromCenterRad + (M_PI / 2.0 - tiltRad); // Tambah offset jika tilted

    // Jika negative (behind), ignore atau clamp
    if (angleFromVerticalRad <= 0.0)
        return 0.01; // Minimal 1cm jika behind/very close

    // Distance horizontal: d = h * tan(alpha_from_vertical) * calibrationFactor
    double distanceCm = cameraHeightCm * std::tan(angleFromVerticalRad) * calibrationFactor;

    // Clamp reasonable range (min 1cm untuk close proximity, max 500cm)
    distanceCm = std::max(1.0, std::min(500.0, distanceCm));

    double distanceM = distanceCm / 100.0;

    if (gVerboseDistanceLog)
    {
        std::cout << "PixelY: " << pixelYPosition << ", NormY: " << normalizedY
                  << ", AlphaDeg: " << (angleFromVerticalRad * 180.0 / M_PI)
                  << ", DistM: " << distanceM << " (Tilt: " << cameraTiltDeg
                  << "°, CalibFactor: " << calibrationFactor << ")" << std::endl;
    }

    return distanceM;
}

// Temporal filter buffer for yellow detection
static std::deque<cv::Mat> yellowMaskHistory;
static const int TEMPORAL_FILTER_SIZE = 3;

// ==================== White Line Detection for Following ====================
// Detect white line only, return errorX (-1 to 1), angleDeg (-45 to 45), and confidence (0-1)
// Pass frame by ref untuk draw contour jika detected
static void detectWhiteLine(cv::Mat &frameBgr, float &errorX, float &angleDeg, double &lineConfidence)
{
    errorX = 0.0f;
    angleDeg = 0.0f;
    lineConfidence = 0.0;

    int h = frameBgr.rows;
    int bandTop = int(h * 0.5); // Bottom 50% untuk deteksi hingga setengah layar bawah
    cv::Mat roi = frameBgr.rowRange(bandTop, h);

    // Step 1: Grayscale + CLAHE for reflection handling
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    // Step 2: Adaptive threshold with Otsu
    cv::Mat maskAdaptive;
    cv::threshold(gray, maskAdaptive, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Step 3: HSV white mask (relaxed for glossy white)
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::Mat maskWhite;
    cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 50, 255), maskWhite); // V>180, S<50

    // Combine masks
    cv::Mat maskLine;
    cv::bitwise_and(maskWhite, maskAdaptive, maskLine);

    // Step 4: Morphology (smaller kernel for speed)
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // Smaller for less computation
    cv::erode(maskLine, maskLine, kernel);
    cv::morphologyEx(maskLine, maskLine, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(maskLine, maskLine, cv::MORPH_CLOSE, kernel);

    // Debug masks (commented out; uncomment for tuning)
    // cv::Mat debugAdaptive, debugWhite, debugLine;
    // cv::resize(maskAdaptive, debugAdaptive, cv::Size(320, 240));
    // cv::imshow("Mask Adaptive Debug", debugAdaptive);
    // cv::resize(maskWhite, debugWhite, cv::Size(320, 240));
    // cv::imshow("Mask White Debug", debugWhite);
    // cv::resize(maskLine, debugLine, cv::Size(320, 240));
    // cv::imshow("Mask Line Debug", debugLine);

    // Step 5: Find largest contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskLine, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return;

    size_t maxIdx = 0;
    double maxArea = 0.0;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxIdx = i;
        }
    }

    // Filter: Min area (relaxed) and in bottom area
    if (maxArea < (roi.rows * roi.cols * 0.01))
        return; // More relaxed for thin lines
    cv::Rect boundRect = cv::boundingRect(contours[maxIdx]);
    double relativeY = static_cast<double>(boundRect.y + bandTop + boundRect.height / 2) / h;
    if (relativeY < 0.5)
        return; // Adjusted for bottom focus

    // Fit line for angle
    cv::Vec4f lineParams;
    cv::fitLine(contours[maxIdx], lineParams, cv::DIST_L2, 0, 0.01, 0.01);
    angleDeg = std::atan2(lineParams[1], lineParams[0]) * 180.0 / M_PI;
    angleDeg = std::clamp(angleDeg, -45.0f, 45.0f);

    // Center calculation
    cv::Moments moments = cv::moments(contours[maxIdx]);
    if (moments.m00 == 0)
        return;
    float centerX = static_cast<float>(moments.m10 / moments.m00);
    errorX = (centerX - roi.cols / 2.0f) / (roi.cols / 2.0f);

    // Confidence: Area + elongation
    cv::RotatedRect rect = cv::minAreaRect(contours[maxIdx]);
    float elongation = std::max(rect.size.width / rect.size.height, rect.size.height / rect.size.width);
    lineConfidence = (maxArea / (roi.rows * roi.cols * 0.01)) * (elongation > 2.0 ? 1.2 : 1.0);
    lineConfidence = std::min(1.0, lineConfidence);

    // No temporal smoothing for speed - use direct values

    // Draw if confident
    if (lineConfidence > 0.5)
    {
        std::vector<std::vector<cv::Point>> fullContours = contours;
        for (auto &pt : fullContours[maxIdx])
            pt.y += bandTop;
        cv::drawContours(frameBgr, fullContours, maxIdx, cv::Scalar(0, 255, 0), 2);
        // Draw fitted line (optional, comment if lag)
        cv::Point pt1(centerX - 1000 * lineParams[0], (boundRect.y + bandTop) - 1000 * lineParams[1]);
        cv::Point pt2(centerX + 1000 * lineParams[0], (boundRect.y + bandTop) + 1000 * lineParams[1]);
        cv::line(frameBgr, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }
}

// ==================== Helper: Kirim instruksi sekali + tunggu selesai ====================
static void sendInstructionOnce(SerialTransport &serial,
                                RobotBridge &bridge,
                                const std::string &instruction,
                                bool sendAsciiProtocol)
{
    std::cout << "→ CMD: " << instruction;
    auto cmds = bridge.mapInstruction(instruction);
    if (!serial.isOpen())
    {
        std::cout << "  [SKIP SEND: serial belum terbuka]\n";
        return;
    }
    if (!sendAsciiProtocol)
    {
        std::cout << "  [SKIP SEND: send_ascii_protocol=false]\n";
        return;
    }
    if (sendAsciiProtocol && !cmds.empty())
    {
        std::cout << "  [SEND " << cmds.size() << " baris]\n";
        for (const auto &c : cmds)
            serial.sendLine(c);
    }
    else
    {
        std::cout << "  [TIADA CMD TERGENERASI]\n";
    }
}

static void sleepMs(int ms)
{
    if (ms <= 0)
        return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ==================== Helper: Cek ArUco beberapa frame dalam jendela waktu ====================
static bool checkArucoDetectedWindow(cv::VideoCapture &cam,
                                     cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                     cv::Ptr<cv::aruco::DetectorParameters> &detectorParams,
                                     int windowMs = 800,
                                     int minHits = 1,
                                     bool lowPowerMode = false,
                                     int lowPowerWidth = 0,
                                     int lowPowerHeight = 0,
                                     int arucoDetectionInterval = 1,
                                     bool showRoutinePreview = true)
{
    std::cout << "   · Cek ArUco (" << windowMs << " ms)...";
    uint64_t start = (uint64_t)cv::getTickCount();
    double freq = cv::getTickFrequency();
    int hits = 0;
    int localFrameCounter = 0;
    std::vector<int> cachedMarkerIds;
    std::vector<std::vector<cv::Point2f>> cachedMarkerCorners;
    bool haveCache = false;

    while (true)
    {
        cv::Mat frame;
        cam >> frame;
        if (frame.empty())
            break;
        localFrameCounter++;

        if (lowPowerMode && lowPowerWidth > 0 && lowPowerHeight > 0)
        {
            if (frame.cols > lowPowerWidth || frame.rows > lowPowerHeight)
            {
                double scale = std::min(static_cast<double>(lowPowerWidth) / frame.cols,
                                        static_cast<double>(lowPowerHeight) / frame.rows);
                if (scale > 0.0 && scale < 1.0)
                {
                    cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_AREA);
                }
            }
        }

        bool runDetection = (arucoDetectionInterval <= 1) ||
                            ((localFrameCounter % arucoDetectionInterval) == 0) ||
                            !haveCache;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        if (runDetection)
        {
            std::vector<std::vector<cv::Point2f>> rejected;
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejected);
            cachedMarkerIds = markerIds;
            cachedMarkerCorners = markerCorners;
            haveCache = true;
        }
        else
        {
            markerIds = cachedMarkerIds;
            markerCorners = cachedMarkerCorners;
        }

        if (showRoutinePreview)
        {
            // Downscale untuk tampilan lebih ringan
            cv::Mat disp;
            double dispScale = (frame.cols > 640) ? (640.0 / frame.cols) : 1.0;
            if (dispScale < 1.0)
                cv::resize(frame, disp, cv::Size(), dispScale, dispScale, cv::INTER_AREA);
            else
                disp = frame;
            // Tampilkan feed kamera + overlay selama rutinitas terarah
            if (!markerIds.empty())
            {
                cv::aruco::drawDetectedMarkers(disp, markerCorners, markerIds);
            }
            cv::putText(disp, "Rutin Cek ArUco", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            cv::imshow("Rover Camera", disp);
            cv::waitKey(1);
        }
        if (!markerIds.empty())
        {
            hits++;
            if (gShowRoutinePopup && !markerCorners.empty())
            {
                ScannedMarkerDetail detail = buildDetailFromCorners(markerIds[0], markerCorners[0]);
                showDetectionPopup(markerIds[0], detail);
            }
            if (hits >= minHits)
            {
                std::cout << " TERDETEKSI (" << hits << " hit)\n";
                return true;
            }
        }
        uint64_t nowTicks = (uint64_t)cv::getTickCount();
        double elapsedMs = (nowTicks - start) * 1000.0 / freq;
        if (elapsedMs >= windowMs)
            break;
    }
    std::cout << " tidak ada.\n";
    return false;
}

// ==================== Rutinitas Autonomous Terarah (scan 4 arah) ====================
static bool runDirectedArucoRoutine(cv::VideoCapture &cam,
                                    SerialTransport &serial,
                                    RobotBridge &bridge,
                                    bool sendAsciiProtocol,
                                    const std::function<void(const std::string &)> &sendCharFn,
                                    cv::Ptr<cv::aruco::Dictionary> &dictionary,
                                    cv::Ptr<cv::aruco::DetectorParameters> &detectorParams,
                                    int msPerCm,
                                    int msPerDeg,
                                    bool lowPowerMode,
                                    int lowPowerWidth,
                                    int lowPowerHeight,
                                    int arucoDetectionInterval,
                                    bool showRoutinePreview)
{
    auto moveWait = [&](const std::string &instr, int cm)
    {
        std::cout << "[RUTIN] " << instr << " (estimasi " << (cm * msPerCm + 300) << " ms)\n";
        // Kirim karakter HC-05 (w/s/...) jika diaktifkan
        if (sendCharFn)
            sendCharFn(instr);
        // Kirim ASCII (F/B/...) jika diaktifkan
        sendInstructionOnce(serial, bridge, instr, sendAsciiProtocol);
        int waitMs = std::max(0, cm * msPerCm + 300); // margin 300ms
        sleepMs(waitMs);
        // Stop setelah selesai durasi
        if (sendCharFn)
            sendCharFn("STOP");
        sendInstructionOnce(serial, bridge, "STOP", sendAsciiProtocol);
    };
    auto turnWait = [&](const std::string &instr, int deg)
    {
        std::cout << "[RUTIN] " << instr << " (estimasi " << (deg * msPerDeg + 300) << " ms)\n";
        if (sendCharFn)
            sendCharFn(instr);
        sendInstructionOnce(serial, bridge, instr, sendAsciiProtocol);
        int waitMs = std::max(0, deg * msPerDeg + 300); // margin 300ms
        sleepMs(waitMs);
        if (sendCharFn)
            sendCharFn("STOP");
        sendInstructionOnce(serial, bridge, "STOP", sendAsciiProtocol);
    };
    auto checkNow = [&]() -> bool
    {
        return checkArucoDetectedWindow(cam, dictionary, detectorParams,
                                        900, 1,
                                        lowPowerMode, lowPowerWidth, lowPowerHeight,
                                        arucoDetectionInterval, showRoutinePreview);
    };

    // 1) Maju 60 cm -> cek
    std::cout << "[LANGKAH 1] ";
    moveWait("MAJU 60 cm", 60);
    if (checkNow())
        return true;

    // 2) Mundur 60 cm
    std::cout << "[LANGKAH 2] ";
    moveWait("MUNDUR 60 cm", 60);
    //    Belok kiri 90°
    turnWait("PUTAR KIRI 90°", 90);

    // 3) Maju 50 cm -> cek
    std::cout << "[LANGKAH 3] ";
    moveWait("MAJU 50 cm", 50);
    if (checkNow())
        return true;

    // 4) Mundur 50 cm
    std::cout << "[LANGKAH 4] ";
    moveWait("MUNDUR 50 cm", 50);
    //    Belok kanan 90° + kanan 90°
    turnWait("PUTAR KANAN 90°", 90);
    turnWait("PUTAR KANAN 90°", 90);

    // 5) Maju 50 cm -> cek
    std::cout << "[LANGKAH 5] ";
    moveWait("MAJU 50 cm", 50);
    if (checkNow())
        return true;

    // 6) Mundur 50 cm
    std::cout << "[LANGKAH 6] ";
    moveWait("MUNDUR 50 cm", 50);
    //    Belok kanan 90°
    turnWait("PUTAR KANAN 90°", 90);

    // 7) Maju 50 cm -> cek
    std::cout << "[LANGKAH 7] ";
    moveWait("MAJU 50 cm", 50);
    if (checkNow())
        return true;

    // 8) Mundur 50 cm -> selesai
    std::cout << "[LANGKAH 8] ";
    moveWait("MUNDUR 50 cm", 50);
    std::cout << "[RUTIN] Selesai tanpa deteksi.\n";
    return false;
}

// Improved yellow detection with adaptive filtering and confidence scoring
static bool detectYellowBoundaryAhead(const cv::Mat &frameBgr, double &coverageRatio, double &confidence)
{
    int h = frameBgr.rows;
    int bandTop = int(h * 0.80); // bottom 20%
    cv::Mat roi = frameBgr.rowRange(bandTop, h);

    // Step 1: Adaptive lighting compensation
    cv::Mat roiLab;
    cv::cvtColor(roi, roiLab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> labChannels;
    cv::split(roiLab, labChannels);

    // Histogram equalization on L channel for better lighting adaptation
    cv::equalizeHist(labChannels[0], labChannels[0]);
    cv::merge(labChannels, roiLab);

    // Step 2: Multi-space color detection
    cv::Mat hsv, maskHSV, maskLAB;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // HSV-based detection (wider range for robustness)
    cv::Scalar lower_yellow_hsv(15, 40, 50);
    cv::Scalar upper_yellow_hsv(45, 255, 255);
    cv::inRange(hsv, lower_yellow_hsv, upper_yellow_hsv, maskHSV);

    // LAB-based detection (B channel: yellow has high B value)
    // L: 50-255 (not too dark), A: 0-135 (yellowish), B: 135-255 (yellow)
    cv::Scalar lower_yellow_lab(50, 0, 135);
    cv::Scalar upper_yellow_lab(255, 135, 255);
    cv::inRange(roiLab, lower_yellow_lab, upper_yellow_lab, maskLAB);

    // Combine both masks (OR operation for better coverage)
    cv::Mat maskCombined;
    cv::bitwise_or(maskHSV, maskLAB, maskCombined);

    // Step 3: Morphological operations for noise removal
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(maskCombined, maskCombined, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(maskCombined, maskCombined, cv::MORPH_CLOSE, kernel);

    // Step 4: Temporal filtering (moving average)
    yellowMaskHistory.push_back(maskCombined.clone());
    if (yellowMaskHistory.size() > TEMPORAL_FILTER_SIZE)
    {
        yellowMaskHistory.pop_front();
    }

    cv::Mat maskFiltered = cv::Mat::zeros(maskCombined.size(), CV_8UC1);
    if (yellowMaskHistory.size() >= 2)
    {
        // Average multiple frames
        cv::Mat accumulator = cv::Mat::zeros(maskCombined.size(), CV_32F);
        for (const auto &m : yellowMaskHistory)
        {
            cv::Mat mFloat;
            m.convertTo(mFloat, CV_32F);
            accumulator += mFloat;
        }
        accumulator /= (float)yellowMaskHistory.size();
        accumulator.convertTo(maskFiltered, CV_8U);
        cv::threshold(maskFiltered, maskFiltered, 127, 255, cv::THRESH_BINARY);
    }
    else
    {
        maskFiltered = maskCombined;
    }

    // Step 5: Confidence scoring
    int yellowPixels = cv::countNonZero(maskFiltered);
    int total = maskFiltered.rows * maskFiltered.cols;
    coverageRatio = total > 0 ? (double)yellowPixels / (double)total : 0.0;

    // Calculate confidence based on multiple factors
    confidence = 0.0;
    if (yellowPixels > 0)
    {
        // Find contours for shape analysis
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskFiltered.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // Factor 1: Coverage ratio (0-1)
            double coverageFactor = std::min(1.0, coverageRatio * 10.0); // Scale up small ratios

            // Factor 2: Shape elongation (lines are elongated, not blobs)
            double maxElongation = 0.0;
            for (const auto &contour : contours)
            {
                if (contour.size() < 5)
                    continue;
                cv::RotatedRect rect = cv::minAreaRect(contour);
                float w = rect.size.width;
                float h = rect.size.height;
                if (w > 0 && h > 0)
                {
                    float elongation = std::max(w / h, h / w);
                    maxElongation = std::max(maxElongation, (double)elongation);
                }
            }
            double shapeFactor = std::min(1.0, maxElongation / 5.0); // Lines have elongation > 5

            // Factor 3: Position consistency (yellow lines usually horizontal at bottom)
            double positionFactor = 1.0; // Assume good by default

            // Combined confidence score
            confidence = (coverageFactor * 0.5 + shapeFactor * 0.3 + positionFactor * 0.2);
        }
    }

    // Threshold: require minimum confidence of 0.3
    return (coverageRatio > 0.02 && confidence > 0.3);
}

// Improved yellow detection with adaptive filtering, LAB space, and confidence scoring
// FOKUS HANYA PADA BOTTOM 30% FRAME (garis kuning ada di bawah, bukan di atas!)
static void detectYellowBoxes(const cv::Mat &frameBgr,
                              int hLow, int hHigh, int sLow, int vLow,
                              bool useAdaptive, int adaptDeltaH,
                              int labALow, int labAHigh, int labBLow, bool combineOr,
                              double bandTopRatio, double relativeMinY,
                              double minAreaRatio,
                              double &coverageRatio,
                              std::vector<cv::Rect> &boxes,
                              int &nearestPx,
                              double &nearestDistanceM,
                              double cameraHeightCm, double fovVerticalDeg,
                              double cameraTiltDeg,     // NEW: Camera tilt angle
                              double calibrationFactor, // NEW: Calibration factor
                              double &confidence)
{
    boxes.clear();
    nearestPx = 0;
    confidence = 0.0;

    int h = frameBgr.rows;
    // Fokus pada bottom portion frame (configurable) - widen ke bottom 40% untuk deteksi lebih awal
    bandTopRatio = std::clamp(bandTopRatio, 0.0, 0.95);
    int bandTop = int(h * 0.60); // CHANGED: Bottom 40% (dari 30%) untuk sensitivity lebih tinggi
    cv::Mat roi = frameBgr.rowRange(bandTop, h);

    // Step 1: Illumination normalization (lightweight, frame-local)
    // 1a) Simple gray-world white balance (channel gain so R,G,B have similar means)
    {
        cv::Scalar meanBGR = cv::mean(roi);
        double meanB = std::max(1.0, meanBGR[0]);
        double meanG = std::max(1.0, meanBGR[1]);
        double meanR = std::max(1.0, meanBGR[2]);
        double meanAvg = (meanB + meanG + meanR) / 3.0;
        double gainB = meanAvg / meanB;
        double gainG = meanAvg / meanG;
        double gainR = meanAvg / meanR;
        cv::Mat roiF;
        roi.convertTo(roiF, CV_32F);
        std::vector<cv::Mat> bgr;
        cv::split(roiF, bgr);
        bgr[0] *= static_cast<float>(gainB);
        bgr[1] *= static_cast<float>(gainG);
        bgr[2] *= static_cast<float>(gainR);
        cv::merge(bgr, roiF);
        cv::Mat roiWB;
        cv::min(roiF, 255.0, roiF);
        roiF.convertTo(roiWB, CV_8U);
        roi = roiWB;
    }

    // NEW: Tambah CLAHE pada channel V (HSV) untuk tingkatkan contrast pada area kuning
    cv::Mat roiHSV;
    cv::cvtColor(roi, roiHSV, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(roiHSV, hsvChannels);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8)); // Clip limit 2.0, tile 8x8
    clahe->apply(hsvChannels[2], hsvChannels[2]);                    // Apply ke Value channel
    cv::merge(hsvChannels, roiHSV);
    cv::cvtColor(roiHSV, roi, cv::COLOR_HSV2BGR); // Konversi kembali ke BGR

    // 1b) Contrast normalization on L channel (CLAHE-like effect with equalizeHist on L)
    cv::Mat roiLab;
    cv::cvtColor(roi, roiLab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> labChannels;
    cv::split(roiLab, labChannels);
    cv::equalizeHist(labChannels[0], labChannels[0]);
    cv::merge(labChannels, roiLab);

    // Step 2: Multi-space detection
    cv::Mat hsv, maskHSV, maskLAB;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // Adaptive HSV range if enabled
    int hLowAdj = hLow;
    int hHighAdj = hHigh;
    int sLowAdj = sLow;
    int vLowAdj = vLow;
    if (useAdaptive)
    {
        // Calculate mean/StdDev for S and V to adapt thresholds
        std::vector<cv::Mat> hsvCh;
        cv::split(hsv, hsvCh);
        cv::Scalar meanS, stdS, meanV, stdV;
        cv::meanStdDev(hsvCh[1], meanS, stdS);
        cv::meanStdDev(hsvCh[2], meanV, stdV);
        // Make S/V more strict in bright scenes, relaxed in dark scenes
        sLowAdj = std::max(sLow, (int)std::round(meanS[0] - 0.25 * stdS[0]));
        vLowAdj = std::max(vLow, (int)std::round(meanV[0] - 0.25 * stdV[0]));
        sLowAdj = std::min(sLowAdj, 255);
        vLowAdj = std::min(vLowAdj, 255);
        // Adjust hue window slightly if very dark
        if (meanV[0] < 100.0)
        {
            hLowAdj = std::max(10, hLow - adaptDeltaH);
            hHighAdj = std::min(50, hHigh + adaptDeltaH);
        }
    }

    // CHANGED: Narrow HSV range untuk ketat (kurangi false positive dari cahaya/non-kuning)
    cv::Scalar lower_yellow_hsv(20, 100, 100); // Narrow H 20-40, S 100+, V 100+ (ketat untuk true yellow)
    cv::Scalar upper_yellow_hsv(40, 255, 255);

    cv::inRange(hsv, lower_yellow_hsv, upper_yellow_hsv, maskHSV);

    // LAB-based detection - Narrow untuk ketat
    // In OpenCV Lab, a* and b* are offset by +128. Yellow tends to have slightly positive a* and high b*.
    // Use configurable a* range to reject greens and reds while keeping yellows.
    int aLow = std::max(0, std::min(255, labALow));
    int aHigh = std::max(aLow, std::min(255, labAHigh));
    // LAB: Narrowing untuk kuning (b* tinggi, a ketat)
    cv::Scalar lower_lab(50, 110, 140); // CHANGED: Ketat a 110-140, b 140+
    cv::Scalar upper_lab(255, 140, 255);
    cv::inRange(roiLab, lower_lab, upper_lab, maskLAB);

    // Additional RGB-ratio gate (R and G should be high and similar, B significantly lower)
    cv::Mat maskRGB;
    {
        std::vector<cv::Mat> bgr;
        cv::split(roi, bgr);
        cv::Mat r = bgr[2], g = bgr[1], b = bgr[0];
        cv::Mat rgClose, bLow;
        // |R-G| / max(R,G) < 0.30 (RELAXED dari 0.25 untuk sensitivity)
        cv::Mat maxRG;
        cv::max(r, g, maxRG);
        cv::Mat diffRG;
        cv::absdiff(r, g, diffRG);
        // Avoid division by zero by adding 1 to denominator
        cv::Mat ratio;
        diffRG.convertTo(diffRG, CV_32F);
        maxRG.convertTo(maxRG, CV_32F);
        ratio = diffRG / (maxRG + 1.0f);
        cv::threshold(ratio, rgClose, 0.30, 255, cv::THRESH_BINARY_INV); // RELAXED threshold
        rgClose.convertTo(rgClose, CV_8U);
        // B <= min(R,G) - 20 (RELAXED dari 30)
        cv::Mat minRG;
        cv::min(r, g, minRG);
        cv::Mat minRGMinus;
        cv::subtract(minRG, 20, minRGMinus);
        cv::compare(b, minRGMinus, bLow, cv::CMP_LE);
        cv::bitwise_and(rgClose, bLow, maskRGB);
    }

    // Combine masks - CHANGED: Selalu gunakan AND untuk ketat (reduce false positive)
    cv::Mat mask;
    cv::Mat maskHsvLab;
    cv::bitwise_and(maskHSV, maskLAB, maskHsvLab); // AND untuk robustness
    cv::bitwise_and(maskHsvLab, maskRGB, mask);

    // Step 3: Advanced morphological operations - RELAX kernel untuk tangkap garis tipis
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // CHANGED: Smaller kernel (3x3 dari 5x5) untuk sensitivity
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // Additional: Remove small isolated pixels with smaller kernel
    cv::Mat kernelSmall = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)); // CHANGED: Even smaller (2x2)
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernelSmall);

    // NEW: Debug - Tampilkan mask hanya jika diaktifkan di config
    if (gShowYellowMaskDebug)
    {
        cv::Mat maskDebug;
        cv::resize(mask, maskDebug, cv::Size(320, 240));
        cv::imshow("Yellow Mask Debug", maskDebug);
    }

    // Step 4: Line confirmation dengan Hough Line Transform (untuk mengurangi false positive)
    cv::Mat edges;
    cv::Canny(mask, edges, 40, 120); // CHANGED: Lower thresholds (40,120 dari 50,150) untuk deteksi edge lebih sensitif
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 30, 5); // CHANGED: Lower threshold (30 dari 50), minLineLength (30 dari 50), maxGap (5 dari 10) untuk garis pendek

    // Jika tidak ada garis terdeteksi, kemungkinan bukan garis kuning sebenarnya
    if (lines.empty())
    {
        // Reset confidence untuk menghindari false positive dari blob acak
        confidence = 0.0;
        coverageRatio = 0.0;
        nearestDistanceM = -1.0;
        return;
    }

    // Hitung jumlah garis horizontal (indikasi garis kuning di lantai)
    int horizontalLines = 0;
    for (const auto &line : lines)
    {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double angle = std::atan2(std::abs(y2 - y1), std::abs(x2 - x1)) * 180.0 / M_PI;
        // Garis horizontal: angle < 30 derajat (KETAT dari 45 untuk fokus horizontal true)
        if (angle < 30.0)
        {
            horizontalLines++;
        }
    }

    // Boost confidence jika banyak garis horizontal terdeteksi
    double lineConfidenceBoost = std::min(1.3, 1.0 + (horizontalLines * 0.1)); // CHANGED: Higher boost (0.1 dari 0.05)

    // Calculate coverage ratio
    int yellowPixels = cv::countNonZero(mask);
    int total = mask.rows * mask.cols;
    coverageRatio = total > 0 ? (double)yellowPixels / (double)total : 0.0;

    // Find contours and filter by quality
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double minArea = std::max(0.0, 0.002) * (double)(roi.rows * roi.cols); // CHANGED: Kurangi minAreaRatio ke 0.002 untuk deteksi garis kecil
    int topMost = h;

    // Confidence calculation factors
    double totalConfidence = 0.0;
    int validContours = 0;

    for (const auto &c : contours)
    {
        double area = cv::contourArea(c);
        if (area < minArea)
            continue;

        cv::Rect r = cv::boundingRect(c);
        cv::Rect rf(r.x, r.y + bandTop, r.width, r.height);

        // CRITICAL: Filter deteksi yang ada di ATAS frame (bukan di bottom)
        // Garis kuning HARUS ada di bottom portion (configurable threshold dari atas)
        double relativeY = static_cast<double>(rf.y + rf.height / 2) / h;
        if (relativeY < 0.8) // CHANGED: Ketatkan relativeMinY ke 0.8 (hanya bottom 20% frame)
        {
            // Deteksi terlalu tinggi di frame - REJECT!
            // Ini kemungkinan bukan garis kuning di lantai
            continue;
        }

        // Shape analysis for confidence
        double contourConfidence = 1.0;
        if (c.size() >= 5)
        {
            cv::RotatedRect rect = cv::minAreaRect(c);
            float w = rect.size.width;
            float h_rect = rect.size.height;
            if (w > 0 && h_rect > 0)
            {
                float elongation = std::max(w / h_rect, h_rect / w);
                // Relax shape requirement when very near bottom (close to camera)
                bool nearBottom = (relativeY > 0.85);
                bool largeArea = (area > (minArea * 3.0));
                if (!nearBottom && !largeArea)
                {
                    // Hard reject blobs with low elongation to avoid false positives
                    if (elongation < 2.0f) // CHANGED: Ketat dari 1.5 ke 2.0 untuk elongated true
                    {
                        continue;
                    }
                }
                // Yellow lines should be elongated (aspect ratio > 2)
                if (elongation > 2.0)
                {
                    contourConfidence *= 1.2; // Boost confidence for line-like shapes
                }
                else if (elongation < 1.5)
                {
                    contourConfidence *= 0.7; // Reduce confidence for blob-like shapes
                }
            }
        }

        // Area-based confidence (larger areas are more reliable)
        double areaFactor = std::min(1.5, std::sqrt(area / minArea));
        contourConfidence *= areaFactor;

        // Position-based confidence boost (closer to bottom = higher confidence)
        if (relativeY > 0.80)
        {
            contourConfidence *= 1.1; // Very bottom of frame
        }

        // Clamp confidence
        contourConfidence = std::min(1.0, contourConfidence);
        totalConfidence += contourConfidence;
        validContours++;

        boxes.push_back(rf);

        if (rf.y < topMost)
            topMost = rf.y;
    }

    // Overall confidence score
    if (validContours > 0)
    {
        confidence = totalConfidence / validContours;
        // Boost confidence if multiple consistent detections
        if (validContours > 2)
        {
            confidence = std::min(1.0, confidence * 1.1);
        }
        // Apply line detection confidence boost
        confidence = std::min(1.0, confidence * lineConfidenceBoost);
    }

    // Calculate distance estimation
    nearestDistanceM = -1.0;
    if (!boxes.empty())
    {
        nearestPx = h - topMost;
        if (nearestPx < 0)
            nearestPx = 0;
        // Estimate distance (with tilt and calibration factor)
        nearestDistanceM = estimateDistanceToYellowLine(topMost, h, cameraHeightCm, fovVerticalDeg,
                                                        cameraTiltDeg, calibrationFactor);
    }
}
// ========== Simple config loader (key: value, dotted keys supported) ==========
static std::map<std::string, std::string> loadConfig(const std::vector<std::string> &candidatePaths)
{
    auto trim = [](const std::string &s)
    {
        size_t b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos)
            return std::string();
        size_t e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    };
    std::map<std::string, std::string> cfg;
    for (const auto &path : candidatePaths)
    {
        std::ifstream in(path);
        if (!in.is_open())
            continue;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty())
                continue;
            if (line[0] == '#')
                continue;
            auto pos = line.find(':');
            if (pos == std::string::npos)
                continue;
            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos + 1));
            // remove surrounding quotes if any
            if (!value.empty() && (value.front() == '"' || value.front() == '\''))
                value.erase(0, 1);
            if (!value.empty() && (value.back() == '"' || value.back() == '\''))
                value.pop_back();
            if (!key.empty())
                cfg[key] = value;
        }
        break; // stop at the first file found
    }
    return cfg;
}

static std::string cfgStr(const std::map<std::string, std::string> &cfg, const std::string &key, const std::string &defVal)
{
    auto it = cfg.find(key);
    return it == cfg.end() ? defVal : it->second;
}

static int cfgInt(const std::map<std::string, std::string> &cfg, const std::string &key, int defVal)
{
    auto it = cfg.find(key);
    if (it == cfg.end())
        return defVal;
    try
    {
        return std::stoi(it->second);
    }
    catch (...)
    {
        return defVal;
    }
}

// ========== Minimal YAML key updater (replace in-place or append) ==========
static bool saveYamlKeys(const std::string &yamlPath,
                         const std::vector<std::pair<std::string, std::string>> &keyValues)
{
    // Read existing lines (if any)
    std::ifstream in(yamlPath);
    std::vector<std::string> lines;
    if (in.is_open())
    {
        std::string line;
        while (std::getline(in, line))
            lines.push_back(line);
    }
    // Track replaced flags
    std::vector<bool> replaced(keyValues.size(), false);
    // Replace lines starting with "key:"
    for (std::string &line : lines)
    {
        for (size_t i = 0; i < keyValues.size(); ++i)
        {
            const auto &kv = keyValues[i];
            const std::string prefix = kv.first + ":";
            if (line.rfind(prefix, 0) == 0)
            {
                line = kv.first + ": " + kv.second;
                replaced[i] = true;
            }
        }
    }
    // Rebuild lines ensuring appended keys are correct
    {
        std::vector<std::string> finalLines;
        finalLines.reserve(lines.size());
        for (const auto &ln : lines)
            finalLines.push_back(ln);
        // Find which keys are still missing after replacement
        for (size_t i = 0; i < keyValues.size(); ++i)
        {
            if (!replaced[i])
            {
                finalLines.push_back(keyValues[i].first + ": " + keyValues[i].second);
            }
        }
        lines.swap(finalLines);
    }
    // Write to temp then rename atomically
    std::string tmpPath = yamlPath + ".tmp";
    {
        std::ofstream out(tmpPath, std::ios::trunc);
        if (!out.is_open())
            return false;
        for (size_t i = 0; i < lines.size(); ++i)
        {
            out << lines[i];
            if (i + 1 < lines.size())
                out << "\n";
        }
    }
    return std::rename(tmpPath.c_str(), yamlPath.c_str()) == 0;
}

// ==================== Visited markers persistence (txt/json fallback) ====================
static std::set<int> loadVisitedTxt(const std::string &path)
{
    std::set<int> visited;
    std::ifstream in(path);
    if (!in.is_open())
        return visited;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;
        try
        {
            int id = std::stoi(line);
            visited.insert(id);
        }
        catch (...)
        {
        }
    }
    return visited;
}

static void saveVisitedTxt(const std::string &path, const std::set<int> &visited)
{
    // ensure parent directory exists
    try
    {
        std::filesystem::path p(path);
        if (p.has_parent_path())
            std::filesystem::create_directories(p.parent_path());
    }
    catch (...)
    {
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open())
        return;
    for (int id : visited)
    {
        out << id << "\n";
    }
}

// legacy JSON loader/saver (kept for backward compatibility)
static std::set<int> loadVisited(const std::string &jsonPath)
{
    std::set<int> visited;
    std::ifstream in(jsonPath);
    if (!in.is_open())
        return visited;
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    // very small parser: expect [1,2,3]
    int num = 0;
    bool inNum = false, neg = false;
    for (char c : content)
    {
        if (c == '-')
        {
            neg = true;
            continue;
        }
        if (c >= '0' && c <= '9')
        {
            inNum = true;
            num = num * 10 + (c - '0');
        }
        else
        {
            if (inNum)
            {
                visited.insert(neg ? -num : num);
                num = 0;
                inNum = false;
                neg = false;
            }
        }
    }
    if (inNum)
        visited.insert(neg ? -num : num);
    return visited;
}

static void saveVisited(const std::string &jsonPath, const std::set<int> &visited)
{
    // ensure parent directory exists
    try
    {
        std::filesystem::path p(jsonPath);
        if (p.has_parent_path())
            std::filesystem::create_directories(p.parent_path());
    }
    catch (...)
    {
    }
    std::ofstream out(jsonPath, std::ios::trunc);
    if (!out.is_open())
        return;
    out << "[";
    bool first = true;
    for (int id : visited)
    {
        if (!first)
            out << ",";
        out << id;
        first = false;
    }
    out << "]";
}

static std::map<int, ScannedMarkerDetail> loadScannedDatabase(const std::string &dbPath)
{
    std::map<int, ScannedMarkerDetail> db;
    std::ifstream in(dbPath);
    if (!in.is_open())
        return db;

    std::string line;
    std::getline(in, line); // skip header
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;
        std::stringstream ss(line);
        ScannedMarkerDetail detail;
        char comma;
        ss >> detail.id >> comma;
        std::getline(ss, detail.timestamp, ',');
        ss >> detail.distance >> comma >> detail.markerSize >> comma;
        ss >> detail.position.x >> comma >> detail.position.y >> comma;
        ss >> detail.scanCount;
        db[detail.id] = detail;
    }
    return db;
}

static void saveScannedDatabase(const std::string &dbPath, const std::map<int, ScannedMarkerDetail> &db)
{
    std::ofstream out(dbPath, std::ios::trunc);
    if (!out.is_open())
        return;
    out << "id,timestamp,distance,size,pos_x,pos_y,scan_count\n";
    for (const auto &pair : db)
    {
        const auto &d = pair.second;
        out << d.id << "," << d.timestamp << "," << d.distance << ","
            << d.markerSize << "," << d.position.x << "," << d.position.y << ","
            << d.scanCount << "\n";
    }
}

// Pop-up notification window
static void showScanSuccessPopup(int markerId, const ScannedMarkerDetail &detail)
{
    // Create notification popup (500x200 pixels)
    cv::Mat popup(200, 500, CV_8UC3, cv::Scalar(20, 120, 20)); // Green background

    // Title
    cv::putText(popup, "ARUCO BERHASIL DISCAN!",
                cv::Point(80, 40), cv::FONT_HERSHEY_DUPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);

    // ID info
    std::string idText = "ID: " + std::to_string(markerId);
    cv::putText(popup, idText,
                cv::Point(180, 80), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                cv::Scalar(255, 255, 0), 2);

    // Distance info
    std::string distText = "Distance: ~" + std::to_string((int)detail.distance) + " cm";
    cv::putText(popup, distText,
                cv::Point(120, 115), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 1);

    // Timestamp
    cv::putText(popup, detail.timestamp,
                cv::Point(100, 145), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);

    // Instruction
    cv::putText(popup, "Tekan tombol untuk lanjut...",
                cv::Point(110, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(255, 255, 255), 1);

    // Show popup
    cv::namedWindow("Scan Success", cv::WINDOW_NORMAL);
    cv::resizeWindow("Scan Success", 500, 200);
    cv::imshow("Scan Success", popup);
    cv::waitKey(5000); // Show for 5 seconds
    cv::destroyWindow("Scan Success");
}

// NEW: Popup untuk detection (bukan scan) - ID BESAR, detail font KECIL
static void showDetectionPopup(int markerId, const ScannedMarkerDetail &detail)
{
    // Create popup (600x300 pixels untuk lebih besar)
    cv::Mat popup(300, 600, CV_8UC3, cv::Scalar(20, 20, 120)); // Blue background untuk detection

    // Title
    cv::putText(popup, "ARUCO TERDETEKSI!",
                cv::Point(100, 50), cv::FONT_HERSHEY_DUPLEX, 1.0,
                cv::Scalar(255, 255, 255), 2);

    // ID besar (font scale 2.0)
    std::string idText = "ID: " + std::to_string(markerId);
    cv::putText(popup, idText,
                cv::Point(200, 120), cv::FONT_HERSHEY_SIMPLEX, 2.0, // Besar!
                cv::Scalar(255, 255, 0), 3);                        // Tebal

    // Detail dengan font kecil (scale 0.5)
    int y = 160;
    std::string distText = "Distance: ~" + std::to_string((int)detail.distance) + " cm";
    cv::putText(popup, distText, cv::Point(150, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y += 25;

    std::string sizeText = "Size: " + std::to_string((int)detail.markerSize) + " px";
    cv::putText(popup, sizeText, cv::Point(150, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y += 25;

    std::string posText = "Position: (" + std::to_string((int)detail.position.x) + ", " + std::to_string((int)detail.position.y) + ")";
    cv::putText(popup, posText, cv::Point(150, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y += 25;

    std::string countText = "Scan Count: " + std::to_string(detail.scanCount);
    cv::putText(popup, countText, cv::Point(150, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);
    y += 25;

    std::string timeText = "Timestamp: " + detail.timestamp;
    cv::putText(popup, timeText, cv::Point(150, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1);

    // Show popup
    cv::namedWindow("ArUco Detected", cv::WINDOW_NORMAL);
    cv::resizeWindow("ArUco Detected", 600, 300);
    cv::imshow("ArUco Detected", popup);
    cv::waitKey(5000); // 5 detik
    cv::destroyWindow("ArUco Detected");
}

static ScannedMarkerDetail buildDetailFromCorners(int markerId, const std::vector<cv::Point2f> &corners)
{
    ScannedMarkerDetail detail;
    detail.id = markerId;
    detail.timestamp = nowString();
    detail.scanCount = 0;
    if (corners.size() >= 4)
    {
        cv::Point2f center(0, 0);
        for (const auto &pt : corners)
            center += pt;
        center *= (1.0f / static_cast<float>(corners.size()));
        detail.position = center;

        float dx = corners[0].x - corners[2].x;
        float dy = corners[0].y - corners[2].y;
        float diag = std::sqrt(dx * dx + dy * dy);
        detail.markerSize = diag;
        detail.distance = std::max(5.0f, 1000.0f / (diag + 1.0f));
    }
    else
    {
        detail.position = cv::Point2f(0, 0);
        detail.markerSize = 0.0f;
        detail.distance = 0.0f;
    }
    return detail;
}

// ==================== Autonomous Navigator (trimmed from existing) ====================
enum RobotState
{
    SCANNING,
    TRACKING,
    LOCKED,
    APPROACHING,
    SCAN_COMPLETE,
    FOLLOW_LINE // NEW: State untuk line following saat no ArUco
};

struct MarkerInfo
{
    int id;
    cv::Point2f center;
    float size;
    float distance;
    float angle;
    time_t lastSeen;
    bool isScanned;
    double confidence = 0.0; // NEW: Confidence based on size/distance (0-1)
};

// ==================== Simple Pose & Path Structures ====================
struct Pose
{
    double xCm = 0.0;      // centimeters
    double yCm = 0.0;      // centimeters
    double thetaDeg = 0.0; // degrees (0 = +x axis)
};

// ArUco Waypoint structure for map display (different from path planning Waypoint)
struct ArUcoWaypoint
{
    int id;
    double x, y; // in cm from origin
    bool visited;
};

// Helper function for timestamp
static std::string nowString()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[64];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tstruct);
    return std::string(buf);
}

static void appendPathCsv(const std::string &csvPath, const Pose &p, const std::string &instruction)
{
    // ensure parent directory exists
    try
    {
        std::filesystem::path pp(csvPath);
        if (pp.has_parent_path())
            std::filesystem::create_directories(pp.parent_path());
    }
    catch (...)
    {
    }
    // create file with header if not exists
    bool needHeader = !std::filesystem::exists(csvPath);
    std::ofstream out(csvPath, std::ios::app);
    if (!out.is_open())
        return;
    if (needHeader)
    {
        out << "timestamp,x_cm,y_cm,theta_deg,instruction\n";
    }
    out << nowString() << "," << p.xCm << "," << p.yCm << "," << p.thetaDeg << ",\"" << instruction << "\"\n";
}

static int parseFirstNumberFromText(const std::string &text, int fallback)
{
    int sign = 1;
    long val = 0;
    bool inNum = false;
    for (size_t i = 0; i < text.size(); ++i)
    {
        char c = text[i];
        if (!inNum)
        {
            if (c == '-')
            {
                sign = -1;
                inNum = true;
            }
            else if (c >= '0' && c <= '9')
            {
                val = c - '0';
                inNum = true;
            }
        }
        else
        {
            if (c >= '0' && c <= '9')
            {
                val = val * 10 + (c - '0');
            }
            else
                break;
        }
    }
    if (!inNum)
        return fallback;
    val *= sign;
    if (val > 10000)
        val = 10000;
    if (val < -10000)
        val = -10000;
    return static_cast<int>(val);
}

static void updatePoseFromInstruction(Pose &p, const std::string &instruction)
{
    std::string u = instruction;
    std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c)
                   { return std::toupper(c); });
    if (u.find("PUTAR KIRI") != std::string::npos)
    {
        int deg = parseFirstNumberFromText(u, 15);
        p.thetaDeg += deg;
    }
    else if (u.find("PUTAR KANAN") != std::string::npos)
    {
        int deg = parseFirstNumberFromText(u, 15);
        p.thetaDeg -= deg;
    }
    else if (u.find("MAJU") != std::string::npos)
    {
        int cm = parseFirstNumberFromText(u, 15);
        double rad = p.thetaDeg * M_PI / 180.0;
        p.xCm += std::cos(rad) * cm;
        p.yCm += std::sin(rad) * cm;
    }
    else if (u.find("MUNDUR") != std::string::npos)
    {
        int cm = parseFirstNumberFromText(u, 10);
        double rad = p.thetaDeg * M_PI / 180.0;
        p.xCm -= std::cos(rad) * cm;
        p.yCm -= std::sin(rad) * cm;
    }
    // normalize theta
    while (p.thetaDeg >= 180.0)
        p.thetaDeg -= 360.0;
    while (p.thetaDeg < -180.0)
        p.thetaDeg += 360.0;
}

static cv::Point poseToGrid(const Pose &p, int gridSize, double cmPerPixel)
{
    int cx = gridSize / 2;
    int cy = gridSize / 2;
    int gx = cx + static_cast<int>(std::round(p.xCm / cmPerPixel));
    int gy = cy - static_cast<int>(std::round(p.yCm / cmPerPixel));
    gx = std::max(0, std::min(gridSize - 1, gx));
    gy = std::max(0, std::min(gridSize - 1, gy));
    return cv::Point(gx, gy);
}

static std::string nodeKey(const Pose &p, double cellSizeCm, int headingBinDeg)
{
    int ix = static_cast<int>(std::floor(p.xCm / cellSizeCm));
    int iy = static_cast<int>(std::floor(p.yCm / cellSizeCm));
    int hb = static_cast<int>(std::floor((p.thetaDeg + 360.0) / headingBinDeg)) % (360 / headingBinDeg);
    return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(hb);
}

static cv::Rect computeViewportROI(const cv::Point &center, int gridSize, int viewSizePx)
{
    int half = viewSizePx / 2;
    int x0 = center.x - half;
    int y0 = center.y - half;
    if (x0 < 0)
        x0 = 0;
    if (y0 < 0)
        y0 = 0;
    if (x0 + viewSizePx > gridSize)
        x0 = std::max(0, gridSize - viewSizePx);
    if (y0 + viewSizePx > gridSize)
        y0 = std::max(0, gridSize - viewSizePx);
    return cv::Rect(x0, y0, std::min(viewSizePx, gridSize), std::min(viewSizePx, gridSize));
}

static void drawGridScaleBar(cv::Mat &bgr, double cmPerPixel)
{
    // Scale bar di pojok kiri bawah
    int barLengthCm = 50; // 50 cm scale bar
    int barLengthPx = (int)(barLengthCm / cmPerPixel);
    cv::Point p1(20, bgr.rows - 30);
    cv::Point p2(20 + barLengthPx, bgr.rows - 30);

    // Background untuk scale bar
    cv::rectangle(bgr, cv::Point(p1.x - 5, p1.y - 25),
                  cv::Point(p2.x + 5, p1.y + 10),
                  cv::Scalar(0, 0, 0), -1);

    // Scale bar line
    cv::line(bgr, p1, p2, cv::Scalar(255, 255, 255), 3);
    cv::line(bgr, cv::Point(p1.x, p1.y - 5), cv::Point(p1.x, p1.y + 5), cv::Scalar(255, 255, 255), 2);
    cv::line(bgr, cv::Point(p2.x, p2.y - 5), cv::Point(p2.x, p2.y + 5), cv::Scalar(255, 255, 255), 2);

    // Text label
    std::string label = std::to_string(barLengthCm) + " cm";
    cv::putText(bgr, label, cv::Point(p1.x, p1.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
}

static void drawCompass(cv::Mat &bgr, double headingDeg)
{
    // Kompas di pojok kanan atas
    int radius = 40;
    cv::Point center(bgr.cols - 60, 60);

    // Background circle
    cv::circle(bgr, center, radius + 5, cv::Scalar(0, 0, 0), -1);
    cv::circle(bgr, center, radius, cv::Scalar(60, 60, 60), -1);
    cv::circle(bgr, center, radius, cv::Scalar(200, 200, 200), 2);

    // Cardinal directions (N, S, E, W)
    cv::putText(bgr, "N", cv::Point(center.x - 7, center.y - radius + 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(bgr, "S", cv::Point(center.x - 5, center.y + radius - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    cv::putText(bgr, "E", cv::Point(center.x + radius - 12, center.y + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);
    cv::putText(bgr, "W", cv::Point(center.x - radius + 5, center.y + 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

    // Heading arrow (robot direction)
    double rad = -headingDeg * M_PI / 180.0; // Negative for screen coords
    cv::Point2f tip(center.x + (float)(std::sin(rad) * radius * 0.8),
                    center.y - (float)(std::cos(rad) * radius * 0.8));
    cv::arrowedLine(bgr, center, tip, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, 0.3);

    // Center dot
    cv::circle(bgr, center, 3, cv::Scalar(255, 255, 0), -1);
}

static void drawWaypoints(cv::Mat &bgr, const std::map<int, ArUcoWaypoint> &waypoints,
                          const Pose &robotPose, int viewSizePx, double cmPerPixel)
{
    // Draw waypoints (ArUco marker positions) relative to robot
    cv::Point2f robotCenter(viewSizePx / 2.0f, viewSizePx / 2.0f);

    for (const auto &pair : waypoints)
    {
        const ArUcoWaypoint &wp = pair.second;
        // Calculate relative position to robot in viewport
        double dx = wp.x - robotPose.xCm;
        double dy = wp.y - robotPose.yCm;

        // Convert to viewport coordinates (centered on robot)
        int px = (int)(robotCenter.x + dx / cmPerPixel);
        int py = (int)(robotCenter.y + dy / cmPerPixel);

        // Only draw if within viewport
        if (px >= 0 && px < bgr.cols && py >= 0 && py < bgr.rows)
        {
            cv::Scalar color = wp.visited ? cv::Scalar(100, 100, 255) : cv::Scalar(0, 255, 255); // Blue if visited, Yellow if not

            // Draw marker
            cv::drawMarker(bgr, cv::Point(px, py), color, cv::MARKER_STAR, 12, 2);

            // Draw ID label
            std::string label = "M" + std::to_string(wp.id);
            cv::putText(bgr, label, cv::Point(px + 8, py - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
        }
    }
}

static void drawRobotOverlay(cv::Mat &bgr, int viewSizePx, double robotSizePx, double headingDeg)
{
    // Robot di tengah viewport
    cv::Point2f center(viewSizePx / 2.0f, viewSizePx / 2.0f);
    cv::RotatedRect rr(center, cv::Size2f(robotSizePx, robotSizePx), (float)(-headingDeg));
    cv::Point2f pts[4];
    rr.points(pts);
    for (int i = 0; i < 4; ++i)
    {
        cv::line(bgr, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
    }
    // Heading arrow
    double rad = (-headingDeg) * M_PI / 180.0;
    cv::Point2f tip(center.x + (float)(std::cos(rad) * robotSizePx),
                    center.y + (float)(std::sin(rad) * robotSizePx));
    cv::arrowedLine(bgr, center, tip, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.25);
    // Crosshair center
    cv::drawMarker(bgr, center, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 12, 2);
}

class AutonomousNavigator
{
private:
    RobotState currentState;
    std::map<int, MarkerInfo> detectedMarkers;
    std::map<int, bool> scannedMarkers;
    int targetMarkerId;

    int totalMarkersFound;
    int totalMarkersScanned;
    time_t sessionStartTime;
    std::deque<std::string> navigationHistory;
    int maxHistorySize;

    float centerThreshold;
    float approachDistance;
    float minMarkerSize;
    float farMarkerSize;
    int frameWidth;
    int frameHeight;
    cv::Point2f frameCenter;
    bool lastTurnLeft = true; // NEW: Track last turn direction untuk alternasi

public:
    AutonomousNavigator()
        : currentState(SCANNING),
          targetMarkerId(-1),
          totalMarkersFound(0),
          totalMarkersScanned(0),
          sessionStartTime(time(0)),
          maxHistorySize(10),
          centerThreshold(120.0f),
          approachDistance(180.0f),
          minMarkerSize(150.0f),
          farMarkerSize(50.0f),
          frameWidth(640),
          frameHeight(480),
          lastTurnLeft(true) // NEW: Init last turn
    {
        frameCenter = cv::Point2f(frameWidth / 2.0f, frameHeight / 2.0f);
    }

    void setFrameSize(int width, int height)
    {
        frameWidth = width;
        frameHeight = height;
        frameCenter = cv::Point2f(frameWidth / 2.0f, frameHeight / 2.0f);
    }

    void preloadVisited(const std::set<int> &visited)
    {
        for (int id : visited)
            scannedMarkers[id] = true;
    }

    void updateMarkers(const std::vector<int> &markerIds,
                       const std::vector<std::vector<cv::Point2f>> &markerCorners)
    {
        time_t currentTime = time(0);
        std::set<int> currentFrameMarkers;
        const float minMarkerSizePx = 20.0f;      // NEW: Minimal size px untuk valid marker (hindari false positive kecil/noisy)
        const float maxMarkerDistanceCm = 100.0f; // NEW: Ignore jika estimasi jarak > 100 cm (asumsi ArUco tak relevan jika terlalu jauh)

        for (size_t i = 0; i < markerIds.size(); i++)
        {
            int id = markerIds[i];

            cv::Point2f center(0, 0);
            for (int j = 0; j < 4; j++)
            {
                center.x += markerCorners[i][j].x;
                center.y += markerCorners[i][j].y;
            }
            center.x /= 4.0f;
            center.y /= 4.0f;

            float dx = markerCorners[i][0].x - markerCorners[i][2].x;
            float dy = markerCorners[i][0].y - markerCorners[i][2].y;
            float size = std::sqrt(dx * dx + dy * dy);

            // NEW: Filter false positive - ignore jika size terlalu kecil atau distance terlalu jauh
            if (size < minMarkerSizePx)
            {
                std::cout << "Ignored ArUco ID " << id << " - too small (" << size << " px < " << minMarkerSizePx << ")\n";
                continue;
            }

            float angle = std::atan2(center.y - frameCenter.y, center.x - frameCenter.x) * 180.0f / M_PI;
            float distance = 1000.0f / (size + 1.0f);

            if (distance > maxMarkerDistanceCm)
            {
                std::cout << "Ignored ArUco ID " << id << " - too far (~" << (int)distance << " cm > " << (int)maxMarkerDistanceCm << ")\n";
                continue;
            }

            currentFrameMarkers.insert(id);

            if (detectedMarkers.find(id) == detectedMarkers.end())
            {
                totalMarkersFound++;
            }

            // NEW: Hitung confidence (size normalized * (1 - dist norm), clamp 0-1)
            double sizeConfidence = std::min(1.0, static_cast<double>(size) / minMarkerSize);
            double distConfidence = std::max(0.0, 1.0 - (distance / maxMarkerDistanceCm));
            double conf = (sizeConfidence * 0.7 + distConfidence * 0.3); // Weighted 70% size, 30% distance

            MarkerInfo info;
            info.id = id;
            info.center = center;
            info.size = size;
            info.distance = distance;
            info.angle = angle;
            info.lastSeen = currentTime;
            info.isScanned = scannedMarkers[id];
            info.confidence = conf; // Set confidence

            detectedMarkers[id] = info;
        }
    }

    int selectTargetMarker()
    {
        if (detectedMarkers.empty())
            return -1;
        int bestTarget = -1;
        float bestScore = FLT_MAX;
        for (const auto &pair : detectedMarkers)
        {
            const MarkerInfo &info = pair.second;
            if (info.isScanned)
                continue;
            float dx = info.center.x - frameCenter.x;
            float dy = info.center.y - frameCenter.y;
            float distFromCenter = std::sqrt(dx * dx + dy * dy);
            float score = distFromCenter - info.size * 0.5f;
            if (score < bestScore)
            {
                bestScore = score;
                bestTarget = info.id;
            }
        }
        return bestTarget;
    }

    std::string getNavigationInstruction(bool hasYellowAhead, bool hasObstacleAhead, float yellowOffsetX = 0.0f, float lineErrorX = 0.0f, float lineAngleDeg = 0.0f, double lineConfidence = 0.0) // CHANGED: Tambah lineAngleDeg untuk angle-aware following
    {
        std::string evadeDir = (yellowOffsetX > 0) ? "KIRI" : "KANAN"; // Belok away dari yellow offset (kanan -> putar kiri)

        if (detectedMarkers.empty())
        {
            currentState = FOLLOW_LINE; // NEW: Switch ke follow line jika no ArUco

            // NEW: Line following jika confident
            if (lineConfidence > 0.5)
            { // Hanya jika line detected baik
                // Steering: Use PID-like (Kp for errorX, add angle compensation)
                float steering = lineErrorX + 0.1f * lineAngleDeg; // Incorporate angle for turns (0.1 factor untuk smooth)

                // Speed: Slow if angle large (anticipate sharp turns)
                bool isSharpTurn = std::abs(lineAngleDeg) > 20.0f;

                if (std::abs(lineErrorX) > 0.3f || isSharpTurn)
                {                                                        // Off-center >30% or sharp turn, belok
                    std::string dir = (steering > 0) ? "KANAN" : "KIRI"; // Steering positive = belok kanan
                    float turnAngle = std::abs(steering) * 30.0f;        // Proportional turn (max 30°)
                    turnAngle = std::min(turnAngle, 45.0f);              // Cap at 45°
                    std::string speed = isSharpTurn ? "PELAN" : "SEDANG";
                    return "MAJU " + speed + " + PUTAR " + dir + " " + std::to_string((int)turnAngle) + "° - Ikuti garis (angle:" + std::to_string((int)lineAngleDeg) + "°)";
                }
                else
                {
                    return "MAJU - Ikuti garis lurus"; // On center, maju straight
                }
            }
            else
            {
                // No line - fallback ke scan with alternasi
                // Cek stuck di putar (history >5 putar berturut)
                int consecutiveTurns = 0;
                for (auto rit = navigationHistory.rbegin(); rit != navigationHistory.rend(); ++rit)
                {
                    if (rit->find("PUTAR") != std::string::npos)
                        consecutiveTurns++;
                    else
                        break;
                }
                if (consecutiveTurns > 5)
                {
                    // Stuck - force maju jika safe
                    if (!hasYellowAhead && !hasObstacleAhead)
                    {
                        return "MAJU 20 cm - Break scan loop";
                    }
                    else if (hasYellowAhead)
                    {
                        return "MUNDUR 10 cm - Avoid yellow";
                    }
                    else
                    {
                        // Obstacle - putar opposite
                        lastTurnLeft = !lastTurnLeft;
                        std::string turnStr = lastTurnLeft ? "PUTAR KIRI 30°" : "PUTAR KANAN 30°";
                        return turnStr + " - Avoid obstacle";
                    }
                }
                // Normal scan: Alternasi putar kiri/kanan untuk variasi
                // Jika yellow ahead, mundur dulu sebelum putar
                if (hasYellowAhead)
                {
                    return "MUNDUR 15 cm + PUTAR " + evadeDir + " 30° - Evade yellow boundary";
                }
                std::string turnDir = lastTurnLeft ? "KIRI" : "KANAN";
                lastTurnLeft = !lastTurnLeft; // Alternasi next time
                return "PUTAR " + turnDir + " 20° - Scan lingkungan";
            }
        }

        if (targetMarkerId == -1 ||
            detectedMarkers.find(targetMarkerId) == detectedMarkers.end() ||
            detectedMarkers[targetMarkerId].isScanned)
        {
            targetMarkerId = selectTargetMarker();
            if (targetMarkerId == -1)
            {
                currentState = FOLLOW_LINE; // NEW: Fallback ke follow line
                // Integrasi avoidance
                if (hasYellowAhead)
                {
                    return "MUNDUR 20 cm + PUTAR " + evadeDir + " 45° - Avoid yellow boundary";
                }
                else if (hasObstacleAhead)
                {
                    return "PUTAR KIRI 30° - Bypass obstacle";
                }
                else if (lineConfidence > 0.5)
                {
                    // Follow line dengan angle awareness
                    float steering = lineErrorX + 0.1f * lineAngleDeg;
                    bool isSharpTurn = std::abs(lineAngleDeg) > 20.0f;

                    if (std::abs(lineErrorX) > 0.3f || isSharpTurn)
                    {
                        std::string dir = (steering > 0) ? "KANAN" : "KIRI";
                        float turnAngle = std::abs(steering) * 30.0f;
                        turnAngle = std::min(turnAngle, 45.0f);
                        std::string speed = isSharpTurn ? "PELAN" : "SEDANG";
                        return "MAJU " + speed + " + PUTAR " + dir + " " + std::to_string((int)turnAngle) + "° - Ikuti garis (angle:" + std::to_string((int)lineAngleDeg) + "°)";
                    }
                    else
                    {
                        return "MAJU - Ikuti garis lurus";
                    }
                }
                else
                {
                    return "MAJU 15 cm - Explore new area";
                }
            }
        }

        const MarkerInfo &target = detectedMarkers[targetMarkerId];
        float offsetX = target.center.x - frameCenter.x;
        float offsetY = target.center.y - frameCenter.y;
        float distFromCenter = std::sqrt(offsetX * offsetX + offsetY * offsetY);

        std::string instruction;
        bool isCentered = distFromCenter < centerThreshold;
        bool isCloseEnough = target.size > minMarkerSize;
        bool isFar = target.size < farMarkerSize;
        bool isModeratelyCentered = distFromCenter < (centerThreshold * 1.5f);

        // NEW: Cerdas scan jika confidence tinggi (>0.8) & centered, meski size belum minMarkerSize
        if (target.confidence > 0.8 && isCentered)
        {
            currentState = LOCKED;
            scannedMarkers[targetMarkerId] = true;
            totalMarkersScanned++;
            return "LOCKED! - Scan otomatis (high confidence)";
        }
        // NEW: Cerdas dekati jika confidence cukup tinggi (>0.5), prioritaskan approach
        else if (target.confidence > 0.5)
        {
            currentState = APPROACHING;
            // Avoidance sebelum dekati
            if (hasYellowAhead)
            {
                return "MUNDUR 10 cm + PUTAR " + evadeDir + " 45° - Avoid yellow while approaching";
            }
            else if (hasObstacleAhead)
            {
                return "PUTAR KANAN 30° - Bypass obstacle to target";
            }
            std::string majuInstr = "MAJU 100 cm";
            if (!isModeratelyCentered)
            {
                if (offsetX > centerThreshold * 0.5f)
                    majuInstr += " + Sedikit KANAN";
                else if (offsetX < -centerThreshold * 0.5f)
                    majuInstr += " + Sedikit KIRI";
            }
            return majuInstr;
        }
        // Existing checks (fallback jika confidence rendah)
        else if (isCentered && isCloseEnough)
        {
            currentState = LOCKED;
            scannedMarkers[targetMarkerId] = true;
            totalMarkersScanned++;
            return "LOCKED! - Scan otomatis";
        }
        else if (isFar)
        {
            currentState = APPROACHING;
            // Check avoidance sebelum maju
            if (hasYellowAhead)
            {
                return "MUNDUR 10 cm + PUTAR " + evadeDir + " 45° - Avoid yellow while approaching";
            }
            else if (hasObstacleAhead)
            {
                return "PUTAR KANAN 30° - Bypass obstacle to target";
            }
            std::string majuInstr = "MAJU 100 cm";
            if (!isModeratelyCentered)
            {
                if (offsetX > centerThreshold * 0.5f)
                    majuInstr += " + Sedikit KANAN";
                else if (offsetX < -centerThreshold * 0.5f)
                    majuInstr += " + Sedikit KIRI";
            }
            return majuInstr;
        }
        else if (target.size < minMarkerSize)
        {
            currentState = APPROACHING;
            // Avoidance check
            if (hasYellowAhead)
            {
                return "MUNDUR 15 cm + PUTAR " + evadeDir + " 30° - Evade yellow to target";
            }
            else if (hasObstacleAhead)
            {
                return "PUTAR KIRI 20° - Dodge obstacle";
            }
            if (isModeratelyCentered)
            {
                float distance = (minMarkerSize - target.size) * 0.8f;
                return "MAJU " + std::to_string((int)distance) + " cm";
            }
            else
            {
                std::string direction = (offsetX > 0) ? "KANAN" : "KIRI";
                float distance = (minMarkerSize - target.size) * 0.5f;
                return "MAJU " + std::to_string((int)distance) + " cm + PUTAR " + direction;
            }
        }
        else
        {
            currentState = TRACKING;
            // Avoidance selama tracking
            if (hasYellowAhead)
            {
                return "MUNDUR 20 cm + PUTAR " + evadeDir + " 45° - Evade yellow";
            }
            else if (hasObstacleAhead)
            {
                return "PUTAR KANAN 15° - Dodge obstacle";
            }
            if (std::abs(offsetX) > std::abs(offsetY))
            {
                if (offsetX > centerThreshold * 0.3f)
                {
                    float angle = std::abs(offsetX) / 8.0f;
                    instruction = "PUTAR KANAN " + std::to_string((int)angle) + "°";
                }
                else if (offsetX < -centerThreshold * 0.3f)
                {
                    float angle = std::abs(offsetX) / 8.0f;
                    instruction = "PUTAR KIRI " + std::to_string((int)angle) + "°";
                }
                else
                {
                    instruction = "MAJU 20 cm";
                }
            }
            else
            {
                instruction = "MAJU 15 cm";
            }
        }

        navigationHistory.push_back(instruction);
        if (navigationHistory.size() > maxHistorySize)
            navigationHistory.pop_front();
        return instruction;
    }

    RobotState getState() const { return currentState; }
    int getTargetId() const { return targetMarkerId; }
    int getTotalFound() const { return totalMarkersFound; }
    int getTotalScanned() const { return totalMarkersScanned; }
    const std::map<int, MarkerInfo> &getMarkers() const { return detectedMarkers; }
};

int main(int argc, char *argv[])
{
    std::cout << "========================================\n";
    std::cout << "AUTONOMOUS ROVER (ArUco -> Robot Bridge)\n";
    std::cout << "========================================\n\n";

    // Load config
    auto cfg = loadConfig({"../config/rover_config.yaml", "config/rover_config.yaml"});
    // Determine which config path is in use (first existing)
    std::string cfgPathUsed = "../config/rover_config.yaml";
    {
        std::ifstream tin(cfgPathUsed);
        if (!tin.is_open())
            cfgPathUsed = "config/rover_config.yaml";
    }
    std::string ipCameraURL = cfgStr(cfg, "camera.url", "http://10.182.43.123:8080/video");
    std::string serialPort = cfgStr(cfg, "robot.port", "/dev/ttyUSB0");
    int baud = cfgInt(cfg, "robot.baud", 115200);
    bool sendCharControls = (cfgStr(cfg, "robot.send_char_controls", "false") == "true");
    bool sendAsciiProtocol = (cfgStr(cfg, "robot.send_ascii_protocol", "true") == "true");

    int speedForward = cfgInt(cfg, "speed.forward", 150);
    int speedBackward = cfgInt(cfg, "speed.back", 150);
    int speedTurn = cfgInt(cfg, "speed.turn", 130);
    int msPerCm = cfgInt(cfg, "calibration.ms_per_cm", 80);
    int msPerDeg = cfgInt(cfg, "calibration.ms_per_deg", 12);
    std::string protocolMode = cfgStr(cfg, "protocol.mode", "standard");
    bool debugPanel = (cfgStr(cfg, "debug.enable_panel", "true") == "true");
    std::string debugCsvPath = cfgStr(cfg, "debug.csv_path", "decisions_log.csv");
    bool debugShowYellowMask = (cfgStr(cfg, "debug.show_yellow_mask", "false") == "true");
    bool debugVerboseDistance = (cfgStr(cfg, "debug.verbose_distance", "false") == "true");
    bool debugShowRoutineFeed = (cfgStr(cfg, "debug.show_routine_feed", "true") == "true");
    bool debugShowRoutinePopup = (cfgStr(cfg, "debug.show_routine_popup", "true") == "true");
    gShowYellowMaskDebug = debugShowYellowMask;
    gVerboseDistanceLog = debugVerboseDistance;
    gShowRoutinePreview = debugShowRoutineFeed;
    gShowRoutinePopup = debugShowRoutinePopup;
    // Yellow thresholds and UI sizes
    bool yellowDetectionEnabled = (cfgStr(cfg, "yellow.enable", "true") == "true");
    std::cout << "✓ Yellow line detection " << (yellowDetectionEnabled ? "enabled" : "disabled") << "\n";
    int yHLow = cfgInt(cfg, "yellow.h_low", 15);
    int yHHigh = cfgInt(cfg, "yellow.h_high", 42);
    int ySLow = cfgInt(cfg, "yellow.s_low", 80);
    int yVLow = cfgInt(cfg, "yellow.v_low", 100);
    double yMinArea = 0.005;
    try
    {
        yMinArea = std::stod(cfgStr(cfg, "yellow.min_area_ratio", "0.005"));
    }
    catch (...)
    {
    }
    int yLabBLow = cfgInt(cfg, "yellow.lab_b_low", 140);
    int yALow = cfgInt(cfg, "yellow.lab_a_low", 120); // OpenCV Lab offset domain [0..255]
    int yAHigh = cfgInt(cfg, "yellow.lab_a_high", 170);
    bool yCombineOr = (cfgStr(cfg, "yellow.combine_or", "true") == "true");
    int yBandTopRatioPct = cfgInt(cfg, "yellow.band_top_ratio", 65); // bottom 50% default
    yBandTopRatioPct = std::max(0, std::min(95, yBandTopRatioPct));
    double yRelativeMinY = 0.50;
    bool announcedFirstDetection = false;
    bool stopHintAnnounced = false;
    bool yellowVoiceActive = false;
    try
    {
        yRelativeMinY = std::stod(cfgStr(cfg, "yellow.relative_min_y", "0.40"));
    }
    catch (...)
    {
    }
    yRelativeMinY = std::max(0.0, std::min(0.95, yRelativeMinY));
    int pxPerCm = cfgInt(cfg, "yellow.px_per_cm", 0);
    int camW = cfgInt(cfg, "ui.camera_w", 960);
    int camH = cfgInt(cfg, "ui.camera_h", 540);
    bool showMap = (cfgStr(cfg, "ui.show_map", "false") == "true");
    int yellowDetectionInterval = cfgInt(cfg, "yellow.detection_interval", 3);
    bool lowPowerMode = (cfgStr(cfg, "performance.low_power_mode", "false") == "true");
    int lowPowerWidth = cfgInt(cfg, "performance.low_power_width", 480);
    int lowPowerHeight = cfgInt(cfg, "performance.low_power_height", 360);
    int arucoDetectionInterval = std::max(1, cfgInt(cfg, "aruco.detection_interval", 1));
    if (lowPowerMode)
    {
        std::cout << "✓ Low-power mode aktif: target frame <= "
                  << lowPowerWidth << "x" << lowPowerHeight << "\n";
    }
    int panelW = cfgInt(cfg, "ui.panel_w", 520);
    int panelH = cfgInt(cfg, "ui.panel_h", 420);
    double arucoMarkerLengthM = 0.0; // optional pose size
    try
    {
        arucoMarkerLengthM = std::stod(cfgStr(cfg, "aruco.marker_length_m", "0.0"));
    }
    catch (...)
    {
    }
    bool yellowTunerEnabled = (cfgStr(cfg, "debug.enable_yellow_tuner", "false") == "true");

    // Camera mounting parameters for distance estimation
    double cameraHeightCm = 11.0; // CHANGED: Default ke 11 cm sesuai spec Anda
    double cameraFovVertical = 50.0;
    double yellowSafetyDistanceM = 0.01;  // CHANGED: Default 1cm untuk peringatan SANGAT close proximity
    double cameraCalibrationFactor = 1.0; // NEW: Factor untuk fine-tune estimasi jarak
    try
    {
        cameraHeightCm = std::stod(cfgStr(cfg, "camera.height_cm", "11.0")); // Prioritaskan config, default 11.0
        cameraFovVertical = std::stod(cfgStr(cfg, "camera.fov_vertical", "50.0"));
        yellowSafetyDistanceM = std::stod(cfgStr(cfg, "yellow.safety_distance_m", "0.01")); // 1cm default
        cameraCalibrationFactor = std::stod(cfgStr(cfg, "camera.calibration_factor", "1.0"));
    }
    catch (...)
    {
    }

    double cameraTiltDeg = 90.0;
    try
    {
        cameraTiltDeg = std::stod(cfgStr(cfg, "camera.tilt_angle", "90.0"));
    }
    catch (...)
    {
    }
    std::cout << "Camera tilt (from horizontal): " << cameraTiltDeg << "°\n";

    std::cout << "Camera parameters: height=" << cameraHeightCm << "cm, FOV=" << cameraFovVertical << "°\n";
    std::cout << "Yellow safety distance: " << yellowSafetyDistanceM << " meters\n";
    std::cout << "Calibration factor: " << cameraCalibrationFactor << " (adjust if distance not matching real)\n";

    // Camera override from argv, if present
    if (argc > 1)
        ipCameraURL = argv[1];

    // Setup camera
    cv::VideoCapture cam;
    std::cout << "Connecting to camera: " << ipCameraURL << "\n";
    cam.open(ipCameraURL, cv::CAP_FFMPEG);
    if (!cam.isOpened())
    {
        std::cout << "ERROR: Cannot open camera!\n";
        return -1;
    }
    // Reduce buffering latency if backend supports it
    try
    {
        cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
    }
    catch (...)
    {
    }

    // Performance optimization: Reduce camera resolution if needed
    int cameraWidth = cfgInt(cfg, "camera.width", 0); // 0 = use default
    int cameraHeight = cfgInt(cfg, "camera.height", 0);
    if (cameraWidth > 0 && cameraHeight > 0)
    {
        cam.set(cv::CAP_PROP_FRAME_WIDTH, cameraWidth);
        cam.set(cv::CAP_PROP_FRAME_HEIGHT, cameraHeight);
        std::cout << "✓ Camera resolution set to " << cameraWidth << "x" << cameraHeight << "\n";
    }

    // Set FPS limit to reduce processing load
    int cameraFPS = cfgInt(cfg, "camera.fps", 0);
    if (cameraFPS > 0)
    {
        cam.set(cv::CAP_PROP_FPS, cameraFPS);
        std::cout << "✓ Camera FPS limited to " << cameraFPS << "\n";
    }

    std::cout << "✓ Camera connected.\n";

    // Setup ArUco - DICT_5X5_1000 supports ID 0-999 (5x5 markers)
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

    // Tune detector parameters for long-range detection (2+ meters)
    detectorParams->adaptiveThreshWinSizeMin = 3;
    detectorParams->adaptiveThreshWinSizeMax = 23; // Increased for far detection
    detectorParams->adaptiveThreshWinSizeStep = 10;
    detectorParams->minMarkerPerimeterRate = 0.05; // CHANGED: Tingkatkan dari 0.01 ke 0.05 untuk ignore marker kecil/noisy (reduce false positive)
    detectorParams->maxMarkerPerimeterRate = 4.0;
    detectorParams->polygonalApproxAccuracyRate = 0.03;
    detectorParams->minCornerDistanceRate = 0.01;
    detectorParams->minDistanceToBorder = 1;                                  // Lowered to detect markers near edges
    detectorParams->errorCorrectionRate = 0.4;                                // CHANGED: Kurangi dari default 0.6 ke 0.4 untuk reduce toleransi error (hindari false positive)
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // Accurate corner detection
    detectorParams->cornerRefinementWinSize = 5;
    detectorParams->cornerRefinementMaxIterations = 30;
    detectorParams->cornerRefinementMinAccuracy = 0.01;

    std::cout << "✓ ArUco DICT_5X5_1000 configured (supports ID 0-999)\n";
    std::cout << "✓ Detector tuned for long-range detection (2+ meters) with reduced false positives\n";

    // Navigator
    AutonomousNavigator navigator;

    // Obstacle Detector Configuration
    ObstacleDetectorConfig obstacleConfig;
    obstacleConfig.cameraHeightCm = cameraHeightCm;
    obstacleConfig.cameraFovVertical = cameraFovVertical;
    obstacleConfig.cameraTiltAngle = 90.0; // Assume tegak down
    obstacleConfig.emergencyDistanceCm = cfgInt(cfg, "obstacle.emergency_distance_cm", 20);
    obstacleConfig.warningDistanceCm = cfgInt(cfg, "obstacle.warning_distance_cm", 40);
    obstacleConfig.safeDistanceCm = cfgInt(cfg, "obstacle.safe_distance_cm", 60);
    obstacleConfig.enableMotionDetection = (cfgStr(cfg, "obstacle.enable_motion", "false") == "true");

    ObstacleDetector obstacleDetector(obstacleConfig);
    bool obstacleDetectionEnabled = (cfgStr(cfg, "obstacle.enable", "true") == "true");
    std::cout << "✓ Obstacle detection " << (obstacleDetectionEnabled ? "enabled" : "disabled") << "\n";

// YOLOv5 Detector (if available)
#ifdef USE_YOLO_DETECTOR
    bool useYolo = (cfgStr(cfg, "yolo.enable", "true") == "true");
    YoloConfig yoloConfig;
    yoloConfig.modelPath = cfgStr(cfg, "yolo.model_path", "models/yolov5s.onnx");
    yoloConfig.confidenceThreshold = 0.5f;
    try
    {
        yoloConfig.confidenceThreshold = std::stof(cfgStr(cfg, "yolo.confidence", "0.5"));
    }
    catch (...)
    {
    }
    yoloConfig.useCUDA = (cfgStr(cfg, "yolo.use_cuda", "false") == "true");

    std::unique_ptr<YoloDetector> yoloDetector;
    if (useYolo)
    {
        yoloDetector = std::make_unique<YoloDetector>(yoloConfig);
        if (yoloDetector->initialize())
        {
            std::cout << "✓ YOLOv5 detector initialized successfully" << std::endl;
            std::cout << "  Model: " << yoloConfig.modelPath << std::endl;
            std::cout << "  Confidence threshold: " << yoloConfig.confidenceThreshold << std::endl;
        }
        else
        {
            std::cerr << "✗ Failed to initialize YOLOv5 detector" << std::endl;
            yoloDetector.reset();
            useYolo = false;
        }
    }
#else
    bool useYolo = false;
    std::cout << "⚠ YOLOv5 not available (ONNX Runtime not linked)" << std::endl;
#endif

    // Path Planning Configuration
    double gridResolutionCm = 5.0;
    try
    {
        gridResolutionCm = std::stod(cfgStr(cfg, "path.grid_resolution_cm", "5.0"));
    }
    catch (...)
    {
    }
    std::string explorationPattern = cfgStr(cfg, "path.exploration_pattern", "spiral");
    bool replanOnObstacle = (cfgStr(cfg, "path.replan_on_obstacle", "true") == "true");

    // Initialize Grid Map (3x3 meters = 300x300 cm)
    GridMap gridMap(300.0, 300.0, gridResolutionCm);
    AStarPlanner astarPlanner(&gridMap);
    SpiralExplorer spiralExplorer(&gridMap);

    // Generate initial spiral exploration waypoints (from origin, 150cm radius)
    std::vector<Waypoint> explorationWaypoints = spiralExplorer.generateSpiralWaypoints(0.0, 0.0, 150.0, 30);
    int currentWaypointIndex = 0;
    std::cout << "✓ Path planner initialized with " << explorationWaypoints.size() << " waypoints\n";
    std::cout << "✓ Exploration pattern: " << explorationPattern << "\n";

    // Visited memory
    const std::string visitedTxtPath = "data/visited_markers.txt";
    const std::string visitedJsonPath = "visited_markers.json"; // legacy
    // ensure directories exist
    try
    {
        std::filesystem::create_directories("data");
    }
    catch (...)
    {
    }
    try
    {
        std::filesystem::create_directories("logs");
    }
    catch (...)
    {
    }
    std::set<int> visited = loadVisitedTxt(visitedTxtPath);
    if (visited.empty())
    {
        // fallback to legacy json if exists
        visited = loadVisited(visitedJsonPath);
    }
    navigator.preloadVisited(visited);
    std::cout << "Visited markers loaded: " << visited.size() << "\n";

    // Detailed scanned markers database
    const std::string scannedDbPath = "scanned_markers_database.csv";
    std::map<int, ScannedMarkerDetail> scannedDb = loadScannedDatabase(scannedDbPath);
    std::cout << "Scanned markers database loaded: " << scannedDb.size() << " entries\n";

    // Optional camera intrinsics for pose estimation
    cv::Mat cameraMatrix, distCoeffs;
    bool hasIntrinsics = false;
    {
        cv::FileStorage fs1("config/camera_intrinsics.yml", cv::FileStorage::READ);
        cv::FileStorage fs2("../config/camera_intrinsics.yml", cv::FileStorage::READ);
        if (fs1.isOpened() || fs2.isOpened())
        {
            cv::FileStorage &fs = fs1.isOpened() ? fs1 : fs2;
            try
            {
                fs["camera_matrix"] >> cameraMatrix;
                fs["distortion_coefficients"] >> distCoeffs;
                hasIntrinsics = !cameraMatrix.empty() && !distCoeffs.empty();
                if (hasIntrinsics)
                    std::cout << "✓ Camera intrinsics loaded for pose estimation.\n";
            }
            catch (...)
            {
            }
        }
    }

    // Path memory & grid map
    Pose currentPose;
    std::vector<Pose> pathTrace;
    cv::Mat occupancyGrid(2000, 2000, CV_8UC1, cv::Scalar(0)); // Larger grid for 1px=1cm
    const double cmPerPixel = 1.0;                             // 1 pixel = 1 cm (higher resolution)
    const std::string pathCsv = "logs/path_trace.csv";
    int gridSaveCounter = 0;
    // breadcrumb topologis: node -> set of tried directions ("L"/"R")
    std::map<std::string, std::set<std::string>> nodeTried;

    // ArUco Waypoints: store detected marker positions in world frame
    std::map<int, ArUcoWaypoint> waypoints;

    // Serial transport
    SerialTransport serial;
    if (!serial.open(serialPort, baud))
    {
        std::cout << "WARN: Serial open failed for " << serialPort << " (baud " << baud << "). Continue without robot.\n";
    }
    else
    {
        std::cout << "✓ Serial connected: " << serialPort << " @" << baud << "\n";
        // optional: push calibration
        if (sendAsciiProtocol)
        {
            serial.sendLine(rovercmd::buildCalSetMsPerCm(msPerCm));
            serial.sendLine(rovercmd::buildCalSetMsPerDeg(msPerDeg));
        }
    }

    // Bridge
    RoverConfig rcfg;
    rcfg.speedForward = speedForward;
    rcfg.speedBackward = speedBackward;
    rcfg.speedTurn = speedTurn;
    rcfg.msPerCm = msPerCm;
    rcfg.msPerDeg = msPerDeg;
    rcfg.useStm32Protocol = (protocolMode == "stm32");
    RobotBridge bridge(rcfg);
    // Throttled single-char sender (HC-05) based on high-level instruction
    uint64_t lastCharSentMs = 0;
    char lastCharSent = 0;
    std::vector<char> toggleControlChars = {'w', 's', 'a', 'd', 'q', 'e', 'g', 'p'};
    std::unordered_map<char, bool> toggleState;
    for (char c : toggleControlChars)
        toggleState[c] = false;

    auto toggleCharSend = [&](char c)
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        serial.sendByte(c);
        toggleState[c] = !toggleState[c];
        lastCharSent = c;
        lastCharSentMs = (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
        std::cout << "[BT] toggle '" << c << "' -> " << (toggleState[c] ? "ON" : "OFF") << "\n";
    };

    auto setCharState = [&](char c, bool desired)
    {
        bool current = toggleState.count(c) ? toggleState[c] : false;
        if (current == desired)
            return;
        toggleCharSend(c);
    };

    auto stopAllChars = [&]()
    {
        for (char c : toggleControlChars)
        {
            if (toggleState[c])
                toggleCharSend(c);
        }
    };
    auto sendCharForInstruction = [&](const std::string &instr)
    {
        static bool warnedNotOpen = false;
        if (!sendCharControls)
            return;
        if (!serial.isOpen())
        {
            if (!warnedNotOpen)
            {
                std::cout << "WARN: BT char send enabled but serial NOT open (" << serialPort << ").\n";
                warnedNotOpen = true;
            }
            return;
        }
        std::string u = instr;
        std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c)
                       { return std::toupper(c); });
        char out = 0;
        // STOP/LOCKED/SCAN COMPLETE -> toggle off all
        bool isStopInstruction = false;
        if (u.find("STOP") != std::string::npos || u.find("LOCKED") != std::string::npos || u.find("SCAN COMPLETE") != std::string::npos)
        {
            isStopInstruction = true;
        }
        // Move forward/back
        else if (u.find("MAJU") != std::string::npos)
            out = 'w';
        else if (u.find("MUNDUR") != std::string::npos)
            out = 's';
        // Turns
        else if (u.find("PUTAR KIRI") != std::string::npos)
            out = 'q';
        else if (u.find("PUTAR KANAN") != std::string::npos)
            out = 'e';
        // Nudge/steer small corrections
        else if (u.find("SEDIKIT KIRI") != std::string::npos)
            out = 'a';
        else if (u.find("SEDIKIT KANAN") != std::string::npos)
            out = 'd';
        // Gripper
        else if (u.find("GRIP CLOSE") != std::string::npos || u.find("PICKUP") != std::string::npos)
            out = 'g';
        else if (u.find("GRIP OPEN") != std::string::npos || u.find("PUT ") != std::string::npos)
            out = 'p';

        if (isStopInstruction)
        {
            stopAllChars();
            return;
        }

        if (out == 0)
            return;

        for (char c : toggleControlChars)
        {
            if (c != out && toggleState[c])
                toggleCharSend(c);
        }

        setCharState(out, true);
    };

    cv::Mat frame;
    std::vector<int> cachedMarkerIds;
    std::vector<std::vector<cv::Point2f>> cachedMarkerCorners;
    std::vector<std::vector<cv::Point2f>> cachedRejectedCandidates;
    bool haveCachedMarkers = false;
    std::string lastInstruction;
    uint64_t lastSentMs = 0;
    auto nowMs = []() -> uint64_t
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
    };

    // Performance optimization: Frame skipping for YOLO
    int yoloDetectionInterval = cfgInt(cfg, "yolo.detection_interval", 3); // Detect every N frames
    int frameCounter = 0;
    std::vector<DetectedObstacle> lastYoloObstacles; // Cache last detection

    // Exploration control (to avoid endless "scan lingkungan")
    int explorationScanCooldownMs = cfgInt(cfg, "exploration.scan_cooldown_ms", 2000);
    int explorationMarkerGraceMs = cfgInt(cfg, "exploration.marker_grace_ms", 1500);
    uint64_t lastMarkerSeenMs = nowMs();
    uint64_t lastScanActionMs = 0;

    // Decision history + CSV setup
    std::deque<DecisionEntry> history;
    const size_t maxHistory = 12;
    std::ofstream csv(debugCsvPath, std::ios::app);
    if (csv.is_open())
    {
        csv << "timestamp,instruction,num_markers,target_id,yellow_ratio\n";
    }
    if (debugPanel)
    {
        cv::namedWindow("Decision Panel", cv::WINDOW_NORMAL);
        cv::resizeWindow("Decision Panel", panelW, panelH);
    }
    cv::namedWindow("Rover Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Rover Camera", camW, camH);
    // Map window (3x3 meters view)
    if (showMap)
    {
        cv::namedWindow("Map 3x3m", cv::WINDOW_NORMAL);
        cv::resizeWindow("Map 3x3m", 800, 800);
    }

    bool espeakSessionActive = false;
    {
        int sampleRate = espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 500, nullptr, 0);
        if (sampleRate >= 0)
        {
            espeak_SetVoiceByName("id");
            gEspeakReady = true;
            espeakSessionActive = true;
            std::cout << "✓ eSpeak TTS aktif (voice: id)\n";
        }
        else
        {
            std::cout << "⚠ eSpeak TTS gagal diinisialisasi. Sistem akan mencoba fallback perintah 'espeak'.\n";
        }
    }

    // Optional: Yellow Tuner UI
    int trk_hLow = yHLow, trk_hHigh = yHHigh, trk_sLow = ySLow, trk_vLow = yVLow;
    int trk_labBLow = yLabBLow, trk_labALow = yALow, trk_labAHigh = yAHigh;
    int trk_combine = yCombineOr ? 1 : 0;
    int trk_minAreaX1e4 = (int)std::round(yMinArea * 10000.0);
    if (yellowTunerEnabled)
    {
        cv::namedWindow("Yellow Tuner", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H Low", "Yellow Tuner", &trk_hLow, 179);
        cv::createTrackbar("H High", "Yellow Tuner", &trk_hHigh, 179);
        cv::createTrackbar("S Low", "Yellow Tuner", &trk_sLow, 255);
        cv::createTrackbar("V Low", "Yellow Tuner", &trk_vLow, 255);
        cv::createTrackbar("Lab a Low", "Yellow Tuner", &trk_labALow, 255);
        cv::createTrackbar("Lab a High", "Yellow Tuner", &trk_labAHigh, 255);
        cv::createTrackbar("Lab b Low", "Yellow Tuner", &trk_labBLow, 255);
        cv::createTrackbar("MinArea x1e4", "Yellow Tuner", &trk_minAreaX1e4, 200);
        cv::createTrackbar("Combine (0=AND,1=OR)", "Yellow Tuner", &trk_combine, 1);
    }

    // ==================== High-level Behavior FSM (INIT_SCAN, FOLLOW_LINE, APPROACH_ARUCO, SEARCH, AVOID) ====================
    enum class BehaviorState
    {
        INIT_SCAN,      // Rotate to recognize environment (one-time 360°)
        FOLLOW_LINE,    // Follow white line while opportunistically scanning ArUco
        APPROACH_ARUCO, // Approach detected ArUco marker
        SEARCH,         // Search when line lost
        AVOID           // Avoid hazards (yellow boundary / obstacle)
    };

    BehaviorState behaviorState = BehaviorState::INIT_SCAN;
    int initScanAccumulatedDeg = 0;
    const int INIT_SCAN_STEP_DEG = 30; // turn step per instruction during initial scan
    bool initScanTurnLeft = true;

    auto behaviorStateSpeechText = [](BehaviorState s) -> std::string
    {
        switch (s)
        {
        case BehaviorState::INIT_SCAN:
            return "Mulai pemindaian awal";
        case BehaviorState::FOLLOW_LINE:
            return "Mulai mengikuti garis putih";
        case BehaviorState::APPROACH_ARUCO:
            return "Mendekati marker ArUco";
        case BehaviorState::SEARCH:
            return "Mode pencarian garis";
        case BehaviorState::AVOID:
            return "Menghindari bahaya";
        default:
            return "";
        }
    };
    BehaviorState lastSpokenBehaviorState = behaviorState;

    int noLineFrames = 0;
    const int NO_LINE_FRAMES_THRESHOLD = 10; // frames before entering SEARCH

    // Optional voice guidance toggle
    bool ttsEnabledChoice = true;
    std::cout << "\nAktifkan suara deteksi? [Y/n]: ";
    std::string ttsChoiceInput;
    std::getline(std::cin, ttsChoiceInput);
    if (!ttsChoiceInput.empty())
    {
        char c = std::tolower(ttsChoiceInput[0]);
        if (c == 'n')
            ttsEnabledChoice = false;
        else if (c == 'y')
            ttsEnabledChoice = true;
    }
    setTtsEnabled(ttsEnabledChoice);
    if (ttsEnabledChoice)
    {
        speakText("Mode suara aktif. Sistem siap memberikan instruksi.");
    }
    else
    {
        std::cout << "→ Suara deteksi dimatikan.\n";
    }

    // ==================== MODE SELECTION (Terarah dulu vs Bebas) ====================
    {
        std::cout << "\n=== PILIH MODE ===\n";
        std::cout << "1) Autonomous Terarah (jalankan pola scan 4 arah dahulu)\n";
        std::cout << "2) Autonomous Bebas (langsung follow line + cari ArUco)\n";
        if (gTtsEnabled.load())
        {
            speakText("Pilih mode mana yang kamu mau? Mode satu untuk scan terarah, mode dua untuk langsung mengikuti garis.");
        }
        std::cout << "Masukkan pilihan [1/2]: ";
        std::string input;
        std::getline(std::cin, input);
        char sel = input.empty() ? '2' : input[0];
        if (sel == '1')
        {
            std::cout << "→ Menjalankan Autonomous Terarah (scan 4 arah)...\n";
            if (gTtsEnabled.load())
                speakText("Mode satu dipilih. Menjalankan scan terarah.");
            bool found = runDirectedArucoRoutine(cam, serial, bridge, sendAsciiProtocol,
                                                 sendCharForInstruction, dictionary, detectorParams,
                                                 msPerCm, msPerDeg,
                                                 lowPowerMode, lowPowerWidth, lowPowerHeight,
                                                 arucoDetectionInterval, gShowRoutinePreview);
            if (found)
            {
                // Stop total program jika aruco terdeteksi saat rutin
                std::cout << "✓ ArUco terdeteksi saat rutinitas terarah. Program berhenti.\n";
                // Kirim STOP untuk jaga-jaga
                sendInstructionOnce(serial, bridge, "STOP", sendAsciiProtocol);
                return 0;
            }
            std::cout << "✗ Tidak ada ArUco terdeteksi pada rutinitas awal. Lanjut autonomous bebas.\n";
            behaviorState = BehaviorState::FOLLOW_LINE; // langsung follow line tanpa INIT_SCAN
        }
        else
        {
            std::cout << "→ Mode Autonomous Bebas dipilih.\n";
            if (gTtsEnabled.load())
                speakText("Mode dua dipilih. Menjalankan rover bebas mengikuti garis.");
        }
    }

    while (true)
    {
        cam >> frame;
        if (frame.empty())
        {
            std::cout << "ERROR: Empty frame\n";
            break;
        }

        if (lowPowerMode && lowPowerWidth > 0 && lowPowerHeight > 0)
        {
            if (frame.cols > lowPowerWidth || frame.rows > lowPowerHeight)
            {
                double scale = std::min(static_cast<double>(lowPowerWidth) / frame.cols,
                                        static_cast<double>(lowPowerHeight) / frame.rows);
                if (scale > 0.0 && scale < 1.0)
                {
                    cv::resize(frame, frame, cv::Size(), scale, scale, cv::INTER_AREA);
                }
            }
        }

        frameCounter++;

        navigator.setFrameSize(frame.cols, frame.rows);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        bool runArucoDetection = (arucoDetectionInterval <= 1) ||
                                 ((frameCounter % arucoDetectionInterval) == 0) ||
                                 !haveCachedMarkers;
        if (runArucoDetection)
        {
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
            cachedMarkerIds = markerIds;
            cachedMarkerCorners = markerCorners;
            cachedRejectedCandidates = rejectedCandidates;
            haveCachedMarkers = true;
        }
        else
        {
            markerIds = cachedMarkerIds;
            markerCorners = cachedMarkerCorners;
            rejectedCandidates = cachedRejectedCandidates;
        }
        if (!markerIds.empty())
        {
            lastMarkerSeenMs = nowMs();
        }

        // Optional pose estimation (visual only; no correction applied)
        if (hasIntrinsics && arucoMarkerLengthM > 0.0 && !markerIds.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            try
            {
                cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoMarkerLengthM, cameraMatrix, distCoeffs, rvecs, tvecs);
                for (size_t i = 0; i < rvecs.size(); ++i)
                {
                    cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], arucoMarkerLengthM * 0.5);
                }
            }
            catch (...)
            {
            }
        }

        // Immediately mark seen markers as visited (option 1b)
        // AND store as waypoints for map display
        bool newVisitedAdded = false;
        for (size_t i = 0; i < markerIds.size(); ++i)
        {
            int id = markerIds[i];

            // Mark as visited
            if (visited.find(id) == visited.end())
            {
                visited.insert(id);
                newVisitedAdded = true;
            }

            // Store as waypoint (first time only)
            if (waypoints.find(id) == waypoints.end())
            {
                // Calculate marker center in frame
                cv::Point2f center(0, 0);
                for (int j = 0; j < 4; j++)
                {
                    center.x += markerCorners[i][j].x;
                    center.y += markerCorners[i][j].y;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                // Estimate rough position relative to robot (simple front-facing assumption)
                // For better accuracy, use pose estimation with camera intrinsics
                float size_px = 0;
                {
                    float dx = markerCorners[i][0].x - markerCorners[i][2].x;
                    float dy = markerCorners[i][0].y - markerCorners[i][2].y;
                    size_px = std::sqrt(dx * dx + dy * dy);
                }
                // Rough distance estimate (inverse proportional to size)
                double distCm = std::max(50.0, 1000.0 / (size_px + 1.0));

                // Rough angle from center of frame
                double angleOffsetDeg = (center.x - frame.cols / 2.0) * 0.1; // Very rough estimate
                double markerAngle = currentPose.thetaDeg + angleOffsetDeg;
                double markerAngleRad = markerAngle * M_PI / 180.0;

                // World position (rough estimate)
                ArUcoWaypoint wp;
                wp.id = id;
                wp.x = currentPose.xCm + distCm * std::cos(markerAngleRad);
                wp.y = currentPose.yCm + distCm * std::sin(markerAngleRad);
                wp.visited = true; // Already visited since we mark on first sight

                waypoints[id] = wp;

                std::cout << "📍 Waypoint added: Marker ID " << id
                          << " at (" << (int)wp.x << ", " << (int)wp.y << ") cm\n";
            }
        }
        if (newVisitedAdded)
        {
            saveVisitedTxt(visitedTxtPath, visited);
            // keep legacy json in sync (optional)
            saveVisited(visitedJsonPath, visited);
            navigator.preloadVisited(visited);
        }

        // Update internal marker tracking AFTER visited preload to reflect isScanned immediately
        navigator.updateMarkers(markerIds, markerCorners);

        // NEW: Track detected IDs and show popup saat ArUco PERTAMA KALI terdeteksi
        // Static set untuk track IDs yang sudah pernah tampil popup (hindari spam)
        static std::set<int> poppedIds;

        if (!markerIds.empty())
        {
            // Track detected IDs in this frame
            std::set<int> currentDetectedIds;
            for (const auto &id : markerIds)
            {
                currentDetectedIds.insert(id);
            }

            // Untuk setiap ID baru terdeteksi (pertama kali)
            for (int id : currentDetectedIds)
            {
                if (poppedIds.find(id) == poppedIds.end())
                {
                    // Cari detail dari markers
                    const auto &markers = navigator.getMarkers();
                    auto it = markers.find(id);
                    if (it != markers.end())
                    {
                        const MarkerInfo &info = it->second;

                        // Buat detail untuk popup
                        ScannedMarkerDetail detail;
                        detail.id = id;
                        detail.timestamp = nowString();
                        detail.distance = info.distance;
                        detail.markerSize = info.size;
                        detail.position = info.center;
                        detail.scanCount = (scannedDb.find(id) != scannedDb.end()) ? scannedDb[id].scanCount : 0;

                        std::cout << "\n🎯 NEW ArUco DETECTED! ID: " << id
                                  << " | Distance: ~" << (int)info.distance << " cm" << std::endl;

                        // Tampilkan popup DETECTED (5 detik)
                        showDetectionPopup(id, detail);

                        if (!announcedFirstDetection)
                        {
                            speakText("Saya melihat ArUco. Jika target sudah terpenuhi, Anda boleh menghentikan program.");
                            announcedFirstDetection = true;
                        }

                        speakText("ArUco dengan id " + std::to_string(id) + " berhasil discan");

                        poppedIds.insert(id); // Mark agar tak popup lagi untuk ID ini
                        break;                // Handle one at a time untuk tidak spam
                    }
                }
            }
        }

        // Proximity-based scan: if any marker is very close, consider it scanned and show popup
        int scanDistanceCmCfg = cfgInt(cfg, "aruco.scan_distance_cm", 35);
        if (!markerIds.empty())
        {
            const auto &mm = navigator.getMarkers();
            for (const auto &kv : mm)
            {
                int mid = kv.first;
                const MarkerInfo &mi = kv.second;
                if (mi.distance > 0 && mi.distance < scanDistanceCmCfg && visited.find(mid) == visited.end())
                {
                    visited.insert(mid);
                    saveVisitedTxt(visitedTxtPath, visited);
                    saveVisited(visitedJsonPath, visited);
                    navigator.preloadVisited(visited);

                    // Record detailed scan
                    ScannedMarkerDetail detail;
                    detail.id = mid;
                    detail.timestamp = nowString();
                    detail.distance = mi.distance;
                    detail.markerSize = mi.size;
                    detail.position = mi.center;

                    // Update scan count
                    if (scannedDb.find(mid) != scannedDb.end())
                    {
                        detail.scanCount = scannedDb[mid].scanCount + 1;
                    }
                    else
                    {
                        detail.scanCount = 1;
                    }
                    scannedDb[mid] = detail;
                    saveScannedDatabase(scannedDbPath, scannedDb);

                    std::cout << "=== ArUco SCANNED (proximity) ===\n";
                    std::cout << "ID: " << mid << "  Dist: ~" << (int)mi.distance << " cm\n";
                    std::cout << "==================================\n";

                    // Stop robot and show popup for 5 seconds
                    {
                        auto stopCmds = bridge.mapInstruction("STOP");
                        if (sendAsciiProtocol && !stopCmds.empty() && serial.isOpen())
                        {
                            // Send single-char control if enabled
                            sendCharForInstruction("STOP");
                            for (const auto &cmd : stopCmds)
                                serial.sendLine(cmd);
                        }
                    }
                    showScanSuccessPopup(mid, detail);
                    speakText("ArUco dengan id " + std::to_string(mid) + " berhasil discan");
                    if (!stopHintAnnounced)
                    {
                        speakText("Tugas satu selesai. Anda boleh menghentikan program.");
                        stopHintAnnounced = true;
                    }
                    break; // handle one at a time
                }
            }
        }

        // NEW: Prepare avoidance status variables untuk navigator
        bool hasYellowAhead = false;
        bool hasObstacleAhead = false;

        // NEW: Calculate yellowOffsetX - will be set later after yellow detection
        float yellowOffsetX = 0.0f;

        // NEW: Detect white line for following
        float lineErrorX = 0.0f;
        float lineAngleDeg = 0.0f;
        double lineConfidence = 0.0;
        detectWhiteLine(frame, lineErrorX, lineAngleDeg, lineConfidence);

        // Default instruction from existing Navigator (may be overridden by FSM below)
        std::string instruction = navigator.getNavigationInstruction(hasYellowAhead, hasObstacleAhead, yellowOffsetX, lineErrorX, lineAngleDeg, lineConfidence); // CHANGED: Pass line params with angle

        // ==================== High-level Behavior FSM transitions/actions ====================
        // Update SEARCH trigger
        if (lineConfidence <= 0.2)
            noLineFrames++;
        else
            noLineFrames = 0;

        // Determine hazard presence early for AVOID state hint (will be enforced later too)
        bool hazardLikely = false; // set after yellow/obstacle computed; placeholder for state only

        // State transitions (lightweight, non-invasive to existing navigator)
        switch (behaviorState)
        {
        case BehaviorState::INIT_SCAN:
        {
            // Rotate in place to recognize environment for one full circle or until confident line/ArUco is detected
            bool detectionReady = (lineConfidence > 0.5) || !markerIds.empty();
            if (initScanAccumulatedDeg >= 360 || detectionReady)
            {
                behaviorState = BehaviorState::FOLLOW_LINE;
                // Stop rotation by keeping navigator instruction result
            }
            else
            {
                // Override instruction with turn step
                std::string dir = initScanTurnLeft ? "KIRI" : "KANAN";
                instruction = std::string("PUTAR ") + dir + " " + std::to_string(INIT_SCAN_STEP_DEG) + "° - Initial scan";
                initScanAccumulatedDeg += INIT_SCAN_STEP_DEG;
            }
            break;
        }
        case BehaviorState::FOLLOW_LINE:
        {
            // Promote to APPROACH_ARUCO when any marker visible
            if (!markerIds.empty())
            {
                behaviorState = BehaviorState::APPROACH_ARUCO;
            }
            else if (noLineFrames >= NO_LINE_FRAMES_THRESHOLD)
            {
                behaviorState = BehaviorState::SEARCH;
            }
            break;
        }
        case BehaviorState::APPROACH_ARUCO:
        {
            // If marker lost, fallback
            if (markerIds.empty())
            {
                behaviorState = (lineConfidence > 0.5) ? BehaviorState::FOLLOW_LINE : BehaviorState::SEARCH;
            }
            break;
        }
        case BehaviorState::SEARCH:
        {
            // Override instruction to search pattern until we find line or ArUco
            if (lineConfidence > 0.5)
            {
                behaviorState = BehaviorState::FOLLOW_LINE;
            }
            else if (!markerIds.empty())
            {
                behaviorState = BehaviorState::APPROACH_ARUCO;
            }
            else
            {
                instruction = "PUTAR KANAN 20° - Search";
            }
            break;
        }
        case BehaviorState::AVOID:
        default:
            break;
        }

        // ============================================
        // 🆕 OPTIMAL ANTI-REVISIT LOGIC (Main Loop Level)
        // ============================================
        // Check jika instruction adalah scan/explore, cek visitCount untuk avoid loop
        std::string instrUpper = instruction;
        std::transform(instrUpper.begin(), instrUpper.end(), instrUpper.begin(), [](unsigned char c)
                       { return std::toupper(c); });

        if ((instrUpper.find("SCAN") != std::string::npos || instrUpper.find("PUTAR") != std::string::npos) &&
            markerIds.empty())
        {
            // Cek visit count di grid cell saat ini
            int visitCount = gridMap.getVisitCount(currentPose.xCm, currentPose.yCm);

            if (visitCount > 2)
            { // Sudah dikunjungi >2x = area revisited, cari escape route!
                std::cout << "⚠️  Anti-revisit: Cell (" << (int)currentPose.xCm << "," << (int)currentPose.yCm
                          << ") visited " << visitCount << "x - forcing exploration\n";

                // Probe directions untuk cari yang visit count rendah
                // Simulasi pose after turn left
                double probeLeftX = currentPose.xCm + 30.0 * std::cos((currentPose.thetaDeg - 30.0) * M_PI / 180.0);
                double probeLeftY = currentPose.yCm + 30.0 * std::sin((currentPose.thetaDeg - 30.0) * M_PI / 180.0);
                int visitLeft = gridMap.getVisitCount(probeLeftX, probeLeftY);

                // Simulasi pose after turn right
                double probeRightX = currentPose.xCm + 30.0 * std::cos((currentPose.thetaDeg + 30.0) * M_PI / 180.0);
                double probeRightY = currentPose.yCm + 30.0 * std::sin((currentPose.thetaDeg + 30.0) * M_PI / 180.0);
                int visitRight = gridMap.getVisitCount(probeRightX, probeRightY);

                std::cout << "  Probe: LEFT visit=" << visitLeft << ", RIGHT visit=" << visitRight << "\n";

                // Pilih arah dengan visit count paling rendah
                if (visitLeft < visitRight && visitLeft < 3)
                {
                    instruction = "PUTAR KIRI 30° - Explore low-visit area";
                    std::cout << "  → Choosing LEFT (lower visit count)\n";
                }
                else if (visitRight < visitLeft && visitRight < 3)
                {
                    instruction = "PUTAR KANAN 30° - Explore low-visit area";
                    std::cout << "  → Choosing RIGHT (lower visit count)\n";
                }
                else if (visitLeft >= 3 && visitRight >= 3)
                {
                    // Kedua arah HIGH visit - force escape dengan mundur + random 180°
                    if (!hasYellowAhead && !hasObstacleAhead)
                    {
                        std::string randomDir = (rand() % 2 == 0) ? "KIRI" : "KANAN";
                        instruction = "MUNDUR 20 cm + PUTAR " + randomDir + " 180° - Escape revisited trap";
                        std::cout << "  → FORCING ESCAPE: Both directions revisited, turning 180°\n";
                    }
                    else
                    {
                        instruction = "MUNDUR 30 cm - Backtrack from stuck area";
                        std::cout << "  → BACKTRACKING from high-visit area\n";
                    }
                }
                else
                {
                    // Salah satu arah OK tapi not much better - alternate
                    std::string alternateDir = (visitLeft <= visitRight) ? "KIRI" : "KANAN";
                    instruction = "PUTAR " + alternateDir + " 30° - Try less-visited direction";
                    std::cout << "  → Trying " << alternateDir << " (moderately better)\n";
                }
            }
        }
        // ============================================
        // END OPTIMAL ANTI-REVISIT LOGIC
        // ============================================

        // ENHANCED: Integrasikan path planning untuk eksplorasi sistematis
        // Jika tidak ada marker dan instruksi adalah SCAN, gunakan waypoint exploration
        if (markerIds.empty() && instruction.find("SCAN") != std::string::npos &&
            currentWaypointIndex < explorationWaypoints.size())
        {
            Waypoint nextWp = explorationWaypoints[currentWaypointIndex];

            // Hitung jarak dan sudut ke waypoint berikutnya
            double dx = nextWp.x - currentPose.xCm;
            double dy = nextWp.y - currentPose.yCm;
            double dist = std::sqrt(dx * dx + dy * dy);
            double targetTheta = std::atan2(dy, dx) * 180.0 / M_PI;
            double turnDeg = targetTheta - currentPose.thetaDeg;

            // Normalize angle to [-180, 180]
            while (turnDeg > 180.0)
                turnDeg -= 360.0;
            while (turnDeg < -180.0)
                turnDeg += 360.0;

            // Jika sudah dekat waypoint (< 20 cm), lanjut ke waypoint berikutnya
            if (dist < 20.0)
            {
                currentWaypointIndex++;
                std::cout << "✓ Waypoint " << currentWaypointIndex << " reached. Moving to next..." << std::endl;
            }
            // Jika belum menghadap waypoint (> 15 derajat), putar dulu
            else if (std::abs(turnDeg) > 15.0)
            {
                std::string direction = (turnDeg > 0) ? "KIRI" : "KANAN";
                instruction = "PUTAR " + direction + " " + std::to_string((int)std::abs(turnDeg)) + "° - Ke waypoint";
                std::cout << "→ Navigating to waypoint #" << currentWaypointIndex
                          << " (" << (int)nextWp.x << "," << (int)nextWp.y << ") - Turning "
                          << (int)std::abs(turnDeg) << "° " << direction << std::endl;
            }
            // Sudah menghadap, maju ke waypoint
            else
            {
                int majuDist = std::min((int)dist, 50); // Max 50 cm per step
                instruction = "MAJU " + std::to_string(majuDist) + " cm - Eksplorasi waypoint";
                std::cout << "→ Navigating to waypoint #" << currentWaypointIndex
                          << " - Moving forward " << majuDist << " cm" << std::endl;
            }
        }

        // Query current target and approximate distance (if available)
        int currentTargetId = navigator.getTargetId();
        double currentTargetDistCm = 1e9;
        if (currentTargetId >= 0)
        {
            const auto &markersMap = navigator.getMarkers();
            auto itm = markersMap.find(currentTargetId);
            if (itm != markersMap.end())
            {
                currentTargetDistCm = itm->second.distance;
            }
        }
        int arucoPriorityDistanceCm = cfgInt(cfg, "aruco.priority_distance_cm", 40); // prioritize scanning when close
        bool arucoClose = (currentTargetId >= 0 && currentTargetDistCm < arucoPriorityDistanceCm);
        // Explore policy: bila instruksi eksplorasi, hindari jalur yang sama
        {
            std::string u = instruction;
            std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c)
                           { return std::toupper(c); });
            if (u.find("SCAN LINGKUNGAN") != std::string::npos)
            {
                // Prefer approaching when we have a target OR recently saw any marker
                uint64_t nowT = nowMs();
                bool recentlySawMarker = (nowT - lastMarkerSeenMs) <= (uint64_t)explorationMarkerGraceMs;
                int currentTargetIdTmp = navigator.getTargetId();
                if (currentTargetIdTmp >= 0 || recentlySawMarker)
                {
                    instruction = "MAJU PELAN - Hold course";
                }
                else if ((nowT - lastScanActionMs) < (uint64_t)explorationScanCooldownMs)
                {
                    instruction = "MAJU PELAN - Cooldown";
                }
                else
                {
                    // Alternate: turn once, then move forward a bit to avoid spinning in place
                    static bool didScanTurnLast = false;
                    static bool scanTurnLeftToggle = false; // strictly alternate L/R across nodes
                    if (didScanTurnLast)
                    {
                        instruction = "MAJU PELAN 10 cm - After scan turn";
                        didScanTurnLast = false;
                        lastScanActionMs = nowT;
                    }
                    else
                    {
                        // Decide left/right using breadcrumb; if ambiguous, use global toggle
                        std::string nk = nodeKey(currentPose, /*cellSizeCm=*/50.0, /*headingBinDeg=*/15);
                        bool triedLeft = nodeTried[nk].count("L") > 0;
                        bool triedRight = nodeTried[nk].count("R") > 0;
                        if (triedLeft && !triedRight)
                        {
                            instruction = "PUTAR KANAN 20° - Scan lingkungan";
                            nodeTried[nk].insert("R");
                        }
                        else if (triedRight && !triedLeft)
                        {
                            instruction = "PUTAR KIRI 20° - Scan lingkungan";
                            nodeTried[nk].insert("L");
                        }
                        else if (triedLeft && triedRight)
                        {
                            nodeTried[nk].clear();
                            if (scanTurnLeftToggle)
                            {
                                instruction = "PUTAR KIRI 20° - Scan lingkungan";
                                nodeTried[nk].insert("L");
                            }
                            else
                            {
                                instruction = "PUTAR KANAN 20° - Scan lingkungan";
                                nodeTried[nk].insert("R");
                            }
                            scanTurnLeftToggle = !scanTurnLeftToggle;
                        }
                        else
                        {
                            if (scanTurnLeftToggle)
                            {
                                instruction = "PUTAR KIRI 20° - Scan lingkungan";
                                nodeTried[nk].insert("L");
                            }
                            else
                            {
                                instruction = "PUTAR KANAN 20° - Scan lingkungan";
                                nodeTried[nk].insert("R");
                            }
                            scanTurnLeftToggle = !scanTurnLeftToggle;
                        }
                        didScanTurnLast = true;
                        lastScanActionMs = nowT;
                    }
                }
            }
        }

        // Check yellow boundary first (for distance-based decision)
        std::vector<cv::Rect> yboxes;
        int nearestPx = 0;
        double ycov = 0.0;
        double yellowDistanceM = -1.0;
        double yellowConfidence = 0.0;

        if (yellowDetectionEnabled)
        {
            // Apply tuner overrides, if enabled
            if (yellowTunerEnabled)
            {
                // Ensure sane ranges
                if (trk_hLow > trk_hHigh)
                    trk_hHigh = trk_hLow;
                if (trk_labALow > trk_labAHigh)
                    trk_labAHigh = trk_labALow;
                yHLow = std::max(0, std::min(179, trk_hLow));
                yHHigh = std::max(0, std::min(179, trk_hHigh));
                ySLow = std::max(0, std::min(255, trk_sLow));
                yVLow = std::max(0, std::min(255, trk_vLow));
                yLabBLow = std::max(0, std::min(255, trk_labBLow));
                yALow = std::max(0, std::min(255, trk_labALow));
                yAHigh = std::max(0, std::min(255, trk_labAHigh));
                yCombineOr = (trk_combine != 0);
                trk_minAreaX1e4 = std::max(0, trk_minAreaX1e4);
                yMinArea = std::max(0.0, (double)trk_minAreaX1e4 / 10000.0);
            }
            // Lightweight caching to reduce per-frame cost
            static std::vector<cv::Rect> yboxesCache;
            static int nearestPxCache = 0;
            static double ycovCache = 0.0;
            static double yellowDistanceMCache = -1.0;
            static double yellowConfidenceCache = 0.0;
            static bool haveYellowCache = false;

            if (yellowDetectionInterval <= 1 || (frameCounter % yellowDetectionInterval) == 0)
            {
                detectYellowBoxes(frame,
                                  yHLow, yHHigh, ySLow, yVLow,
                                  /*useAdaptive=*/true, /*adaptDeltaH=*/10,
                                  /*labALow=*/yALow, /*labAHigh=*/yAHigh, /*labBLow=*/yLabBLow, /*combineOr=*/yCombineOr,
                                  /*bandTopRatio=*/yBandTopRatioPct / 100.0,
                                  /*relativeMinY=*/yRelativeMinY,
                                  yMinArea, ycov, yboxes, nearestPx, yellowDistanceM,
                                  cameraHeightCm, cameraFovVertical, cameraTiltDeg, cameraCalibrationFactor, yellowConfidence);
                yboxesCache = yboxes;
                nearestPxCache = nearestPx;
                ycovCache = ycov;
                yellowDistanceMCache = yellowDistanceM;
                yellowConfidenceCache = yellowConfidence;
                haveYellowCache = true;
            }
            else if (haveYellowCache)
            {
                yboxes = yboxesCache;
                nearestPx = nearestPxCache;
                ycov = ycovCache;
                yellowDistanceM = yellowDistanceMCache;
                yellowConfidence = yellowConfidenceCache;
            }
        }

        // Obstacle Detection (HIGHEST PRIORITY)
        std::vector<DetectedObstacle> obstacles;
        CollisionAvoidance::ThreatLevel obstacleThreat = CollisionAvoidance::ThreatLevel::SAFE;

#ifdef USE_YOLO_DETECTOR
        // Use YOLOv5 for more accurate obstacle detection
        // OPTIMIZATION: Only run YOLO every N frames to reduce lag
        if (useYolo && yoloDetector && (frameCounter % yoloDetectionInterval == 0))
        {
            std::vector<YoloDetection> yoloDetections = yoloDetector->detectObstacles(frame);

            // Clear cache and update with new detections
            lastYoloObstacles.clear();

            // Convert YOLO detections to DetectedObstacle format
            for (const auto &yoloDet : yoloDetections)
            {
                DetectedObstacle obs;
                obs.boundingBox = yoloDet.box;
                obs.centerPoint = cv::Point2f(yoloDet.box.x + yoloDet.box.width / 2.0f,
                                              yoloDet.box.y + yoloDet.box.height / 2.0f);
                obs.confidence = yoloDet.confidence;

                // Estimate distance based on box position in frame
                // Objects at bottom of frame are closer
                double normalizedY = static_cast<double>(obs.centerPoint.y) / frame.rows;
                obs.distanceCm = 200.0 * (1.0 - normalizedY); // Simple approximation
                obs.distanceCm = std::max(10.0, std::min(300.0, obs.distanceCm));

                obstacles.push_back(obs);
                lastYoloObstacles.push_back(obs); // Cache for next frames

                // Draw YOLO detections
                cv::Scalar color = cv::Scalar(0, 255, 255); // Yellow for YOLO
                cv::rectangle(frame, yoloDet.box, color, 2);
                std::string label = yoloDet.className + " " +
                                    std::to_string(static_cast<int>(yoloDet.confidence * 100)) + "%";
                cv::putText(frame, label,
                            cv::Point(yoloDet.box.x, yoloDet.box.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }

            obstacleThreat = CollisionAvoidance::assessThreat(obstacles, obstacleConfig);
        }
        else if (useYolo && !lastYoloObstacles.empty())
        {
            // Use cached YOLO results on non-detection frames
            obstacles = lastYoloObstacles;
            obstacleThreat = CollisionAvoidance::assessThreat(obstacles, obstacleConfig);

            // Draw cached detections (slightly dimmed)
            for (const auto &obs : obstacles)
            {
                cv::Scalar color = cv::Scalar(0, 200, 200); // Slightly dimmed yellow
                cv::rectangle(frame, obs.boundingBox, color, 1);
                std::string label = std::to_string((int)obs.distanceCm) + "cm (cached)";
                cv::putText(frame, label,
                            cv::Point(obs.boundingBox.x, obs.boundingBox.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
        }
        else
#endif
            // Fallback to traditional CV-based detection
            if (obstacleDetectionEnabled)
            {
                obstacles = obstacleDetector.detectObstacles(frame);
                obstacleThreat = CollisionAvoidance::assessThreat(obstacles, obstacleConfig);

                // Update grid map with obstacles
                for (const auto &obs : obstacles)
                {
                    // Convert obstacle position to world coordinates (relative to robot)
                    double obstacleWorldX = currentPose.xCm + obs.distanceCm * std::cos(currentPose.thetaDeg * M_PI / 180.0);
                    double obstacleWorldY = currentPose.yCm + obs.distanceCm * std::sin(currentPose.thetaDeg * M_PI / 180.0);

                    if (obs.confidence > 0.5)
                    {
                        gridMap.setCell(obstacleWorldX, obstacleWorldY, CellType::OBSTACLE);
                    }
                }

                // Override instruction if obstacle detected
                if (obstacleThreat == CollisionAvoidance::ThreatLevel::EMERGENCY)
                {
                    instruction = "STOP";
                }
                else if (obstacleThreat == CollisionAvoidance::ThreatLevel::DANGER)
                {
                    instruction = "MUNDUR 20 cm";
                }
                else if (obstacleThreat == CollisionAvoidance::ThreatLevel::WARNING)
                {
                    // Only slow down if moving forward
                    if (instruction.find("MAJU") != std::string::npos)
                    {
                        instruction = "MAJU PELAN";
                    }
                }
            }

        // High-priority: Yellow boundary avoidance based on REAL DISTANCE
        // Only act on high-confidence detections to avoid false positives
        double yellowRatio = ycov;
        bool yellowDetectedConfident = (yellowDetectionEnabled && !yboxes.empty() && yellowDistanceM >= 0 && yellowConfidence > 0.5); // CHANGED: Tingkatkan min confidence ke 0.5

        // NEW: Calculate yellowOffsetX from yellow boxes (untuk directional avoidance)
        if (!yboxes.empty())
        {
            float sumX = 0.0f;
            for (const auto &box : yboxes)
            {
                sumX += (box.x + box.width / 2.0f);
            }
            yellowOffsetX = sumX / yboxes.size() - frame.cols / 2.0f; // Offset from center (-left, +right)
        }

        // NEW: Update avoidance status untuk navigator (dilakukan SETELAH deteksi)
        hasYellowAhead = yellowDetectedConfident && (yellowDistanceM < yellowSafetyDistanceM * 2.0); // Ahead jika < 2x safety
        hasObstacleAhead = (obstacleThreat >= CollisionAvoidance::ThreatLevel::WARNING);             // Ahead jika warning+

        // NEW: Tambah check jika garis di bottom extreme (nearestPx > h*0.9) untuk peringatan hanya saat mendekati
        if (yellowDetectedConfident && nearestPx > (frame.rows * 0.9))
        { // Hanya jika dekat bottom frame
            if (arucoClose && yellowDistanceM >= yellowSafetyDistanceM * 0.8)
            {
                // Allow cautious approach
                if (instruction.find("MAJU") == std::string::npos)
                {
                    instruction = "MAJU PELAN";
                }
            }
            // Smart decision based on distance - CHANGED: Ketat threshold
            else if (yellowDistanceM < yellowSafetyDistanceM)
            {
                // TOO CLOSE! Immediate stop or backup
                instruction = "STOP";
                if (yellowDistanceM < yellowSafetyDistanceM * 0.5) // Very close (< 0.5 cm)
                {
                    instruction = "MUNDUR 30 cm"; // Back up!
                }
            }
            else if (yellowDistanceM < yellowSafetyDistanceM * 1.5) // Warning zone (1-1.5 cm)
            {
                // Slow down, prepare to stop
                if (instruction.find("MAJU") != std::string::npos)
                {
                    instruction = "MAJU PELAN"; // Slow forward
                }
            }
            // else: yellowDistanceM > 1.5 cm, safe to continue normal operation, no warning
        }
        else if (yellowDetectionEnabled && !yboxes.empty() && yellowConfidence <= 0.5)
        {
            // Low confidence detection - ignore but log for debugging
            // This helps reduce false positives from lighting changes
        }

        // If navigator locks, mark visited AND save detailed info
        if (instruction.find("LOCKED") != std::string::npos)
        {
            int tid = navigator.getTargetId();
            if (tid >= 0 && visited.find(tid) == visited.end())
            {
                // Mark as visited
                visited.insert(tid);
                saveVisitedTxt(visitedTxtPath, visited);
                saveVisited(visitedJsonPath, visited);

                // Get marker info from navigator
                const auto &markers = navigator.getMarkers();
                auto it = markers.find(tid);
                if (it != markers.end())
                {
                    const MarkerInfo &info = it->second;

                    // Create or update detailed scan info
                    ScannedMarkerDetail detail;
                    detail.id = tid;
                    detail.timestamp = nowString();
                    detail.distance = info.distance;
                    detail.markerSize = info.size;
                    detail.position = info.center;

                    // Update scan count
                    if (scannedDb.find(tid) != scannedDb.end())
                    {
                        detail.scanCount = scannedDb[tid].scanCount + 1;
                    }
                    else
                    {
                        detail.scanCount = 1;
                    }

                    // Save to database
                    scannedDb[tid] = detail;
                    saveScannedDatabase(scannedDbPath, scannedDb);

                    // Show success popup
                    std::cout << "\n";
                    std::cout << "========================================\n";
                    std::cout << "   ARUCO MARKER BERHASIL DISCAN!       \n";
                    std::cout << "========================================\n";
                    std::cout << "ID: " << tid << "\n";
                    std::cout << "Timestamp: " << detail.timestamp << "\n";
                    std::cout << "Distance: ~" << (int)detail.distance << " cm\n";
                    std::cout << "Size: " << (int)detail.markerSize << " px\n";
                    std::cout << "Position: (" << (int)detail.position.x << ", " << (int)detail.position.y << ")\n";
                    std::cout << "Scan Count: " << detail.scanCount << "\n";
                    std::cout << "========================================\n\n";

                    // Immediately stop robot before showing popup
                    {
                        auto stopCmds = bridge.mapInstruction("STOP");
                        if (sendAsciiProtocol && !stopCmds.empty() && serial.isOpen())
                        {
                            // Send single-char control if enabled
                            sendCharForInstruction("STOP");
                            for (const auto &cmd : stopCmds)
                                serial.sendLine(cmd);
                        }
                    }

                    showScanSuccessPopup(tid, detail);
                    speakText("ArUco dengan id " + std::to_string(tid) + " berhasil discan");
                    if (!stopHintAnnounced)
                    {
                        speakText("Tugas satu selesai. Anda boleh menghentikan program.");
                        stopHintAnnounced = true;
                    }
                }
            }
        }

        // NEW: Agresif override jika warning yellow muncul (force evade with direction)
        if (yellowDetectedConfident && !yellowVoiceActive)
        {
            speakText("Peringatan, garis kuning terdeteksi.");
            yellowVoiceActive = true;
        }
        else if (!yellowDetectedConfident && yellowVoiceActive)
        {
            yellowVoiceActive = false;
        }

        if (yellowDetectedConfident)
        {
            std::cout << "Yellow detected - triggering avoidance override (offsetX: " << yellowOffsetX << ")\n";
            std::string evadeDir = (yellowOffsetX > 0) ? "KIRI" : "KANAN";
            // Reflect state for display/logic
            behaviorState = BehaviorState::AVOID;
            if (yellowDistanceM < yellowSafetyDistanceM)
            {
                // TOO CLOSE! Immediate stop or backup
                instruction = "STOP - Garis kuning terlalu dekat!";
                if (yellowDistanceM < yellowSafetyDistanceM * 0.5) // Very close
                {
                    instruction = "MUNDUR 30 cm - Hindari garis kuning!";
                }
            }
            else if (yellowDistanceM < yellowSafetyDistanceM * 1.5)
            { // Warning zone - force hinder with direction
                instruction = "MUNDUR 20 cm + PUTAR " + evadeDir + " 45° - Hindari garis kuning mendekati";
            }
            else
            {
                // Detected tapi jauh - slow down jika maju, atau putar slight jika offset besar
                if (instruction.find("MAJU") != std::string::npos)
                {
                    if (std::abs(yellowOffsetX) > frame.cols * 0.2f)
                    {
                        instruction = "MAJU PELAN + PUTAR " + evadeDir + " 15° - Garis kuning terlihat";
                    }
                    else
                    {
                        instruction = "MAJU PELAN - Garis kuning terlihat";
                    }
                }
            }
        }
        else if (hasObstacleAhead && obstacleThreat == CollisionAvoidance::ThreatLevel::EMERGENCY)
        {
            instruction = "STOP - Emergency obstacle";
        }

        // NEW: ArUco handling refined - only stop when actually locked/ready to scan, otherwise allow approach
        if (!markerIds.empty())
        {
            int currentTargetIdTmp = navigator.getTargetId();
            double currDist = currentTargetDistCm;
            bool readyToScan = (navigator.getState() == LOCKED) || (currentTargetIdTmp >= 0 && currDist < scanDistanceCmCfg);
            if (readyToScan)
            {
                instruction = "STOP - ArUco terdeteksi"; // Ready to scan
                if (serial.isOpen())
                {
                    serial.sendLine(" "); // Kirim space ke HC-05
                    std::cout << "Sent ' ' to HC-05 for ArUco detection\n";
                }
            }
            else
            {
                // Ensure behavior state reflects intent to approach
                if (behaviorState == BehaviorState::FOLLOW_LINE)
                    behaviorState = BehaviorState::APPROACH_ARUCO;
            }
        }

        if (behaviorState != lastSpokenBehaviorState)
        {
            std::string speech = behaviorStateSpeechText(behaviorState);
            if (!speech.empty())
                speakText(speech);
            lastSpokenBehaviorState = behaviorState;
        }

        // Rate-limit sending to robot to avoid spamming
        bool changed = (instruction != lastInstruction);
        uint64_t now = nowMs();
        bool timeout = (now - lastSentMs) > 600; // ms
        if (changed || timeout)
        {
            if (changed)
            {
                speakNavigationHint(instruction);
            }
            auto commands = bridge.mapInstruction(instruction);
            if (serial.isOpen())
            {
                // Send single-char control if enabled
                sendCharForInstruction(instruction);
                if (sendAsciiProtocol && !commands.empty())
                {
                    for (const auto &cmd : commands)
                    {
                        serial.sendLine(cmd);
                    }
                }
            }
            // Update path memory (dead-reckoning) based on high-level instruction
            Pose prev = currentPose;
            updatePoseFromInstruction(currentPose, instruction);
            pathTrace.push_back(currentPose);
            appendPathCsv(pathCsv, currentPose, instruction);

            // Update GridMap with current robot position
            gridMap.markExplored(currentPose.xCm, currentPose.yCm);
            gridMap.incrementVisitCount(currentPose.xCm, currentPose.yCm);

            // Mark yellow boundaries in grid map
            if (yellowDetectedConfident)
            {
                double yellowWorldX = currentPose.xCm + yellowDistanceM * 100.0 * std::cos(currentPose.thetaDeg * M_PI / 180.0);
                double yellowWorldY = currentPose.yCm + yellowDistanceM * 100.0 * std::sin(currentPose.thetaDeg * M_PI / 180.0);
                gridMap.setCell(yellowWorldX, yellowWorldY, CellType::YELLOW_LINE);
            }

            // Draw path segment onto occupancy grid (legacy visualization)
            cv::Point p0 = poseToGrid(prev, occupancyGrid.cols, cmPerPixel);
            cv::Point p1 = poseToGrid(currentPose, occupancyGrid.cols, cmPerPixel);
            cv::line(occupancyGrid, p0, p1, cv::Scalar(255), 1);
            // Save grid periodically
            gridSaveCounter++;
            if (gridSaveCounter % 50 == 0)
            {
                cv::imwrite("logs/grid_map.png", occupancyGrid);
            }
            lastInstruction = instruction;
            lastSentMs = now;
        }

        // Draw detected ArUco markers on frame
        if (!markerIds.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // ENHANCED: Tampilkan ID ArUco secara real-time dengan info lebih lengkap
            std::cout << "\n=== ArUco Detection ===" << std::endl;
            std::cout << "Total markers detected: " << markerIds.size() << std::endl;

            // Draw additional info for each marker
            for (size_t i = 0; i < markerIds.size(); i++)
            {
                int id = markerIds[i];

                // Calculate marker center
                cv::Point2f center(0, 0);
                for (int j = 0; j < 4; j++)
                {
                    center.x += markerCorners[i][j].x;
                    center.y += markerCorners[i][j].y;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                // Calculate marker size
                float dx = markerCorners[i][0].x - markerCorners[i][2].x;
                float dy = markerCorners[i][0].y - markerCorners[i][2].y;
                float size = std::sqrt(dx * dx + dy * dy);

                // Estimate distance (rough approximation)
                float distance = 1000.0f / (size + 1.0f);

                // Log to console
                std::string status = (visited.find(id) != visited.end()) ? "✓ VISITED" : "○ NEW";
                std::cout << "  Marker ID: " << id << " | Distance: ~" << (int)distance
                          << " cm | Size: " << (int)size << " px | Status: " << status << std::endl;

                // ENHANCED: Draw ID prominently di frame dengan warna berbeda
                cv::Scalar idColor = (visited.find(id) != visited.end()) ? cv::Scalar(128, 128, 128) : // Gray untuk visited
                                         cv::Scalar(0, 255, 0);                                        // Green untuk new

                // Draw large ID text near marker
                std::string idText = "ID: " + std::to_string(id);
                cv::putText(frame, idText, cv::Point(center.x + 10, center.y - 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, idColor, 2);

                // Draw distance info
                std::string distText = "~" + std::to_string((int)distance) + " cm";
                cv::putText(frame, distText, cv::Point(center.x + 10, center.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

                // Draw center circle
                cv::circle(frame, center, 5, idColor, -1);

                // Jika dekat dan belum visited, tampilkan highlight
                if (distance < 50.0f && visited.find(id) == visited.end())
                {
                    std::cout << "  >>> READY TO SCAN: Marker ID " << id << " is close!" << std::endl;
                    cv::circle(frame, center, 30, cv::Scalar(0, 255, 255), 3);
                    cv::putText(frame, "READY", cv::Point(center.x - 30, center.y + 50),
                                cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
                }
            }
            std::cout << "======================\n"
                      << std::endl;
        }

        // Show small status on frame
        cv::putText(frame, "Instruksi: " + instruction, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Overlay BehaviorState for debugging/clarity
        auto behaviorToStr = [](BehaviorState s)
        {
            switch (s)
            {
            case BehaviorState::INIT_SCAN:
                return "INIT_SCAN";
            case BehaviorState::FOLLOW_LINE:
                return "FOLLOW_LINE";
            case BehaviorState::APPROACH_ARUCO:
                return "APPROACH_ARUCO";
            case BehaviorState::SEARCH:
                return "SEARCH";
            case BehaviorState::AVOID:
                return "AVOID";
            default:
                return "UNKNOWN";
            }
        };
        cv::putText(frame, std::string("State: ") + behaviorToStr(behaviorState), cv::Point(10, 55),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);

        // Show ArUco detection count
        std::string aruco_status = "ArUco: " + std::to_string(markerIds.size()) + " detected";
        cv::putText(frame, aruco_status, cv::Point(10, frame.rows - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 100, 255), 2);

        // Visual for yellow boundary (boxes already detected above for decision making)
        // Draw yellow boxes (only if detection enabled)
        if (yellowDetectionEnabled)
        {
            for (const auto &rb : yboxes)
            {
                cv::rectangle(frame, rb, cv::Scalar(0, 255, 255), 2);
            }

            // Display yellow detection info with real distance
            if (!yboxes.empty())
            {
                // Color-code by confidence: high=green, medium=yellow, low=red
                cv::Scalar warnColor = yellowConfidence > 0.7 ? cv::Scalar(0, 255, 0) : yellowConfidence > 0.4 ? cv::Scalar(0, 200, 255)
                                                                                                               : cv::Scalar(0, 100, 255);

                std::string warn = "YELLOW BOUNDARY!";
                cv::putText(frame, warn, cv::Point(10, 60), cv::FONT_HERSHEY_DUPLEX, 0.7,
                            warnColor, 2);

                // Display confidence score
                char confBuf[32];
                snprintf(confBuf, sizeof(confBuf), "Confidence: %.2f", yellowConfidence);
                cv::putText(frame, std::string(confBuf), cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            warnColor, 1);

                // Display distance in METERS (big and clear!)
                if (yellowDistanceM >= 0)
                {
                    std::string distText = "Distance: " + std::to_string((int)(yellowDistanceM * 100)) + " cm";
                    if (yellowDistanceM < 1.0)
                    {
                        // For distances < 1m, show in cm
                        distText = "Distance: " + std::to_string((int)(yellowDistanceM * 100)) + " cm";
                    }
                    else
                    {
                        // For distances >= 1m, show in meters
                        char buf[32];
                        snprintf(buf, sizeof(buf), "Distance: %.2f m", yellowDistanceM);
                        distText = std::string(buf);
                    }
                    cv::putText(frame, distText, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                                cv::Scalar(255, 255, 0), 2);

                    // Warning if too close AND high confidence!
                    if (yellowDetectedConfident && yellowDistanceM < yellowSafetyDistanceM)
                    {
                        cv::putText(frame, "!!! TOO CLOSE !!!", cv::Point(10, 140),
                                    cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    }
                }
            }
        } // End yellowDetectionEnabled

        // Draw obstacles
        if (obstacleDetectionEnabled && !obstacles.empty())
        {
            obstacleDetector.drawObstacles(frame, obstacles);

            // Display obstacle status
            DetectedObstacle nearest = CollisionAvoidance::findNearestObstacle(obstacles);
            if (nearest.distanceCm < 500.0)
            {                                                                         // Valid detection
                int yOffset = (yellowDetectionEnabled && !yboxes.empty()) ? 170 : 60; // Adjust position if yellow also shown

                std::string obsText = "OBSTACLE: " + std::to_string((int)nearest.distanceCm) + "cm";
                cv::Scalar obsColor = obstacleThreat == CollisionAvoidance::ThreatLevel::EMERGENCY ? cv::Scalar(0, 0, 255) : obstacleThreat == CollisionAvoidance::ThreatLevel::DANGER ? cv::Scalar(0, 100, 255)
                                                                                                                         : obstacleThreat == CollisionAvoidance::ThreatLevel::WARNING  ? cv::Scalar(0, 200, 255)
                                                                                                                                                                                       : cv::Scalar(0, 255, 0);

                cv::putText(frame, obsText, cv::Point(10, yOffset), cv::FONT_HERSHEY_DUPLEX, 0.7,
                            obsColor, 2);

                if (obstacleThreat == CollisionAvoidance::ThreatLevel::EMERGENCY)
                {
                    cv::putText(frame, "!!! EMERGENCY STOP !!!", cv::Point(10, yOffset + 30),
                                cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        // Map 3x3m rendering
        if (showMap)
        {
            // 3m -> 300cm; with 1 px = cmPerPixel, view size in pixels:
            int viewSizePx = (int)std::round(300.0 / cmPerPixel);
            viewSizePx = std::max(50, std::min(viewSizePx, occupancyGrid.cols));
            cv::Point centerGrid = poseToGrid(currentPose, occupancyGrid.cols, cmPerPixel);
            cv::Rect roi = computeViewportROI(centerGrid, occupancyGrid.cols, viewSizePx);
            cv::Mat crop = occupancyGrid(roi).clone();
            cv::Mat mapBgr;
            cv::cvtColor(crop, mapBgr, cv::COLOR_GRAY2BGR);

            // Draw waypoints BEFORE robot overlay so robot is on top
            drawWaypoints(mapBgr, waypoints, currentPose, mapBgr.cols, cmPerPixel);

            double robotSizePx = 40.0 / cmPerPixel; // 40 cm robot
            drawRobotOverlay(mapBgr, mapBgr.cols, robotSizePx, currentPose.thetaDeg);

            cv::Mat mapDisp;
            cv::resize(mapBgr, mapDisp, cv::Size(600, 600), 0, 0, cv::INTER_NEAREST);

            // Draw grid scale bar
            drawGridScaleBar(mapDisp, cmPerPixel);

            // Draw compass
            drawCompass(mapDisp, currentPose.thetaDeg);

            // Border and label
            cv::rectangle(mapDisp, cv::Rect(0, 0, mapDisp.cols, mapDisp.rows), cv::Scalar(200, 200, 200), 2);
            cv::putText(mapDisp, "Map 3x3 m (robot center)", cv::Point(14, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            // Show waypoint count
            std::string wpInfo = "Waypoints: " + std::to_string(waypoints.size());
            cv::putText(mapDisp, wpInfo, cv::Point(14, 55),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);

            cv::imshow("Map 3x3m", mapDisp);
        }

        // ENHANCED: Tambahkan warning visual untuk garis kuning di frame utama
        // NEW: Sesuaikan dengan zone baru (< 1cm, 1-3cm, 3-5cm) - VERY STRICT!
        if (yellowDetectedConfident)
        {
            cv::Scalar warningColor;
            std::string warningText;

            if (yellowDistanceM < 0.01) // CRITICAL: < 1 cm
            {
                // CRITICAL - Red flashing
                warningColor = cv::Scalar(0, 0, 255);
                warningText = "!!! GARIS KUNING < 1 cm - MUNDUR !!!";
            }
            else if (yellowDistanceM < 0.03) // WARNING: 1-3 cm
            {
                // WARNING - Orange/Red
                warningColor = cv::Scalar(0, 100, 255);
                warningText = "WARNING: Garis Kuning ~" + std::to_string((int)(yellowDistanceM * 100)) + " cm";
            }
            else if (yellowDistanceM < 0.05) // CAUTION: 3-5 cm
            {
                // CAUTION - Yellow
                warningColor = cv::Scalar(0, 255, 255);
                warningText = "CAUTION: Garis Kuning " + std::to_string((int)(yellowDistanceM * 100)) + " cm";
            }

            // Tampilkan warning di frame dengan background semi-transparent
            if (!warningText.empty())
            {
                int baseline = 0;
                cv::Size textSize = cv::getTextSize(warningText, cv::FONT_HERSHEY_DUPLEX, 0.8, 2, &baseline);
                cv::Point textOrg(10, 150);

                // Background rectangle
                cv::rectangle(frame,
                              cv::Point(textOrg.x - 5, textOrg.y - textSize.height - 5),
                              cv::Point(textOrg.x + textSize.width + 5, textOrg.y + 5),
                              cv::Scalar(0, 0, 0), -1);

                // Warning text
                cv::putText(frame, warningText, textOrg,
                            cv::FONT_HERSHEY_DUPLEX, 0.8, warningColor, 2);
            }
        }

        cv::imshow("Rover Camera", frame);

        // Log decision + show panel
        DecisionEntry e{nowString(), instruction, (int)markerIds.size(), navigator.getTargetId(), yellowRatio};
        history.push_back(e);
        if (history.size() > maxHistory)
            history.pop_front();
        if (csv.is_open())
        {
            csv << e.timeStr << "," << e.instruction << "," << e.numMarkers << "," << e.targetId << "," << e.yellowRatio << "\n";
        }
        if (debugPanel)
        {
            cv::Mat panel(panelH, panelW, CV_8UC3, cv::Scalar(20, 20, 20));
            cv::putText(panel, "Decision Panel", cv::Point(18, 30), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            cv::putText(panel, "Markers: " + std::to_string((int)markerIds.size()) + "  Target: " + std::to_string(navigator.getTargetId()),
                        cv::Point(18, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
            cv::putText(panel, "Yellow ratio: " + std::to_string((int)(yellowRatio * 100)) + "%", cv::Point(18, 82),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
            int y = 110;
            cv::putText(panel, "Recent decisions:", cv::Point(18, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(180, 180, 180), 1);
            y += 18;
            for (auto it = history.rbegin(); it != history.rend(); ++it)
            {
                std::string row = it->timeStr + " | " + it->instruction;
                if ((int)row.size() > 50)
                    row = row.substr(0, 50) + "...";
                cv::putText(panel, row, cv::Point(18, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                y += 18;
                if (y > panel.rows - 20)
                    break;
            }
            cv::imshow("Decision Panel", panel);
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
            break;
        if (key == 'v' || key == 'V')
        {
            // Reset visited markers
            visited.clear();
            saveVisitedTxt(visitedTxtPath, visited);
            saveVisited(visitedJsonPath, visited);
            navigator.preloadVisited(visited);
            std::cout << "Visited markers reset.\n";
        }
        if (key == 'r' || key == 'R')
        {
            // NEW: Reset detection popups (untuk test ulang popup)
            poppedIds.clear();
            std::cout << "Detection popups reset. Will show again for all markers.\n";
        }
        if (key == 'p' || key == 'P')
        {
            // Reset path memory
            currentPose = Pose();
            pathTrace.clear();
            occupancyGrid.setTo(cv::Scalar(0));
            cv::imwrite("logs/grid_map.png", occupancyGrid);
            std::ofstream pathOut(pathCsv, std::ios::trunc);
            if (pathOut.is_open())
            {
                pathOut << "timestamp,x_cm,y_cm,theta_deg,instruction\n";
            }
            nodeTried.clear();
            std::cout << "Path memory reset.\n";
        }
        if (key == 's' || key == 'S')
        {
            // Save current yellow thresholds back to YAML
            char minAreaBuf[32];
            std::snprintf(minAreaBuf, sizeof(minAreaBuf), "%.4f", std::max(0.0, yMinArea));
            std::vector<std::pair<std::string, std::string>> kv = {
                {"yellow.h_low", std::to_string(std::max(0, std::min(179, yHLow)))},
                {"yellow.h_high", std::to_string(std::max(0, std::min(179, yHHigh)))},
                {"yellow.s_low", std::to_string(std::max(0, std::min(255, ySLow)))},
                {"yellow.v_low", std::to_string(std::max(0, std::min(255, yVLow)))},
                {"yellow.min_area_ratio", std::string(minAreaBuf)},
                {"yellow.lab_b_low", std::to_string(std::max(0, std::min(255, yLabBLow)))},
                {"yellow.lab_a_low", std::to_string(std::max(0, std::min(255, yALow)))},
                {"yellow.lab_a_high", std::to_string(std::max(0, std::min(255, yAHigh)))},
                {"yellow.combine_or", (yCombineOr ? std::string("true") : std::string("false"))}};
            bool ok = saveYamlKeys(cfgPathUsed, kv);
            if (ok)
            {
                std::cout << "✓ Yellow thresholds saved to " << cfgPathUsed << "\n";
            }
            else
            {
                std::cout << "✗ Failed to save yellow thresholds to " << cfgPathUsed << "\n";
            }
        }
    }

    cam.release();
    cv::destroyAllWindows();
    if (serial.isOpen())
        serial.close();
    if (csv.is_open())
        csv.close();
    if (espeakSessionActive)
    {
        espeak_Synchronize();
        espeak_Terminate();
    }
    std::cout << "Shutdown.\n";
    return 0;
}
