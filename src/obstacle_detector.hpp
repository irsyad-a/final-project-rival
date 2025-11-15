#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>

// ==================== Obstacle Type ====================
enum class ObstacleType {
    STATIC,      // Stationary obstacle
    MOVING,      // Moving obstacle
    UNKNOWN      // Cannot determine
};

// ==================== Detected Obstacle ====================
struct DetectedObstacle {
    cv::Rect boundingBox;        // Bounding box in frame
    cv::Point2f centerPoint;     // Center point in frame
    double distanceCm;           // Estimated distance in cm
    double heightCm;             // Estimated height in cm
    double confidence;           // Detection confidence (0-1)
    ObstacleType type;           // Static or moving
    cv::Scalar dominantColor;    // Dominant color (for tracking)
    
    DetectedObstacle()
        : distanceCm(0.0), heightCm(0.0), confidence(0.0), type(ObstacleType::UNKNOWN) {}
};

// ==================== Obstacle Detector Configuration ====================
struct ObstacleDetectorConfig {
    // Camera parameters
    double cameraHeightCm;       // Camera height from ground
    double cameraFovVertical;    // Vertical FOV in degrees
    double cameraTiltAngle;      // Tilt angle (90 = straight down, 0 = horizontal)
    
    // Detection parameters
    int cannyThreshold1;         // Canny edge detection threshold 1
    int cannyThreshold2;         // Canny edge detection threshold 2
    double minAreaRatio;         // Minimum obstacle area ratio (of frame)
    double maxAreaRatio;         // Maximum obstacle area ratio
    double minAspectRatio;       // Minimum height/width ratio
    double maxAspectRatio;       // Maximum height/width ratio
    
    // Distance thresholds
    double emergencyDistanceCm;  // Emergency stop distance
    double warningDistanceCm;    // Warning distance
    double safeDistanceCm;       // Safe distance
    
    // Motion detection
    bool enableMotionDetection;  // Enable motion detection
    int motionHistoryFrames;     // Number of frames for motion history
    double motionThreshold;      // Motion threshold (pixels)
    
    ObstacleDetectorConfig()
        : cameraHeightCm(10.0), cameraFovVertical(50.0), cameraTiltAngle(90.0),
          cannyThreshold1(50), cannyThreshold2(150),
          minAreaRatio(0.005), maxAreaRatio(0.5),
          minAspectRatio(0.5), maxAspectRatio(5.0),
          emergencyDistanceCm(20.0), warningDistanceCm(40.0), safeDistanceCm(60.0),
          enableMotionDetection(false), motionHistoryFrames(5), motionThreshold(5.0) {}
};

// ==================== Obstacle Detector ====================
class ObstacleDetector {
public:
    explicit ObstacleDetector(const ObstacleDetectorConfig& config);
    
    // Main detection function
    std::vector<DetectedObstacle> detectObstacles(const cv::Mat& frame);
    
    // Get obstacle in specific direction (forward, left, right)
    DetectedObstacle getObstacleInDirection(const cv::Mat& frame, const std::string& direction);
    
    // Check if path ahead is clear
    bool isPathClear(const cv::Mat& frame, double minSafeDistanceCm);
    
    // Distance estimation for an obstacle
    double estimateDistance(const cv::Rect& bbox, int frameHeight);
    double estimateHeight(const cv::Rect& bbox, double distance);
    
    // Visualization
    void drawObstacles(cv::Mat& frame, const std::vector<DetectedObstacle>& obstacles);
    
    // Update configuration
    void updateConfig(const ObstacleDetectorConfig& config);
    
private:
    ObstacleDetectorConfig config_;
    std::deque<cv::Mat> frameHistory_;  // For motion detection
    std::deque<std::vector<DetectedObstacle>> obstacleHistory_;  // Track obstacles over time
    
    // Detection methods
    std::vector<cv::Rect> detectByEdges(const cv::Mat& frame);
    std::vector<cv::Rect> detectByColor(const cv::Mat& frame);
    std::vector<cv::Rect> detectByMotion(const cv::Mat& frame);
    
    // Filtering and validation
    std::vector<cv::Rect> filterContours(const std::vector<std::vector<cv::Point>>& contours,
                                          int frameWidth, int frameHeight);
    bool isValidObstacle(const cv::Rect& bbox, int frameWidth, int frameHeight);
    
    // Color analysis
    cv::Scalar getDominantColor(const cv::Mat& frame, const cv::Rect& roi);
    bool isFloorColor(const cv::Scalar& color);
    bool isYellowBoundary(const cv::Scalar& color);
    
    // Obstacle tracking and classification
    ObstacleType classifyObstacle(const DetectedObstacle& obstacle);
    double calculateConfidence(const cv::Rect& bbox, const cv::Mat& frame);
    
    // Merge nearby detections
    std::vector<cv::Rect> mergeOverlapping(const std::vector<cv::Rect>& boxes, double overlapThreshold = 0.5);
};

// ==================== Collision Avoidance Helper ====================
class CollisionAvoidance {
public:
    enum class ThreatLevel {
        SAFE,       // No threat
        WARNING,    // Approaching obstacle
        DANGER,     // Close to obstacle
        EMERGENCY   // Immediate collision risk
    };
    
    static ThreatLevel assessThreat(const std::vector<DetectedObstacle>& obstacles,
                                     const ObstacleDetectorConfig& config);
    
    static std::string getAvoidanceAction(ThreatLevel threat);
    
    static DetectedObstacle findNearestObstacle(const std::vector<DetectedObstacle>& obstacles);
};

