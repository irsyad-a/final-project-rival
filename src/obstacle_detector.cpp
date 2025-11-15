#include "obstacle_detector.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

// ==================== Obstacle Detector Implementation ====================

ObstacleDetector::ObstacleDetector(const ObstacleDetectorConfig& config)
    : config_(config) {}

void ObstacleDetector::updateConfig(const ObstacleDetectorConfig& config)
{
    config_ = config;
}

std::vector<cv::Rect> ObstacleDetector::detectByEdges(const cv::Mat& frame)
{
    cv::Mat gray, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);
    
    // Canny edge detection
    cv::Canny(gray, edges, config_.cannyThreshold1, config_.cannyThreshold2);
    
    // Morphological operations to connect edges
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(edges, edges, kernel);
    cv::erode(edges, edges, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    return filterContours(contours, frame.cols, frame.rows);
}

cv::Scalar ObstacleDetector::getDominantColor(const cv::Mat& frame, const cv::Rect& roi)
{
    cv::Mat roiMat = frame(roi);
    cv::Scalar meanColor = cv::mean(roiMat);
    return meanColor;
}

bool ObstacleDetector::isFloorColor(const cv::Scalar& color)
{
    // Convert BGR to HSV for better color classification
    cv::Mat colorMat(1, 1, CV_8UC3, color);
    cv::Mat hsvMat;
    cv::cvtColor(colorMat, hsvMat, cv::COLOR_BGR2HSV);
    
    cv::Vec3b hsv = hsvMat.at<cv::Vec3b>(0, 0);
    
    // Floor is typically gray/white (low saturation, medium-high value)
    // or brownish (certain hue range)
    bool isGrayish = (hsv[1] < 50);  // Low saturation
    bool isBrownish = (hsv[0] >= 10 && hsv[0] <= 30 && hsv[1] > 30);  // Brown hue
    
    return isGrayish || isBrownish;
}

bool ObstacleDetector::isYellowBoundary(const cv::Scalar& color)
{
    cv::Mat colorMat(1, 1, CV_8UC3, color);
    cv::Mat hsvMat;
    cv::cvtColor(colorMat, hsvMat, cv::COLOR_BGR2HSV);
    
    cv::Vec3b hsv = hsvMat.at<cv::Vec3b>(0, 0);
    
    // Yellow: H in [15, 45], S > 40, V > 50
    return (hsv[0] >= 15 && hsv[0] <= 45 && hsv[1] > 40 && hsv[2] > 50);
}

std::vector<cv::Rect> ObstacleDetector::detectByColor(const cv::Mat& frame)
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // Detect non-floor colors (objects that stand out)
    cv::Mat mask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    
    // Focus on middle-bottom region where obstacles would appear
    int roiTop = static_cast<int>(frame.rows * 0.3);
    int roiBottom = static_cast<int>(frame.rows * 0.9);
    cv::Mat roiHsv = hsv.rowRange(roiTop, roiBottom);
    cv::Mat roiBgr = frame.rowRange(roiTop, roiBottom);
    
    // Split into channels
    std::vector<cv::Mat> channels;
    cv::split(roiHsv, channels);
    
    // High saturation objects (colorful objects = likely obstacles, not floor)
    cv::Mat highSat;
    cv::threshold(channels[1], highSat, 80, 255, cv::THRESH_BINARY);
    
    // Exclude yellow (boundary lines)
    cv::Mat yellowMask;
    cv::inRange(roiHsv, cv::Scalar(15, 40, 50), cv::Scalar(45, 255, 255), yellowMask);
    cv::bitwise_not(yellowMask, yellowMask);
    cv::bitwise_and(highSat, yellowMask, highSat);
    
    // Morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(highSat, highSat, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(highSat, highSat, cv::MORPH_OPEN, kernel);
    
    // Copy to full mask
    highSat.copyTo(mask.rowRange(roiTop, roiBottom));
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    return filterContours(contours, frame.cols, frame.rows);
}

std::vector<cv::Rect> ObstacleDetector::detectByMotion(const cv::Mat& frame)
{
    if (!config_.enableMotionDetection) {
        return {};
    }
    
    // Store current frame in history
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    
    frameHistory_.push_back(gray.clone());
    if (frameHistory_.size() > static_cast<size_t>(config_.motionHistoryFrames)) {
        frameHistory_.pop_front();
    }
    
    if (frameHistory_.size() < 2) {
        return {};  // Need at least 2 frames
    }
    
    // Calculate frame difference
    cv::Mat diff;
    cv::absdiff(frameHistory_.back(), frameHistory_.front(), diff);
    
    // Threshold
    cv::Mat motionMask;
    cv::threshold(diff, motionMask, 25, 255, cv::THRESH_BINARY);
    
    // Morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(motionMask, motionMask, cv::MORPH_OPEN, kernel);
    cv::dilate(motionMask, motionMask, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(motionMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    return filterContours(contours, frame.cols, frame.rows);
}

std::vector<cv::Rect> ObstacleDetector::filterContours(const std::vector<std::vector<cv::Point>>& contours,
                                                         int frameWidth, int frameHeight)
{
    std::vector<cv::Rect> boxes;
    
    double minArea = config_.minAreaRatio * frameWidth * frameHeight;
    double maxArea = config_.maxAreaRatio * frameWidth * frameHeight;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        if (area < minArea || area > maxArea) {
            continue;
        }
        
        cv::Rect bbox = cv::boundingRect(contour);
        
        if (isValidObstacle(bbox, frameWidth, frameHeight)) {
            boxes.push_back(bbox);
        }
    }
    
    return boxes;
}

bool ObstacleDetector::isValidObstacle(const cv::Rect& bbox, int frameWidth, int frameHeight)
{
    // Check aspect ratio
    double aspectRatio = static_cast<double>(bbox.height) / static_cast<double>(bbox.width);
    
    if (aspectRatio < config_.minAspectRatio || aspectRatio > config_.maxAspectRatio) {
        return false;
    }
    
    // Check if in reasonable position (obstacles usually in middle-bottom of frame)
    double centerY = bbox.y + bbox.height / 2.0;
    double relativeY = centerY / frameHeight;
    
    if (relativeY < 0.3) {  // Too high in frame (likely not an obstacle)
        return false;
    }
    
    // Check width (obstacles shouldn't span entire width)
    double widthRatio = static_cast<double>(bbox.width) / frameWidth;
    if (widthRatio > 0.9) {
        return false;
    }
    
    return true;
}

std::vector<cv::Rect> ObstacleDetector::mergeOverlapping(const std::vector<cv::Rect>& boxes, 
                                                          double overlapThreshold)
{
    if (boxes.empty()) {
        return {};
    }
    
    std::vector<cv::Rect> merged;
    std::vector<bool> used(boxes.size(), false);
    
    for (size_t i = 0; i < boxes.size(); i++) {
        if (used[i]) {
            continue;
        }
        
        cv::Rect current = boxes[i];
        used[i] = true;
        
        // Find all overlapping boxes
        for (size_t j = i + 1; j < boxes.size(); j++) {
            if (used[j]) {
                continue;
            }
            
            cv::Rect intersection = current & boxes[j];
            double intersectArea = intersection.area();
            double minArea = std::min(current.area(), boxes[j].area());
            
            if (intersectArea / minArea > overlapThreshold) {
                // Merge boxes
                current = current | boxes[j];
                used[j] = true;
            }
        }
        
        merged.push_back(current);
    }
    
    return merged;
}

double ObstacleDetector::estimateDistance(const cv::Rect& bbox, int frameHeight)
{
    // Similar to yellow line distance estimation
    // Assume obstacle base is at bottom of bounding box
    int baseY = bbox.y + bbox.height;
    
    // Simple perspective projection
    // Closer to bottom = closer to camera
    double normalizedY = static_cast<double>(baseY) / frameHeight;
    
    // Convert camera tilt to radians
    double tiltRad = config_.cameraTiltAngle * M_PI / 180.0;
    
    // For camera tilted down (typical case)
    // Objects at bottom of frame are closer
    double fovRad = config_.cameraFovVertical * M_PI / 180.0;
    double pixelAngle = (normalizedY - 0.5) * fovRad;
    
    // Angle from horizontal
    double angleFromHorizontal = M_PI / 2.0 - tiltRad + pixelAngle;
    
    if (std::abs(angleFromHorizontal) < 0.01) {
        return 999.0;  // Too far or parallel
    }
    
    // Distance calculation
    double distanceCm = config_.cameraHeightCm / std::tan(angleFromHorizontal);
    
    // Clamp to reasonable range
    distanceCm = std::max(5.0, std::min(500.0, distanceCm));
    
    return distanceCm;
}

double ObstacleDetector::estimateHeight(const cv::Rect& bbox, double distance)
{
    // Estimate real-world height based on pixel height and distance
    // This is approximate and depends on camera parameters
    
    double fovRad = config_.cameraFovVertical * M_PI / 180.0;
    double pixelHeight = bbox.height;
    
    // Rough approximation: heightCm = distance * pixelHeight * tan(fov/2) / (frameHeight/2)
    // This is very approximate for monocular camera
    double estimatedHeight = distance * 0.02 * pixelHeight;  // Rough scaling factor
    
    return std::max(5.0, std::min(200.0, estimatedHeight));  // Clamp to reasonable range
}

double ObstacleDetector::calculateConfidence(const cv::Rect& bbox, const cv::Mat& frame)
{
    double confidence = 0.5;  // Base confidence
    
    // Factor 1: Size (larger = more confident)
    double area = bbox.area();
    double frameArea = frame.cols * frame.rows;
    double sizeRatio = area / frameArea;
    confidence += std::min(0.3, sizeRatio * 10.0);
    
    // Factor 2: Position (middle-bottom = more confident)
    double centerY = (bbox.y + bbox.height / 2.0) / frame.rows;
    if (centerY > 0.5 && centerY < 0.9) {
        confidence += 0.2;
    }
    
    // Clamp to [0, 1]
    return std::max(0.0, std::min(1.0, confidence));
}

ObstacleType ObstacleDetector::classifyObstacle(const DetectedObstacle& obstacle)
{
    // Check if obstacle has moved in recent history
    if (obstacleHistory_.size() < 3) {
        return ObstacleType::UNKNOWN;
    }
    
    // Simple heuristic: check if similar obstacle moved in previous frames
    int movementCount = 0;
    cv::Point2f currentPos = obstacle.centerPoint;
    
    for (const auto& prevObstacles : obstacleHistory_) {
        for (const auto& prevObs : prevObstacles) {
            double dx = currentPos.x - prevObs.centerPoint.x;
            double dy = currentPos.y - prevObs.centerPoint.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist > config_.motionThreshold) {
                movementCount++;
                break;
            }
        }
    }
    
    return (movementCount >= 2) ? ObstacleType::MOVING : ObstacleType::STATIC;
}

std::vector<DetectedObstacle> ObstacleDetector::detectObstacles(const cv::Mat& frame)
{
    std::vector<DetectedObstacle> obstacles;
    
    // Detect using multiple methods
    std::vector<cv::Rect> edgeBoxes = detectByEdges(frame);
    std::vector<cv::Rect> colorBoxes = detectByColor(frame);
    std::vector<cv::Rect> motionBoxes = detectByMotion(frame);
    
    // Combine all detections
    std::vector<cv::Rect> allBoxes;
    allBoxes.insert(allBoxes.end(), edgeBoxes.begin(), edgeBoxes.end());
    allBoxes.insert(allBoxes.end(), colorBoxes.begin(), colorBoxes.end());
    allBoxes.insert(allBoxes.end(), motionBoxes.begin(), motionBoxes.end());
    
    // Merge overlapping detections
    std::vector<cv::Rect> mergedBoxes = mergeOverlapping(allBoxes, 0.5);
    
    // Create DetectedObstacle objects
    for (const auto& bbox : mergedBoxes) {
        DetectedObstacle obs;
        obs.boundingBox = bbox;
        obs.centerPoint = cv::Point2f(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
        obs.distanceCm = estimateDistance(bbox, frame.rows);
        obs.heightCm = estimateHeight(bbox, obs.distanceCm);
        obs.confidence = calculateConfidence(bbox, frame);
        obs.dominantColor = getDominantColor(frame, bbox);
        obs.type = ObstacleType::UNKNOWN;  // Will be classified if motion detection enabled
        
        // Filter out yellow boundary false positives
        if (!isYellowBoundary(obs.dominantColor)) {
            obstacles.push_back(obs);
        }
    }
    
    // Store in history for tracking
    obstacleHistory_.push_back(obstacles);
    if (obstacleHistory_.size() > static_cast<size_t>(config_.motionHistoryFrames)) {
        obstacleHistory_.pop_front();
    }
    
    // Classify obstacles if motion detection enabled
    if (config_.enableMotionDetection) {
        for (auto& obs : obstacles) {
            obs.type = classifyObstacle(obs);
        }
    }
    
    return obstacles;
}

DetectedObstacle ObstacleDetector::getObstacleInDirection(const cv::Mat& frame, const std::string& direction)
{
    auto obstacles = detectObstacles(frame);
    
    DetectedObstacle nearest;
    nearest.distanceCm = 999.0;  // Very far
    
    int centerX = frame.cols / 2;
    int leftThreshold = frame.cols / 3;
    int rightThreshold = 2 * frame.cols / 3;
    
    for (const auto& obs : obstacles) {
        bool inDirection = false;
        
        if (direction == "forward") {
            // Forward = center region
            inDirection = (obs.centerPoint.x > leftThreshold && obs.centerPoint.x < rightThreshold);
        } else if (direction == "left") {
            inDirection = (obs.centerPoint.x < centerX);
        } else if (direction == "right") {
            inDirection = (obs.centerPoint.x > centerX);
        }
        
        if (inDirection && obs.distanceCm < nearest.distanceCm) {
            nearest = obs;
        }
    }
    
    return nearest;
}

bool ObstacleDetector::isPathClear(const cv::Mat& frame, double minSafeDistanceCm)
{
    DetectedObstacle forward = getObstacleInDirection(frame, "forward");
    return (forward.distanceCm > minSafeDistanceCm);
}

void ObstacleDetector::drawObstacles(cv::Mat& frame, const std::vector<DetectedObstacle>& obstacles)
{
    for (const auto& obs : obstacles) {
        // Color based on distance/threat
        cv::Scalar color;
        if (obs.distanceCm < config_.emergencyDistanceCm) {
            color = cv::Scalar(0, 0, 255);  // Red - emergency
        } else if (obs.distanceCm < config_.warningDistanceCm) {
            color = cv::Scalar(0, 165, 255);  // Orange - warning
        } else {
            color = cv::Scalar(0, 255, 0);  // Green - safe
        }
        
        // Draw bounding box
        cv::rectangle(frame, obs.boundingBox, color, 2);
        
        // Draw info text
        std::string info = std::to_string(static_cast<int>(obs.distanceCm)) + "cm";
        cv::putText(frame, info, 
                   cv::Point(obs.boundingBox.x, obs.boundingBox.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        
        // Draw center point
        cv::circle(frame, obs.centerPoint, 3, color, -1);
    }
}

// ==================== Collision Avoidance Implementation ====================

CollisionAvoidance::ThreatLevel CollisionAvoidance::assessThreat(
    const std::vector<DetectedObstacle>& obstacles,
    const ObstacleDetectorConfig& config)
{
    if (obstacles.empty()) {
        return ThreatLevel::SAFE;
    }
    
    // Find nearest obstacle with high confidence
    double minDistance = 999.0;
    for (const auto& obs : obstacles) {
        if (obs.confidence > 0.5 && obs.distanceCm < minDistance) {
            minDistance = obs.distanceCm;
        }
    }
    
    if (minDistance < config.emergencyDistanceCm) {
        return ThreatLevel::EMERGENCY;
    } else if (minDistance < config.warningDistanceCm) {
        return ThreatLevel::DANGER;
    } else if (minDistance < config.safeDistanceCm) {
        return ThreatLevel::WARNING;
    }
    
    return ThreatLevel::SAFE;
}

std::string CollisionAvoidance::getAvoidanceAction(ThreatLevel threat)
{
    switch (threat) {
        case ThreatLevel::EMERGENCY:
            return "STOP";
        case ThreatLevel::DANGER:
            return "MUNDUR 20 cm";
        case ThreatLevel::WARNING:
            return "MAJU PELAN";
        case ThreatLevel::SAFE:
        default:
            return "CLEAR";
    }
}

DetectedObstacle CollisionAvoidance::findNearestObstacle(const std::vector<DetectedObstacle>& obstacles)
{
    DetectedObstacle nearest;
    nearest.distanceCm = 999.0;
    
    for (const auto& obs : obstacles) {
        if (obs.distanceCm < nearest.distanceCm) {
            nearest = obs;
        }
    }
    
    return nearest;
}

