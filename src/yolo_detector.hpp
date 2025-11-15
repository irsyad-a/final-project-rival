#pragma once
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>
#include <memory>

// ==================== YOLO Detection Result ====================
struct YoloDetection {
    int classId;
    std::string className;
    float confidence;
    cv::Rect box;
    
    YoloDetection() : classId(-1), confidence(0.0f) {}
    YoloDetection(int id, const std::string& name, float conf, const cv::Rect& bbox)
        : classId(id), className(name), confidence(conf), box(bbox) {}
};

// ==================== YOLO Class Names (COCO Dataset) ====================
static const std::vector<std::string> COCO_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

// ==================== YOLOv5 Detector Configuration ====================
struct YoloConfig {
    std::string modelPath;          // Path to ONNX model file
    float confidenceThreshold;      // Minimum confidence for detection
    float nmsThreshold;             // Non-maximum suppression threshold
    int inputWidth;                 // Model input width (typically 640)
    int inputHeight;                // Model input height (typically 640)
    bool useCUDA;                   // Use CUDA if available
    
    YoloConfig()
        : modelPath("models/yolov5s.onnx"),
          confidenceThreshold(0.45f),
          nmsThreshold(0.50f),
          inputWidth(640),
          inputHeight(640),
          useCUDA(false) {}
};

// ==================== YOLOv5 Detector ====================
class YoloDetector {
public:
    explicit YoloDetector(const YoloConfig& config);
    ~YoloDetector();
    
    // Initialize the detector (load model)
    bool initialize();
    
    // Detect objects in frame
    std::vector<YoloDetection> detect(const cv::Mat& frame);
    
    // Filter detections by class (e.g., only detect persons, chairs, etc.)
    std::vector<YoloDetection> detectSpecificClasses(const cv::Mat& frame, 
                                                      const std::vector<int>& classIds);
    
    // Detect only obstacles (things that block the path)
    std::vector<YoloDetection> detectObstacles(const cv::Mat& frame);
    
    // Draw detections on frame
    void drawDetections(cv::Mat& frame, const std::vector<YoloDetection>& detections);
    
    // Check if model is loaded
    bool isInitialized() const { return initialized_; }
    
    // Get model info
    std::string getModelInfo() const;
    
private:
    YoloConfig config_;
    bool initialized_;
    
    // ONNX Runtime session
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::SessionOptions> sessionOptions_;
    
    // Model info
    std::vector<const char*> inputNames_;
    std::vector<const char*> outputNames_;
    std::vector<int64_t> inputShape_;
    std::vector<int64_t> outputShape_;
    
    // Preprocessing
    cv::Mat preprocessImage(const cv::Mat& frame);
    void letterbox(const cv::Mat& src, cv::Mat& dst, const cv::Size& outSize);
    
    // Postprocessing
    std::vector<YoloDetection> postprocess(const std::vector<float>& output,
                                            int rows,
                                            const cv::Size& frameSize,
                                            const cv::Size& inputSize);
    
    // Non-maximum suppression
    std::vector<int> nms(const std::vector<cv::Rect>& boxes,
                         const std::vector<float>& scores,
                         float threshold);
    
    // Utility
    float computeIOU(const cv::Rect& box1, const cv::Rect& box2);
    
    // Obstacle class IDs (things that typically block robot's path)
    static const std::vector<int> OBSTACLE_CLASS_IDS;
};

