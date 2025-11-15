#include "yolo_detector.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>

// Obstacle class IDs from COCO dataset (objects that typically block path)
const std::vector<int> YoloDetector::OBSTACLE_CLASS_IDS = {
    0,  // person
    1,  // bicycle
    2,  // car
    3,  // motorcycle
    5,  // bus
    7,  // truck
    14, // bird
    15, // cat
    16, // dog
    17, // horse
    56, // chair
    57, // couch
    58, // potted plant
    60, // dining table
    62, // tv
    63, // laptop
    73, // book
    74, // clock
    75  // vase
};

YoloDetector::YoloDetector(const YoloConfig& config)
    : config_(config), initialized_(false) {}

YoloDetector::~YoloDetector() {}

bool YoloDetector::initialize()
{
    try {
        // Check if model file exists
        std::ifstream modelFile(config_.modelPath);
        if (!modelFile.good()) {
            std::cerr << "ERROR: Model file not found: " << config_.modelPath << std::endl;
            std::cerr << "Please download YOLOv5 ONNX model first!" << std::endl;
            return false;
        }
        
        // Create ONNX Runtime environment
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv5Detector");
        
        // Create session options
        sessionOptions_ = std::make_unique<Ort::SessionOptions>();
        sessionOptions_->SetIntraOpNumThreads(4); // Use 4 threads for inference
        sessionOptions_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        
        // Enable CUDA if requested and available
        if (config_.useCUDA) {
            try {
                OrtCUDAProviderOptions cudaOptions;
                sessionOptions_->AppendExecutionProvider_CUDA(cudaOptions);
                std::cout << "✓ CUDA enabled for YOLOv5" << std::endl;
            } catch (...) {
                std::cout << "⚠ CUDA not available, using CPU" << std::endl;
            }
        }
        
        // Create session
        #ifdef _WIN32
            std::wstring modelPathW(config_.modelPath.begin(), config_.modelPath.end());
            session_ = std::make_unique<Ort::Session>(*env_, modelPathW.c_str(), *sessionOptions_);
        #else
            session_ = std::make_unique<Ort::Session>(*env_, config_.modelPath.c_str(), *sessionOptions_);
        #endif
        
        // Get input/output info
        Ort::AllocatorWithDefaultOptions allocator;
        
        // Input info
        size_t numInputNodes = session_->GetInputCount();
        if (numInputNodes > 0) {
            inputNames_.push_back(session_->GetInputName(0, allocator));
            Ort::TypeInfo inputTypeInfo = session_->GetInputTypeInfo(0);
            auto tensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
            inputShape_ = tensorInfo.GetShape();
        }
        
        // Output info
        size_t numOutputNodes = session_->GetOutputCount();
        if (numOutputNodes > 0) {
            outputNames_.push_back(session_->GetOutputName(0, allocator));
            Ort::TypeInfo outputTypeInfo = session_->GetOutputTypeInfo(0);
            auto tensorInfo = outputTypeInfo.GetTensorTypeAndShapeInfo();
            outputShape_ = tensorInfo.GetShape();
        }
        
        initialized_ = true;
        std::cout << "✓ YOLOv5 model loaded: " << config_.modelPath << std::endl;
        std::cout << "  Input shape: [";
        for (size_t i = 0; i < inputShape_.size(); i++) {
            std::cout << inputShape_[i];
            if (i < inputShape_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        return true;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "ERROR: ONNX Runtime exception: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Exception initializing YOLOv5: " << e.what() << std::endl;
        return false;
    }
}

void YoloDetector::letterbox(const cv::Mat& src, cv::Mat& dst, const cv::Size& outSize)
{
    // Letterbox resize: maintain aspect ratio and pad
    int srcW = src.cols;
    int srcH = src.rows;
    int dstW = outSize.width;
    int dstH = outSize.height;
    
    float r = std::min(static_cast<float>(dstW) / srcW, 
                       static_cast<float>(dstH) / srcH);
    
    int newW = static_cast<int>(srcW * r);
    int newH = static_cast<int>(srcH * r);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(newW, newH));
    
    // Create padded image
    dst = cv::Mat(dstH, dstW, CV_8UC3, cv::Scalar(114, 114, 114));
    
    int padX = (dstW - newW) / 2;
    int padY = (dstH - newH) / 2;
    
    resized.copyTo(dst(cv::Rect(padX, padY, newW, newH)));
}

cv::Mat YoloDetector::preprocessImage(const cv::Mat& frame)
{
    cv::Mat blob;
    
    // Letterbox resize
    cv::Mat resized;
    letterbox(frame, resized, cv::Size(config_.inputWidth, config_.inputHeight));
    
    // Convert BGR to RGB
    cv::cvtColor(resized, blob, cv::COLOR_BGR2RGB);
    
    // Convert to float and normalize to [0, 1]
    blob.convertTo(blob, CV_32F, 1.0 / 255.0);
    
    return blob;
}

std::vector<YoloDetection> YoloDetector::detect(const cv::Mat& frame)
{
    if (!initialized_) {
        std::cerr << "ERROR: YOLOv5 not initialized!" << std::endl;
        return {};
    }
    
    try {
        // Preprocess
        cv::Mat blob = preprocessImage(frame);
        
        // Convert to CHW format (channels first)
        std::vector<cv::Mat> channels;
        cv::split(blob, channels);
        
        std::vector<float> inputData;
        inputData.reserve(config_.inputWidth * config_.inputHeight * 3);
        
        for (const auto& ch : channels) {
            inputData.insert(inputData.end(), (float*)ch.data, 
                           (float*)ch.data + config_.inputWidth * config_.inputHeight);
        }
        
        // Create input tensor
        std::vector<int64_t> inputShape = {1, 3, config_.inputHeight, config_.inputWidth};
        auto memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, inputData.data(), inputData.size(), 
            inputShape.data(), inputShape.size());
        
        // Run inference
        auto outputTensors = session_->Run(
            Ort::RunOptions{nullptr},
            inputNames_.data(), &inputTensor, 1,
            outputNames_.data(), 1);
        
        // Get output
        float* outputData = outputTensors[0].GetTensorMutableData<float>();
        auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
        
        int rows = static_cast<int>(outputShape[1]); // Number of detections
        
        std::vector<float> output(outputData, outputData + rows * outputShape[2]);
        
        // Postprocess
        return postprocess(output, rows, frame.size(), 
                          cv::Size(config_.inputWidth, config_.inputHeight));
        
    } catch (const Ort::Exception& e) {
        std::cerr << "ERROR: ONNX Runtime inference failed: " << e.what() << std::endl;
        return {};
    }
}

std::vector<YoloDetection> YoloDetector::postprocess(const std::vector<float>& output,
                                                      int rows,
                                                      const cv::Size& frameSize,
                                                      const cv::Size& inputSize)
{
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;
    
    // Parse detections
    int dimensions = static_cast<int>(output.size()) / rows;
    
    for (int i = 0; i < rows; ++i) {
        int index = i * dimensions;
        
        float confidence = output[index + 4]; // Objectness score
        
        if (confidence < config_.confidenceThreshold) {
            continue;
        }
        
        // Find class with highest score
        float maxClassScore = 0.0f;
        int maxClassId = -1;
        
        for (int j = 5; j < dimensions; ++j) {
            float classScore = output[index + j];
            if (classScore > maxClassScore) {
                maxClassScore = classScore;
                maxClassId = j - 5;
            }
        }
        
        float finalConfidence = confidence * maxClassScore;
        
        if (finalConfidence < config_.confidenceThreshold) {
            continue;
        }
        
        // Get box coordinates (center_x, center_y, width, height)
        float cx = output[index + 0];
        float cy = output[index + 1];
        float w = output[index + 2];
        float h = output[index + 3];
        
        // Convert to (x1, y1, x2, y2) format
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        
        // Scale to original image size
        float scaleX = static_cast<float>(frameSize.width) / inputSize.width;
        float scaleY = static_cast<float>(frameSize.height) / inputSize.height;
        
        x1 *= scaleX;
        y1 *= scaleY;
        w *= scaleX;
        h *= scaleY;
        
        cv::Rect box(static_cast<int>(x1), static_cast<int>(y1),
                     static_cast<int>(w), static_cast<int>(h));
        
        boxes.push_back(box);
        confidences.push_back(finalConfidence);
        classIds.push_back(maxClassId);
    }
    
    // Apply NMS
    std::vector<int> indices = nms(boxes, confidences, config_.nmsThreshold);
    
    // Create final detections
    std::vector<YoloDetection> detections;
    for (int idx : indices) {
        if (idx < static_cast<int>(boxes.size())) {
            int classId = classIds[idx];
            std::string className = (classId < static_cast<int>(COCO_CLASSES.size())) 
                                   ? COCO_CLASSES[classId] : "unknown";
            
            detections.emplace_back(classId, className, confidences[idx], boxes[idx]);
        }
    }
    
    return detections;
}

float YoloDetector::computeIOU(const cv::Rect& box1, const cv::Rect& box2)
{
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);
    
    int intersectionArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = box1.area() + box2.area() - intersectionArea;
    
    return (unionArea > 0) ? static_cast<float>(intersectionArea) / unionArea : 0.0f;
}

std::vector<int> YoloDetector::nms(const std::vector<cv::Rect>& boxes,
                                    const std::vector<float>& scores,
                                    float threshold)
{
    std::vector<std::pair<float, int>> scoreIndex;
    for (size_t i = 0; i < scores.size(); ++i) {
        scoreIndex.emplace_back(scores[i], static_cast<int>(i));
    }
    
    // Sort by score descending
    std::sort(scoreIndex.begin(), scoreIndex.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    std::vector<bool> suppressed(boxes.size(), false);
    std::vector<int> keep;
    
    for (size_t i = 0; i < scoreIndex.size(); ++i) {
        int idx = scoreIndex[i].second;
        
        if (suppressed[idx]) {
            continue;
        }
        
        keep.push_back(idx);
        
        for (size_t j = i + 1; j < scoreIndex.size(); ++j) {
            int idx2 = scoreIndex[j].second;
            
            if (suppressed[idx2]) {
                continue;
            }
            
            float iou = computeIOU(boxes[idx], boxes[idx2]);
            if (iou > threshold) {
                suppressed[idx2] = true;
            }
        }
    }
    
    return keep;
}

std::vector<YoloDetection> YoloDetector::detectSpecificClasses(const cv::Mat& frame,
                                                                 const std::vector<int>& classIds)
{
    auto allDetections = detect(frame);
    
    std::vector<YoloDetection> filtered;
    for (const auto& det : allDetections) {
        if (std::find(classIds.begin(), classIds.end(), det.classId) != classIds.end()) {
            filtered.push_back(det);
        }
    }
    
    return filtered;
}

std::vector<YoloDetection> YoloDetector::detectObstacles(const cv::Mat& frame)
{
    return detectSpecificClasses(frame, OBSTACLE_CLASS_IDS);
}

void YoloDetector::drawDetections(cv::Mat& frame, const std::vector<YoloDetection>& detections)
{
    for (const auto& det : detections) {
        // Draw bounding box
        cv::Scalar color = cv::Scalar(0, 255, 0); // Green
        cv::rectangle(frame, det.box, color, 2);
        
        // Draw label
        std::string label = det.className + " " + 
                           std::to_string(static_cast<int>(det.confidence * 100)) + "%";
        
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        
        int top = std::max(det.box.y, labelSize.height);
        cv::rectangle(frame, cv::Point(det.box.x, top - labelSize.height),
                     cv::Point(det.box.x + labelSize.width, top + baseLine),
                     color, cv::FILLED);
        
        cv::putText(frame, label, cv::Point(det.box.x, top),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
}

std::string YoloDetector::getModelInfo() const
{
    if (!initialized_) {
        return "Model not initialized";
    }
    
    std::string info = "YOLOv5 Model Info:\n";
    info += "  Path: " + config_.modelPath + "\n";
    info += "  Input: " + std::to_string(config_.inputWidth) + "x" + 
            std::to_string(config_.inputHeight) + "\n";
    info += "  Confidence threshold: " + std::to_string(config_.confidenceThreshold) + "\n";
    info += "  NMS threshold: " + std::to_string(config_.nmsThreshold) + "\n";
    
    return info;
}

