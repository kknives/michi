#include <algorithm>
#include <iterator>

#include <onnxruntime_cxx_api.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/video.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include "mobilenet_arrow.hpp"
#include "classification_model.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    return -1;
  }
  std::string vidf(argv[1]);
    // Open the video stream
    cv::VideoCapture cap(vidf);
    
    // Check if the video opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file." << std::endl;
        return -1;
    }

    // Get the video's frames per second (fps) and frame size
    double fps = cap.get(cv::CAP_PROP_FPS);
    cv::Size frame_size(640, 480);  // Set the desired frame size

    // Create a VideoWriter object to write the output video
  cv::VideoWriter video_writer("output_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame_size, true);

    // Check if the VideoWriter opened successfully
    if (!video_writer.isOpened()) {
        std::cerr << "Error opening video writer." << std::endl;
        return -1;
    }

  auto mac = ClassificationModel(MobilenetArrowClassifier::make_waseem2_model("lib/w_model2.onnx"));
    // Process each frame
    cv::Mat frame;
    while (cap.read(frame)) {
        // Resize the frame to 640x480
        cv::resize(frame, frame, frame_size);

        // Apply bounding box
        if (cv::Mat copy_frame = frame.clone(); classify(mac, copy_frame, 0.6) != ClassificationModel::Detection::NONE)
        {
          std::cout << "Rectangle" << '\n';
          auto bb = get_bounding_box(mac);
          cv::Rect bounding_box(640*bb[1], 480*bb[0], (bb[3]-bb[1])*640, (bb[2] - bb[0])*480); // Example bounding box (x, y, width, height)
          cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);
          std::string text = "LEFT";
          int font = cv::FONT_HERSHEY_SIMPLEX;
          double font_scale = 0.5;
          int thickness = 1;
          int baseline = 0;
          cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
          cv::Point text_org(bounding_box.x + (bounding_box.width - text_size.width) / 2, bounding_box.y + bounding_box.height + text_size.height + 5);
          cv::putText(frame, text, text_org, font, font_scale, cv::Scalar(255, 255, 255), thickness);
        }

        // Write the frame to the output video
        video_writer.write(frame);
    }

    // Release the VideoCapture and VideoWriter
    cap.release();
    video_writer.release();

    return 0;
}
