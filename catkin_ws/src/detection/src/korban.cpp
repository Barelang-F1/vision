#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>

#include <jetson-inference/detectNet.h>
#include <jetson-inference/objectTracker.h>

#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/String.h> // Include for publishing strings

bool signal_recieved = false;

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        LogVerbose("received SIGINT\n");
        signal_recieved = true;
    }
}

int usage()
{
    printf("usage: detectnet [--help] [--network=NETWORK] [--threshold=THRESHOLD] ...\n");
    printf("                 input [output]\n\n");
    printf("Locate objects in a video/image stream using an object detection DNN.\n");
    printf("See below for additional arguments that may not be shown above.\n\n");
    printf("positional arguments:\n");
    printf("    input           resource URI of input stream  (see videoSource below)\n");
    printf("    output          resource URI of output stream (see videoOutput below)\n\n");

    printf("%s", detectNet::Usage());
    printf("%s", objectTracker::Usage());
    printf("%s", videoSource::Usage());
    printf("%s", videoOutput::Usage());
    printf("%s", Log::Usage());

    return 0;
}

int main(int argc, char **argv)
{
    /*
     * Initialize ROS
     */
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::Publisher detection_pub = nh.advertise<std_msgs::String>("detections", 1000);

    /*
     * parse command line
     */
    commandLine cmdLine(argc, argv);

    if (cmdLine.GetFlag("help"))
        return usage();

    /*
     * attach signal handler
     */
    if (signal(SIGINT, sig_handler) == SIG_ERR)
        LogError("can't catch SIGINT\n");

    videoOptions options;
    options.width = 300;
    options.height = 300;
    options.frameRate = 30;

    videoSource *input = videoSource::Create("/dev/video0", options);

    if (!input)
    {
        LogError("detectnet:  failed to create input stream\n");
        return 1;
    }

    /*
     * create output stream
     */
    videoOutput *output = videoOutput::Create(cmdLine, ARG_POSITION(1));

    if (!output)
    {
        LogError("detectnet:  failed to create output stream\n");
        return 1;
    }

    /*
     * create detection network
     */
    detectNet *net = detectNet::Create(NULL, "/home/bf1/jetson-inference/python/training/detection/ssd/models/test/ssd-mobilenet.onnx", NULL, "/home/bf1/jetson-inference/python/training/detection/ssd/models/test/labels.txt", 0.5f, "input_0", "scores", "boxes");

    if (!net)
    {
        LogError("detectnet:  failed to load detectNet model\n");
        return 1;
    }

    // parse overlay flags
    const uint32_t overlayFlags = detectNet::OverlayFlagsFromStr(cmdLine.GetString("overlay", "box,labels,conf"));

    /*
     * processing loop
     */
    while (!signal_recieved)
    {
        // capture next image
        uchar3 *image = NULL;
        int status = 0;

        if (!input->Capture(&image, &status))
        {
            if (status == videoSource::TIMEOUT)
                continue;

            break; // EOS
        }

        // detect objects in the frame
        detectNet::Detection *detections = NULL;
        const int numDetections = net->Detect(image, input->GetWidth(), input->GetHeight(), &detections, overlayFlags);

        std::ostringstream oss; // For collecting detection results as a string

        if (numDetections > 0)
        {
            LogVerbose("%i objects detected\n", numDetections);

            for (int n = 0; n < numDetections; n++)
            {
                //LogVerbose("\ndetected obj %i  class #%u (%s)  confidence=%f\n", n, detections[n].ClassID, net->GetClassDesc(detections[n].ClassID), detections[n].Confidence);
                //LogVerbose("bounding box %i  (%.2f, %.2f)  (%.2f, %.2f)  w=%.2f  h=%.2f\n", n, detections[n].Left, detections[n].Top, detections[n].Right, detections[n].Bottom, detections[n].Width(), detections[n].Height());

                // Calculate and log center of bounding box
                float centerX = (detections[n].Left + detections[n].Right) / 2;
                //float centerY = (detections[n].Top + detections[n].Bottom) / 2;

                const std::string class_name = net->GetClassDesc(detections[n].ClassID);
                if (class_name == "Korban")
                {
                    LogVerbose("Center of bounding box Korban %i: (%.2f)\n", n, centerX);
                    // Add detection info to ROS message
                    oss << "Detected: " << class_name << " at (" << centerX << ")\n";
                }

                if (detections[n].TrackID >= 0) // is this a tracked object?
                    LogVerbose("tracking  ID %i  status=%i  frames=%i  lost=%i\n", detections[n].TrackID, detections[n].TrackStatus, detections[n].TrackFrames, detections[n].TrackLost);
            }

            // Publish the detection results as a string
            std_msgs::String msg;
            msg.data = oss.str();
            detection_pub.publish(msg);
        }

        // render outputs
        if (output != NULL)
        {
            output->Render(image, input->GetWidth(), input->GetHeight());

            // update the status bar
            char str[256];
            sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
            output->SetStatus(str);

            // check if the user quit
            if (!output->IsStreaming())
                break;
        }

        // print out timing info
        net->PrintProfilerTimes();
    }

    /*
     * destroy resources
     */
    LogVerbose("detectnet:  shutting down...\n");

    SAFE_DELETE(input);
    SAFE_DELETE(output);
    SAFE_DELETE(net);

    LogVerbose("detectnet:  shutdown complete.\n");
    return 0;
}
