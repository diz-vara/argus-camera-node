#include <iostream>
#include <iomanip>
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <unistd.h>
#include <exception>
#include <experimental/filesystem>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <cmath>
#include <pthread.h>
#include <atomic>
#include "cxxopts.hpp"
#include <algorithm>
#include <NvJpegEncoder.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <fstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <camera_info_manager/camera_info_manager.h>

#include <camera_calibration_parsers/parse_ini.h>



#include "argus-cam.h"

using namespace std;
using namespace Argus;
using namespace EGLStream;
namespace fs = std::experimental::filesystem;

constexpr double NANO = 1e-9;

namespace {
    template<class T>
    void CheckNotZero(T val, const char * msg)
    {
        if (val == 0)
        {
            ROS_FATAL_STREAM(msg);
            throw runtime_error(msg);
        }
    }

    template<class T>
    void CheckZero(T val, const char * msg)
    {
        if (val != 0)
        {
            ROS_FATAL_STREAM(msg);
            throw runtime_error(msg);
        }
    }

#ifndef USE_GPIO
# define USE_GPIO 0
#endif


#if USE_GPIO
    FILE *io = NULL, *iodir = NULL, *ioedge = NULL;
    int iovalfd = -1;
    constexpr int GPIO_BUTTON  = 388;

    constexpr double NANO = 1e-9;

    atomic_bool runGPIO;
    atomic<double> lastGPIOTriggerTime;

    void CloseGpioInput(int gpio)
    {
        if (io != NULL)
        {
            fclose(io);
            io = NULL;
        }
        if (iodir != NULL)
        {
            fclose(iodir);
            iodir = NULL;

        }
        if (iovalfd >= 0)
        {
            close(iovalfd);
            iovalfd = -1;
        }
        if (ioedge != NULL)
        {
            fclose(ioedge);
            ioedge = NULL;
        }

        io = fopen("/sys/class/gpio/unexport", "w");
        fseek(io,0,SEEK_SET);
        fprintf(io,"%d",gpio);
        fflush(io);
        fclose(io);
    }

    bool OpenGpioInput(int gpio)
    {
        io = fopen("/sys/class/gpio/export", "w");
        if(io == NULL)
        {
            fprintf(stdout, "could not open gpio/export (error %d\r\n", errno);
            return false;
        }

        fseek(io,0,SEEK_SET);
        fprintf(io,"%d",gpio);
        fflush(io);

        char str[64];
        sprintf(str, "/sys/class/gpio/gpio%d/direction", gpio);
        iodir = fopen(str, "w");
        if(iodir == NULL)
        {
            CloseGpioInput(gpio);
            fprintf(stdout, "could not open %s (error %d\r\n", str, errno);
            return false;
        }

        fseek(iodir,0,SEEK_SET);
        fprintf(iodir,"in"); // in / out
        fflush(iodir);

        sprintf(str, "/sys/class/gpio/gpio%d/edge", gpio);
        ioedge = fopen(str, "w");
        if(ioedge == NULL)
        {
            CloseGpioInput(gpio);
            fprintf(stdout, "could not open %s (error %d\r\n", str, errno);
            return false;
        }

        fseek(ioedge,0,SEEK_SET);
        fprintf(ioedge,"falling"); // rising / falling
        fflush(ioedge);

        sprintf(str, "/sys/class/gpio/gpio%d/value", gpio);
        iovalfd = open(str, O_RDONLY);
        if(iovalfd  < 0)
        {
            CloseGpioInput(gpio);
            fprintf(stdout, "could not open %s (error %d\r\n", str, errno);
            return false;
        }
        // first read needed to enable POLL
        char val;
        int nRead = read(iovalfd, &val, sizeof(val));
        if(nRead < 1)
        {
            CloseGpioInput(gpio);
            fprintf(stdout, "could not read %s (error %d\r\n", str, errno);
            return false;
        }

        if(io == NULL || iodir == NULL || iovalfd < 0 || ioedge == NULL)
        {
            CloseGpioInput(gpio);
            return false;
        }

        return true;
    }

    // Waits for zero level on GPIO
    // returns false on timeout/error. TimeoutMs value must be in range 0...5000 ms
    bool WaitZeroLevel(int TimeoutMs)
    {
        struct pollfd pollfd[1];
        char c;
        int err;

        pollfd[0].fd = iovalfd;
        pollfd[0].events = POLLPRI | POLLERR;
        pollfd[0].revents = 0;

        err =  poll(pollfd, 1, TimeoutMs);
        if(err != 1)
        {
            cout << "poll error: " << errno << endl;
            return false;
        }

        lseek(iovalfd, 0, SEEK_SET);
        err = read(iovalfd, &c, sizeof(c));
        if(err < 1)
        {
            cout << "read ioval error: " << errno << endl;
            return false;
        }

        if (c != '0')
        {
            cout << "GPIO value not zero: " << c << endl;
            return false;
        }

        return true;
    }
#endif

    double GetSysTime()
    {
        struct timespec now;
        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
            throw runtime_error("timer error");
        double sysTime = now.tv_sec * 1.0 + now.tv_nsec * NANO;
        return sysTime;
    }

    double ToRealTime(double monotonic)
    {
        struct timespec nowMonotonic;
        if (clock_gettime(CLOCK_MONOTONIC, &nowMonotonic) != 0) {
            ROS_FATAL_STREAM("timer error");
            throw runtime_error("timer error");
        }

        struct timespec nowReal;
        if (clock_gettime(CLOCK_REALTIME, &nowReal) != 0) {
            ROS_FATAL_STREAM("timer error");
            throw runtime_error("timer error");
        }

        double nowMonoSec = nowMonotonic.tv_sec * 1.0 + nowMonotonic.tv_nsec * NANO;
        double nowRealSec = nowReal.tv_sec * 1.0 + nowReal.tv_nsec * NANO;
        double diff = nowRealSec - nowMonoSec;
        return monotonic + diff;
    }


    constexpr bool enableTimer = false;
    UniqueObj<Request> CreateRequest(ICaptureSession * iCaptureSession,
                                     OutputStream * stream,
                                     const CameraModeInfo & cameraModeInfo,
                                     double maxExposureDurationSec,
                                     double frameDurationSec,
                                     int maxGain)
    {
        Argus::Status status;

        UniqueObj<Request> request(iCaptureSession->createRequest(CAPTURE_INTENT_PREVIEW));
        IRequest * iRequest = interface_cast<IRequest>(request);
        CheckNotZero(iRequest, "Failed to create Request");

        ISourceSettings * iSourceSettings =
                interface_cast<ISourceSettings>(iRequest->getSourceSettings());
        CheckNotZero(iSourceSettings, "getSourceSettings error");

        uint64_t maxExposureDurationNanoSec = (uint64_t) std::round(maxExposureDurationSec / NANO);
        maxExposureDurationNanoSec = std::min(
                    maxExposureDurationNanoSec,
                    cameraModeInfo.exposureRange.second);
        maxExposureDurationNanoSec = std::max(
                    maxExposureDurationNanoSec,
                    cameraModeInfo.exposureRange.first);

        status = iSourceSettings->setExposureTimeRange(
                    Range<uint64_t>(cameraModeInfo.exposureRange.first, maxExposureDurationNanoSec));
        CheckZero(status, "setExposureTimeRange error");

        uint64_t frameDurationNanoSec = (uint64_t) std::round(frameDurationSec / NANO);
        frameDurationNanoSec = std::min(
                    frameDurationNanoSec,
                    cameraModeInfo.frameDurationRange.second);
        frameDurationNanoSec = std::max(
                    frameDurationNanoSec,
                    cameraModeInfo.frameDurationRange.first);

        status = iSourceSettings->setFrameDurationRange(
                    Range<uint64_t>(frameDurationNanoSec, frameDurationNanoSec));

        float maxGainValue(maxGain);
        maxGainValue = std::min(maxGainValue,
                                     cameraModeInfo.gainRange.second);
        maxGainValue = std::max(maxGainValue,
                                     cameraModeInfo.gainRange.first);

        CheckZero(status, "setFrameDurationRange error");
        status = iSourceSettings->setGainRange(
                    Range<float>(cameraModeInfo.gainRange.first, maxGainValue));
        CheckZero(status, "setGain error");

        ROS_INFO_STREAM("maxExposureDurationNanoSec " << maxExposureDurationNanoSec);
        ROS_INFO_STREAM("frameDurationNanoSec " << frameDurationNanoSec);
        ROS_INFO_STREAM("maxGain " << maxGain);

        // Enable stream in the request.
        status = iRequest->enableOutputStream(stream);
        CheckZero(status, "enableOutputStream error");

        return std::move(request);
    }



    void FillCameraModeInfo(CameraModeInfo & cameraModeInfo,
                            ISensorMode * iSensorMode)
    {
        cameraModeInfo.exposureRange = make_pair(
                    iSensorMode->getExposureTimeRange().min(),
                    iSensorMode->getExposureTimeRange().max());
        cameraModeInfo.frameDurationRange = make_pair(
                    iSensorMode->getFrameDurationRange().min(),
                    iSensorMode->getFrameDurationRange().max());
        cameraModeInfo.gainRange = make_pair(
                    iSensorMode->getAnalogGainRange().min(),
                    iSensorMode->getAnalogGainRange().max());
        cameraModeInfo.resolutionWidthHeight = make_pair(
                    iSensorMode->getResolution().width(),
                    iSensorMode->getResolution().height());
    }



    void RunCameraDevice(ros::NodeHandle nh)
    {
        Argus::Status status;

        camera_info_manager::CameraInfoManager camera_info_manager_(nh);

        int device = nh.param("device",1);
        double maxExposureTimeSec = nh.param("max_exposure_time",0.001);
        double frameDurationSec = nh.param("frame_duration",0.1);
        int maxGain = nh.param("max_gain",8);
        int width = nh.param("width",1280);
        int height = nh.param("height", 720);


        ROS_INFO_STREAM("device" << device);
        ROS_INFO_STREAM("max_exposure_time" << maxExposureTimeSec);
        ROS_INFO_STREAM("max_gain" << maxGain);


        // Initialize the Argus camera provider.
        UniqueObj<CameraProvider> cameraProvider(CameraProvider::create());

        // Get the ICameraProvider interface from the global CameraProvider.
        ICameraProvider * iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
        CheckNotZero(iCameraProvider, "Failed to get ICameraProvider interface");
        ROS_INFO_STREAM("Argus Version: " << iCameraProvider->getVersion().c_str());

        // Get the camera devices.
        vector<CameraDevice*> cameraDevices;
        iCameraProvider->getCameraDevices(&cameraDevices);
        ROS_INFO_STREAM("Found " << cameraDevices.size() << " cameras\n");
        CheckNotZero(cameraDevices.size(), "No cameras found");

        ICameraProperties * iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[0]);
        CheckNotZero(iCameraProperties, "iCameraProperties error");
        vector<SensorMode*> modes;
        iCameraProperties->getAllSensorModes(&modes);
        CheckNotZero(modes.size(), "get camera modes error");
        CameraModeInfo cameraModeInfo;
        for (int i = 0; i < modes.size(); i++)
        {
            SensorMode * mode = modes[i];
            ISensorMode * iSensorMode = interface_cast<ISensorMode>(mode);
            CheckNotZero(iSensorMode, "iSensorMode");
            ROS_INFO_STREAM("Sensor mode:");
            ROS_INFO_STREAM("  exposure duration "
                 << iSensorMode->getExposureTimeRange().min()
                 << " - " << iSensorMode->getExposureTimeRange().max());
            ROS_INFO_STREAM("  frame duration "
                 << iSensorMode->getFrameDurationRange().min()
                 << " - " << iSensorMode->getFrameDurationRange().max());
            ROS_INFO_STREAM("  gain range "
                 << iSensorMode->getAnalogGainRange().min()
                 << " - " << iSensorMode->getAnalogGainRange().max());
            ROS_INFO_STREAM("  resolution "
                 << iSensorMode->getResolution().width()
                 << " - " << iSensorMode->getResolution().height());
            ROS_INFO_STREAM("  name " << iSensorMode->getSensorModeType().getName());

            if (i == 0)
            {
                FillCameraModeInfo(cameraModeInfo, iSensorMode);
            }
        }

        if (device < 0 || device > cameraDevices.size() - 1)
        {
            ROS_FATAL("device number is out of range");
            exit(-1);
        }

        CameraDevice* activeCamera = cameraDevices.at(device);

        // Create the capture session
        UniqueObj<CaptureSession> captureSession(
                    iCameraProvider->createCaptureSession(activeCamera, &status));
        CheckZero(status, "createCaptureSession error");
        ICaptureSession * iCaptureSession = interface_cast<ICaptureSession>(captureSession);
        CheckNotZero(iCaptureSession, "Failed to get capture session interface");

        // Create stream settings object and set settings common to streams.
        UniqueObj<OutputStreamSettings> streamSettings(iCaptureSession->createOutputStreamSettings());
        IOutputStreamSettings * iStreamSettings = interface_cast<IOutputStreamSettings>(streamSettings);
        CheckNotZero(iStreamSettings, "Failed to create OutputStreamSettings");

        iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
        Size2D<uint32_t> resolution(width, height);
        ROS_INFO_STREAM("Resolution: " << resolution.width() << "x" << resolution.height());
        iStreamSettings->setResolution(resolution);
        iStreamSettings->setMetadataEnable(true);

        // Create egl stream
        ROS_INFO_STREAM("Creating egl stream");
        iStreamSettings->setCameraDevice(activeCamera);
        UniqueObj<OutputStream> stream(iCaptureSession->createOutputStream(streamSettings.get()));
        IStream * iStream = interface_cast<IStream>(stream);
        CheckNotZero(iStream, "Failed to create egl stream");

        // Create consumer
        UniqueObj<FrameConsumer> consumer(FrameConsumer::create(stream.get()));
        IFrameConsumer * iConsumer = interface_cast<IFrameConsumer>(consumer);
        CheckNotZero(iConsumer, "Failed to initialize Consumer");

        // Create a request
        UniqueObj<Request> request = CreateRequest(iCaptureSession,
                                                   stream.get(),
                                                   cameraModeInfo,
                                                   maxExposureTimeSec,
                                                   frameDurationSec,
                                                   maxGain);


        ros::Publisher jpeg_pub_ = nh.advertise<sensor_msgs::CompressedImage>("image_raw/compressed",1);
        ros::Publisher cinfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info",1);



        /*
         * Acquire a frame generated by the capture request, get the images from the frame
         * and create a .JPG files of the captured images
         */

        ROS_INFO_STREAM("frames acquiring");
        uint64_t currentFrameNumber = 0;

        status = iCaptureSession->repeat(request.get());
        CheckZero(status, "repeatBurst");

        int nativeBufFd = -1;
        JpegEncoder jpegEncoder;

        while(ros::ok()) {

            UniqueObj<EGLStream::Frame> frame;
            IFrame * iFrame = NULL;
            {
                frame.reset(iConsumer->acquireFrame(1.0 / NANO));
                iFrame = interface_cast<IFrame>(frame);
                if (!iFrame)
                    continue;
            }

            currentFrameNumber = iFrame->getNumber();

            IArgusCaptureMetadata * iArgusMetadata = interface_cast<IArgusCaptureMetadata>(frame);
            CaptureMetadata * metadata = iArgusMetadata->getMetadata();
            ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(metadata);

            uint64_t u64TimeSensorStamp = iMetadata->getSensorTimestamp();
            double sensorTimestampSec = u64TimeSensorStamp * NANO;

            double currSysTime = GetSysTime();
            double sensorRealTimeSec = ToRealTime(sensorTimestampSec);

            ROS_DEBUG_STREAM("  "
                 << std::fixed
                 << std::setprecision(5)
                 << " sys time " << currSysTime
                 << " sensor time " << sensorTimestampSec
                 << " sensor real time " << sensorRealTimeSec);

            const unsigned char * jpegBuf = nullptr;
            unsigned long jpegBufSize = 0;
            {
                NV::IImageNativeBuffer * iNativeBuffer =
                    interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
                CheckNotZero(iNativeBuffer, "IImageNativeBuffer error");

                if (nativeBufFd == -1)
                {
                    nativeBufFd = iNativeBuffer->createNvBuffer(iStream->getResolution(),
                                                                NvBufferColorFormat_YUV420,
                                                                NvBufferLayout_BlockLinear);
                    if (nativeBufFd == -1)
                    {
                        throw runtime_error("createNvBuffer error");
                    }
                }

                status = iNativeBuffer->copyToNvBuffer(nativeBufFd);
                CheckZero(status, "copyToNvBuffer error");
                jpegBuf = jpegEncoder.EncodeFromFd(nativeBufFd, jpegBufSize);

                sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
                sensor_msgs::CameraInfoPtr cinfo;
                cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));
                cinfo->header.stamp = ros::Time(sensorRealTimeSec);

                sensor_msgs::CompressedImagePtr img(new sensor_msgs::CompressedImage());
                img->header = cinfo->header;
                img->format = "jpeg";
                img->data.resize(jpegBufSize);

                std::copy(jpegBuf, (jpegBuf) +  (jpegBufSize),
                        img->data.begin());
                jpeg_pub_.publish(img);
                cinfo_pub_.publish(cinfo);
            }
            ros::spinOnce();

        }

        if (nativeBufFd != -1)
        {
            CheckZero(NvBufferDestroy(nativeBufFd), "NvBufferDestroy error");
        }
        return;
    }

#if USE_GPIO
    void * RunGPIO(void *)
    {
        if(OpenGpioInput(GPIO_BUTTON) == false)
        {
            cout << "Could not open GPIO 388 for read\n" << endl;
            exit(-1);
        }

        while (runGPIO)
        {
            bool res = WaitZeroLevel(-1);
            if (!res)
            {
                cout << "WaitZeroLevel error" << endl;
                continue;
            }
            lastGPIOTriggerTime = GetSysTime();
            cout << "lastGPIOTriggerTime update " << lastGPIOTriggerTime << endl;
        }
        return NULL;
    }
#endif

}


int main(int argc, char **argv)
{
      ros::init(argc, argv, "argus_cam");
      ros::NodeHandle nh("~");
      RunCameraDevice(nh);
      return 0;
}

