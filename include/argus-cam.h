#ifndef __ARGUS_CAM_H
#define __ARGUS_CAM_H


#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <stdexcept>

using namespace std;
using namespace Argus;
using namespace EGLStream;
namespace fs = std::experimental::filesystem;


struct CameraModeInfo
{
    pair<uint64_t, uint64_t> exposureRange;
    pair<uint64_t, uint64_t> frameDurationRange;
    pair<float, float> gainRange;
    pair<uint32_t, uint32_t> resolutionWidthHeight;
};


struct Params
{
    const int device;
    const double maxExposureTimeSec;
    const double frameDurationSec;
    const int maxISO;
    const int requestsCount;
    const int resolutionWidth;
    const int resolutionHeight;
    const bool runGPIOThread;
    const string outFolder;

    Params(const int device,
            const double maxExposureTimeSec,
            const double frameDurationSec,
            const int maxISO,
            const int requestsCount,
            const int resolutionWidth,
            const int resolutionHeight,
            const bool runGPIOThread,
            const string & outFolder)
        : device(device),
          maxExposureTimeSec(maxExposureTimeSec),
          frameDurationSec(frameDurationSec),
          maxISO(maxISO),
          requestsCount(requestsCount),
          resolutionWidth(resolutionWidth),
          resolutionHeight(resolutionHeight),
          runGPIOThread(runGPIOThread),
          outFolder(outFolder)
    {
    }

    void Print()
    {
        cout << "Params:"
            << "\n  device index " << device
            << "\n  maximum exposure time (seconds) " << maxExposureTimeSec
            << "\n  frame duration (seconds) " << frameDurationSec
            << "\n  maximum ISO value " << maxISO
            << "\n  requests count " << requestsCount
            << "\n  resolution width " << resolutionWidth
            << "\n  resolution height " << resolutionHeight
            << "\n  run GPIO thread " << runGPIOThread
            << "\n  output folder " << outFolder
            << endl;
    }
};


class JpegEncoder
{
public:
    JpegEncoder()
        : enc(NvJPEGEncoder::createJPEGEncoder("jpenenc"))
    {
        bufSize = 1280 * 720 * 1.5;
        buf = new unsigned char[bufSize];
    }
    ~JpegEncoder()
    {
        if (buf)
        {
            delete [] buf;
            buf = nullptr;
            bufSize = 0;
        }
    }
    const unsigned char * EncodeFromFd(int fd, unsigned long & outBufSize, int quality = 100)
    {
        size_t tmpSize = bufSize;
        enc->encodeFromFd(fd, JCS_YCbCr, &buf, tmpSize, quality);
        if (tmpSize > bufSize)
        {
            bufSize = tmpSize;
        }
        outBufSize = tmpSize;
        return buf;
    }

private:
    unique_ptr<NvJPEGEncoder> enc;
    unsigned char * buf;
    size_t bufSize;
};



#endif // ifndef __GSCAM_GSCAM_H
