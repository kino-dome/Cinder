/*
 Copyright (c) 2010, The Barbarian Group
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "cinder/Buffer.h"
#include "cinder/Capture.h"
#include "cinder/Cinder.h"
#include "cinder/Surface.h"
#include <linux/videodev2.h>

namespace cinder {

class CaptureImplV4l2 {
 public:
    typedef std::shared_ptr<class Device> DeviceRef;

    // taken from http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
    class Device : public Capture::Device {
        friend class CaptureImplV4l2;

        class Parameters {
         public:
            typedef std::vector<std::pair<v4l2_queryctrl, int32_t>>                                                               Controls;
            typedef std::vector<std::pair<v4l2_fmtdesc, std::vector<std::pair<v4l2_frmsizeenum, std::vector<v4l2_frmivalenum>>>>> ConfigsV4l2;
            typedef std::vector<std::pair<std::string, std::vector<std::pair<ci::ivec2, std::vector<float>>>>>                    Configs;
            Parameters() {}
            ~Parameters() {}

            Controls &        getControls() { return mAvailableControls; }
            const Controls &  getControls() const { return mAvailableControls; }
            ConfigsV4l2 &     getConfigsV4l2() { return mAvailableConfigsV4l2; }
            const ConfigsV4l2 getConfigsV4l2() const { return mAvailableConfigsV4l2; }
            Configs &         getConfigs() { return mAvailableConfigs; }
            const Configs &   getConfigs() const { return mAvailableConfigs; }

            //control info + current value
            Controls mAvailableControls;
            //available formats    available sizes    availabe intervals ---------- fps = 1/interval
            ConfigsV4l2 mAvailableConfigsV4l2;
            // availabe formatStrings 		available ivec2 sizes 		avaialble fps
            Configs mAvailableConfigs;
        };

     public:
        // constructors
        Device( const std::string &name, int32_t width, int32_t height );
        Device( const std::string &name );

        // overrides from Capture::Device
        bool                      checkAvailable() const override;
        bool                      isConnected() const override;
        Capture::DeviceIdentifier getUniqueId() const override { return mUniqueId; }

        // setters/getters
        std::string &                     getName() { return mName; }
        const std::string &               getDriverName() const { return mDriverName; }
        std::string &                     getDriverName() { return mDriverName; }
        const std::string &               getPath() const { return mPath; }
        std::string &                     getPath() { return mPath; }
        int                               getFd() const { return mFd; }
        v4l2_capability                   getCapability() const { return mCapability; }
        v4l2_pix_format                   getImageFormat() const { return mImageFormat; }
        int32_t                           getWidth() const { return mWidth; }
        int32_t                           getHeight() const { return mHeight; }
        int32_t                           getStride() const { return mStride; }
        ci::BufferRef                     getCurrentBuffer() { return mBuffers[mBufferIndex]; }
        std::vector<ci::BufferRef> &      getBuffers() { return mBuffers; }
        const std::vector<ci::BufferRef> &getBuffers() const { return mBuffers; }
        int                               getBufferIndex() const { return mBufferIndex; }
        const Parameters &                getParameters() const { return mParameters; }
        Parameters &                      getParameters() { return mParameters; }
        void                              setFrameRate( uint32_t frameRate );
        void                              setControl( const std::string &controlName, int32_t value );
        void                              setControl( uint32_t controlName, int32_t value );

     protected:
        void print();
        void init();
        void initMmap();
        void openDevice();
        void start();
        void stop();
        void uninit();
        void closeDevice();
        bool readFrame();

        bool                       mIsAvailable, mIsConnected;
        Capture::DeviceIdentifier  mUniqueId;  // number of the video in path
        int                        mFd;        // device handle
        v4l2_capability            mCapability;
        v4l2_pix_format            mImageFormat;
        std::string                mDriverName, mPath;
        int32_t                    mWidth, mHeight;  // could be different from the requested dimensions
        int32_t                    mStride;          //bytes per line from the driver
        std::vector<ci::BufferRef> mBuffers;         // buffers mapped to device memory using mmap
        int                        mBufferIndex;     // holds the index of the current buffer
        Parameters                 mParameters;
    };

    // methods similar to ci::Capture
    CaptureImplV4l2( int32_t width, int32_t height, const Capture::DeviceRef device );
    ~CaptureImplV4l2();
    void                                          start();
    void                                          stop();
    bool                                          isCapturing() const;
    bool                                          checkNewFrame() const;
    int32_t                                       getWidth() const;
    int32_t                                       getHeight() const;
    Surface8uRef                                  getSurface() const;
    const Capture::DeviceRef                      getDevice() const { return mDevice; }
    static const std::vector<Capture::DeviceRef> &getDevices( bool forceRefresh = false );

    // print all the devices
    static void printDevices( bool forceRefresh = false );

 protected:
    static void         clear( void *data );
    static uint8_t      clip( uint8_t color );
    static int          xioctl( int fd, int request, void *arg );
    static void         yuyvToRgb24( const uint8_t *src, uint8_t *dest, int width, int height, int stride );
    static bool         hasHuffmanTable( uint8_t *buf );  // checks if jpeg has huffman table
    static bool         jpgToRgb( uint8_t *jpgData, unsigned int jpgSize, uint8_t *rgbData );
    static unsigned int mjpgToJpg( uint8_t *mjpg, unsigned int mjpgSize, uint8_t *jpg );

    std::string          mName;
    bool                 mIsCapturing;
    int32_t              mWidth, mHeight;
    mutable Surface8uRef mCurrentFrame;
    Capture::DeviceRef   mDevice;

    // static fields for getDevices()
    static bool                            sDevicesEnumerated;
    static std::vector<Capture::DeviceRef> sDevices;
    // static methods to get name of V4l2 properties in string and vice versa
    static const std::string &getControlTypeToString( v4l2_ctrl_type ctrlType );
    static const std::string &getControlIdToString( uint32_t id );
    static uint32_t           getControlIdFromString( const std::string &name );
    static const std::string &getPixelFormatToString( uint32_t pixelFormat );
    static uint32_t           getPixelFormatFromString( const std::string &name );

    // static maps that assign strings to control types to make them human-readable
    static const std::map<v4l2_ctrl_type, std::string> V4L2_CONTROL_TYPE_TO_STRING_MAP;
    static const std::map<uint32_t, std::string>       V4L2_CONTROL_ID_TO_STRING_MAP;
    static const std::map<std::string, uint32_t>       V4L2_CONTROL_ID_FROM_STRING_MAP;
    static const std::map<uint32_t, std::string>       V4L2_PIXEL_FORMAT_TO_STRING_MAP;
    static const std::map<std::string, uint32_t>       V4L2_PIXEL_FORMAT_FROM_STRING_MAP;
};

}  //namespace cinder
