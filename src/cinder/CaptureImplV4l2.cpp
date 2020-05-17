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
#include "cinder/CaptureImplV4l2.h"
#include "cinder/ImageIo.h"
#include "cinder/Log.h"
#include "cinder/Utilities.h"
#include "cinder/app/App.h"

#include <jpeglib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

using namespace std;

namespace cinder {

////////////////////////////////////////////////////////////////////////////////////////////////////
// CaptureImplV4l2::Device

CaptureImplV4l2::Device::Device( const ::string &name ) : Capture::Device(), mWidth( 0 ), mHeight( 0 ), mIsAvailable( false ), mIsConnected( false )
{
    mName = name;
    mUniqueId = ::atoi( &mName.back() );
}

CaptureImplV4l2::Device::Device( const ::string &name, int32_t width, int32_t height ) : Capture::Device(), mWidth( width ), mHeight( height ), mIsAvailable( false ), mIsConnected( false )
{
    mName = name;
    mUniqueId = ::atoi( &mName.back() );
    // make device and open it
    openDevice();
    init();
}

bool CaptureImplV4l2::Device::checkAvailable() const
{
    return mIsAvailable;
}

bool CaptureImplV4l2::Device::isConnected() const
{
    return mIsConnected;
}

void CaptureImplV4l2::Device::init()
{
    struct v4l2_cropcap cropcap;
    struct v4l2_crop    crop;

    if ( ! ( mCapability.capabilities & V4L2_CAP_VIDEO_CAPTURE ) ) {
        throw runtime_error( mPath + " is no video capture device" );
    }

    if ( ! ( mCapability.capabilities & V4L2_CAP_STREAMING ) ) {
        throw runtime_error( mPath + " does not support streaming i/o" );
    }

    /* Select video input, video standard and tune here. */

    clear( &cropcap );
    clear( &crop );
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if ( 0 == ioctl( mFd, VIDIOC_CROPCAP, &cropcap ) ) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if ( -1 == ioctl( mFd, VIDIOC_S_CROP, &crop ) ) {
            switch ( errno ) {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    }
    else {
        /* Errors ignored. */
    }

    v4l2_format format;
    clear( &mImageFormat );
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = mWidth;
    format.fmt.pix.height = mHeight;

    format.fmt.pix.pixelformat = mParameters.getConfigsV4l2().front().first.pixelformat;

    if ( format.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV ) {
        format.fmt.pix.field = V4L2_FIELD_INTERLACED;  //interlaced
    }
    else {
        format.fmt.pix.field = V4L2_FIELD_NONE;  // progressive
    }

    if ( -1 == ioctl( mFd, VIDIOC_S_FMT, &format ) )
        throw runtime_error( "VIDIOC_S_FMT" );

    /* Note VIDIOC_S_FMT may change width and height. */
    mImageFormat = format.fmt.pix;
    mWidth = mImageFormat.width;
    mHeight = mImageFormat.height;
    mStride = mImageFormat.bytesperline;

    initMmap();
    mIsConnected = false;
}

void CaptureImplV4l2::Device::initMmap()
{
    v4l2_requestbuffers req;
    clear( &req );

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if ( -1 == ioctl( mFd, VIDIOC_REQBUFS, &req ) ) {
        if ( EINVAL == errno ) {
            throw runtime_error( mPath + " does not support memory mapping" );
        }
        else {
            throw runtime_error( "VIDIOC_REQBUFS" );
        }
    }

    if ( req.count < 2 ) {
        throw runtime_error( string( "Insufficient buffer memory on " ) + mPath );
    }

    for ( int i = 0; i < req.count; i++ ) {
        v4l2_buffer buf;
        clear( &buf );

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if ( -1 == ioctl( mFd, VIDIOC_QUERYBUF, &buf ) ) {
            throw runtime_error( "VIDIOC_QUERYBUF" );
        }

        void *data =
            mmap( NULL /* start anywhere */,
                  buf.length,
                  PROT_READ | PROT_WRITE /* required */,
                  MAP_SHARED /* recommended */,
                  mFd, buf.m.offset );

        if ( MAP_FAILED == data ) {
            throw runtime_error( "mmap" );
        }

        mBuffers.push_back( ci::Buffer::create( data, buf.length ) );
    }
}

void CaptureImplV4l2::Device::openDevice()
{
    struct stat st;

    if ( -1 == stat( mPath.c_str(), &st ) ) {
        throw runtime_error( mPath + ": cannot identify! " + to_string( errno ) + ": " + strerror( errno ) );
    }

    if ( ! S_ISCHR( st.st_mode ) ) {
        throw runtime_error( mPath + " is no device" );
    }

    mFd = open( mPath.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0 );

    if ( -1 == mFd ) {
        throw runtime_error( mPath + ": cannot open! " + to_string( errno ) + ": " + strerror( errno ) );
    }

    mIsAvailable = true;
}

void CaptureImplV4l2::Device::setFrameRate( uint32_t frameRate )
{
    if ( ! mIsAvailable )
        return;

    v4l2_streamparm parm;
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = frameRate;

    if ( -1 == xioctl( mFd, VIDIOC_S_PARM, &parm ) ) {
        CI_LOG_W( "Couldn't set frame rate to " + toString( frameRate ) );
    }
}

void CaptureImplV4l2::Device::setControl( uint32_t controlName, int32_t value )
{
    v4l2_control ctrl;
    clear( &ctrl );
    ctrl.id = controlName;
    ctrl.value = value;

    if ( -1 == ioctl( mFd, VIDIOC_S_CTRL, &ctrl ) ) {
        cerr << "VIDIOC_S_CTR"
             << "   ::  " << errno << endl;
    }
}

void CaptureImplV4l2::Device::setControl( const std::string &controlName, int32_t value )
{
    setControl( getControlIdFromString( controlName ), value );
}

void CaptureImplV4l2::Device::start()
{
    // set control for exposure_auto_priority to 0 to solve a headache with slow fps
    setControl( "exposure_auto_priority", 0 );
    // start
    unsigned int  i;
    v4l2_buf_type type;

    for ( int i = 0; i < mBuffers.size(); i++ ) {
        v4l2_buffer buf;
        clear( &buf );
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if ( -1 == ioctl( mFd, VIDIOC_QBUF, &buf ) )
            throw runtime_error( "VIDIOC_QBUF" );
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ( -1 == ioctl( mFd, VIDIOC_STREAMON, &type ) )
        throw runtime_error( "VIDIOC_STREAMON" );
}

void CaptureImplV4l2::Device::stop()
{
    // stop
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ( -1 == ioctl( mFd, VIDIOC_STREAMOFF, &type ) )
        throw runtime_error( "VIDIOC_STREAMOFF" );

    uninit();
    closeDevice();
}

void CaptureImplV4l2::Device::uninit()
{
    for ( int i = 0; i < mBuffers.size(); i++ ) {
        if ( -1 == munmap( mBuffers[i]->getData(), mBuffers[i]->getSize() ) )
            throw runtime_error( "munmap" );
    }
    mIsConnected = false;
}

void CaptureImplV4l2::Device::closeDevice()
{
    if ( -1 == close( mFd ) )
        throw runtime_error( "close" );

    mFd = -1;
    mIsAvailable = false;
}

bool CaptureImplV4l2::Device::readFrame()
{
    v4l2_buffer buf;
    clear( &buf );

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if ( -1 == ioctl( mFd, VIDIOC_DQBUF, &buf ) ) {
        switch ( errno ) {
            case EAGAIN:
                return false;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw runtime_error( "VIDIOC_DQBUF" );
        }
    }
    mBufferIndex = buf.index;

    if ( -1 == ioctl( mFd, VIDIOC_QBUF, &buf ) )
        throw runtime_error( "VIDIOC_QBUF" );

    return true;
}

void CaptureImplV4l2::Device::print()
{
    app::console() << "." << endl
                   << "." << endl
                   << "." << endl
                   << "." << endl
                   << "." << endl
                   << "." << endl;
    app::console() << "------------DEVICE INFO---------------" << endl;
    app::console() << "Name: " << mName << endl;
    app::console() << "Driver: " << mDriverName << endl;
    app::console() << "Path: " << mPath << endl;
    app::console() << "." << endl
                   << "." << endl;
    app::console() << "------------Available Params----------" << endl;
    for ( auto &formatPair : mParameters.getConfigs() ) {
        app::console() << "Format: " << formatPair.first << endl;
        for ( auto &sizePair : formatPair.second ) {
            app::console() << "			"
                           << " Size: " << sizePair.first << endl;
            ;
            for ( auto &fps : sizePair.second ) {
                app::console() << "			"
                               << " 			"
                               << " FPS: " << fps << endl;
            }
        }
    }

    app::console() << "." << endl
                   << "." << endl;
    app::console() << "------------Available Controls----------" << endl;

    for ( auto &control : mParameters.getControls() ) {
        app::console() << "name: " << control.first.name << " id: " << getControlIdToString( control.first.id ) << " type:" << getControlTypeToString( (v4l2_ctrl_type)control.first.type ) << " min: " << control.first.minimum << " max: " << control.first.maximum << " step: " << control.first.step << " default: " << control.first.default_value << " value: " << control.second << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// CaptureImplV4l2

bool                       CaptureImplV4l2::sDevicesEnumerated = false;
vector<Capture::DeviceRef> CaptureImplV4l2::sDevices;

const std::map<v4l2_ctrl_type, std::string> CaptureImplV4l2::V4L2_CONTROL_TYPE_TO_STRING_MAP = {
    {V4L2_CTRL_TYPE_INTEGER, "int"},
    {V4L2_CTRL_TYPE_BOOLEAN, "bool"},
    {V4L2_CTRL_TYPE_STRING, "string"},
    {V4L2_CTRL_TYPE_MENU, "menu"},
    {V4L2_CTRL_TYPE_INTEGER64, "int 64"},
    {V4L2_CTRL_TYPE_INTEGER_MENU, "int menu"},
    {V4L2_CTRL_TYPE_BITMASK, "bitmask"},
    {V4L2_CTRL_TYPE_BUTTON, "button"},
    {V4L2_CTRL_TYPE_CTRL_CLASS, "control class"}

};

const std::map<uint32_t, std::string> CaptureImplV4l2::V4L2_CONTROL_ID_TO_STRING_MAP = {
    {V4L2_CID_BRIGHTNESS, "brightness"},
    {V4L2_CID_CONTRAST, "contrast"},
    {V4L2_CID_SATURATION, "saturation"},
    {V4L2_CID_HUE, "hue"},
    {V4L2_CID_WHITE_BALANCE_TEMPERATURE, "white_balance_temperature"},
    {V4L2_CID_AUTO_WHITE_BALANCE, "auto_white_balance"},
    {V4L2_CID_GAMMA, "gamma"},
    {V4L2_CID_POWER_LINE_FREQUENCY, "power_line_frequency"},
    {V4L2_CID_SHARPNESS, "sharpness"},
    {V4L2_CID_BACKLIGHT_COMPENSATION, "backlight_compensation"},
    {V4L2_CID_EXPOSURE_AUTO, "exposure_auto"},
    {V4L2_CID_EXPOSURE_ABSOLUTE, "exposure_absolute"},
    {V4L2_CID_EXPOSURE_AUTO_PRIORITY, "exposure_auto_priority"}

};

const std::map<std::string, uint32_t> CaptureImplV4l2::V4L2_CONTROL_ID_FROM_STRING_MAP = {
    {"brightness", V4L2_CID_BRIGHTNESS},
    {"contrast", V4L2_CID_CONTRAST},
    {"saturation", V4L2_CID_SATURATION},
    {"hue", V4L2_CID_HUE},
    {"white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE},
    {"auto_white_balance", V4L2_CID_AUTO_WHITE_BALANCE},
    {"gamma", V4L2_CID_GAMMA},
    {"power_line_frequency", V4L2_CID_POWER_LINE_FREQUENCY},
    {"sharpness", V4L2_CID_SHARPNESS},
    {"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION},
    {"exposure_auto", V4L2_CID_EXPOSURE_AUTO},
    {"exposure_absolute", V4L2_CID_EXPOSURE_ABSOLUTE},
    {"exposure_auto_priority", V4L2_CID_EXPOSURE_AUTO_PRIORITY}

};

const std::map<uint32_t, std::string> CaptureImplV4l2::V4L2_PIXEL_FORMAT_TO_STRING_MAP = {
    {V4L2_PIX_FMT_MJPEG, "MJPG"},
    {V4L2_PIX_FMT_JPEG, "JPEG"},
    {V4L2_PIX_FMT_DV, "dvsd"},
    {V4L2_PIX_FMT_MPEG, "MPEG"},
    {V4L2_PIX_FMT_H264, "H264"},
    {V4L2_PIX_FMT_H264_NO_SC, "AVC1"},
    {V4L2_PIX_FMT_H264_MVC, "M264"},
    {V4L2_PIX_FMT_H263, "H263"},
    {V4L2_PIX_FMT_MPEG1, "MPG1"},
    {V4L2_PIX_FMT_MPEG2, "MPG2"},
    {V4L2_PIX_FMT_MPEG4, "MPG4"},
    {V4L2_PIX_FMT_XVID, "XVID"},
    {V4L2_PIX_FMT_VC1_ANNEX_G, "VC1G"},
    {V4L2_PIX_FMT_VC1_ANNEX_L, "VC1L"},
    {V4L2_PIX_FMT_VP8, "VP8"},
    {V4L2_PIX_FMT_VP9, "VP9"}

};

const std::map<std::string, uint32_t> CaptureImplV4l2::V4L2_PIXEL_FORMAT_FROM_STRING_MAP = {
    {"MJPG", V4L2_PIX_FMT_MJPEG},
    {"JPEG", V4L2_PIX_FMT_JPEG},
    {"dvsd", V4L2_PIX_FMT_DV},
    {"MPEG", V4L2_PIX_FMT_MPEG},
    {"H264", V4L2_PIX_FMT_H264},
    {"AVC1", V4L2_PIX_FMT_H264_NO_SC},
    {"M264", V4L2_PIX_FMT_H264_MVC},
    {"H263", V4L2_PIX_FMT_H263},
    {"MPG1", V4L2_PIX_FMT_MPEG1},
    {"MPG2", V4L2_PIX_FMT_MPEG2},
    {"MPG4", V4L2_PIX_FMT_MPEG4},
    {"XVID", V4L2_PIX_FMT_XVID},
    {"VC1G", V4L2_PIX_FMT_VC1_ANNEX_G},
    {"VC1L", V4L2_PIX_FMT_VC1_ANNEX_L},
    {"VP8", V4L2_PIX_FMT_VP8},
    {"VP9", V4L2_PIX_FMT_VP9},
};

CaptureImplV4l2::CaptureImplV4l2( int width, int height, const Capture::DeviceRef device )
    : mWidth( width ), mHeight( height ), mDevice( device )
{
    if ( ! mDevice ) {
        auto devices = getDevices();
        mDevice = devices.front();
    }

    auto fullDevice = static_pointer_cast<CaptureImplV4l2::Device>( mDevice );

    if ( fullDevice->getWidth() == 0 ) {
        fullDevice->mWidth = width;
        fullDevice->mHeight = height;
        // make device and open it
        fullDevice->openDevice();
        fullDevice->init();
    }

    // get the final width from the device because it might differ from user input
    mName = fullDevice->getName();
    mWidth = fullDevice->getWidth();
    mHeight = fullDevice->getHeight();
}

CaptureImplV4l2::~CaptureImplV4l2()
{
    if ( mIsCapturing ) {
        stop();
    }
}

void CaptureImplV4l2::start()
{
    if ( ! mDevice )
        return;

    auto fullDevice = static_pointer_cast<CaptureImplV4l2::Device>( mDevice );
    if ( fullDevice ) {
        fullDevice->start();
        mIsCapturing = true;
    }
}

void CaptureImplV4l2::stop()
{
    mIsCapturing = false;
    if ( mDevice ) {
        auto fullDevice = static_pointer_cast<CaptureImplV4l2::Device>( mDevice );
        fullDevice->stop();
        mDevice.reset();
    }
}

bool CaptureImplV4l2::isCapturing() const
{
    return mIsCapturing;
}

bool CaptureImplV4l2::checkNewFrame() const
{
    if ( isCapturing() && mDevice ) {
        auto fullDevice = static_pointer_cast<CaptureImplV4l2::Device>( mDevice );
        if ( fullDevice ) {
            timeval tv;
            int     fd = fullDevice->getFd();
            fd_set  fdset;
            FD_ZERO( &fdset );
            FD_SET( fd, &fdset );
            return select( fd + 1, &fdset, nullptr, nullptr, &tv ) == 1;
        }
    }
    return false;
}

Surface8uRef CaptureImplV4l2::getSurface() const
{
    auto fullDevice = ::static_pointer_cast<CaptureImplV4l2::Device>( mDevice );
    fullDevice->readFrame();
    auto currentBuffer = fullDevice->getCurrentBuffer();

    if ( fullDevice->getImageFormat().pixelformat == V4L2_PIX_FMT_MJPEG ) {  // decompress jpeg
        // check if it has huffman and if not add it so we can correctly decompress as jpeg
        if ( hasHuffmanTable( (uint8_t *)currentBuffer->getData() ) ) {
            //mCurrentFrame = Surface::create( loadImage( DataSourceBuffer::create( currentBuffer ) ), SurfaceConstraintsDefault(), false );
            mCurrentFrame = Surface::create( mWidth, mHeight, false, SurfaceChannelOrder::RGB );
            jpgToRgb( (uint8_t *)currentBuffer->getData(), currentBuffer->getSize(), mCurrentFrame->getData() );
        }
        else {
            auto     bufferSize = fullDevice->getImageFormat().sizeimage;
            uint32_t jpgSize = mjpgToJpg( (uint8_t *)currentBuffer->getData(), bufferSize, NULL );
            uint8_t *jpg = (uint8_t *)malloc( jpgSize );
            mjpgToJpg( (uint8_t *)currentBuffer->getData(), bufferSize, jpg );
            auto buffer = Buffer::create( jpg, jpgSize );
            mCurrentFrame = Surface::create( loadImage( DataSourceBuffer::create( buffer ) ), SurfaceConstraintsDefault(), false );
        }
    }
    else if ( fullDevice->getImageFormat().pixelformat == V4L2_PIX_FMT_YUYV ) {  //decompress yuv
        mCurrentFrame = Surface::create( mWidth, mHeight, false );
        auto yuvBuffer = (uint8_t *)currentBuffer->getData();
        yuyvToRgb24( yuvBuffer, (uint8_t *)mCurrentFrame->getData(), fullDevice->getWidth(), fullDevice->getHeight(), fullDevice->getStride() );
    }

    return mCurrentFrame;
}
int32_t CaptureImplV4l2::getWidth() const
{
    if ( ! mDevice )
        return -1;

    return mWidth;
}

int32_t CaptureImplV4l2::getHeight() const
{
    if ( ! mDevice )
        return -1;

    return mHeight;
}

const vector<Capture::DeviceRef> &CaptureImplV4l2::getDevices( bool forceRefresh )
{
    if ( ! CaptureImplV4l2::sDevicesEnumerated || forceRefresh ) {
        CaptureImplV4l2::sDevices.clear();
        string nameBase( "/dev/video" );
        for ( int i = 0; i < 64; i++ ) {  // v4l2 can have up to 64 devices so check all
            int             fd;
            bool            isAvailable = true;
            string          name = nameBase + ::to_string( i );
            v4l2_capability videoCap;

            if ( ( fd = open( name.c_str(), O_RDWR ) ) == -1 ) {
                isAvailable = false;
                continue;
            }

            if ( ioctl( fd, VIDIOC_QUERYCAP, &videoCap ) == -1 ) {
                isAvailable = false;
                continue;
            }

            // camera is available, make the device
            auto device = ::make_shared<CaptureImplV4l2::Device>( toString( videoCap.card ) );
            device->mCapability = videoCap;
            device->mDriverName = toString( videoCap.driver );
            device->mPath = nameBase + toString( i );

            // check for avaialble formats and avaialble sizes
            // https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/vidioc-enum-fmt.html
            auto& availableConfigs = device->getParameters().getConfigs();
            availableConfigs.clear();
            auto& availableConfigsV4l2 = device->getParameters().getConfigsV4l2();
            availableConfigsV4l2.clear();
            v4l2_fmtdesc fmtDesc;
            clear( &fmtDesc );
            fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            while ( ioctl( fd, VIDIOC_ENUM_FMT, &fmtDesc ) == 0 ) {
                // add the format to the available formats vector
                // check available sizes and frame intervals
                // https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/vidioc-enum-framesizes.html
                v4l2_frmsizeenum frmSize;
                clear( &frmSize );
                frmSize.pixel_format = fmtDesc.pixelformat;
                vector<pair<v4l2_frmsizeenum, vector<v4l2_frmivalenum>>> frameSizes;
                vector<pair<ivec2, vector<float>>>                       frameSizesCi;
                while ( ioctl( fd, VIDIOC_ENUM_FRAMESIZES, &frmSize ) == 0 ) {
                    if ( frmSize.type == V4L2_FRMSIZE_TYPE_DISCRETE ) {
                        // found a size, add it
                        ivec2 size = ivec2( frmSize.discrete.width, frmSize.discrete.height );

                        // look for intervals/fps
                        v4l2_frmivalenum frmIval;
                        clear( &frmIval );
                        frmIval.pixel_format = fmtDesc.pixelformat;
                        frmIval.width = frmSize.discrete.width;
                        frmIval.height = frmSize.discrete.height;
                        vector<v4l2_frmivalenum> frameIntervals;
                        vector<float>            frameRates;
                        ioctl( fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmIval );
                        if ( frmIval.type == V4L2_FRMIVAL_TYPE_DISCRETE ) {
                            while ( ioctl( fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmIval ) == 0 ) {
                                frameIntervals.push_back( frmIval );
                                frameRates.push_back( (float)frmIval.discrete.denominator / frmIval.discrete.numerator );
                                frmIval.index += 1;
                            }
                        }
                        auto frameSizePair = make_pair( frmSize, frameIntervals );
                        auto frameSizePairCi = make_pair( size, frameRates );
                        frameSizes.push_back( frameSizePair );
                        frameSizesCi.push_back( frameSizePairCi );
                        frmSize.index++;
                    }
                    else {  // according to the guide if the result is not discrete, there's no use in searching anymore
                        break;
                    }
                }
                availableConfigsV4l2.push_back( make_pair( fmtDesc, frameSizes ) );
                availableConfigs.push_back( make_pair( toString( fmtDesc.description ), frameSizesCi ) );
                fmtDesc.index++;
            }

            // get available controls
            auto& availableControls = device->getParameters().getControls();
            v4l2_queryctrl qctrl;
            v4l2_control   ctrl;
            clear( &qctrl );
            clear( &ctrl );
            qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
            while ( 0 == ioctl( fd, VIDIOC_QUERYCTRL, &qctrl ) ) {
                ctrl.id = qctrl.id;
                if ( -1 == ioctl( fd, VIDIOC_G_CTRL, &ctrl ) ) {
                    cerr << "VIDIOC_G_CTR"
                         << "   ::  " << errno << endl;
                }
                else {
                    availableControls.push_back( make_pair( qctrl, ctrl.value ) );
                }

                qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
            }

            close( fd );

            // add device to the list
            CaptureImplV4l2::sDevices.push_back( device );
        }
        CaptureImplV4l2::sDevicesEnumerated = true;
    }

    return CaptureImplV4l2::sDevices;
}

void CaptureImplV4l2::printDevices( bool forceRefresh )
{
    auto devices = getDevices( forceRefresh );
    for ( auto &device : devices ) {
        auto fullDevice = static_pointer_cast<CaptureImplV4l2::Device>( device );
        fullDevice->print();
    }
}

// taken from https://github.com/severin-lemaignan/webcam-v4l2/blob/master/webcam.cpp

void CaptureImplV4l2::clear( void *data )
{
    memset( data, 0, sizeof( data ) );
}

uint8_t CaptureImplV4l2::clip( uint8_t color )
{
    return ( ( ( color ) > 0xFF ) ? 0xff : ( ( ( color ) < 0 ) ? 0 : ( color ) ) );
}

int CaptureImplV4l2::xioctl( int fd, int request, void *arg )
{
    int r;
    do
        r = ioctl( fd, request, arg );
    while ( -1 == r && EINTR == errno );
    return r;
}

void CaptureImplV4l2::yuyvToRgb24( const uint8_t *src, uint8_t *dest, int width, int height, int stride )
{
    int j;
    while ( --height >= 0 ) {
        for ( j = 0; j + 1 < width; j += 2 ) {
            int u = src[1];
            int v = src[3];
            int u1 = ( ( ( u - 128 ) << 7 ) + ( u - 128 ) ) >> 6;
            int rg = ( ( ( u - 128 ) << 1 ) + ( u - 128 ) +
                       ( ( v - 128 ) << 2 ) + ( ( v - 128 ) << 1 ) ) >>
                     3;
            int v1 = ( ( ( v - 128 ) << 1 ) + ( v - 128 ) ) >> 1;

            *dest++ = clip( src[0] + v1 );
            *dest++ = clip( src[0] - rg );
            *dest++ = clip( src[0] + u1 );

            *dest++ = clip( src[2] + v1 );
            *dest++ = clip( src[2] - rg );
            *dest++ = clip( src[2] + u1 );
            src += 4;
        }
        src += stride - ( width * 2 );
    }
}

bool CaptureImplV4l2::hasHuffmanTable( uint8_t *buf )
{
    int      i = 0;
    uint8_t *pbuf = buf;

    while ( ( ( pbuf[0] << 8 ) | pbuf[1] ) != 0xffda ) {
        if ( i++ > 2048 ) {
            return 0;
        }

        if ( ( ( pbuf[0] << 8 ) | pbuf[1] ) == 0xffc4 ) {
            return 1;
        }

        pbuf++;
    }

    return 0;
}

bool CaptureImplV4l2::jpgToRgb( uint8_t *jpgData, unsigned int jpgSize, uint8_t *rgbData )
{
    // Variables for the decompressor itself
    jpeg_decompress_struct cinfo;
    jpeg_error_mgr         jerr;

    // Variables for the output buffer, and how long each row is
    unsigned long bmp_size;
    int           row_stride, width, height, pixel_size;

    // Allocate a new decompress struct, with the default error handler.
    // The default error handler will exit() on pretty much any issue,
    // so it's likely you'll want to replace it or supplement it with
    // your own.
    cinfo.err = jpeg_std_error( &jerr );
    jpeg_create_decompress( &cinfo );

    // Configure this decompressor to read its data from a memory
    // buffer starting at unsigned char *jpg_buffer, which is jpg_size
    // long, and which must contain a complete jpg already.
    //
    // If you need something fancier than this, you must write your
    // own data source manager, which shouldn't be too hard if you know
    // what it is you need it to do. See jpeg-8d/jdatasrc.c for the
    // implementation of the standard jpeg_mem_src and jpeg_stdio_src
    // managers as examples to work from.
    jpeg_mem_src( &cinfo, jpgData, jpgSize );

    // Have the decompressor scan the jpeg header. This won't populate
    // the cinfo struct output fields, but will indicate if the
    // jpeg is valid.
    int rc = jpeg_read_header( &cinfo, TRUE );

    if ( rc != 1 ) {
        exit( EXIT_FAILURE );
    }

    // By calling jpeg_start_decompress, you populate cinfo
    // and can then allocate your output bitmap buffers for
    // each scanline.
    jpeg_start_decompress( &cinfo );

    width = cinfo.output_width;
    height = cinfo.output_height;
    pixel_size = cinfo.output_components;

    bmp_size = width * height * pixel_size;

    // The row_stride is the total number of bytes it takes to store an
    // entire scanline (row).
    row_stride = width * pixel_size;

    //
    // Now that you have the decompressor entirely configured, it's time
    // to read out all of the scanlines of the jpeg.
    //
    // By default, scanlines will come out in RGBRGBRGB...  order,
    // but this can be changed by setting cinfo.out_color_space
    //
    // jpeg_read_scanlines takes an array of buffers, one for each scanline.
    // Even if you give it a complete set of buffers for the whole image,
    // it will only ever decompress a few lines at a time. For best
    // performance, you should pass it an array with cinfo.rec_outbuf_height
    // scanline buffers. rec_outbuf_height is typically 1, 2, or 4, and
    // at the default high quality decompression setting is always 1.
    while ( cinfo.output_scanline < cinfo.output_height ) {
        uint8_t *buffer_array[1];
        buffer_array[0] = rgbData +
                          ( cinfo.output_scanline ) * row_stride;

        jpeg_read_scanlines( &cinfo, buffer_array, 1 );
    }

    // Once done reading *all* scanlines, release all internal buffers,
    // etc by calling jpeg_finish_decompress. This lets you go back and
    // reuse the same cinfo object with the same settings, if you
    // want to decompress several jpegs in a row.
    //
    // If you didn't read all the scanlines, but want to stop early,
    // you instead need to call jpeg_abort_decompress(&cinfo)
    jpeg_finish_decompress( &cinfo );

    // At this point, optionally go back and either load a new jpg into
    // the jpg_buffer, or define a new jpeg_mem_src, and then start
    // another decompress operation.

    // Once you're really really done, destroy the object to free everything
    jpeg_destroy_decompress( &cinfo );
    // And free the input buffer
    return true;
}

// taken from https://github.com/lightbits/usbcam/blob/master/mjpg_to_jpg.cpp
// also check this https://lightbits.github.io/v4l2_huffman/
unsigned int CaptureImplV4l2::mjpgToJpg( uint8_t *mjpg, unsigned int mjpgSize, uint8_t *jpg )
{
    // To understand how this works I suggest you run
    //   $ xxd <file> | less
    // on one of your MJPG images, and try to relate the binary data with
    // the JPEG specification on wikipedia (en.wikipedia.org/wiki/JPEG#JPEG_files).
    // Specifically, look for the ff** markers, starting with ffd8.
    // Then compare that with a JPG image.

    static const uint8_t huffman[] =
        {
            // JPEG magic
            // 0xff, 0xd8,

            // Text comment
            // 0xff, 0xfe, 0x00, 0x10 0x4c, 0x61 0x76, 0x63 0x35, 0x36 0x2e, 0x36 0x30, 0x2e 0x31, 0x30 0x30, 0x00,

            // Huffman table
            0xff, 0xc4, 0x01, 0xa2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
            0x0b, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x10, 0x00,
            0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d, 0x01,
            0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
            0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24,
            0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29,
            0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a,
            0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a,
            0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a,
            0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8,
            0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6,
            0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3,
            0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9,
            0xfa, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01,
            0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07,
            0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33,
            0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19,
            0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46,
            0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66,
            0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85,
            0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
            0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
            0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8,
            0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
            0xf7, 0xf8, 0xf9, 0xfa};

    unsigned int jpg_size = mjpgSize + sizeof( huffman );
    if ( ! jpg )
        return jpg_size;

    // search for Start of Frame (SOF0) marker
    unsigned int i = 0;
    while ( ( i + 1 ) < mjpgSize && ! ( mjpg[i] == 0xff && mjpg[i + 1] == 0xc0 ) )
        i++;

    // and squeeze huffman table inbetween
    memcpy( jpg, mjpg, i );
    memcpy( jpg + i, huffman, sizeof( huffman ) );
    memcpy( jpg + i + sizeof( huffman ), mjpg + i, mjpgSize - i );
    return jpg_size;
}

const std::string &CaptureImplV4l2::getControlTypeToString( v4l2_ctrl_type ctrlType )
{
    return V4L2_CONTROL_TYPE_TO_STRING_MAP.at( ctrlType );
}

const std::string &CaptureImplV4l2::getControlIdToString( uint32_t id )
{
    return V4L2_CONTROL_ID_TO_STRING_MAP.at( id );
}

uint32_t CaptureImplV4l2::getControlIdFromString( const string &name )
{
    return V4L2_CONTROL_ID_FROM_STRING_MAP.at( name );
}

const std::string &CaptureImplV4l2::getPixelFormatToString( uint32_t pixelFormat )
{
    return V4L2_PIXEL_FORMAT_TO_STRING_MAP.at( pixelFormat );
}
uint32_t CaptureImplV4l2::getPixelFormatFromString( const std::string &name )
{
    return V4L2_PIXEL_FORMAT_FROM_STRING_MAP.at( name );
}

}  // namespace cinder