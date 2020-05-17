#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CaptureImplV4l2.h"
#include "cinder/Log.h"
#include "cinder/params/Params.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#if defined( CINDER_ANDROID )
    #define USE_HW_TEXTURE
#endif

class CaptureAdvancedLinuxApp : public App {
 public:
    void setup() override;
    void update() override;
    void draw() override;

    void makeParams();

    CaptureRef     mCapture;
    gl::TextureRef mTexture;

    // params
    params::InterfaceGlRef mParams;
    int                    mDeviceIndex = 0;
    int                    mPixelFormatIndex;
    int                    mResolutionIndex;
    int                    mFpsIndex;
};

void CaptureAdvancedLinuxApp::setup()
{
    // print info about all the available devices
    CaptureImplV4l2::printDevices();

    try {
        mCapture = Capture::create( 1280, 720 );
        mCapture->start();
    } catch ( ci::Exception& exc ) {
        CI_LOG_EXCEPTION( "Failed to init capture ", exc );
    }

    makeParams();
}

void CaptureAdvancedLinuxApp::update()
{
    if ( mCapture && mCapture->checkNewFrame() ) {
        if ( ! mTexture ) {
            // Capture images come back as top-down, and it's more efficient to keep them that way
            mTexture = gl::Texture::create( *mCapture->getSurface(), gl::Texture::Format().loadTopDown() );
        }
        else {
            mTexture->update( *mCapture->getSurface() );
        }
    }
}

void CaptureAdvancedLinuxApp::draw()
{
    gl::clear();

    if ( mTexture ) {
        gl::ScopedModelMatrix modelScope;
        gl::draw( mTexture );
    }

    mParams->draw();
}

void CaptureAdvancedLinuxApp::makeParams()
{
    mParams = params::InterfaceGl::create( "Camera Params", ivec2( 500, 400 ) );
	// reset texture so if we changed resolution, a new one is created accordingly in the first update
    mTexture.reset();
	mTexture = nullptr;

    auto fullDevice = std::static_pointer_cast<CaptureImplV4l2::Device>( mCapture->getDevice() );
    mParams->addText( "Device" );

    vector<string> deviceNames;
    for ( auto& device : Capture::getDevices() ) {
        deviceNames.push_back( device->getName() );
    }
    mParams->addParam( "Name", deviceNames, &mDeviceIndex ).updateFn( [&]() {
        try {
            mCapture = Capture::create( 1280, 720, Capture::getDevices().at( mDeviceIndex ) );
            mCapture->start();
        } catch ( ci::Exception& exc ) {
            CI_LOG_EXCEPTION( "Failed to init capture ", exc );
        }
        makeParams();
    } );

    mParams->addParam<string>( "Driver", &fullDevice->getDriverName(), true );
    mParams->addParam<string>( "Path", &fullDevice->getPath(), true );



    mParams->addSeparator();
	mParams->addText("Pixel Format");

	vector<string> pixelFormatNames;
	for (auto& pixelFormatPair : fullDevice->getParameters().getConfigsV4l2()) {
		pixelFormatNames.push_back(toString(pixelFormatPair.first.description));
	}
	mParams->addParam("Format", pixelFormatNames, &mPixelFormatIndex).updateFn([&](){

	});
}

void prepareSettings( CaptureAdvancedLinuxApp::Settings* settings )
{
    settings->setWindowSize( ivec2( 1280, 720 ) );
}

CINDER_APP( CaptureAdvancedLinuxApp, RendererGl, prepareSettings )
