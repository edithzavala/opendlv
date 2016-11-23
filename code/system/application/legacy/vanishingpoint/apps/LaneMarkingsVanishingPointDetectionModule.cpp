#include <cstdlib>
#include <iostream>

#include <opendavinci/generated/odcore/data/image/SharedImage.h>
#include <opendavinci/odcore/base/module/DataTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/wrapper/SharedMemoryFactory.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VanishingPointDetection.h"
#include "ConfigDeserialization.h"

namespace lmvp {

using odcore::base::module::DataTriggeredConferenceClientModule;
using odcore::base::Lock;
using odcore::data::Container;
using odcore::data::image::SharedImage;
using odcore::exceptions::ValueForKeyNotFoundException;
using odcore::wrapper::SharedMemoryFactory;
using odcore::wrapper::SharedMemory;

void onMouse(int event, int x, int y, int flags, void * userdata);

void onMouse(int event, int x, int y, int flags, void * userdata) {
    LMVP_UNUSED(flags);
    LMVP_UNUSED(userdata);

    if(event == cv::EVENT_MOUSEMOVE) {
        std::cout << "x: " << x << " y: " << y << std::endl;
    }
}

class VanishingPointLowPass {
public:
    VanishingPointLowPass(float dampingFactor) : dampingFactor_(dampingFactor), lastVanishingPoint_() { }

    std::shared_ptr<cv::Point2f> lowpass(std::shared_ptr<cv::Point2f> vanishingPoint) {
        if(!vanishingPoint) {
            //lastVanishingPoint_ = vanishingPoint;
            return vanishingPoint;
        }

        if(!lastVanishingPoint_) {
            lastVanishingPoint_ = vanishingPoint;
            return vanishingPoint;
        }

        cv::Point2f distance = *vanishingPoint - *lastVanishingPoint_;

        lastVanishingPoint_ = std::make_shared<cv::Point2f>(dampingFactor_ * distance + *lastVanishingPoint_);

        return lastVanishingPoint_;
    }

private:
    const float dampingFactor_;

    std::shared_ptr<cv::Point2f> lastVanishingPoint_;
};

class FailedAttachingToSharedMemoryException : public std::runtime_error {
public:
    FailedAttachingToSharedMemoryException(const std::string & name)
        : std::runtime_error("Failed attaching to shared memory " + name) {
    }
};

/**
 * Class linking a SharedImage to a cv::Mat. Attachment to the shared memory happens on construction.
 * Construction only succeeds when attaching to the shared memory was successful.
 */
class SharedImageCvAdapter {
public:
    SharedImageCvAdapter(const SharedImage & sharedImage) : sharedImageMemory_(), imageHeader_() {
        attachToSharedMemory(sharedImage);
    }

    cv::Mat fetchImage() {
        Lock lock(sharedImageMemory_);

        return imageHeader_.clone();
    }

private:
    void attachToSharedMemory(const SharedImage & sharedImage) {
        sharedImageMemory_ = SharedMemoryFactory::attachToSharedMemory(sharedImage.getName());

        if(!sharedImageMemory_->isValid()) {
            throw FailedAttachingToSharedMemoryException(sharedImage.getName());
        }

        imageHeader_ = cv::Mat(sharedImage.getHeight(), sharedImage.getWidth(),
                               CV_MAKETYPE(CV_8U, sharedImage.getBytesPerPixel()),
                               sharedImageMemory_->getSharedMemory());
    }

    std::shared_ptr<SharedMemory> sharedImageMemory_;
    cv::Mat imageHeader_;
};

class LaneMarkingsVanishingPointDetectionModule : public DataTriggeredConferenceClientModule {
public:
    LaneMarkingsVanishingPointDetectionModule(int32_t argc, char ** argv)
        : DataTriggeredConferenceClientModule(argc, argv, MODULE_NAME)
        , lowpass_(.125f)
        , sharedImageName_()
        , leftScanRegion_()
        , rightScanRegion_()
        , vanishingPointDetection_()
        , sharedImageAdapter_() {}

    void setUp() {
        sharedImageName_ = getSharedImageName();

        auto & kvConfig = getKeyValueConfiguration();

        leftScanRegion_.reset(new ScanRegion(
            kvConfig.getValue<ScanRegionBuilder>(MODULE_NAME + ".leftscanregion")));

        rightScanRegion_.reset(new ScanRegion(
            kvConfig.getValue<ScanRegionBuilder>(MODULE_NAME + ".rightscanregion")));

        vanishingPointDetection_.reset(new VanishingPointDetection(
            kvConfig.getValue<RoiBuilder>(MODULE_NAME + ".roi"),
            kvConfig.getValue<int>(MODULE_NAME + ".blurradius"),
            static_cast<uint8_t>(kvConfig.getValue<int>(MODULE_NAME + ".threshold")),
            *leftScanRegion_, *rightScanRegion_,
            kvConfig.getValue<ColumnIndex>(MODULE_NAME + ".maxlanemarkingwidth"),
            kvConfig.getValue<ColumnIndex>(MODULE_NAME + ".maxmarkingrowcount"),
            kvConfig.getValue<double>(MODULE_NAME + ".maxmarkinginclination"),
            kvConfig.getValue<size_t>(MODULE_NAME + ".minpixelcount")));

        if(DEBUG_IS_SET) {
            cv::namedWindow(MODULE_NAME);
            cv::setMouseCallback(MODULE_NAME, onMouse);
        }
    }

    void nextContainer(Container & container) {
        if(SharedImage::ID() == container.getDataType()) {
            SharedImage image = container.getData<SharedImage>();

            if(isDesiredSharedImage(image)) {
                if(!attachToSharedMemory(image)) {
                    return;
                }

                cv::Mat cvImage = sharedImageAdapter_->fetchImage();
                processImage(cvImage);
            }
        }
    }

    void tearDown() {
        if(DEBUG_IS_SET) {
            cv::destroyWindow(MODULE_NAME);
        }
    }

private:
    static const std::string MODULE_NAME;
    static const std::string CONFIG_NAME;

    /**
     * Run upon construction to fetch the desired sharedImage name from the configuration
     *
     * @return The sharedImageName configuration value. Empty string if not set.
     */
    std::string getSharedImageName() {
        try {
            return getKeyValueConfiguration().getValue<std::string>(MODULE_NAME + ".sharedimagename");
        }
        catch(const ValueForKeyNotFoundException & e) {
            CLOG << "Didn't find key " << MODULE_NAME + ".sharedimagename" << std::endl;
            return std::string();
        }
    }

    /**
     * Checks whether the given image has the name set in sharedImageName_. If it is not set, it is
     * overwritten with the name of the given shared image, so that this is used in the future.
     */
    bool isDesiredSharedImage(const SharedImage & image) {
        if(sharedImageName_.empty()) {
            CLOG << "no \"sharedImageName\" config value set, using shared image " << image.getName() << std::endl;
            sharedImageName_ = image.getName();
            return true;
        }

        return 0 == image.getName().compare(sharedImageName_);
    }

    /**
     * Try to attach to the shared memory, initializing the sharedImageAdapter_ member.
     *
     * @return true if attaching succeeded or already done, false if it failed
     */
    bool attachToSharedMemory(const SharedImage & image) {
        if(!sharedImageAdapter_) {
            try {
                sharedImageAdapter_.reset(new SharedImageCvAdapter(image));
            }
            catch(const FailedAttachingToSharedMemoryException & e) {
                return false;
            }
        }

        return true;
    }

#ifndef NDEBUG
    static void drawVanishingPoint(cv::Mat & image, const cv::Point2f & vanishingPoint, const cv::Scalar & color) {
        int radius = 10;
        float offset = 1.5f * static_cast<float>(radius);
        cv::circle(image, vanishingPoint, radius, color, 2);
        cv::line(image, vanishingPoint - cv::Point2f(0, offset), vanishingPoint + cv::Point2f(0, offset),
                 color, 1);
        cv::line(image, vanishingPoint - cv::Point2f(offset, 0), vanishingPoint + cv::Point2f(offset, 0),
                 color, 1);
    }
#endif

    void processImage(cv::Mat & image) {
        std::shared_ptr<cv::Point2f> vanishingPoint = vanishingPointDetection_->detectVanishingPoint(image);
        vanishingPoint = lowpass_.lowpass(vanishingPoint);

        if(DEBUG_IS_SET) {
            if(DEBUG_SHOW_SCAN_REGIONS) {
                leftScanRegion_->draw(image, cv::Scalar(0,212,89));
                rightScanRegion_->draw(image, cv::Scalar(212,195,0));
            }

            if(DEBUG_SHOW_VANISHING_POINT) {
                if(vanishingPoint) {
                    drawVanishingPoint(image, *vanishingPoint, cv::Scalar(0,0,255));
                }
            }

            cv::imshow(MODULE_NAME, image);

            int waitDelay = pause_ ? WAIT_DELAY_PAUSE : WAIT_DELAY_PLAY;
            char key = cv::waitKey(waitDelay);
            if(key == ' ') {
                pause_ = !pause_;
            }
        }
    }

    static const int WAIT_DELAY_PLAY = 25,
                     WAIT_DELAY_PAUSE = 0;

    bool pause_ = false;

    VanishingPointLowPass lowpass_;

    std::string sharedImageName_;

    // the following members are defined as unique_ptr to allow for delayed initialization
    // (in setUp() instead of the constructor)
    std::unique_ptr<ScanRegion> leftScanRegion_,
                                rightScanRegion_;

    std::unique_ptr<VanishingPointDetection> vanishingPointDetection_;
    std::unique_ptr<SharedImageCvAdapter> sharedImageAdapter_;

};

const std::string LaneMarkingsVanishingPointDetectionModule::MODULE_NAME =
                "vanishingpoint";

}

int32_t main(int32_t argc, char ** argv) {
    using namespace lmvp;

    LaneMarkingsVanishingPointDetectionModule vpModule(argc, argv);
    vpModule.runModule();

    return EXIT_SUCCESS;
}