/**
 * This file is a wrapper of Basler API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
 * ROS node.
 */
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// Basler API
#include <pylon/PylonIncludes.h>
// #include "acA1600_60gc_pylon_gen/BaslerCamera.h"
// #include "acA1600_60gc_pylon_gen/BaslerCameraArray.h"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "camera_adapter.h"

// Check if available and readable and writable; and combinations of them
#define CHECK_A(node){if (!IsAvailable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not available." << std::endl; \
    return false; \
}}
#define CHECK_R(node){if (!IsReadable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not readable." << std::endl; \
    return false; \
}}
#define CHECK_W(node){if (!IsWritable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not writable." << std::endl; \
    return false; \
}}
#define CHECK_AR(node) {CHECK_A(node); CHECK_R(node); }
#define CHECK_AW(node) {CHECK_A(node); CHECK_W(node); }
#define CHECK_ARW(node) {CHECK_A(node); CHECK_R(node); CHECK_W(node); }

#define CHECK_POINTER(pointer){if (!pointer) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #pointer << " is not available." << std::endl; \
    return false; \
}}

// Reference to basler camera to be handled
std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> pBasler;
std::string camera_name = "Default:acA1600-60gc";
/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    if(pBasler)
    {
        // camera_name = std::string(pBasler->GetDeviceInfo().GetFriendlyName())+std::string(":")+std::string(pBasler->GetDeviceInfo().GetModelName());
        camera_name = std::string("Basler ") + std::string(pBasler->GetDeviceInfo().GetModelName());
    }
    return camera_name;
}

/**
 * @brief Return image type to store it correctly
 */
std::string getType()
{
    return "visible";
}

/**
 * @brief Function that handle all Pylon and Camera initializacion and configuration.
 * @param frame_rate frames per second to capture with camera
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate, std::string camera_ip)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized. 
        Pylon::PylonInitialize();
        //Basler with IP: '192.168.4.5'
        // This takes first abailable
        pBasler = std::unique_ptr<Pylon::CBaslerUniversalInstantCamera>(new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice()));
        
        // Not working :( creates camera that cannot get images
        // This access it by IP
        // Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance(); 
        // Pylon::CDeviceInfo info; 
        // info.SetDeviceClass(Pylon::BaslerGigEDeviceClass); 
        // info.SetIpAddress(camera_ip.c_str()); 
        // Pylon::IPylonDevice* device = tlFactory.CreateDevice(info);
        // pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(device));

        if (!pBasler)
        {
            std::cerr << "[BaslerAdapter::initCamera] Camera with configured IP ("<<camera_ip<<") was not found."  << std::endl;
            return false;
        }
        else
        {
            std::cerr << "[BaslerAdapter::initCamera] Opening camera with: " << std::endl;
            std::cout << "\t\t· Model Name " << pBasler->GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "\t\t· Friendly Name: " << pBasler->GetDeviceInfo().GetFriendlyName() << std::endl;
            std::cout << "\t\t· Current IP Addr: " << pBasler->GevCurrentIPAddress.ToStringOrDefault("<not readable>") << std::endl;
            std::cout << "\t\t· Requested IP: " << camera_ip << std::endl;
        }

        pBasler->Open();

        // With autoexposure the camera is not triggering?¿
        // Enable Auto Exposure (set to Continuous mode)
        CHECK_ARW(pBasler->ExposureAuto)
        std::cout << "[BaslerAdapter::initCamera] Autoexposure enabled in continuous mode." << std::endl;
        pBasler->ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
        // ExposureAutoEnums e = camera.ExposureAuto.GetValue();
        CHECK_ARW(pBasler->AutoTargetValue)
        pBasler->AutoTargetValue.SetValue(70);
        std::cout << "[BaslerAdapter::initCamera] Current autoexposure set to target value: " << pBasler->AutoTargetValue.GetValue() << std::endl;
        // To set a fixed exposure time, disable auto exposure and set exposure time manually
        
        CHECK_ARW(pBasler->BalanceWhiteAuto)
        std::cout << "[BaslerAdapter::initCamera] Auto Balance White enabled in continuous mode." << std::endl;
        pBasler->BalanceWhiteAuto.SetValue(Basler_UniversalCameraParams::BalanceWhiteAuto_Continuous);


        pBasler->AcquisitionFrameRateEnable.SetValue(true);
        pBasler->AcquisitionFrameRateAbs.SetValue(frame_rate);

        ////////////////////////////////////
        //  Metadata extraction enabling  //
        ////////////////////////////////////

        // GenApi::StringList_t entries;
        // pBasler->ChunkSelector.GetSymbolics(entries);
        // std::cout << "Chunks disponibles:" << std::endl;
        // for (auto &entry : entries)
        //     std::cout << " - " << entry << std::endl;

        CHECK_ARW(pBasler->ChunkModeActive);
        pBasler->ChunkModeActive.SetValue(true);

        pBasler->ChunkSelector.SetValue("Timestamp");
        pBasler->ChunkEnable.SetValue(true);

        pBasler->ChunkSelector.SetValue("Framecounter");
        pBasler->ChunkEnable.SetValue(true);

        pBasler->ChunkSelector.SetValue("ExposureTime");
        pBasler->ChunkEnable.SetValue(true);

        pBasler->ChunkSelector.SetValue("GainAll");
        pBasler->ChunkEnable.SetValue(true);

        // Enable PTP and set camera as slave
        // pBasler->GevIEEE1588.SetValue(true);

        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::initCamera] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
}

/**
 * @brief Function that handle acquisition init. Note that it has to start after all configuration is set.
 * @return true or false depending on image acquisition result
 */
bool beginAcquisition()
{
    CHECK_POINTER(pBasler);
    pBasler->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    return true;
}

/**
 * @brief Function that handle acquisition end.
 */
bool endAcquisition()
{
    CHECK_POINTER(pBasler);
    pBasler->StopGrabbing();
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
    std::cout << "No master configuration for now :)" << std::endl;
    return true;

    CHECK_POINTER(pBasler);
    try
    {
        // Select Line 2 (output line)
        CHECK_AW(pBasler->LineSelector);
        pBasler->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Out1);

        // Set it as output
        CHECK_AW(pBasler->LineMode);
        pBasler->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Output);

        // Set the source signal to User Output 1
        CHECK_AW(pBasler->LineSource);
        pBasler->LineSource.SetValue(Basler_UniversalCameraParams::LineSource_ExposureActive);
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "[BaslerAdapter::setAsMaster] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }

    // Setup continuous acquisition as trigger
    // In theory continuous acquisition is already set by default
    // pBasler->AcquisitionMode.TrySetValue( Basler_UniversalCameraParams::AcquisitionMode_Continuous );
    std::cout << "[BaslerAdapter::setAsMaster] Configured internal trigger and signal output." << std::endl;

    return true;   
}

/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave()
{
    CHECK_POINTER(pBasler);
    std::cerr << "[BaslerAdapter::setAsSlave] ************************************" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] * EMPTY FUNCTION. NOT IMPLEMENTED. *" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] *  Calling setAsMaster() function  *" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] ************************************" << std::endl;
    bool result = setAsMaster();
    return result;   
}

/**
 * @brief Function that handles image acquisition. Returns image in CV format.
 * @param image CV mat reference to be filled with image
 * @return true or false depending on image acquisition
 */
bool acquireImage(cv::Mat& image, uint64_t& timestamp, ImageMetadata& metadata)
{
    
    CHECK_POINTER(pBasler);
    try
    {
        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;
        cv::Mat openCvImage;

        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        Pylon::CPylonImage pylonImage;

        // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
        pBasler->RetrieveResult( 2000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
        // Image grabbed successfully?
        if (!ptrGrabResult)
        {
            std::cout << "[BaslerAdapter::acquireImage] No grab result reference." << std::endl;
            return false;
        }
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            // const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();

            formatConverter.Convert(pylonImage, ptrGrabResult);
            // needs to be cloned so to not keep pointing to local raw data that will be destroyed after function finishes
            image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer()).clone();
            timestamp = ptrGrabResult->GetTimeStamp();

            /*****************************************
            **   Extract metadata from chunk data   **
            ******************************************/
            metadata.width = ptrGrabResult->GetWidth();
            metadata.height = ptrGrabResult->GetHeight();
            
            GenApi::CEnumerationPtr pixelFormatNode(pBasler->GetNodeMap().GetNode("PixelFormat"));
            if (GenApi::IsReadable(pixelFormatNode))
            {
                std::string pixelFormatName = std::string(pixelFormatNode->GetCurrentEntry()->GetSymbolic());
                metadata.pixelFormat = pixelFormatName;
            }

            GenApi::INodeMap& chunkDataMap = ptrGrabResult->GetChunkDataNodeMap();

            // Timestamp
            GenApi::CIntegerPtr chunkTimestamp(chunkDataMap.GetNode("ChunkTimestamp"));
            if (GenApi::IsReadable(chunkTimestamp))
            {
                int64_t ts = chunkTimestamp->GetValue();
                // std::cout << "Timestamp: " << ts << std::endl;
                metadata.timestamp = ts;
            }
        
            // Framecounter
            GenApi::CIntegerPtr chunkFrameCounter(chunkDataMap.GetNode("ChunkFramecounter"));
            if (GenApi::IsReadable(chunkFrameCounter))
            {
                int64_t frameCounter = chunkFrameCounter->GetValue();
                // std::cout << "Frame #: " << frameCounter << std::endl;
                metadata.frameCounter = frameCounter;
            }
        
            // ExposureTime
            GenApi::CFloatPtr chunkExposure(chunkDataMap.GetNode("ChunkExposureTime"));
            if (GenApi::IsReadable(chunkExposure))
            {
                double exposure = chunkExposure->GetValue();
                // std::cout << "Exposure: " << exposure << " µs" << std::endl;
                metadata.exposureTime = exposure;
            }
        
            // GainAll
            GenApi::CFloatPtr chunkGain(chunkDataMap.GetNode("ChunkGainAll"));
            if (GenApi::IsReadable(chunkGain))
            {
                double gain = chunkGain->GetValue();
                // std::cout << "GainAll: " << gain << " dB" << std::endl;
                metadata.gain = gain;
            }

            // Gain
            GenApi::CFloatPtr gainNode(pBasler->GetNodeMap().GetNode("Gain"));
            if (GenApi::IsReadable(gainNode))
            {
                double gain = gainNode->GetValue();
                // std::cout << "[BaslerAdapter::acquireImage] Gain actual: " << gain << " dB" << std::endl;
                metadata.gain = gain;
            }
            metadata.systemTime = getTimeTag();
        }
        else
        {
            std::cout << "[BaslerAdapter::acquireImage] Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            return false;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "[BaslerAdapter::acquireImage] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
    return true;   
}

/**
 * @brief Function that handle all camera de-initialization, port closing and Pylon clean finishing.
 * @return true or false depending on image acquisition
 */
bool closeCamera()
{
    std::cout << "[BaslerAdapter::closeCamera] Close camera requested." << std::endl;
    // Deinitialize Basler
    if (pBasler)
    {
        endAcquisition();
        pBasler->Close();
        pBasler.reset();
    }

    // Releases all pylon resources. 
    Pylon::PylonTerminate(); 

    return true;
}