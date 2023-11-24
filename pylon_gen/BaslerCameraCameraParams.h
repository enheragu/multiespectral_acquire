//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2023 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief A parameter class containing all parameters as members that are available for acA1600-60gc

Sources:
acA1600-60gc 106204-22;U;acA1600_60g;V5.0-0;1
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_BASLERCAMERACAMERAPARAMS_H
#define BASLER_PYLON_BASLERCAMERACAMERAPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the a control interface and related enumeration types for acA1600-60gc
namespace Pylon
{
namespace BaslerCameraCameraParams_Params
{
    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************
    //! Valid values for AcquisitionMode
    enum AcquisitionModeEnums
    {
        AcquisitionMode_Continuous,  //!< Sets the acquisition mode to continuous - Applies to: acA1600-60gc
        AcquisitionMode_SingleFrame  //!< Sets the acquisition mode to single frame - Applies to: acA1600-60gc
    };

    //! Valid values for AcquisitionStatusSelector
    enum AcquisitionStatusSelectorEnums
    {
        AcquisitionStatusSelector_AcquisitionActive,  //!< Device is currently doing an acquisition of one or many frames - Applies to: acA1600-60gc
        AcquisitionStatusSelector_AcquisitionTransfer,  //!< Device is currently transferring an acquisition of one or many frames - Applies to: acA1600-60gc
        AcquisitionStatusSelector_AcquisitionTriggerWait,  //!< Device is currently waiting for a trigger for the capture of one or many frames - Applies to: acA1600-60gc
        AcquisitionStatusSelector_ExposureActive,  //!< Device is doing the exposure of a frame - Applies to: acA1600-60gc
        AcquisitionStatusSelector_FrameActive,  //!< Device is currently doing the capture of a frame - Applies to: acA1600-60gc
        AcquisitionStatusSelector_FrameTransfer,  //!< Device is currently transferring a frame - Applies to: acA1600-60gc
        AcquisitionStatusSelector_FrameTriggerWait,  //!< Device is currently waiting for a Frame trigger - Applies to: acA1600-60gc
        AcquisitionStatusSelector_LineTriggerWait  //!< Device is currently waiting for a line trigger - Applies to: acA1600-60gc
    };

    //! Valid values for AutoFunctionAOISelector
    enum AutoFunctionAOISelectorEnums
    {
        AutoFunctionAOISelector_AOI1,  //!< Selects Auto Function AOI 1 - Applies to: acA1600-60gc
        AutoFunctionAOISelector_AOI2  //!< Selects Auto Function AOI 2 - Applies to: acA1600-60gc
    };

    //! Valid values for AutoFunctionProfile
    enum AutoFunctionProfileEnums
    {
        AutoFunctionProfile_ExposureMinimum,  //!< Exposure Time at minimum - Applies to: acA1600-60gc
        AutoFunctionProfile_GainMinimum  //!< Keep gain at minimum - Applies to: acA1600-60gc
    };

    //! Valid values for BalanceRatioSelector
    enum BalanceRatioSelectorEnums
    {
        BalanceRatioSelector_Blue,  //!< Selects the blue balance ratio control for adjustment - Applies to: acA1600-60gc
        BalanceRatioSelector_Green,  //!< Selects the green balance ratio control for adjustment - Applies to: acA1600-60gc
        BalanceRatioSelector_Red  //!< Selects the red balance ratio control for adjustment - Applies to: acA1600-60gc
    };

    //! Valid values for BalanceWhiteAuto
    enum BalanceWhiteAutoEnums
    {
        BalanceWhiteAuto_Continuous,  //!< Sets operation mode to 'continuous' - Applies to: acA1600-60gc
        BalanceWhiteAuto_Off,  //!< Disables the Balance White Auto function - Applies to: acA1600-60gc
        BalanceWhiteAuto_Once  //!< Sets operation mode to 'once' - Applies to: acA1600-60gc
    };

    //! Valid values for BinningModeHorizontal
    enum BinningModeHorizontalEnums
    {
        BinningModeHorizontal_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for BinningModeVertical
    enum BinningModeVerticalEnums
    {
        BinningModeVertical_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for BlackLevelSelector
    enum BlackLevelSelectorEnums
    {
        BlackLevelSelector_All  //!< Selects all black level controls for adjustment - Applies to: acA1600-60gc
    };

    //! Valid values for ChunkPixelFormat
    enum ChunkPixelFormatEnums
    {
        ChunkPixelFormat_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ChunkSelector
    enum ChunkSelectorEnums
    {
        ChunkSelector_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ColorAdjustmentSelector
    enum ColorAdjustmentSelectorEnums
    {
        ColorAdjustmentSelector_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ColorTransformationSelector
    enum ColorTransformationSelectorEnums
    {
        ColorTransformationSelector_RGBtoRGB  //!< Matrix color transformation from RGB to RGB - Applies to: acA1600-60gc
    };

    //! Valid values for ColorTransformationValueSelector
    enum ColorTransformationValueSelectorEnums
    {
        ColorTransformationValueSelector_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ContrastMode
    enum ContrastModeEnums
    {
        ContrastMode_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for CounterEventSource
    enum CounterEventSourceEnums
    {
        CounterEventSource_FrameTrigger  //!< Counts the number of Frame Trigger - Applies to: acA1600-60gc
    };

    //! Valid values for CounterResetSource
    enum CounterResetSourceEnums
    {
        CounterResetSource_Line1,  //!< Selects line 1 as the source for counter reset - Applies to: acA1600-60gc
        CounterResetSource_Off,  //!< Disables counter reset - Applies to: acA1600-60gc
        CounterResetSource_Software  //!< Selects software command as the source for counter reset - Applies to: acA1600-60gc
    };

    //! Valid values for CounterSelector
    enum CounterSelectorEnums
    {
        CounterSelector_Counter1,  //!< Selects counter 1 for configuration - Applies to: acA1600-60gc
        CounterSelector_Counter2  //!< Selects counter 2 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for DeviceScanType
    enum DeviceScanTypeEnums
    {
        DeviceScanType_Areascan,  //!< Indicates that the device has an area scan type of sensor - Applies to: acA1600-60gc
        DeviceScanType_Linescan  //!< Indicates that the device has an Line scan type of sensor - Applies to: acA1600-60gc
    };

    //! Valid values for EventNotification
    enum EventNotificationEnums
    {
        EventNotification_GenICamEvent,  //!< Sets the event notification type to GenICam event - Applies to: acA1600-60gc
        EventNotification_Off,  //!< Sets event notification to off - Applies to: acA1600-60gc
        EventNotification_On  //!< Sets the enables the event notification - Applies to: acA1600-60gc
    };

    //! Valid values for EventSelector
    enum EventSelectorEnums
    {
        EventSelector_AcquisitionStart,  //!< Selects the acquisition start trigger event for enabling - Applies to: acA1600-60gc
        EventSelector_AcquisitionStartOvertrigger,  //!< Selects the acquisition start overtrigger event for enabling - Applies to: acA1600-60gc
        EventSelector_AcquisitionWait,  //!< Selects the acquisition wait trigger event for enabling - Applies to: acA1600-60gc
        EventSelector_EventOverrun,  //!< Selects the event overrun event for enabling - Applies to: acA1600-60gc
        EventSelector_ExposureEnd,  //!< Selects the end of exposure event for enabling - Applies to: acA1600-60gc
        EventSelector_FrameStart,  //!< Selects the frame start trigger event for enabling - Applies to: acA1600-60gc
        EventSelector_FrameStartOvertrigger,  //!< Selects the frame start overtrigger event for enabling - Applies to: acA1600-60gc
        EventSelector_FrameWait  //!< Selects the frame wait trigger event for enabling - Applies to: acA1600-60gc
    };

    //! Valid values for ExpertFeatureAccessSelector
    enum ExpertFeatureAccessSelectorEnums
    {
        ExpertFeatureAccessSelector_ExpertFeature1,  //!< Selects the Expert Feature 1 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature1_Legacy,  //!< Selects the Expert Feature 1 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature2,  //!< Selects the Expert Feature 2 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature3,  //!< Selects the Expert Feature 3 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature4,  //!< Selects the Expert Feature 4 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature5,  //!< Selects the Expert Feature 5 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature6,  //!< Selects the Expert Feature 6 for configuration - Applies to: acA1600-60gc
        ExpertFeatureAccessSelector_ExpertFeature7  //!< Selects the Expert Feature 7 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for ExposureAuto
    enum ExposureAutoEnums
    {
        ExposureAuto_Continuous,  //!< Sets operation mode to 'continuous' - Applies to: acA1600-60gc
        ExposureAuto_Off,  //!< Disables the Exposure Auto function - Applies to: acA1600-60gc
        ExposureAuto_Once  //!< Sets operation mode to 'once' - Applies to: acA1600-60gc
    };

    //! Valid values for ExposureMode
    enum ExposureModeEnums
    {
        ExposureMode_Timed  //!< Sets the exposure mode to timed - Applies to: acA1600-60gc
    };

    //! Valid values for FeatureSet
    enum FeatureSetEnums
    {
        FeatureSet_Basic,  //!< The 'Basic' camera description file provides nearly all features - Applies to: acA1600-60gc
        FeatureSet_Full  //!< The 'Full' camera description file provides all features - Applies to: acA1600-60gc
    };

    //! Valid values for FileOpenMode
    enum FileOpenModeEnums
    {
        FileOpenMode_Read,  //!< Selects read-only open mode - Applies to: acA1600-60gc
        FileOpenMode_Write  //!< Selects write-only open mode - Applies to: acA1600-60gc
    };

    //! Valid values for FileOperationSelector
    enum FileOperationSelectorEnums
    {
        FileOperationSelector_Close,  //!< Closes the file selected by FileSelector - Applies to: acA1600-60gc
        FileOperationSelector_Open,  //!< Opens the file selected by FileSelector - Applies to: acA1600-60gc
        FileOperationSelector_Read,  //!< Reads data from the selected file - Applies to: acA1600-60gc
        FileOperationSelector_Write  //!< Writes data to the selected file - Applies to: acA1600-60gc
    };

    //! Valid values for FileOperationStatus
    enum FileOperationStatusEnums
    {
        FileOperationStatus_Failure,  //!< Failing file operation - Applies to: acA1600-60gc
        FileOperationStatus_Success  //!< Successful file operation - Applies to: acA1600-60gc
    };

    //! Valid values for FileSelector
    enum FileSelectorEnums
    {
        FileSelector_ExpertFeature7File,  //!< Selects the file 'Expert Feature 7 File' - Applies to: acA1600-60gc
        FileSelector_UserData,  //!< Selects the file 'User Data' - Applies to: acA1600-60gc
        FileSelector_UserGainShading1,  //!< Selects the file 'User Gain Shading 1' - Applies to: acA1600-60gc
        FileSelector_UserGainShading2,  //!< Selects the file 'User Gain Shading 2' - Applies to: acA1600-60gc
        FileSelector_UserOffsetShading1,  //!< Selects the file 'User Offset Shading 1' - Applies to: acA1600-60gc
        FileSelector_UserOffsetShading2,  //!< Selects the file 'User Offset Shading 2' - Applies to: acA1600-60gc
        FileSelector_UserSet1,  //!< Selects the file 'User Set 1' - Applies to: acA1600-60gc
        FileSelector_UserSet2,  //!< Selects the file 'User Set 2' - Applies to: acA1600-60gc
        FileSelector_UserSet3  //!< Selects the file 'User Set 3' - Applies to: acA1600-60gc
    };

    //! Valid values for GainAuto
    enum GainAutoEnums
    {
        GainAuto_Continuous,  //!< Sets operation mode to 'continuous' - Applies to: acA1600-60gc
        GainAuto_Off,  //!< Disables the Gain Auto function - Applies to: acA1600-60gc
        GainAuto_Once  //!< Sets operation mode to 'once' - Applies to: acA1600-60gc
    };

    //! Valid values for GainSelector
    enum GainSelectorEnums
    {
        GainSelector_All,  //!< Selects all gain controls for adjustment - Applies to: acA1600-60gc
        GainSelector_AnalogAll,  //!< Selects all analog gain controls for adjustment - Applies to: acA1600-60gc
        GainSelector_DigitalAll  //!< Selects all digital gain controls for adjustment - Applies to: acA1600-60gc
    };

    //! Valid values for GammaSelector
    enum GammaSelectorEnums
    {
        GammaSelector_User,  //!< Sets gamma to user defined curve - Applies to: acA1600-60gc
        GammaSelector_sRGB  //!< Sets gamma to fixed sRGB curve - Applies to: acA1600-60gc
    };

    //! Valid values for GevCCP
    enum GevCCPEnums
    {
        GevCCP_Control,  //!< Sets the control channel privilege feature to control - Applies to: acA1600-60gc
        GevCCP_Exclusive,  //!< Sets the control channel privilege feature to exclusive - Applies to: acA1600-60gc
        GevCCP_ExclusiveControl  //!< Sets the control channel privilege feature to exclusive control - Applies to: acA1600-60gc
    };

    //! Valid values for GevIEEE1588Status
    enum GevIEEE1588StatusEnums
    {
        GevIEEE1588Status_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for GevInterfaceSelector
    enum GevInterfaceSelectorEnums
    {
        GevInterfaceSelector_NetworkInterface0  //!< Selects network interface 0 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for GevStreamChannelSelector
    enum GevStreamChannelSelectorEnums
    {
        GevStreamChannelSelector_StreamChannel0  //!< Selects stream channel 0 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for LUTSelector
    enum LUTSelectorEnums
    {
        LUTSelector_Luminance  //!< Selects the luminance LUT for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for LastError
    enum LastErrorEnums
    {
        LastError_InsufficientTriggerWidth,  //!< The trigger width was too short - Applies to: acA1600-60gc
        LastError_InvalidParameter,  //!< Indicates that a parameter was set to an invalid value - Applies to: acA1600-60gc
        LastError_NoError,  //!< Indicates that no error was detected - Applies to: acA1600-60gc
        LastError_Overtrigger,  //!< Indicates that the camera was overtriggered - Applies to: acA1600-60gc
        LastError_PowerFailure,  //!< Indicates that the power supply is not sufficient - Applies to: acA1600-60gc
        LastError_Userset  //!< Indicates an error was detected while loading a userset - Applies to: acA1600-60gc
    };

    //! Valid values for LightSourceSelector
    enum LightSourceSelectorEnums
    {
        LightSourceSelector_Custom,  //!< Allows using a custom defined color transformation matrix - Applies to: acA1600-60gc
        LightSourceSelector_Daylight,  //!< The matrix is optimized for image acquisition with daylight of 5000 K - Applies to: acA1600-60gc
        LightSourceSelector_Daylight6500K,  //!< The matrix is optimized for image acquisition with daylight of 6500 K - Applies to: acA1600-60gc
        LightSourceSelector_Off,  //!< No matrix color transformation for specific light source is performed - Applies to: acA1600-60gc
        LightSourceSelector_Tungsten  //!< The matrix is optimized for image acquisition with tungsten incandescent light (3100 K) - Applies to: acA1600-60gc
    };

    //! Valid values for LineFormat
    enum LineFormatEnums
    {
        LineFormat_LVDS,  //!< Sets the electrical configuration of the selected line to LVDS - Applies to: acA1600-60gc
        LineFormat_NoConnect,  //!< Sets the electrical configuration of the selected line to not connected - Applies to: acA1600-60gc
        LineFormat_OptoCoupled,  //!< Sets the electrical configuration of the selected line to opto-coupled - Applies to: acA1600-60gc
        LineFormat_RS422,  //!< Sets the electrical configuration of the selected line to RS-422 - Applies to: acA1600-60gc
        LineFormat_TTL,  //!< Sets the electrical configuration of the selected line to TTL - Applies to: acA1600-60gc
        LineFormat_TriState  //!< Sets the electrical configuration of the selected line to tri-state - Applies to: acA1600-60gc
    };

    //! Valid values for LineLogic
    enum LineLogicEnums
    {
        LineLogic_Negative,  //!< Applies to: acA1600-60gc
        LineLogic_Positive  //!< Applies to: acA1600-60gc
    };

    //! Valid values for LineMode
    enum LineModeEnums
    {
        LineMode_Input,  //!< Sets the mode for the selected line to input - Applies to: acA1600-60gc
        LineMode_Output  //!< Sets the mode for the selected line to output - Applies to: acA1600-60gc
    };

    //! Valid values for LineSelector
    enum LineSelectorEnums
    {
        LineSelector_Line1,  //!< Selects line 1 for configuration - Applies to: acA1600-60gc
        LineSelector_Out1  //!< Selects output line 1 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for LineSource
    enum LineSourceEnums
    {
        LineSource_ExposureActive
        //LineSource_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ParameterSelector
    enum ParameterSelectorEnums
    {
        ParameterSelector_Gain  //!< Selects the gain limits for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for PixelColorFilter
    enum PixelColorFilterEnums
    {
        PixelColorFilter_Bayer_RG,  //!< Indicates that the Bayer filter has an RG/GB alignment to the pixels in the acquired images - Applies to: acA1600-60gc
        PixelColorFilter_None  //!< Indicates that no Bayer filter is present on the camera - Applies to: acA1600-60gc
    };

    //! Valid values for PixelFormat
    enum PixelFormatEnums
    {
        PixelFormat_BGR8Packed,  //!< Sets the pixel format to BGR 8 Packed - Applies to: acA1600-60gc
        PixelFormat_BayerBG10,  //!< Sets the pixel format to Bayer BG 10 - Applies to: acA1600-60gc
        PixelFormat_BayerBG12,  //!< Sets the pixel format to Bayer BG 12 - Applies to: acA1600-60gc
        PixelFormat_BayerBG12Packed,  //!< Sets the pixel format to Bayer BG 12 Packed - Applies to: acA1600-60gc
        PixelFormat_BayerBG8,  //!< Sets the pixel format to Bayer BG 8 - Applies to: acA1600-60gc
        PixelFormat_BayerGB10,  //!< Sets the pixel format to Bayer GB 10 - Applies to: acA1600-60gc
        PixelFormat_BayerGB12,  //!< Sets the pixel format to Bayer GB 12 - Applies to: acA1600-60gc
        PixelFormat_BayerGB12Packed,  //!< Sets the pixel format to Bayer GB 12 Packed - Applies to: acA1600-60gc
        PixelFormat_BayerGB8,  //!< Sets the pixel format to Bayer GB 8 - Applies to: acA1600-60gc
        PixelFormat_BayerGR10,  //!< Sets the pixel format to Bayer GR 10 - Applies to: acA1600-60gc
        PixelFormat_BayerGR12,  //!< Sets the pixel format to Bayer GR 12 - Applies to: acA1600-60gc
        PixelFormat_BayerGR12Packed,  //!< Sets the pixel format to Bayer GR 12 Packed - Applies to: acA1600-60gc
        PixelFormat_BayerGR8,  //!< Sets the pixel format to Bayer GR 8 - Applies to: acA1600-60gc
        PixelFormat_BayerRG10,  //!< Sets the pixel format to Bayer RG 10 - Applies to: acA1600-60gc
        PixelFormat_BayerRG12,  //!< Sets the pixel format to Bayer RG 12 - Applies to: acA1600-60gc
        PixelFormat_BayerRG12Packed,  //!< Sets the pixel format to Bayer RG 12 Packed - Applies to: acA1600-60gc
        PixelFormat_BayerRG8,  //!< Sets the pixel format to Bayer RG 8 - Applies to: acA1600-60gc
        PixelFormat_Mono10,  //!< Sets the pixel format to Mono 10 - Applies to: acA1600-60gc
        PixelFormat_Mono12,  //!< Sets the pixel format to Mono 12 - Applies to: acA1600-60gc
        PixelFormat_Mono12Packed,  //!< Sets the pixel format to Mono 12 Packed - Applies to: acA1600-60gc
        PixelFormat_Mono8,  //!< Sets the pixel format to Mono 8 - Applies to: acA1600-60gc
        PixelFormat_RGB8Packed,  //!< Sets the pixel format to RGB 8 Packed - Applies to: acA1600-60gc
        PixelFormat_YUV422Packed,  //!< Sets the pixel format to YUV 422 Packed - Applies to: acA1600-60gc
        PixelFormat_YUV422_YUYV_Packed  //!< Sets the pixel format to YUV 422 (YUYV) Packed - Applies to: acA1600-60gc
    };

    //! Valid values for PixelSize
    enum PixelSizeEnums
    {
        PixelSize_Bpp12,  //!< Indicates that the depth of the pixel values in the acquired images is 12 bits per pixel - Applies to: acA1600-60gc
        PixelSize_Bpp16,  //!< Indicates that the depth of the pixel values in the acquired images is 16 bits per pixel - Applies to: acA1600-60gc
        PixelSize_Bpp8  //!< Indicates that the depth of the pixel values in the acquired images is 8 bits per pixel - Applies to: acA1600-60gc
    };

    //! Valid values for SequenceAddressBitSelector
    enum SequenceAddressBitSelectorEnums
    {
        SequenceAddressBitSelector_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for SequenceAddressBitSource
    enum SequenceAddressBitSourceEnums
    {
        SequenceAddressBitSource_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for SequenceAdvanceMode
    enum SequenceAdvanceModeEnums
    {
        SequenceAdvanceMode_Auto,  //!< Automatic sequence set advance - Applies to: acA1600-60gc
        SequenceAdvanceMode_Controlled,  //!< Sequence set advance controlled by settable source - Applies to: acA1600-60gc
        SequenceAdvanceMode_FreeSelection  //!< The sequence sets are selected according to the states of the input lines - Applies to: acA1600-60gc
    };

    //! Valid values for SequenceControlSelector
    enum SequenceControlSelectorEnums
    {
        SequenceControlSelector_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for SequenceControlSource
    enum SequenceControlSourceEnums
    {
        SequenceControlSource_Todo  //!< TODO - Applies to: acA1600-60gc
    };

    //! Valid values for ShutterMode
    enum ShutterModeEnums
    {
        ShutterMode_Global,  //!< Sets the shutter mode to global shutter - Applies to: acA1600-60gc
        ShutterMode_GlobalResetRelease,  //!< Sets the shutter mode to global reset release shutter - Applies to: acA1600-60gc
        ShutterMode_Rolling  //!< Sets the shutter mode to rolling shutter - Applies to: acA1600-60gc
    };

    //! Valid values for SyncUserOutputSelector
    enum SyncUserOutputSelectorEnums
    {
        SyncUserOutputSelector_SyncUserOutput1  //!< Selects user settable synchronous output signal 1 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for TestImageSelector
    enum TestImageSelectorEnums
    {
        TestImageSelector_Off,  //!< Sets the camera's test image generation capability to disabled - Applies to: acA1600-60gc
        TestImageSelector_Testimage1,  //!< Sets the camera to generate and transmit test images with a test image 1 pattern - Applies to: acA1600-60gc
        TestImageSelector_Testimage2,  //!< Sets the camera to generate and transmit test images with a test image 2 pattern - Applies to: acA1600-60gc
        TestImageSelector_Testimage3,  //!< Sets the camera to generate and transmit test images with a test image 3 pattern - Applies to: acA1600-60gc
        TestImageSelector_Testimage4,  //!< Sets the camera to generate and transmit test images with a test image 4 pattern - Applies to: acA1600-60gc
        TestImageSelector_Testimage5,  //!< Sets the camera to generate and transmit test images with a test image 5 pattern - Applies to: acA1600-60gc
        TestImageSelector_Testimage6  //!< Sets the camera to generate and transmit test images with a test image 6 pattern - Applies to: acA1600-60gc
    };

    //! Valid values for TimerSelector
    enum TimerSelectorEnums
    {
        TimerSelector_Timer1  //!< Selects timer 1 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for TimerTriggerActivation
    enum TimerTriggerActivationEnums
    {
        TimerTriggerActivation_RisingEdge  //!< Sets the type of signal change that will start the timer to rising edge - Applies to: acA1600-60gc
    };

    //! Valid values for TimerTriggerSource
    enum TimerTriggerSourceEnums
    {
        TimerTriggerSource_ExposureStart  //!< Sets the source signal for the selected timer to exposure active - Applies to: acA1600-60gc
    };

    //! Valid values for TriggerActivation
    enum TriggerActivationEnums
    {
        TriggerActivation_FallingEdge,  //!< Sets the selected trigger to become active on the falling edge of the source signal - Applies to: acA1600-60gc
        TriggerActivation_RisingEdge  //!< Sets the selected trigger to become active on the rising edge of the source signal - Applies to: acA1600-60gc
    };

    //! Valid values for TriggerMode
    enum TriggerModeEnums
    {
        TriggerMode_Off,  //!< Sets the mode for the selected trigger to off - Applies to: acA1600-60gc
        TriggerMode_On  //!< Sets the mode for the selected trigger to on - Applies to: acA1600-60gc
    };

    //! Valid values for TriggerSelector
    enum TriggerSelectorEnums
    {
        TriggerSelector_AcquisitionStart,  //!< Selects the acquisition start trigger for configuration - Applies to: acA1600-60gc
        TriggerSelector_FrameStart  //!< Selects the frame start trigger for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for TriggerSource
    enum TriggerSourceEnums
    {
        TriggerSource_Action1,  //!< Sets the signal source for the selected trigger to action command signal 1 - Applies to: acA1600-60gc
        TriggerSource_Line1,  //!< Sets the signal source for the selected trigger to line 1 - Applies to: acA1600-60gc
        TriggerSource_Software  //!< Sets the software trigger as the signal source for the selected trigger - Applies to: acA1600-60gc
    };

    //! Valid values for UserDefinedValueSelector
    enum UserDefinedValueSelectorEnums
    {
        UserDefinedValueSelector_Value1,  //!< Applies to: acA1600-60gc
        UserDefinedValueSelector_Value2,  //!< Applies to: acA1600-60gc
        UserDefinedValueSelector_Value3,  //!< Applies to: acA1600-60gc
        UserDefinedValueSelector_Value4,  //!< Applies to: acA1600-60gc
        UserDefinedValueSelector_Value5  //!< Applies to: acA1600-60gc
    };

    //! Valid values for UserOutputSelector
    enum UserOutputSelectorEnums
    {
        UserOutputSelector_UserOutput1  //!< Selects user settable output signal 1 for configuration - Applies to: acA1600-60gc
    };

    //! Valid values for UserSetDefaultSelector
    enum UserSetDefaultSelectorEnums
    {
        UserSetDefaultSelector_AutoFunctions,  //!< Selects the auto function user set as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_Color,  //!< Selects the color user set as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_Default,  //!< Selects the default user set as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_HighGain,  //!< Selects the high gain user set as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_UserSet1,  //!< Selects user set 1 as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_UserSet2,  //!< Selects user set 2 as the default startup set - Applies to: acA1600-60gc
        UserSetDefaultSelector_UserSet3  //!< Selects user set 3 as the default startup set - Applies to: acA1600-60gc
    };

    //! Valid values for UserSetSelector
    enum UserSetSelectorEnums
    {
        UserSetSelector_AutoFunctions,  //!< Factory set using auto functions - Applies to: acA1600-60gc
        UserSetSelector_Color,  //!< Factory set enabling color adjustments - Applies to: acA1600-60gc
        UserSetSelector_Default,  //!< Selects the default configuration set - Applies to: acA1600-60gc
        UserSetSelector_HighGain,  //!< High gain factory set - Applies to: acA1600-60gc
        UserSetSelector_UserSet1,  //!< Selects user set 1 - Applies to: acA1600-60gc
        UserSetSelector_UserSet2,  //!< Selects user set 2 - Applies to: acA1600-60gc
        UserSetSelector_UserSet3  //!< Selects user set 3 - Applies to: acA1600-60gc
    };


    
    
    //**************************************************************************************************
    // Parameter class BaslerCameraCameraParams
    //**************************************************************************************************
    

    /*!
    \brief A parameter class containing all parameters as members that are available for acA1600-60gc

    Sources:
    acA1600-60gc 106204-22;U;acA1600_60g;V5.0-0;1
    */
    class BaslerCameraCameraParams
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            BaslerCameraCameraParams(void);

            //! Destructor
            ~BaslerCameraCameraParams(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

    //! \endcond

    private:
        class BaslerCameraCameraParams_Data;
        BaslerCameraCameraParams_Data* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the number of frames acquired in the multiframe acquisition mode - Applies to: acA1600-60gc

        This value sets the number of frames acquired in the multiframe acquisition mode
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionFrameCount" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionFrameCount;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief If the acquisition frame rate feature is enabled, this value sets the camera's acquisition frame rate in frames per second - Applies to: acA1600-60gc

        Sets the 'absolute' value of the acquisition frame rate. The 'absolute' value is a float value that sets the acquisition frame rate in frames per second.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionFrameRateAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& AcquisitionFrameRateAbs;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Enables setting the camera's acquisition frame rate to a specified value - Applies to: acA1600-60gc

        This boolean value enables setting  the camera's acquisition frame rate to a specified value.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionFrameRateEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& AcquisitionFrameRateEnable;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the image acquisition mode - Applies to: acA1600-60gc

        This enumeration sets the image acquisition mode. 
    
        Visibility: Beginner

        Selecting Parameters: AcquisitionStart and AcquisitionStop

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<AcquisitionModeEnums>& AcquisitionMode;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Starts the acquisition of images - Applies to: acA1600-60gc

        This command starts the acquisition of images. If the camera is set for single frame acquisition, it will start acquisition of one frame. If the camera is set for continuous frame acquisition, it will start continuous acquisition of frames.
    
        Visibility: Beginner

        Selected by: AcquisitionMode

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStart" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& AcquisitionStart;

    //@}


    //! \name Categories: AcquisitionStartEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an acquisition start  event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an acquisition start  event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStartEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionStartEventStreamChannelIndex;

    //@}


    //! \name Categories: AcquisitionStartEventData
    //@{
    /*!
        \brief Indicates the time stamp for an acquisition start  event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an acquisition start  event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStartEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionStartEventTimestamp;

    //@}


    //! \name Categories: AcquisitionStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an acquisition start overtrigger event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an acquisition start overtrigger event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStartOvertriggerEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionStartOvertriggerEventStreamChannelIndex;

    //@}


    //! \name Categories: AcquisitionStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the time stamp for an Acquisition start overtrigger event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an Acquisition start overtrigger event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStartOvertriggerEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionStartOvertriggerEventTimestamp;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Reads the selected acquisition status - Applies to: acA1600-60gc

        This feature is used to read the state (True or False) of the internal acquisition signal selected using AcquisitionStatusSelector.
    
        Visibility: Expert

        Selected by: AcquisitionStatusSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStatus" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& AcquisitionStatus;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief This enumeration is used to select which internal acquisition signal to read using AcquisitionStatus - Applies to: acA1600-60gc

    
        Visibility: Expert

        Selecting Parameters: AcquisitionStatus

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStatusSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<AcquisitionStatusSelectorEnums>& AcquisitionStatusSelector;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Stops the acquisition of images - Applies to: acA1600-60gc

        If the camera is set for continuous image acquisition and acquisition has been started, this command stops the acquisition of images.
    
        Visibility: Beginner

        Selected by: AcquisitionMode

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionStop" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& AcquisitionStop;

    //@}


    //! \name Categories: AcquisitionWaitEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an acquisition wait event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an acquisition wait event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionWaitEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionWaitEventStreamChannelIndex;

    //@}


    //! \name Categories: AcquisitionWaitEventData
    //@{
    /*!
        \brief Indicates the time stamp for an acquisition wait event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an acquisition wait event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AcquisitionWaitEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AcquisitionWaitEventTimestamp;

    //@}


    //! \name Categories: ActionControl
    //@{
    /*!
        \brief Number of action command interfaces - Applies to: acA1600-60gc

        Available number of action command interfaces on the camera device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ActionCommandCount" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ActionCommandCount;

    //@}


    //! \name Categories: ActionControl
    //@{
    /*!
        \brief Authorization key - Applies to: acA1600-60gc

        Key to authorize the action for the device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ActionDeviceKey" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ActionDeviceKey;

    //@}


    //! \name Categories: ActionControl
    //@{
    /*!
        \brief Defines a group of devices - Applies to: acA1600-60gc

        Used to define a group of devices on which actions can be executed.
    
        Visibility: Guru

        Selected by: ActionSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ActionGroupKey" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ActionGroupKey;

    //@}


    //! \name Categories: ActionControl
    //@{
    /*!
        \brief Filters out particular devices from its group - Applies to: acA1600-60gc

        Used to filter out some particular devices from the group of devices defined by the action group key.
    
        Visibility: Guru

        Selected by: ActionSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ActionGroupMask" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ActionGroupMask;

    //@}


    //! \name Categories: ActionControl
    //@{
    /*!
        \brief Selects the action command to configure  Once an action command has been selected, all changes to the action command settings will be applied to the selected action command - Applies to: acA1600-60gc

        This enumeration selects the action command to configure. Once an action command has been selected, all changes to the action command settings will be applied to the selected action command.
    
        Visibility: Beginner

        Selecting Parameters: ActionGroupKey and ActionGroupMask

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ActionSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ActionSelector;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Lower limit of the Auto Exposure Time (Abs) [us] parameter - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoExposureTimeAbsLowerLimit" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& AutoExposureTimeAbsLowerLimit;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Upper limit of the Auto Exposure Time (Abs) [us] parameter - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoExposureTimeAbsUpperLimit" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& AutoExposureTimeAbsUpperLimit;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief Sets the height of the auto function area of interest in pixels - Applies to: acA1600-60gc

        This value sets the height of the auto function area of interest in pixels.
    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIHeight" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoFunctionAOIHeight;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief Sets the starting column of the auto function area of interest in pixels - Applies to: acA1600-60gc

        This value sets the starting column of the auto function area of interest in pixels.
    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIOffsetX" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoFunctionAOIOffsetX;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief Sets the starting line of the auto function area of interest in pixels - Applies to: acA1600-60gc

        This value sets the starting line of the auto function area of interest in pixels.
    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIOffsetY" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoFunctionAOIOffsetY;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief Selects the Auto Function AOI - Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selecting Parameters: AutoFunctionAOIWidth, AutoFunctionAOIHeight, AutoFunctionAOIOffsetX, AutoFunctionAOIOffsetY, AutoFunctionAOIUsageIntensity, AutoFunctionAOIUsageWhiteBalance and AutoFunctionAOIUsageRedLightCorrection

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOISelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<AutoFunctionAOISelectorEnums>& AutoFunctionAOISelector;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIUsageIntensity" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& AutoFunctionAOIUsageIntensity;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIUsageRedLightCorrection" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& AutoFunctionAOIUsageRedLightCorrection;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIUsageWhiteBalance" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& AutoFunctionAOIUsageWhiteBalance;

    //@}


    //! \name Categories: AutoFunctionAOIs
    //@{
    /*!
        \brief Sets the width of the auto function area of interest in pixels - Applies to: acA1600-60gc

        This value sets the width of the auto function area of interest in pixels.
    
        Visibility: Beginner

        Selected by: AutoFunctionAOISelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionAOIWidth" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoFunctionAOIWidth;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Selects the strategy for controlling gain and shutter simultaneously - Applies to: acA1600-60gc

        Selects the profile for controlling gain and shutter simultaneously.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoFunctionProfile" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<AutoFunctionProfileEnums>& AutoFunctionProfile;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Lower limit of the Auto Gain (Raw) parameter - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoGainRawLowerLimit" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoGainRawLowerLimit;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Upper limit of the Auto Gain (Raw) parameter - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoGainRawUpperLimit" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoGainRawUpperLimit;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Target average grey value for Gain Auto and Exposure Auto - Applies to: acA1600-60gc

        The target average grey value may range from nearly black to nearly white. Note that this range of gray values applies to 8 bit and to 16 bit (12 bit effective) output modes. Accordingly, also for 16 bit output modes, black is represented by 0 and white by 255.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=AutoTargetValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& AutoTargetValue;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Sets the value of the selected balance ratio control as a float - Applies to: acA1600-60gc

        This value sets the selected balance ratio control as a float value.
    
        Visibility: Beginner

        Selected by: BalanceRatioSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceRatioAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& BalanceRatioAbs;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Sets the value of the selected balance ratio control as an integer - Applies to: acA1600-60gc

        This value sets the selected balance ratio control as an integer.
    
        Visibility: Beginner

        Selected by: BalanceRatioSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceRatioRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& BalanceRatioRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Selects a balance ratio to configure  Once a balance ratio control has been selected, all changes to the balance ratio settings will be applied to the selected control - Applies to: acA1600-60gc

        This enumeration selects a balance ratio control to configuration. Once a balance ratio control has been selected, all changes to the balance ratio settings will be applied to the selected control.
    
        Visibility: Beginner

        Selecting Parameters: BalanceRatioAbs and BalanceRatioRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceRatioSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<BalanceRatioSelectorEnums>& BalanceRatioSelector;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Balance White adjustment damping for Balance White Auto - Applies to: acA1600-60gc

        The Balance White adjustment damping parameter controls the rate by which the color components are changed when Balance White Auto is enabled.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceWhiteAdjustmentDampingAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& BalanceWhiteAdjustmentDampingAbs;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Balance White adjustment damping for Balance White Auto - Applies to: acA1600-60gc

        The Balance White adjustment damping parameter controls the rate by which the color components are changed when Balance White Auto is enabled.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceWhiteAdjustmentDampingRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& BalanceWhiteAdjustmentDampingRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Balance White Auto is the 'automatic' counterpart of the manual white balance feature - Applies to: acA1600-60gc

        The automatic white balance is a two-step process: First, the Balance Ratio Abs parameter values for red, green, and blue are each set to 1.5. Then, assuming a 'grey world' model, the Balance Ratio Abs parameter values are adjusted such that the average gray values for the 'red' and 'blue' pixels match the average gray value for the 'green' pixels.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceWhiteAuto" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<BalanceWhiteAutoEnums>& BalanceWhiteAuto;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Allows returning to previous settings - Applies to: acA1600-60gc

        Allows returning to the color adjustment settings extant before the latest changes of the settings.
This allows you undoing the latest unwanted changes of the color adjustment settings and returning to the preceding settings.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BalanceWhiteReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& BalanceWhiteReset;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the number of adjacent horizontal pixes to be summed - Applies to: acA1600-60gc

        Sets the number of binned adjacent horizontal pixels. Their charges will be summed and reported out of the camera as a single pixel.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BinningHorizontal" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& BinningHorizontal;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the horizontal binning mode - Applies to: acA1600-60gc

        This enumeration sets the horizontal binning mode.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BinningModeHorizontal" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<BinningModeHorizontalEnums>& BinningModeHorizontal;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the vertical binning mode - Applies to: acA1600-60gc

        This enumeration sets the vertical binning mode.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BinningModeVertical" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<BinningModeVerticalEnums>& BinningModeVertical;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the number of adjacent vertical pixes to be summed - Applies to: acA1600-60gc

        Sets the number of binned adjacent vertical pixels. Their charges will be summed and reported out of the camera as a single pixel.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BinningVertical" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& BinningVertical;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Sets the value of the selected black level control as an integer - Applies to: acA1600-60gc

        This value sets the selected black level control as an integer.
    
        Visibility: Beginner

        Selected by: BlackLevelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BlackLevelRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& BlackLevelRaw;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Selcts a black level control to configure  Once a black level control has been selected, all changes to the black level settings will be applied to the selected control - Applies to: acA1600-60gc

        This enumeration selects the black level control to configure. Once a black level control has been selected, all changes to the black level settings will be applied to the selected control.
    
        Visibility: Beginner

        Selecting Parameters: BlackLevelRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=BlackLevelSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<BlackLevelSelectorEnums>& BlackLevelSelector;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Enables the horizontal centering of the image - Applies to: acA1600-60gc

        This feature is used to center the image horizontally.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CenterX" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& CenterX;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Enables the vertical centering of the image - Applies to: acA1600-60gc

        This feature is used to center the image vertically.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CenterY" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& CenterY;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the maximum possible pixel value in the acquired image - Applies to: acA1600-60gc

        This value indicates indicates the maximum possible pixel value acquired in the image
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkDynamicRangeMax" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkDynamicRangeMax;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the minimum possible pixel value in the acquired image - Applies to: acA1600-60gc

        This value indicates the minimum possible pixel value in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkDynamicRangeMin" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkDynamicRangeMin;

    //@}


    //! \name Categories: ChunkDataStreams
    //@{
    /*!
        \brief Enables the inclusion of the selected chunk in the payload data - Applies to: acA1600-60gc

        This boolean value enables the inclusion of the selected chunk in the payload data.
    
        Visibility: Beginner

        Selected by: ChunkSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ChunkEnable;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkExposureTime" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ChunkExposureTime;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkFrameTriggerCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkFrameTriggerCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkFrameTriggerIgnoredCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkFrameTriggerIgnoredCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the value of the frame counter when the image was acquired - Applies to: acA1600-60gc

        This integer indicates the value of the frame counter when the image was acquired.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkFramecounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkFramecounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkFramesPerTriggerCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkFramesPerTriggerCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkGainAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkGainAll;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the height of the area of interest represented in the acquired image - Applies to: acA1600-60gc

        This value Indicates the height of the area of interest represented in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkHeight" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkHeight;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Number of bits per status - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkInputStatusAtLineTriggerBitsPerLine" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerBitsPerLine;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Used to select a certain status - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkInputStatusAtLineTriggerIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerIndex;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Value of the status selected by 'Index' - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkInputStatusAtLineTriggerValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkInputStatusAtLineTriggerValue;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief A bit field that indicates the status of all of the camera's input and output lines when the image was acquired - Applies to: acA1600-60gc

        This value is a bit field that indicates the status of all of the camera's input and output lines when the image was acquired.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkLineStatusAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkLineStatusAll;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkLineTriggerCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkLineTriggerCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkLineTriggerEndToEndCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkLineTriggerEndToEndCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkLineTriggerIgnoredCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkLineTriggerIgnoredCounter;

    //@}


    //! \name Categories: ChunkDataStreams
    //@{
    /*!
        \brief Enables the chunk mode - Applies to: acA1600-60gc

        This boolean value enables the camera's chunk mode.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkModeActive" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ChunkModeActive;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the X offset of the area of interest represented in the acquired image - Applies to: acA1600-60gc

        This value Indicates the X offset of the area of interest represented in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkOffsetX" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkOffsetX;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the Y offset of the area of interest represented in the acquired image - Applies to: acA1600-60gc

        This value Indicates the Y offset of the area of interest represented in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkOffsetY" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkOffsetY;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the value of CRC checksum - Applies to: acA1600-60gc

        This integer indicates the value of CRC checksum.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkPayloadCRC16" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkPayloadCRC16;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the format of the pixel data in the acquired image - Applies to: acA1600-60gc

        This enumeration lists the pixel formats that can be indicated by the pixel format chunk.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkPixelFormat" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ChunkPixelFormatEnums>& ChunkPixelFormat;

    //@}


    //! \name Categories: ChunkDataStreams
    //@{
    /*!
        \brief Selects chunks for enabling - Applies to: acA1600-60gc

        This enumeration selects chunks for enabling.
    
        Visibility: Beginner

        Selecting Parameters: ChunkEnable

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ChunkSelectorEnums>& ChunkSelector;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the sequence set index number related to the acquired image - Applies to: acA1600-60gc

        This value indicates the sequence set index number related to the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkSequenceSetIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkSequenceSetIndex;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Shaft encoder counter at frame trigger - Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkShaftEncoderCounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkShaftEncoderCounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the number of bytes of data between the beginning of one line in the acquired image and the beginning of the next line in the acquired image - Applies to: acA1600-60gc

        This value indicates the number of bytes of data between the beginning of one line in the acquired image and the beginning of the next line in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkStride" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkStride;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the value of the timestamp when the image was acquired - Applies to: acA1600-60gc

        This integer indicates the value of the timestamp when the image was acquired.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkTimestamp;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the value of the trigger input counter when the image was acquired - Applies to: acA1600-60gc

        This integer indicates the value of the trigger input counter when the image was acquired.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkTriggerinputcounter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkTriggerinputcounter;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief A bit field that indicates the status of all of the camera's virtual input and output lines when the image was acquired - Applies to: acA1600-60gc

        This value is a bit field that indicates the status of all of the camera's virtual input and output lines when the image was acquired.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkVirtLineStatusAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkVirtLineStatusAll;

    //@}


    //! \name Categories: ChunkData
    //@{
    /*!
        \brief Indicates the widtth of the area of interest represented in the acquired image - Applies to: acA1600-60gc

        This value Indicates the width of the area of interest represented in the acquired image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ChunkWidth" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ChunkWidth;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Erases the last error and possibly reveals a previous error - Applies to: acA1600-60gc

        Erases the last error and possibly reveals a previous error.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ClearLastError" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& ClearLastError;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Enables color adjustment - Applies to: acA1600-60gc

        Enables color adjustment.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ColorAdjustmentEnable;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Adjustment of hue of the selected color (float) - Applies to: acA1600-60gc

        Sets a floating point value for the adjustment of hue of the selected color.
    
        Visibility: Expert

        Selected by: ColorAdjustmentSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentHue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ColorAdjustmentHue;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Adjustment of hue of the selected color (integer) - Applies to: acA1600-60gc

        Sets an integer value for the adjustment of hue of the selected color.
    
        Visibility: Expert

        Selected by: ColorAdjustmentSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentHueRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ColorAdjustmentHueRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Allows returning to previous settings - Applies to: acA1600-60gc

        Allows returning to the color adjustment settings extant before the latest changes of the settings.
This allows you undoing the latest unwanted changes of the color adjustment settings and returning to the preceding settings.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& ColorAdjustmentReset;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Adjustment of saturation of the selected color (float) - Applies to: acA1600-60gc

        Sets a floating point value for the adjustment of saturation of the selected color.
    
        Visibility: Expert

        Selected by: ColorAdjustmentSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentSaturation" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ColorAdjustmentSaturation;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Adjustment of saturation of the selected color (integer) - Applies to: acA1600-60gc

        Sets an integer value for the adjustment of saturation of the selected color.
    
        Visibility: Expert

        Selected by: ColorAdjustmentSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentSaturationRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ColorAdjustmentSaturationRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Selects the color for color adjustment - Applies to: acA1600-60gc

        Selects the color for color adjustment.
Those colors in the image will be adjusted where the selected color predominates.
    
        Visibility: Expert

        Selecting Parameters: ColorAdjustmentHue, ColorAdjustmentHueRaw, ColorAdjustmentSaturation and ColorAdjustmentSaturationRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorAdjustmentSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ColorAdjustmentSelectorEnums>& ColorAdjustmentSelector;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Defines the extent to which the selected light source will be considered (float) - Applies to: acA1600-60gc

        Sets a floating point value to define the extent to which the selected light source will be considered in color matrix transformation.
If the value is set to 1 the selected light source will be fully considered. If the value is set to 0 the selected light source will not be considered.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationMatrixFactor" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ColorTransformationMatrixFactor;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Defines the extent to which the selected light source will be considered (integer) - Applies to: acA1600-60gc

        Sets an integer value to define the extent to which the selected light source will be considered in color matrix transformation. If the value is set to 65536 the selected light source will be fully considered. If the value is set to 0 the selected light source will not be considered.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationMatrixFactorRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ColorTransformationMatrixFactorRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Selects the matrix color transformation between color spaces - Applies to: acA1600-60gc

        Selects the matrix color transformation between color spaces.
Generally, the related color spaces are used for internal processing: The color signals provided by the sensor are transformed to the RGB color space to allow further transformations (to account for the type of light source, for color adjustment, for white balance, etc.).
The color transformation selected here does not refer to the color space selected for the transmission of image data out of the camera.

    
        Visibility: Guru

        Selecting Parameters: LightSourceSelector, ColorTransformationValueSelector and ColorTransformationValue

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ColorTransformationSelectorEnums>& ColorTransformationSelector;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Sets a floating point value for the selected element in the color transformation matrix - Applies to: acA1600-60gc

        Sets a floating point value for the selected element in the color transformation matrix.
    
        Visibility: Guru

        Selected by: ColorTransformationSelector and ColorTransformationValueSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ColorTransformationValue;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Sets an integer value for the selected element in the color transformation matrix - Applies to: acA1600-60gc

        Sets an integer value for the selected element in the color transformation matrix.
    
        Visibility: Guru

        Selected by: ColorTransformationValueSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationValueRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ColorTransformationValueRaw;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Selects the element to be entered in the color transformation matrix - Applies to: acA1600-60gc

        Selects the element to be entered in the 3 x 3 color transformation matrix for custom matrix color transformation.
Note: Depending on the camera model, some elements in the color transformation matrix may be preset and can not be changed.

    
        Visibility: Guru

        Selected by: ColorTransformationSelector

        Selecting Parameters: ColorTransformationValue and ColorTransformationValueRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ColorTransformationValueSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ColorTransformationValueSelectorEnums>& ColorTransformationValueSelector;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief Shows the actual value of the contrast used in the function for controlling image contrast - Applies to: acA1600-60gc

        Shows tha actual value of the contrast used in the function for controlling image contrast. Image contrast will increase as values are increased.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ContrastAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ContrastAbs;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ContrastMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ContrastModeEnums>& ContrastMode;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief Sets the position of the pivot point used in the function for controlling image contrast - Applies to: acA1600-60gc

        Sets the position of the pivot point used in the function for controlling image contrast. Pixel values below the pivot point will be decreased. Pixel values above the pivot point will be increased.
The range of available settings (0..255) applies to all pixel data format bit depths.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ContrastPivot" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ContrastPivot;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief Sets the 'raw' value (integer) of the constant used in the function for controlling image contrast - Applies to: acA1600-60gc

        Sets the 'raw' value (integer) of the constant used in the function for controlling image contrast. The actual value used equals the set raw value times a factory-set base value. Image contrast will increase as the raw values are increased.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ContrastRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ContrastRaw;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Selects the event that will be the source to increment the counter - Applies to: acA1600-60gc

        This enumeration selects the event that will be the source to increment the counter.
    
        Visibility: Expert

        Selected by: CounterSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CounterEventSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<CounterEventSourceEnums>& CounterEventSource;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Immediately resets the selected counter - Applies to: acA1600-60gc

        This command will immediately reset the selected counter. Note that the counter starts counting immediately after the reset.
    
        Visibility: Expert

        Selected by: CounterSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CounterReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& CounterReset;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Selects the source of the reset for the selected counter - Applies to: acA1600-60gc

        This enumeration selects the source of the reset for the selected counter.
    
        Visibility: Expert

        Selected by: CounterSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CounterResetSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<CounterResetSourceEnums>& CounterResetSource;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Selects the counter to configure   Once a counter has been selected, all changes to the counter settings will be applied to the selected counter - Applies to: acA1600-60gc

        This enumeration selects the counter to configure. Once a counter has been selected, all changes to the counter settings will be applied to the selected counter.
    
        Visibility: Expert

        Selecting Parameters: CounterEventSource, CounterReset and CounterResetSource

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CounterSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<CounterSelectorEnums>& CounterSelector;

    //@}


    //! \name Categories: CriticalTemperatureEventData
    //@{
    /*!
        \brief Indicates the stream channel index for a critical temperature event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for a critical temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CriticalTemperatureEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& CriticalTemperatureEventStreamChannelIndex;

    //@}


    //! \name Categories: CriticalTemperatureEventData
    //@{
    /*!
        \brief Indicates the time stamp for a critical temperature event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for a critical temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=CriticalTemperatureEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& CriticalTemperatureEventTimestamp;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets horizontal sub-sampling - Applies to: acA1600-60gc

        Horizontal sub-sampling of the image. This has the net effect of reducing the horizontal resolution (width) of the image by the specified horizontal decimation factor. A value of 1 indicates that the camera performs no horizontal decimation.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DecimationHorizontal" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& DecimationHorizontal;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets vertical sub-sampling - Applies to: acA1600-60gc

        Vertical sub-sampling of the image. This has the net effect of reducing the vertical resolution (height) of the image by the specified vertical decimation factor. A value of 1 indicates that the camera performs no vertical decimation.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DecimationVertical" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& DecimationVertical;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief  - Applies to: acA1600-60gc

    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Denoising" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Denoising;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief Enables denoising - Applies to: acA1600-60gc

        Enables denoising.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DenoisingEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& DenoisingEnable;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the version of the device's firmware and software - Applies to: acA1600-60gc

        This is a read only element. It is a string that indicates the version of the device's firmware and software.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceFirmwareVersion" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceFirmwareVersion;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief A unique identifier for the device such as a serial number or a GUID - Applies to: acA1600-60gc

        This is a read only element. It is a string that provides a unique identifier for the device such as a serial number or a GUID.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceID" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceID;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Provides additional information from the vendor about the device - Applies to: acA1600-60gc

        This is a read only element. It is a string that provides additional information from the vendor about the camera.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceManufacturerInfo" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceManufacturerInfo;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the model name of the device - Applies to: acA1600-60gc

        This is a read only element. It is a text description that indicates the model name of the device.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceModelName" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceModelName;

    //@}


    //! \name Categories: DeviceControl
    //@{
    /*!
        \brief Announce the end of registers streaming - Applies to: acA1600-60gc

        StopFeatureStreaming
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceRegistersStreamingEnd" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& DeviceRegistersStreamingEnd;

    //@}


    //! \name Categories: DeviceControl
    //@{
    /*!
        \brief Prepare the device for registers streaming - Applies to: acA1600-60gc

        StartFeatureStreaming
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceRegistersStreamingStart" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& DeviceRegistersStreamingStart;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Immediately resets and reboots the device - Applies to: acA1600-60gc

        This is a command that immediately resets and reboots the device.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& DeviceReset;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the scan type of the device's sensor - Applies to: acA1600-60gc

        This enumeration lists the possible scan types for the sensor in the device.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceScanType" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<DeviceScanTypeEnums>& DeviceScanType;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief A device ID that is user programmable - Applies to: acA1600-60gc

        This is a read/write element. It is a user programmable string.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceUserID" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceUserID;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the name of the device's vendor - Applies to: acA1600-60gc

        This is a read only element. It is a text description that indicates the name of the device's vendor.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceVendorName" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceVendorName;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the version of the device - Applies to: acA1600-60gc

        This is a read only element. It is a string that indicates the version of the device.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DeviceVersion" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& DeviceVersion;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Sets the value of the selected digital shift control - Applies to: acA1600-60gc

        This value sets the selected digital shift control
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=DigitalShift" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& DigitalShift;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief When enabled, the maximum frame rate does not depend on the image transfer rate out of the camera - Applies to: acA1600-60gc

        When enabled, the maximum frame rate onyl depends on sensor timing and timing of the trigger sequence, and not on the image transfer rate out of the camera.

Note: The maximum number of triggers within a burst sequence is limited. If the maximum number is exceeded, images may be damaged or lost.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EnableBurstAcquisition" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& EnableBurstAcquisition;

    //@}


    //! \name Categories: EventsGeneration
    //@{
    /*!
        \brief Sets the notification type that will be sent to the host application for the selected event  - Applies to: acA1600-60gc

        This enumeration sets the notification type that will be sent to the host application for the selected event.
    
        Visibility: Beginner

        Selected by: EventSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EventNotification" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<EventNotificationEnums>& EventNotification;

    //@}


    //! \name Categories: EventOverrunEventData
    //@{
    /*!
        \brief Indicates the frame ID for an event overrun event - Applies to: acA1600-60gc

        This enumeration value indicates the frame ID for an event overrun event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EventOverrunEventFrameID" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& EventOverrunEventFrameID;

    //@}


    //! \name Categories: EventOverrunEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an event overrun event - Applies to: acA1600-60gc

        This enumeration value indicates the stream channel index for an event overrun event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EventOverrunEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& EventOverrunEventStreamChannelIndex;

    //@}


    //! \name Categories: EventOverrunEventData
    //@{
    /*!
        \brief Indicates the time stamp for an event overrun event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an event overrun event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EventOverrunEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& EventOverrunEventTimestamp;

    //@}


    //! \name Categories: EventsGeneration
    //@{
    /*!
        \brief Selects the type of event for enabling - Applies to: acA1600-60gc

        This enumeration selects the type of event for enabling.
    
        Visibility: Beginner

        Selecting Parameters: EventNotification

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=EventSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<EventSelectorEnums>& EventSelector;

    //@}


    //! \name Categories: ExpertFeatureAccess
    //@{
    /*!
        \brief Sets the key to access the selected feature - Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: ExpertFeatureAccessSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExpertFeatureAccessKey" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExpertFeatureAccessKey;

    //@}


    //! \name Categories: ExpertFeatureAccess
    //@{
    /*!
        \brief Selects the feature to configure  Once a feature has been selected, all changes made using the feature enable feature will be applied to the selected feature - Applies to: acA1600-60gc

    
        Visibility: Guru

        Selecting Parameters: ExpertFeatureEnable and ExpertFeatureAccessKey

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExpertFeatureAccessSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ExpertFeatureAccessSelectorEnums>& ExpertFeatureAccessSelector;

    //@}


    //! \name Categories: ExpertFeatureAccess
    //@{
    /*!
        \brief Enable the selected Feature - Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: ExpertFeatureAccessSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExpertFeatureEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ExpertFeatureEnable;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Exposure Auto is the 'automatic' counterpart to manually setting an 'absolute' exposure time - Applies to: acA1600-60gc

        The exposure auto function automatically adjusts the Auto Exposure Time Abs parameter value within set limits, until a target average gray value for the pixel data of the related Auto Function AOI is reached.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureAuto" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ExposureAutoEnums>& ExposureAuto;

    //@}


    //! \name Categories: ExposureEndEventData
    //@{
    /*!
        \brief Indicates the frame ID for an exposure end event - Applies to: acA1600-60gc

        This enumeration value indicates the frame ID for an exposure end event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureEndEventFrameID" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExposureEndEventFrameID;

    //@}


    //! \name Categories: ExposureEndEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an exposure end event - Applies to: acA1600-60gc

        This enumeration value indicates the stream channel index for an exposure end event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureEndEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExposureEndEventStreamChannelIndex;

    //@}


    //! \name Categories: ExposureEndEventData
    //@{
    /*!
        \brief Indicates the time stamp for an exposure end event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an exposure end event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureEndEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExposureEndEventTimestamp;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the exposure mode - Applies to: acA1600-60gc

        This enumeration sets the exposure mode.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ExposureModeEnums>& ExposureMode;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the maximum overlap of the sensor exposure with sensor readout in TriggerWidth exposure mode in microseconds - Applies to: acA1600-60gc

        This float value sets the maximum overlap time (in microseconds) of the sensor exposure with sensor readout in TriggerWidth exposure mode.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureOverlapTimeMaxAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ExposureOverlapTimeMaxAbs;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the maximum overlap of the sensor exposure with the sensor readout in TriggerWidth exposure mode in raw units - Applies to: acA1600-60gc

        This integer value sets the maximum overlap time (in raw units) of the sensor exposure with sensor readout in TriggerWidth exposure mode.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureOverlapTimeMaxRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExposureOverlapTimeMaxRaw;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Directly sets the camera's exposure time in microseconds - Applies to: acA1600-60gc

        This float value sets the camera's exposure time in microseconds.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureTimeAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ExposureTimeAbs;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the 'raw' exposure time   Actual exposure time = raw exposure setting  x  exposure time base abs setting - Applies to: acA1600-60gc

        This value sets an integer that will be used as a multiplier for the exposure timebase. The actual exposure time equals the current exposure time raw setting times the current exposure time base abs setting.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ExposureTimeRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& ExposureTimeRaw;

    //@}


    //! \name Categories: FeatureSets
    //@{
    /*!
        \brief Select a camera description file - Applies to: acA1600-60gc

        Selects a feature set description file.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FeatureSet" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<FeatureSetEnums>& FeatureSet;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Defines the intermediate access buffer - Applies to: acA1600-60gc

        The File Access Buffer feature defines the intermediate access buffer that allows the exchange of data between the device file storage and the application.
    
        Visibility: Guru

        Selected by: FileSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileAccessBuffer" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IRegisterEx& FileAccessBuffer;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Controls the mapping between the device file storage and the FileAccessBuffer - Applies to: acA1600-60gc

        This feature controls the mapping between the device file storage and the FileAccessBuffer.
    
        Visibility: Guru

        Selected by: FileSelector and FileOperationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileAccessLength" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FileAccessLength;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Controls the mapping between the device file storage and the FileAccessBuffer - Applies to: acA1600-60gc

        This feature controls the mapping between the device file storage and the FileAccessBuffer.
    
        Visibility: Guru

        Selected by: FileSelector and FileOperationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileAccessOffset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FileAccessOffset;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Selects the access mode in which a file is opened - Applies to: acA1600-60gc

        The File Open Mode feature selects the access mode in which a file is opened in the device.
    
        Visibility: Guru

        Selected by: FileSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileOpenMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<FileOpenModeEnums>& FileOpenMode;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Executes the selected operation - Applies to: acA1600-60gc

        The File Operation Execute feature is the command that executes the operation selected by FileOperationSelector on the selected file.
    
        Visibility: Guru

        Selected by: FileSelector and FileOperationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileOperationExecute" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& FileOperationExecute;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Represents the file operation result - Applies to: acA1600-60gc

        The File Operation Result feature represents the file operation result. For Read or Write operations, the number of successfully read/written bytes is returned.
    
        Visibility: Guru

        Selected by: FileSelector and FileOperationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileOperationResult" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FileOperationResult;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Selects the target operation for the selected file - Applies to: acA1600-60gc

        The File Operation Selector feature selects the target operation for the selected file in the device. This Operation is executed when the FileOperationExecute feature is called.
    
        Visibility: Guru

        Selected by: FileSelector

        Selecting Parameters: FileAccessOffset, FileAccessLength, FileOperationStatus, FileOperationResult and FileOperationExecute

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileOperationSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<FileOperationSelectorEnums>& FileOperationSelector;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Represents the file operation execution status - Applies to: acA1600-60gc

        The File Operation Status feature represents the file operation execution status.
    
        Visibility: Guru

        Selected by: FileSelector and FileOperationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileOperationStatus" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<FileOperationStatusEnums>& FileOperationStatus;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief This feature selects the target file in the device - Applies to: acA1600-60gc

        The File Selector feature selects the target file in the device.
    
        Visibility: Guru

        Selecting Parameters: FileOperationSelector, FileOpenMode, FileAccessBuffer, FileAccessOffset, FileAccessLength, FileOperationStatus, FileOperationResult, FileSize and FileOperationExecute

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<FileSelectorEnums>& FileSelector;

    //@}


    //! \name Categories: FileAccessControl
    //@{
    /*!
        \brief Represents the size of the selected file - Applies to: acA1600-60gc

        The File Size feature represents the size of the selected file in bytes.
    
        Visibility: Guru

        Selected by: FileSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FileSize" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FileSize;

    //@}


    //! \name Categories: FrameStartEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an frame start  event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an frame start  event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameStartEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameStartEventStreamChannelIndex;

    //@}


    //! \name Categories: FrameStartEventData
    //@{
    /*!
        \brief Indicates the time stamp for an frame start  event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an frame start  event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameStartEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameStartEventTimestamp;

    //@}


    //! \name Categories: FrameStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an frame start overtrigger event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an frame start overtrigger event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameStartOvertriggerEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameStartOvertriggerEventStreamChannelIndex;

    //@}


    //! \name Categories: FrameStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the time stamp for an frame start overtrigger event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an frame start overtrigger event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameStartOvertriggerEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameStartOvertriggerEventTimestamp;

    //@}


    //! \name Categories: FrameTimeoutEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an frame timeout event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an frame timeout event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameTimeoutEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameTimeoutEventStreamChannelIndex;

    //@}


    //! \name Categories: FrameTimeoutEventData
    //@{
    /*!
        \brief Indicates the time stamp for an frame timeout event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an frame timeout event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameTimeoutEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameTimeoutEventTimestamp;

    //@}


    //! \name Categories: FrameWaitEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an frame wait event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an frame wait event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameWaitEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameWaitEventStreamChannelIndex;

    //@}


    //! \name Categories: FrameWaitEventData
    //@{
    /*!
        \brief Indicates the time stamp for an frame wait event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an frame wait event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=FrameWaitEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& FrameWaitEventTimestamp;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Gain Auto is the 'automatic' counterpart of the manual gain feature - Applies to: acA1600-60gc

        The gain auto function automatically adjusts the Auto Gain Raw parameter value within set limits, until a target average gray value for the pixel data from Auto Function AOI1 is reached.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GainAuto" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GainAutoEnums>& GainAuto;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief This is an integer value that sets the selected gain control in device specific units - Applies to: acA1600-60gc

        Sets the 'raw' value of the selected gain control. The 'raw' value is an integer value that sets the selected gain control in units specific to the camera.
    
        Visibility: Beginner

        Selected by: GainSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GainRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GainRaw;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Selects the gain control to configure  Once a gain control has been selected, all changes to the gain settings will be applied to the selected control - Applies to: acA1600-60gc

        This enumeration selects the gain control to configure. Once a gain control has been selected, all changes to the gain settings will be applied to the selected control.
    
        Visibility: Beginner

        Selecting Parameters: GainRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GainSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GainSelectorEnums>& GainSelector;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief This feature is used to perform gamma correction of pixel intensity - Applies to: acA1600-60gc

        This feature is used to perform gamma correction of pixel  intensity. This is typically used to compensate for non-linearity of the display system (such as CRT).
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Gamma" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& Gamma;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Enables the gamma correction - Applies to: acA1600-60gc

        This boolean value enables the gamma correction.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GammaEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GammaEnable;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

        This enumeration selects the type of gamma to apply.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GammaSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GammaSelectorEnums>& GammaSelector;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the control channel privilege feature - Applies to: acA1600-60gc

        This enumeration sets the control channel privilege feature.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevCCP" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GevCCPEnums>& GevCCP;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the current default gateway for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the current default gateway for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevCurrentDefaultGateway" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevCurrentDefaultGateway;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the current IP address for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the current IP address for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevCurrentIPAddress" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevCurrentIPAddress;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the current IP configuration of the selected network interface - Applies to: acA1600-60gc

        This value sets the IP configuration of the selected network interface, i.e., fixed IP, DHCP, auto IP. 
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevCurrentIPConfiguration" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevCurrentIPConfiguration;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the current subnet mask for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the current subnet mask for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevCurrentSubnetMask" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevCurrentSubnetMask;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indictes the character set - Applies to: acA1600-60gc

        This is a read only element. Its value indicates the character set. 1 = UTF8
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevDeviceModeCharacterSet" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevDeviceModeCharacterSet;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the endianess of the bootstrap registers - Applies to: acA1600-60gc

        This is a read only element. It indicates the endianess of the bootstrap registers. True = big endian.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevDeviceModeIsBigEndian" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevDeviceModeIsBigEndian;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the first URL to the XML device description file - Applies to: acA1600-60gc

        This is a read only element. It indicates the first URL to the XML device description file.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevFirstURL" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& GevFirstURL;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the heartbeat timeout in milliseconds - Applies to: acA1600-60gc

        This value sets the heartbeat timeout in milliseconds.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevHeartbeatTimeout" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevHeartbeatTimeout;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Enable usage of the IEEE 1588 V2 Precision Time Protocol to source the timestamp register  Only available when the IEEE1588_support bit of the GVCP Capability register is set  When PTP is enabled, the Timestamp Control register cannot be used to reset the timestamp  Factory default is devicespecific  When PTP is enabled or disabled, the value of Timestamp Tick Frequency and Timestamp Value registers might change to reflect the new time domain - Applies to: acA1600-60gc

        This value indicates whether IEEE 1588 V2 (PTP) is enabled.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevIEEE1588" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevIEEE1588;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Provides the state of the IEEE 1588 clock - Applies to: acA1600-60gc

        Provides the state of the IEEE 1588 clock. Values of this field must match the IEEE 1588 PTP port state enumeration (INITIALIZING, FAULTY, DISABLED, LISTENING, PRE_MASTER, MASTER, PASSIVE, UNCALIBRATED, SLAVE). Please refer to IEEE 1588 for additional information.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevIEEE1588Status" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GevIEEE1588StatusEnums>& GevIEEE1588Status;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Selects the physical network interface to configure  Once a network interface has been selected, all changes to the network interface settings will be applied to the selected interface - Applies to: acA1600-60gc

        This selects the physical network interface to configure. Once a network interface has been selected, all changes to the network interface settings will be applied to the selected interface.
    
        Visibility: Guru

        Selecting Parameters: GevMACAddress, GevSupportedIPConfigurationLLA, GevSupportedIPConfigurationDHCP, GevSupportedIPConfigurationPersistentIP, GevCurrentIPConfiguration, GevCurrentIPAddress, GevCurrentSubnetMask, GevCurrentDefaultGateway, GevPersistentIPAddress, GevPersistentSubnetMask, GevPersistentDefaultGateway, GevLinkSpeed, GevLinkMaster, GevLinkFullDuplex and GevLinkCrossover

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevInterfaceSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GevInterfaceSelectorEnums>& GevInterfaceSelector;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the state of medium-dependent interface crossover (MDIX) for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the state of medium-dependent interface crossover (MDIX) for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevLinkCrossover" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevLinkCrossover;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether the selected network interface operates in full-duplex mode - Applies to: acA1600-60gc

        This is a read only element. It indicates whether the selected network interface operates in full-duplex mode.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevLinkFullDuplex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevLinkFullDuplex;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether the selected network interface is the clock master - Applies to: acA1600-60gc

        This is a read only element. It indicates whether the selected network interface is the clock master.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevLinkMaster" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevLinkMaster;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the connection speed in Mbps for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the connection speed in Mbps for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevLinkSpeed" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevLinkSpeed;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the MAC address for the selected network interface - Applies to: acA1600-60gc

        This is a read only element. It indicates the MAC address for the selected network interface.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevMACAddress" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevMACAddress;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the number of message channels supported by the device - Applies to: acA1600-60gc

        This is a read only element. It indicates the number of message channels supported by the device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevMessageChannelCount" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevMessageChannelCount;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the number of network interfaces on the device - Applies to: acA1600-60gc

        This is a read only element. It indicates the number of network interfaces on the device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevNumberOfInterfaces" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevNumberOfInterfaces;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief If fixed (persistent) IP addressing is supported by the device and enabled, sets the fixed default gateway for the selected network interface  - Applies to: acA1600-60gc

        This value sets the fixed default gateway for the selected network interface (if fixed IP addressing is supported by the device and enabled).
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevPersistentDefaultGateway" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevPersistentDefaultGateway;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief If fixed (persistent) IP addressing is supported by the device and enabled, sets the fixed IP address for the selected network interface  - Applies to: acA1600-60gc

        This value sets the fixed IP address for the selected network interface (if fixed IP addressing is supported by the device and enabled).
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevPersistentIPAddress" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevPersistentIPAddress;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief If fixed (persistent) IP addressing is supported by the device and enabled, sets the fixed subnet mask for the selected network interface  - Applies to: acA1600-60gc

        This value sets the fixed subnet mask for the selected network interface (if fixed IP addressing is supported by the device and enabled).
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevPersistentSubnetMask" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevPersistentSubnetMask;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the bandwidth (in bytes per second) that will be used by the camera to transmit image and chunk feature data and to handle resends and control data transmissions - Applies to: acA1600-60gc

        This value indicates the base bandwidth in bytes per second that will be used by the camera to transmit image and chunk feature data and to handle resends and control data transmissions. This parameter represents a combination of the packet size and the inter-packet delay.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCBWA" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCBWA;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets a percentage of the Ethernet bandwidth assigned to the camera to be held in reserve  The reserve is used for packet resends and control data transmissions   - Applies to: acA1600-60gc

        This value reserves a portion of Ethernet bandwidth assigned to the camera for packet resends and for the transmission of control data between the camera and the host PC. The setting is expressed as a percentage of the bandwidth assigned parameter. For example, if the Bandwidth Assigned parameter indicates that 30 MBytes/s have been assigned to the camera and the Bandwidth Reserve parameter is set to 5%, then the bandwidth reserve will be 1.5 MBytes/s.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCBWR" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCBWR;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets a multiplier for the Bandwidth Reserve parameter  The multiplier is used to establish an extra pool of reserved bandwidth that can be used if an unusually large burst of packet resends is needed - Applies to: acA1600-60gc

        This value sets a multiplier for the Bandwidth Reserve parameter. The multiplier is used to establish an extra pool of reserved bandwidth that can be used if an unusually large burst of packet resends is needed.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCBWRA" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCBWRA;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the stream channel destination IPv4 address for the selected stream channel - Applies to: acA1600-60gc

        This value sets the stream channel destination IPv4 address for the selected stream channel. The destination can be a unicast or a multicast.
    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCDA" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCDA;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the actual bandwidth (in bytes per second) that the camera will use to transmit image data and chunk data given the current AOI settings, chunk feature settings, and the pixel format setting - Applies to: acA1600-60gc

        This value indicates the actual bandwidth (in bytes per second) that the camera will use to transmit image data and chunk data given the current AOI settings, chunk feature settings, and the pixel format setting.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCDCT" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCDCT;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the maximum amount of data (in bytes per second) that the camera could generate given its current settings and ideal conditions, i e , unlimited bandwidth and no packet resends - Applies to: acA1600-60gc

        This value indicates the maximum amount of data (in bytes per second) that the camera could generate given its current settings and ideal conditions, i.e., unlimited bandwidth and no packet resends.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCDMT" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCDMT;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the maximum time (in ticks) that the next frame transmission could be delayed due to a burst of resends - Applies to: acA1600-60gc

        If the Bandwidth Reserve Accumulation parameter is set to a high value, the camera can experience periods where there is a large burst of data resends. This burst of resends will delay the start of transmission of the next acquired image. The Frame Max Jitter parameter indicates the maximum time in ticks that the next frame transmission could be delayed due to a burst of resends.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCFJM" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCFJM;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the frame transfer start delay (in ticks) for the selected stream channel - Applies to: acA1600-60gc

        This value sets the frame transfer delay for the selected stream channel. This value sets a delay betweem when the camera would normally begin transmitted an acquired image (frame) and when it actually begins transmitting the acquired image.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCFTD" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCFTD;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the inter-packet delay (in ticks) for the selected stream channel - Applies to: acA1600-60gc

        This value sets a delay between the transmission of each packet for the selected stream channel. The delay is measured in ticks.
    
        Visibility: Expert

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPD" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCPD;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the port to which the device must send data streams - Applies to: acA1600-60gc

        This value sets the port to which the device must send data streams.
    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPHostPort" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCPHostPort;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the index of the network interface to use - Applies to: acA1600-60gc

        This value sets the index of the network interface to use.
    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPInterfaceIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCPInterfaceIndex;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPSBigEndian" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSCPSBigEndian;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPSDoNotFragment" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSCPSDoNotFragment;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPSFireTestPacket" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& GevSCPSFireTestPacket;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Sets the packet size in bytes for the selected stream channel - Applies to: acA1600-60gc

        This value sets the packet size in bytes for the selected stream channel. Excludes data leader and data trailer. (The last packet may be smaller because the packet size is not necessarily a multiple of the block size for the stream channel.)
    
        Visibility: Beginner

        Selected by: GevStreamChannelSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSCPSPacketSize" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevSCPSPacketSize;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the second URL to the XML device description file - Applies to: acA1600-60gc

        This is a read only element. It indicates the second URL to the XML device description file.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSecondURL" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IStringEx& GevSecondURL;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the number of stream channels supported by the device - Applies to: acA1600-60gc

        This is a read only element. It indicates the number of stream channels supported by the device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevStreamChannelCount" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevStreamChannelCount;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Selects the stream channel to configure  Once a stream channel has been selected, all changes to the stream channel settings will be applied to the selected stream channel - Applies to: acA1600-60gc

        This enumeration selects the stream channels to configure. Once a stream channel has been selected, all changes to the stream channel settings will be applied to the selected stream channel.
    
        Visibility: Guru

        Selecting Parameters: GevSCPInterfaceIndex, GevSCPHostPort, GevSCPSFireTestPacket, GevSCPSDoNotFragment, GevSCPSBigEndian, GevSCPSPacketSize, GevSCPD, GevSCFTD, GevSCDA, GevSCBWR, GevSCBWRA, GevSCBWA, GevSCDMT, GevSCDCT and GevSCFJM

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevStreamChannelSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<GevStreamChannelSelectorEnums>& GevStreamChannelSelector;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether IEEE 1588 (PTP) is supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether whether IEEE 1588 (PTP) is supported.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedIEEE1588" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedIEEE1588;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether the selected network interface supports DHCP IP addressing - Applies to: acA1600-60gc

        This is a read only element. It indicates whether the selected network interface supports DHCP IP addressing.
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedIPConfigurationDHCP" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedIPConfigurationDHCP;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether the selected network interface supports auto IP addressing (also known as LLA) - Applies to: acA1600-60gc

        This is a read only element. It indicates whether the selected network interface supports auto IP addressing (also known as LLA).
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedIPConfigurationLLA" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedIPConfigurationLLA;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether the selected network interface supports fixed IP addressing (also known as persistent IP addressing) - Applies to: acA1600-60gc

        This is a read only element. It indicates whether the selected network interface supports fixed IP addressing (also known as persistent IP addressing).
    
        Visibility: Guru

        Selected by: GevInterfaceSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedIPConfigurationPersistentIP" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedIPConfigurationPersistentIP;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether multiple operations in a single message are supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether multiple operations in a single message are supported.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedOptionalCommandsConcatenation" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedOptionalCommandsConcatenation;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether EVENT_CMD and EVENT_ACK are supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether EVENT_CMD and EVENT_ACK are supported.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedOptionalCommandsEVENT" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedOptionalCommandsEVENT;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether EVENTDATA_CMD and EVENTDATA_ACK are supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether EVENTDATA_CMD and EVENTDATA_ACK are supported.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedOptionalCommandsEVENTDATA" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedOptionalCommandsEVENTDATA;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether PACKETRESEND_CMD is supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether PACKETRESEND_CMD is supported.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedOptionalCommandsPACKETRESEND" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedOptionalCommandsPACKETRESEND;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates whether WRITEMEM_CMD and WRITEMEM_ACK are supported - Applies to: acA1600-60gc

        This is a read only element. It indicates whether WRITEMEM_CMD and WRITEMEM_ACK are supported
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevSupportedOptionalCommandsWRITEMEM" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& GevSupportedOptionalCommandsWRITEMEM;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Latches the current timestamp value of the device - Applies to: acA1600-60gc

        This command latches the current timestamp value of the device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevTimestampControlLatch" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& GevTimestampControlLatch;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Resets the timestamp control latch - Applies to: acA1600-60gc

        This command resets the timestamp control latch.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevTimestampControlLatchReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& GevTimestampControlLatchReset;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Resets the timestamp value for the device - Applies to: acA1600-60gc

        This command resets the timestamp value for the device
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevTimestampControlReset" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& GevTimestampControlReset;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the number of timestamp clock ticks in 1 second - Applies to: acA1600-60gc

        This is a read only element. It indicates the number of timestamp clock ticks in 1 second.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevTimestampTickFrequency" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevTimestampTickFrequency;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the latched value of the timestamp   (The timestamp must first be latched using the Timestamp Control Latch command ) - Applies to: acA1600-60gc

        This is a read only element. It indicates the latched value of the timestamp.  (The timestamp must first be latched using the Timestamp Control Latch command.)
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevTimestampValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevTimestampValue;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the major version number of the GigE Vision specification supported by this device - Applies to: acA1600-60gc

        This is a read only element. It indicates the major version number of the GigE Vision specification supported by this device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevVersionMajor" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevVersionMajor;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Indicates the minor version number of the GigE Vision specification supported by this device - Applies to: acA1600-60gc

        This is a read only element. It indicates the minor version number of the GigE Vision specification supported by this device.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GevVersionMinor" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GevVersionMinor;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Gray value adjustment damping for Gain Auto and Exposure Auto - Applies to: acA1600-60gc

        The gray value adjustment damping parameter controls the rate by which pixel gray values are changed when Exposure Auto and/or Gain Auto are enabled.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GrayValueAdjustmentDampingAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& GrayValueAdjustmentDampingAbs;

    //@}


    //! \name Categories: AutoFunctions
    //@{
    /*!
        \brief Gray value adjustment damping for Gain Auto and Exposure Auto - Applies to: acA1600-60gc

        The gray value adjustment damping parameter controls the rate by which pixel gray values are changed when Exposure Auto and/or Gain Auto are enabled.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=GrayValueAdjustmentDampingRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& GrayValueAdjustmentDampingRaw;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the height of the area of interest in pixels - Applies to: acA1600-60gc

        This value sets the height of the area of interest in pixels.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Height" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Height;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the maximum allowed height of the image in pixels - Applies to: acA1600-60gc

        This is a read only element. It is an integer that indicates maximum allowed height of the image in pixels taking into account any function that may limit the allowed height.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=HeightMax" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& HeightMax;

    //@}


    //! \name Categories: LUTControls
    //@{
    /*!
        \brief Enables the selected LUT - Applies to: acA1600-60gc

        This boolean value enables the selected LUT.
    
        Visibility: Beginner

        Selected by: LUTSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LUTEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& LUTEnable;

    //@}


    //! \name Categories: LUTControls
    //@{
    /*!
        \brief Sets the LUT element to access  - Applies to: acA1600-60gc

        This value sets the LUT element to access. This value is used to index into a LUT array.
    
        Visibility: Beginner

        Selected by: LUTSelector

        Selecting Parameters: LUTValue

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LUTIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LUTIndex;

    //@}


    //! \name Categories: LUTControls
    //@{
    /*!
        \brief Selects the lookup table (LUT) to configure  Once a LUT has been selected, all changes to the LUT settings will be applied to the selected LUT - Applies to: acA1600-60gc

        This enumeration the lookup table (LUT) to configure. Once a LUT has been selected, all changes to the LUT settings will be applied to the selected LUT.
    
        Visibility: Beginner

        Selecting Parameters: LUTEnable, LUTIndex, LUTValue and LUTValueAll

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LUTSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LUTSelectorEnums>& LUTSelector;

    //@}


    //! \name Categories: LUTControls
    //@{
    /*!
        \brief Sets the value of the LUT element at the LUT index - Applies to: acA1600-60gc

        This value sets the value of the LUT element at the LUT index.
    
        Visibility: Beginner

        Selected by: LUTSelector and LUTIndex

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LUTValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LUTValue;

    //@}


    //! \name Categories: LUTControls
    //@{
    /*!
        \brief Accesses the entire content of the selected LUT in one chunk access - Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selected by: LUTSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LUTValueAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IRegisterEx& LUTValueAll;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the error that was detected last - Applies to: acA1600-60gc

        Indicates the error that was detected last.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LastError" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LastErrorEnums>& LastError;

    //@}


    //! \name Categories: LateActionEventData
    //@{
    /*!
        \brief Indicates the stream channel index for a critical temperature event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for a critical temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LateActionEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LateActionEventStreamChannelIndex;

    //@}


    //! \name Categories: LateActionEventData
    //@{
    /*!
        \brief Indicates the time stamp for a critical temperature event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for a critical temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LateActionEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LateActionEventTimestamp;

    //@}


    //! \name Categories: ColorImprovementsControl
    //@{
    /*!
        \brief Selects the type of light source to be considered for matrix color transformation - Applies to: acA1600-60gc

        Selects the color transformation mode to select the type of light source to be considered for matrix color transformation.
    
        Visibility: Expert

        Selected by: ColorTransformationSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LightSourceSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LightSourceSelectorEnums>& LightSourceSelector;

    //@}


    //! \name Categories: Line1RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io line 1 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io line 1 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line1RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line1RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: Line1RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a line 1 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io line 1 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line1RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line1RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: Line2RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io line 2 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io line 2 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line2RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line2RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: Line2RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a line 2 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io line 2 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line2RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line2RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: Line3RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io line 3 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io line 3 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line3RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line3RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: Line3RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a line 3 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io line 3 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line3RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line3RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: Line4RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io line 4 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io line 4 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line4RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line4RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: Line4RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a line 4 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io line 4 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Line4RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Line4RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the absolute value of the selected line debouncer time in microseconds - Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineDebouncerTimeAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& LineDebouncerTimeAbs;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the electrical configuration of the selected line - Applies to: acA1600-60gc

        This feature controls the current electrical format of the selected physical input or output Line. Line Format can take any of the following values: No Connect: The Line is not connected. Tri-state: The Line is currently in Tri-state mode (Not driven). TTL: The Line is currently accepting or sending TTL level signals. LVDS: The Line is currently accepting or sending LVDS level signals. RS-422: The Line is currently accepting or sending RS-422 level signals. Opto-coupled: The Line is Opto-coupled. 
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineFormat" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LineFormatEnums>& LineFormat;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Enables the signal inverter function for the selected input or output line - Applies to: acA1600-60gc

        This boolean value enables the signal inverter function for the selected input or output line.
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineInverter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& LineInverter;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineLogic" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LineLogicEnums>& LineLogic;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the mode for the selected line - Applies to: acA1600-60gc

        This feature controls whether the physical Line is used to Input or Output a signal. When a Line supports input and output mode, the default state is Input to avoid possible electrical contention. Line Mode can take any of the following values: Input: The selected physical line is used to input an electrical signal. Output: The selected physical line is used to output an electrical signal.
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LineModeEnums>& LineMode;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Selects the I/O line to configure  Once a line has been selected, all changes to the line settings will be applied to the selected line - Applies to: acA1600-60gc

        This enumeration selects the I/O line to configure. Once a line has been selected, all changes to the line settings will be applied to the selected line.
    
        Visibility: Beginner

        Selecting Parameters: LineMode, LineDebouncerTimeAbs, LineDebouncerTimeRaw, LineInverter, LineTermination, LineStatus, LineSource, LineFormat, MinOutPulseWidthAbs and MinOutPulseWidthRaw

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LineSelectorEnums>& LineSelector;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the source signal for the selected line (if the selected line is an output) - Applies to: acA1600-60gc

        This enumeration selects the internally generated camera signal (source signal) for the selected line when the selected line is an output. 
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<LineSourceEnums>& LineSource;

    //@}


    //! \name Categories: LineStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an line start overtrigger event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an line start overtrigger event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineStartOvertriggerEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LineStartOvertriggerEventStreamChannelIndex;

    //@}


    //! \name Categories: LineStartOvertriggerEventData
    //@{
    /*!
        \brief Indicates the time stamp for an line start overtrigger event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an line start overtrigger event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineStartOvertriggerEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LineStartOvertriggerEventTimestamp;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Indicates the current logical state for the selected line - Applies to: acA1600-60gc

        This boolean value indicates the current logical state for the selected line at the time of polling.
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineStatus" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& LineStatus;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief A single bitfield indicating the current logical state of all available line signals at time of polling - Applies to: acA1600-60gc

        This integer value is a single bitfield that indicates the current logical state of all available lines at time of polling.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineStatusAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& LineStatusAll;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Enables the termination resistor for the selected input line - Applies to: acA1600-60gc

        This boolean value enables the termination resistor for the selected input line.
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=LineTermination" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& LineTermination;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief Enables the median filter - Applies to: acA1600-60gc

        This boolean enables the median filter.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=MedianFilter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& MedianFilter;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the absolute value (in microseconds) for the minimum signal width of an output signal - Applies to: acA1600-60gc

        This float value sets the absolute value (in microseconds) for the minimum signal width of a signal that is received from the frequency converter or from the shaft encoder module and that is associated with a digital output line.
    
        Visibility: Beginner

        Selected by: LineSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=MinOutPulseWidthAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& MinOutPulseWidthAbs;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the X offset (left offset) of the area of interest in pixels - Applies to: acA1600-60gc

        This value sets the X offset (left offset) for the area of interest in pixels, i.e., the distance in pixels between the left side of the sensor and the left side of the image area.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=OffsetX" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& OffsetX;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the Y offset (top offset) for the area of interest in pixels - Applies to: acA1600-60gc

        This value sets the Y offset (top offset) for the area of interest, i.e., the distance in pixels between the top of the sensor and the top of the image area.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=OffsetY" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& OffsetY;

    //@}


    //! \name Categories: OverTemperatureEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an over temperature event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an over temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=OverTemperatureEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& OverTemperatureEventStreamChannelIndex;

    //@}


    //! \name Categories: OverTemperatureEventData
    //@{
    /*!
        \brief Indicates the time stamp for an over temperature event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an over temperature event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=OverTemperatureEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& OverTemperatureEventTimestamp;

    //@}


    //! \name Categories: RemoveParamLimits
    //@{
    /*!
        \brief Selects the parameter to configure  Once a parameter has been selected, all changes made using the Remove Limits feature will be applied to the selected parameter - Applies to: acA1600-60gc

        This enumeration selects the parameter to configure. Selects the parameter to configure. Once a parameter has been selected, all changes made using the Remove Limits feature will be applied to the selected parameter
    
        Visibility: Guru

        Selecting Parameters: RemoveLimits

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ParameterSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ParameterSelectorEnums>& ParameterSelector;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Size of the payload in bytes - Applies to: acA1600-60gc

        Size of the payload in bytes. This is the total number of bytes sent in the payload. Image data + chunk data if present. No packet headers.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PayloadSize" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& PayloadSize;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

         This is a read only feature. This enumeration provides a list of values that indicate the alignment of the camera's Bayer filter to the pixels in the acquired images.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelColorFilter" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<PixelColorFilterEnums>& PixelColorFilter;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Indicates the maximum possible pixel value that could be transferred from the camera - Applies to: acA1600-60gc

        This a read only feature. It indicates the maximum possible pixel value that could be transferred from the camera.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelDynamicRangeMax" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& PixelDynamicRangeMax;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Indicates the minimum possible pixel value that could be transferred from the camera - Applies to: acA1600-60gc

        This a read only feature. It indicates the minimum possible pixel value that could be transferred from the camera.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelDynamicRangeMin" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& PixelDynamicRangeMin;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Sets the format of the pixel data transmitted for acquired images - Applies to: acA1600-60gc

        This enumeration sets the format of the pixel data transmitted for acquired images. 
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelFormat" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<PixelFormatEnums>& PixelFormat;

    //@}


    //! \name Categories: TransportLayer
    //@{
    /*!
        \brief Select legacy pixel format encoding - Applies to: acA1600-60gc

        This switch selects a legacy GVSP pixel format encoding, for compatibility with older camera models.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelFormatLegacy" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& PixelFormatLegacy;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Indicates the depth of the pixel values in the image in bits per pixel - Applies to: acA1600-60gc

        This is a read only feature. This enumeration provides a list of values that indicate the depth of the pixel values in the acquired images in bits per pixel. This value will always be coherent with the pixel format setting.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=PixelSize" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<PixelSizeEnums>& PixelSize;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Enables color improved RGB raw output - Applies to: acA1600-60gc

        Enables color improvement of RGB data and provides for their output as RGB raw data. Only available for cameras with an RGB Bayer filter.
Note: Make sure to also select a suitable raw pixel data output format.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ProcessedRawEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ProcessedRawEnable;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Indicates the sensor readout time given the current settings - Applies to: acA1600-60gc

    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ReadoutTimeAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ReadoutTimeAbs;

    //@}


    //! \name Categories: RemoveParamLimits
    //@{
    /*!
        \brief Removes the factory-set limits of the selected parameter - Applies to: acA1600-60gc

        Removes the factory-set limits of the selected parameter. Having removed the factory-set limits you may set the parameter within extended limits. These are only defined by technical restrictions. Note:  Inferior image quality may result.
    
        Visibility: Guru

        Selected by: ParameterSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=RemoveLimits" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& RemoveLimits;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Indicates the minimum allowed frame acquisition period (in microseconds) given the current settings for the area of interest, exposure time, and bandwidth - Applies to: acA1600-60gc

        Indicates the 'absolute' value of the minimum allowed acquisition frame period. The 'absolute' value is a float value that indicates the minimum allowed acquisition frame period in microseconds given the current settings for the area of interest, exposure time, and bandwidth.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ResultingFramePeriodAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ResultingFramePeriodAbs;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Indicates the maximum allowed frame acquisition rate (in frames per second) given the current settings for the area of interest, exposure time, and bandwidth - Applies to: acA1600-60gc

        Indicates the 'absolute' value of the maximum allowed acquisition frame rate. The 'absolute' value is a float value that indicates the maximum allowed acquisition frame rate in frames per second given the current settings for the area of interest, exposure time, and bandwidth.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ResultingFrameRateAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& ResultingFrameRateAbs;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Enables the horizontal flipping of the image - Applies to: acA1600-60gc

        This feature is used to flip horizontally the image sent by the device. The AOI is applied after the flipping.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ReverseX" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ReverseX;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Enables the vertical flipping of the image - Applies to: acA1600-60gc

        This feature is used to flip vertically the image sent by the device. The AOI is applied after the flipping.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ReverseY" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& ReverseY;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the height of the camera's sensor in pixels - Applies to: acA1600-60gc

        This is a read only element. It is an integer that indicates the actual height of the camera's sensor in pixels.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SensorHeight" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SensorHeight;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the width of the camera's sensor in pixels - Applies to: acA1600-60gc

        This is a read only element. It is an integer that indicates the actual width of the camera's sensor in pixels.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SensorWidth" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SensorWidth;

    //@}


    //! \name Categories: SequenceControlConfiguration
    //@{
    /*!
        \brief Selects a bit of the sequence set address - Applies to: acA1600-60gc

        Selects a bit of the sequence set address.
    
        Visibility: Guru

        Selecting Parameters: SequenceAddressBitSource

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceAddressBitSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SequenceAddressBitSelectorEnums>& SequenceAddressBitSelector;

    //@}


    //! \name Categories: SequenceControlConfiguration
    //@{
    /*!
        \brief Selects the source for the selected bit of the sequence set address - Applies to: acA1600-60gc

        Selects the source for setting the selected bit of the sequence set address.
    
        Visibility: Guru

        Selected by: SequenceAddressBitSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceAddressBitSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SequenceAddressBitSourceEnums>& SequenceAddressBitSource;

    //@}


    //! \name Categories: SequenceControlConfiguration
    //@{
    /*!
        \brief Selects the sequence set advance mode - Applies to: acA1600-60gc

        Selects the sequence set advance mode. Possible values: Auto - automatic sequence set advance as images are acquired. Controlled - sequence set advance controlled by settable source. Free selection - the sequence sets are selected according to the states of the input lines.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceAdvanceMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SequenceAdvanceModeEnums>& SequenceAdvanceMode;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Allows asynchronous advance from one sequence set to the next - Applies to: acA1600-60gc

        Allows to advance from the current sequence set to the next one. The advance is asynchronous to the cameras's frame trigger. Only available in Controlled sequence advance mode.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceAsyncAdvance" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& SequenceAsyncAdvance;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Allows asynchronous restart of the sequence of sequence sets  - Applies to: acA1600-60gc

        Allows to restart the sequence of sequence sets to image acquisition, starting with the sequence set of lowest index number. The restart is asynchronous to the cameras's frame trigger. Only available in Auto and Controlled sequence advance mode.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceAsyncRestart" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& SequenceAsyncRestart;

    //@}


    //! \name Categories: SequenceControlConfiguration
    //@{
    /*!
        \brief Selects between sequence restart or sequence set advance - Applies to: acA1600-60gc

        Selects between controls for sequence restart or sequence set advance.
    
        Visibility: Guru

        Selecting Parameters: SequenceControlSource

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceControlSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SequenceControlSelectorEnums>& SequenceControlSelector;

    //@}


    //! \name Categories: SequenceControlConfiguration
    //@{
    /*!
        \brief Selects the source for sequence control - Applies to: acA1600-60gc

        Selects the source for sequence control. Possible values: Disabled - advance via asynchronous advance. Always Active - automatic sequence set advance. The sequence repeat starts with sequence set index number 1. Line N - the source for sequence restart or sequence set advance is line N. CCN - the source for sequence restart or sequence set advance is CCN.
    
        Visibility: Guru

        Selected by: SequenceControlSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceControlSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SequenceControlSourceEnums>& SequenceControlSource;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Current sequence set - Applies to: acA1600-60gc

        Indicates the current sequence set.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceCurrentSet" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SequenceCurrentSet;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Enables the sequencer - Applies to: acA1600-60gc

        Enables the existing sequence sets for image acquisition.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceEnable" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& SequenceEnable;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Sets the number of sequence set executions - Applies to: acA1600-60gc

        Sets the number of consecutive executions per sequence cycle for the selected sequence set. Only available in Auto sequence advance mode.
    
        Visibility: Guru

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceSetExecutions" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SequenceSetExecutions;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Selects the index number of a sequence set - Applies to: acA1600-60gc

        Selects the index number of a sequence set.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceSetIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SequenceSetIndex;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Loads a sequence set - Applies to: acA1600-60gc

        Loads an existing sequence set to make it the current sequence set.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceSetLoad" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& SequenceSetLoad;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Stores the current sequence set - Applies to: acA1600-60gc

        Stores the current sequence set as one of the sequence sets of the sequence. Note: Storing the current sequence set will overwrite any already existing sequence set bearing the same index number. Note: The sequence set is stored in the volatile memory and will therefore be lost if the camera is reset or if power is switched off.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceSetStore" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& SequenceSetStore;

    //@}


    //! \name Categories: SequenceControl
    //@{
    /*!
        \brief Total number of sequence sets - Applies to: acA1600-60gc

        Sets the total number of sequence sets in the sequence.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SequenceSetTotalNumber" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SequenceSetTotalNumber;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief  - Applies to: acA1600-60gc

    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Sharpness" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& Sharpness;

    //@}


    //! \name Categories: ImageQuality
    //@{
    /*!
        \brief  - Applies to: acA1600-60gc

    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SharpnessRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SharpnessRaw;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the shutter mode - Applies to: acA1600-60gc

        This enumeration sets the shutter mode.
    
        Visibility: Expert

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=ShutterMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<ShutterModeEnums>& ShutterMode;

    //@}


    //! \name Categories: AnalogControls
    //@{
    /*!
        \brief Sets the substrate voltage - Applies to: acA1600-60gc

        This value sets the substrate voltage
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SubstrateVoltage" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SubstrateVoltage;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Beginner

        Selecting Parameters: SyncUserOutputValue

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SyncUserOutputSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<SyncUserOutputSelectorEnums>& SyncUserOutputSelector;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the state of the selected user settable synchronous output signal - Applies to: acA1600-60gc

        This boolean value sets the state of the selected user settable synchronous output signal.
    
        Visibility: Beginner

        Selected by: SyncUserOutputSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SyncUserOutputValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& SyncUserOutputValue;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief A single bitfield that sets the state of all user settable synchronous output signals in one access - Applies to: acA1600-60gc

        This integer value is a single bitfield that sets the state of all user settable synchronous output signals in one access.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=SyncUserOutputValueAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& SyncUserOutputValueAll;

    //@}


    //! \name Categories: ImageFormat
    //@{
    /*!
        \brief Selecting a test image from the list will enable the test image - Applies to: acA1600-60gc

        This enumeration provides a list of the available test images. Selecting a test image from the list will enable the test image.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TestImageSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TestImageSelectorEnums>& TestImageSelector;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Directly sets the delay for the selected timer in microseconds - Applies to: acA1600-60gc

        This float value sets the delay for the selected timer in microseconds.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDelayAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& TimerDelayAbs;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the 'raw' delay for the selected timer   Actual delay = raw timer delay setting  x  timer delay time base abs setting - Applies to: acA1600-60gc

        This value sets an integer that will be used as a multiplier for the timer delay timebase. The actual delay time equals the current timer delay raw setting times the current timer delay time base abs setting.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDelayRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& TimerDelayRaw;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the time base (in microseconds) that is used when a timer delay is set with the 'timer delay raw' setting - Applies to: acA1600-60gc

        This float value sets the time base (in microseconds) that is used when a timer delay is set with the 'raw' setting.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDelayTimebaseAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& TimerDelayTimebaseAbs;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Directly sets the duration for the selected timer in microseconds - Applies to: acA1600-60gc

        This float value sets the duration for the selected timer in microseconds.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDurationAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& TimerDurationAbs;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the 'raw' duration for the selected timer   Actual duration = raw timer duration setting  x  timer duration time base abs setting - Applies to: acA1600-60gc

        This value sets an integer that will be used as a multiplier for the timer duration timebase. The actual duration time equals the current timer duration raw setting times the current timer duration time base abs setting.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDurationRaw" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& TimerDurationRaw;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the time base (in microseconds) that is used when a timer duration is set with the 'timer duration raw' setting - Applies to: acA1600-60gc

        This float value sets the time base (in microseconds) that is used when a timer duration is set with the 'raw' setting.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerDurationTimebaseAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& TimerDurationTimebaseAbs;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Selects the timer to configure   Once a timer has been selected, all changes to the timer settings will be applied to the selected timer - Applies to: acA1600-60gc

        This enumeration selects the timer to configure. . Once a timer has been selected, all changes to the timer settings will be applied to the selected timer.
    
        Visibility: Beginner

        Selecting Parameters: TimerDurationAbs, TimerDurationRaw, TimerDelayAbs, TimerDelayRaw, TimerTriggerSource and TimerTriggerActivation

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TimerSelectorEnums>& TimerSelector;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the type of signal transistion that will start the timer - Applies to: acA1600-60gc

        This enumeration sets the type of signal transistion that will start the timer.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerTriggerActivation" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TimerTriggerActivationEnums>& TimerTriggerActivation;

    //@}


    //! \name Categories: TimerControls
    //@{
    /*!
        \brief Sets the internal camera signal used to trigger the selected timer - Applies to: acA1600-60gc

        This enumeration sets the internal camera signal used to trigger the selected timer.
    
        Visibility: Beginner

        Selected by: TimerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TimerTriggerSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TimerTriggerSourceEnums>& TimerTriggerSource;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the signal transition needed to activate the selected trigger - Applies to: acA1600-60gc

        This enumeration sets the signal transition needed to activate the selected trigger.
    
        Visibility: Beginner

        Selected by: TriggerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerActivation" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TriggerActivationEnums>& TriggerActivation;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the trigger delay time in microseconds - Applies to: acA1600-60gc

        This float value sets the absolute trigger delay in microseconds to apply after the trigger reception before effectively activating it.
    
        Visibility: Expert

        Selected by: TriggerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerDelayAbs" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IFloatEx& TriggerDelayAbs;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the mode for the selected trigger - Applies to: acA1600-60gc

        This enumeration sets the trigger mode for the selected trigger.
    
        Visibility: Beginner

        Selected by: TriggerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerMode" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TriggerModeEnums>& TriggerMode;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Selects the trigger type to configure  Once a trigger type has been selected, all changes to the trigger settings will be applied to the selected trigger - Applies to: acA1600-60gc

        This enumeration selects the trigger type to configure. Once a trigger type has been selected, all changes to the trigger settings will be applied to the selected trigger.
    
        Visibility: Beginner

        Selecting Parameters: TriggerMode, TriggerSoftware, TriggerSource, TriggerActivation and TriggerDelayAbs

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TriggerSelectorEnums>& TriggerSelector;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Generates a software trigger signal that is used when the trigger source is set to 'software' - Applies to: acA1600-60gc

        This command generates a software trigger signal. The software trigger signal will be used if the trigger source is set to 'software'.
    
        Visibility: Beginner

        Selected by: TriggerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerSoftware" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& TriggerSoftware;

    //@}


    //! \name Categories: AcquisitionTrigger
    //@{
    /*!
        \brief Sets the signal source for the selected trigger - Applies to: acA1600-60gc

        This enumeration sets the signal source for the selected trigger.
    
        Visibility: Beginner

        Selected by: TriggerSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=TriggerSource" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<TriggerSourceEnums>& TriggerSource;

    //@}


    //! \name Categories: UserDefinedValues
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        Selected by: UserDefinedValueSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserDefinedValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& UserDefinedValue;

    //@}


    //! \name Categories: UserDefinedValues
    //@{
    /*!
        \brief  Applies to: acA1600-60gc

    
        Visibility: Guru

        Selecting Parameters: UserDefinedValue

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserDefinedValueSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<UserDefinedValueSelectorEnums>& UserDefinedValueSelector;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Selects the user settable output signal to configure  Once a user settable output signal has been selected, all changes to the user settable output signal settings will be applied to the selected user settable output signal - Applies to: acA1600-60gc

        This enumeration selects the user settable output signal to configure. Once a user settable output signal has been selected, all changes to the user settable output signal settings will be applied to the selected user settable output signal.
    
        Visibility: Beginner

        Selecting Parameters: UserOutputValue

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserOutputSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<UserOutputSelectorEnums>& UserOutputSelector;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief Sets the state of the selected user settable output signal - Applies to: acA1600-60gc

        This boolean value sets the state of the selected user settable output signal.
    
        Visibility: Beginner

        Selected by: UserOutputSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserOutputValue" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IBooleanEx& UserOutputValue;

    //@}


    //! \name Categories: DigitalIO
    //@{
    /*!
        \brief A single bitfield that sets the state of all user settable output signals in one access - Applies to: acA1600-60gc

        This integer value is a single bitfield that sets the state of all user settable output signals in one access.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserOutputValueAll" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& UserOutputValueAll;

    //@}


    //! \name Categories: UserSets
    //@{
    /*!
        \brief Sets the configuration set to be used as the default startup set  The configuration set that has been selected as the default startup set will be loaded as the active set whenever the camera is powered on or reset - Applies to: acA1600-60gc

        This enumeration sets the configuration set to be used as the default startup set. The configuration set that has been selected as the default startup set will be loaded as the active set whenever the camera is powered on or reset.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserSetDefaultSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<UserSetDefaultSelectorEnums>& UserSetDefaultSelector;

    //@}


    //! \name Categories: UserSets
    //@{
    /*!
        \brief Loads the selected configuration into the camera's volatile memory and makes it the active configuration set  Once the selected set is loaded, the parameters in the selected set will control the camera - Applies to: acA1600-60gc

        This command loads the selected configuration set from the non-volatile memory in the camera to the volatile memory and makes the selected set the active configuration set. Once the selected set is loaded, the parameters in the selected set will control the camera.
    
        Visibility: Beginner

        Selected by: UserSetSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserSetLoad" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& UserSetLoad;

    //@}


    //! \name Categories: UserSets
    //@{
    /*!
        \brief Saves the current active configuration set into the selected user set   - Applies to: acA1600-60gc

        This command copies the parameters in the current active configuration set into the selected user set in the camera's non-volatile memory.
    
        Visibility: Beginner

        Selected by: UserSetSelector

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserSetSave" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::ICommandEx& UserSetSave;

    //@}


    //! \name Categories: UserSets
    //@{
    /*!
        \brief Selects the configuration set to load, save, or configure  Once a configuration set has been selected, all changes to the configuration set settings will be applied to the selected configuration set - Applies to: acA1600-60gc

        This enumeration selects the configuration set to load, save or configure. Possible values for the User Set Selector are: Default: Selects a configuration set that contains factory settings. User Set 1: Selects the first user set. When the Default configuration set is selected and loaded using User Set Load, the device must be in default factory settings state and must make sure the mandatory continuous acquisition use case works directly. Default User Set is read-only and cannot be modified.
    
        Visibility: Beginner

        Selecting Parameters: UserSetLoad and UserSetSave

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=UserSetSelector" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IEnumParameterT<UserSetSelectorEnums>& UserSetSelector;

    //@}


    //! \name Categories: VirtualLine1RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io virtual line 1 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io virtual line 1 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine1RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine1RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: VirtualLine1RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a virtual line 1 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io virtual line 1 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine1RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine1RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: VirtualLine2RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io virtual line 2 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io virtual line 2 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine2RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine2RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: VirtualLine2RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a virtual line 2 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io virtual line 2 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine2RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine2RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: VirtualLine3RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io virtual line 3 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io virtual line 3 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine3RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine3RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: VirtualLine3RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a virtual line 3 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io virtual line 3 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine3RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine3RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: VirtualLine4RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the stream channel index for an io virtual line 4 rising edge event - Applies to: acA1600-60gc

        This enumeration Indicates the stream channel index for an io virtual line 4 rising edge event
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine4RisingEdgeEventStreamChannelIndex" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine4RisingEdgeEventStreamChannelIndex;

    //@}


    //! \name Categories: VirtualLine4RisingEdgeEventData
    //@{
    /*!
        \brief Indicates the time stamp for a virtual line 4 rising edge event - Applies to: acA1600-60gc

        This enumeration value indicates the time stamp for an io virtual line 4 rising edge event.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=VirtualLine4RisingEdgeEventTimestamp" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& VirtualLine4RisingEdgeEventTimestamp;

    //@}


    //! \name Categories: AOI
    //@{
    /*!
        \brief Sets the width of the area of interest in pixels - Applies to: acA1600-60gc

        This value sets the width of the area of interest in pixels.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=Width" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& Width;

    //@}


    //! \name Categories: DeviceInformation
    //@{
    /*!
        \brief Indicates the maximum allowed width of the image in pixels - Applies to: acA1600-60gc

        This is a read only element. It is an integer that indicates maximum allowed width of the image in pixels taking into account any function that may limit the allowed width.
    
        Visibility: Beginner

        The <a href="https://docs.baslerweb.com/?rhcsh=1&rhmapid=WidthMax" target="_blank">Basler Product Documentation</a> may provide more information.
    */
    Pylon::IIntegerEx& WidthMax;

    //@}


        private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            BaslerCameraCameraParams(BaslerCameraCameraParams&);

            //! not implemented assignment operator
            BaslerCameraCameraParams& operator=(BaslerCameraCameraParams&);

        //! \endcond
    };

} // namespace Pylon
} // namespace BaslerCameraCameraParams_Params

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_BASLERCAMERACAMERAPARAMS_H