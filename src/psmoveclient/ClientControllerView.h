#ifndef CLIENT_CONTROLLER_VIEW_H
#define CLIENT_CONTROLLER_VIEW_H

//-- includes -----
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include "ClientGeometry.h"
#include <cassert>

//-- pre-declarations -----
namespace PSMoveProtocol
{
    class DeviceOutputDataFrame;
    class DeviceOutputDataFrame_ControllerDataPacket;
    class DeviceInputDataFrame;
    class DeviceInputDataFrame_ControllerDataPacket;
};

//-- constants -----
enum PSMoveButtonState {
    PSMoveButton_UP = 0x00,       // (00b) Not pressed
    PSMoveButton_PRESSED = 0x01,  // (01b) Down for one frame only
    PSMoveButton_DOWN = 0x03,     // (11b) Down for >1 frame
    PSMoveButton_RELEASED = 0x02, // (10b) Up for one frame only
};

enum PSMoveBatteryState
{
    PSMoveBattery_0        = 0,
    PSMoveBattery_20       = 1,
    PSMoveBattery_40       = 2,
    PSMoveBattery_60       = 3,
    PSMoveBattery_80       = 4,
    PSMoveBattery_100      = 5,
    PSMoveBattery_Charging = 0xEE,
    PSMoveBattery_Charged  = 0xEF
};

enum PSMoveTrackingColorType {
    Magenta,    // R:0xFF, G:0x00, B:0xFF
    Cyan,       // R:0x00, G:0xFF, B:0xFF
    Yellow,     // R:0xFF, G:0xFF, B:0x00
    Red,        // R:0xFF, G:0x00, B:0x00
    Green,      // R:0x00, G:0xFF, B:0x00
    Blue,       // R:0x00, G:0x00, B:0xFF

    MAX_PSMOVE_COLOR_TYPES
};

//-- declarations -----
struct PSM_CPP_PUBLIC_CLASS PSMovePhysicsData
{
    PSMoveFloatVector3 VelocityCmPerSec;
    PSMoveFloatVector3 AccelerationCmPerSecSqr;
    PSMoveFloatVector3 AngularVelocityRadPerSec;
    PSMoveFloatVector3 AngularAccelerationRadPerSecSqr;

    inline void Clear()
    {
        VelocityCmPerSec = *k_psmove_float_vector3_zero;
        AccelerationCmPerSecSqr = *k_psmove_float_vector3_zero;
        AngularVelocityRadPerSec = *k_psmove_float_vector3_zero;
        AngularAccelerationRadPerSecSqr = *k_psmove_float_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSMoveRawSensorData
{
    PSMoveIntVector3 Magnetometer;
    PSMoveIntVector3 Accelerometer;
    PSMoveIntVector3 Gyroscope;

    inline void Clear()
    {
        Magnetometer= *k_psmove_int_vector3_zero;
        Accelerometer = *k_psmove_int_vector3_zero;
        Gyroscope = *k_psmove_int_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSMoveCalibratedSensorData
{
    PSMoveFloatVector3 Magnetometer;
    PSMoveFloatVector3 Accelerometer;
    PSMoveFloatVector3 Gyroscope;

    inline void Clear()
    {
        Magnetometer = *k_psmove_float_vector3_zero;
        Accelerometer = *k_psmove_float_vector3_zero;
        Gyroscope = *k_psmove_float_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSMoveRawTrackerData
{
    // Parallel arrays: ScreenLocations, Positions and the TrackerID associated with them
    PSMoveScreenLocation ScreenLocations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMovePosition RelativePositionsCm[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMoveQuaternion RelativeOrientations[PSMOVESERVICE_MAX_TRACKER_COUNT];
    PSMoveTrackingProjection TrackingProjections[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int TrackerIDs[PSMOVESERVICE_MAX_TRACKER_COUNT];
    int ValidTrackerLocations;

	PSMovePosition MulticamPositionCm;
	PSMoveQuaternion MulticamOrientation;
	bool bMulticamPositionValid;
	bool bMulticamOrientationValid;

    inline void Clear()
    {
        for (int index = 0; index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++index)
        {
            ScreenLocations[index] = PSMoveScreenLocation::create(0, 0);
            RelativePositionsCm[index] = *k_psmove_position_origin;
            RelativeOrientations[index]= *k_psmove_quaternion_identity;
            TrackerIDs[index] = -1;
        }
        ValidTrackerLocations = 0;

		MulticamPositionCm= PSMovePosition::create(0.f, 0.f, 0.f);
		MulticamOrientation= PSMoveQuaternion::identity();
		bMulticamPositionValid= false;
		bMulticamOrientationValid= false;
    }

    inline bool GetPixelLocationOnTrackerId(int trackerId, PSMoveScreenLocation &outLocation) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outLocation = ScreenLocations[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }

    inline bool GetPositionOnTrackerId(int trackerId, PSMovePosition &outPosition) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outPosition = RelativePositionsCm[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }

    inline bool GetOrientationOnTrackerId(int trackerId, PSMoveQuaternion &outOrientation) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outOrientation = RelativeOrientations[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }

    inline bool GetProjectionOnTrackerId(int trackerId, PSMoveTrackingProjection &outProjection) const
    {
        bool bFound = false;

        for (int listIndex = 0; listIndex < ValidTrackerLocations; ++listIndex)
        {
            if (TrackerIDs[listIndex] == trackerId)
            {
                outProjection = TrackingProjections[listIndex];
                bFound = true;
                break;
            }
        }

        return bFound;
    }
};

struct PSM_CPP_PUBLIC_CLASS ClientPSMoveView
{
private:
    bool bValid;
    bool bHasValidHardwareCalibration;
    bool bIsTrackingEnabled;
    bool bIsCurrentlyTracking;
    bool bIsOrientationValid;
    bool bIsPositionValid;
    bool bHasUnpublishedState;

    PSMovePose Pose;
    PSMovePhysicsData PhysicsData;
    PSMoveRawSensorData RawSensorData;
    PSMoveCalibratedSensorData CalibratedSensorData;
    PSMoveRawTrackerData RawTrackerData;

    PSMoveButtonState TriangleButton;
    PSMoveButtonState CircleButton;
    PSMoveButtonState CrossButton;
    PSMoveButtonState SquareButton;
    PSMoveButtonState SelectButton;
    PSMoveButtonState StartButton;
    PSMoveButtonState PSButton;
    PSMoveButtonState MoveButton;
    PSMoveButtonState TriggerButton;

    unsigned char TriggerValue;
	PSMoveBatteryState BatteryValue;

    unsigned char Rumble;
    unsigned char LED_r, LED_g, LED_b;

	long long ResetPoseButtonPressTime;
	bool bResetPoseRequestSent;
	bool bPoseResetButtonEnabled;

public:
    void Clear();
    void ApplyControllerDataFrame(class ClientControllerView *parentView, const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame);

    void SetRumble(float rumbleFraction);
    void SetLEDOverride(unsigned char red, unsigned char g, unsigned char b);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline void SetValid(bool flag)
    {
        bValid= flag;
    }

    inline bool GetHasValidHardwareCalibration() const
    {
        return IsValid() ? bHasValidHardwareCalibration : false;
    }

    inline bool GetIsCurrentlyTracking() const
    {
        return IsValid() ? bIsCurrentlyTracking : false;
    }
    
    inline bool GetIsTrackingEnabled() const
    {
        return IsValid() ? bIsTrackingEnabled : false;
    }

    inline bool GetIsOrientationValid() const
    {
        return IsValid() ? bIsOrientationValid : false;
    }

    inline bool GetIsPositionValid() const
    {
        return IsValid() ? bIsPositionValid : false;
    }

    inline bool GetHasUnpublishedState() const
    {
        return IsValid() ? bHasUnpublishedState : false;
    }

    inline const PSMovePose &GetPose() const
    {
        return IsValid() ? Pose : *k_psmove_pose_identity;
    }

    inline const PSMovePosition &GetPosition() const
    {
        return IsValid() ? Pose.Position : *k_psmove_position_origin;
    }

    inline const PSMoveQuaternion &GetOrientation() const
    {
        return IsValid() ? Pose.Orientation : *k_psmove_quaternion_identity;
    }

    inline PSMoveButtonState GetButtonTriangle() const
    {
        return IsValid() ? TriangleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCircle() const
    {
        return IsValid() ? CircleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCross() const
    {
        return IsValid() ? CrossButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonSquare() const
    {
        return IsValid() ? SquareButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonSelect() const
    {
        return IsValid() ? SelectButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonStart() const
    {
        return IsValid() ? StartButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonPS() const
    {
        return IsValid() ? PSButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonMove() const
    {
        return IsValid() ? MoveButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTrigger() const
    {
        return IsValid() ? TriggerButton : PSMoveButton_UP;
    }

	inline PSMoveBatteryState GetBatteryValue() const 
	{
		return IsValid() ? BatteryValue : PSMoveBattery_0;
	}

    inline unsigned char GetTriggerValue() const
    {
        return IsValid() ? TriggerValue : 0;
    }

    inline float GetRumble() const
    {
        return IsValid() ? static_cast<float>(Rumble) / 255.f : 0.f;
    }

    const PSMovePhysicsData &GetPhysicsData() const;
    const PSMoveRawSensorData &GetRawSensorData() const;
    const PSMoveCalibratedSensorData &GetCalibratedSensorData() const;
    bool GetIsStable() const;
	bool GetIsGyroStable() const;
    bool GetIsStableAndAlignedWithGravity() const;


    const PSMoveRawTrackerData &GetRawTrackerData() const;

	inline void SetPoseResetButtonEnabled(bool bEnabled)
	{
		bPoseResetButtonEnabled = bEnabled;
	}

protected:
	void ProcessRecenterAction(class ClientControllerView *parentView);
};

class PSM_CPP_PUBLIC_CLASS ClientPSNaviView
{
private:
    bool bValid;

    PSMoveButtonState L1Button;
    PSMoveButtonState L2Button;
    PSMoveButtonState L3Button;
    PSMoveButtonState CircleButton;
    PSMoveButtonState CrossButton;
    PSMoveButtonState PSButton;
    PSMoveButtonState TriggerButton;
    PSMoveButtonState DPadUpButton;
    PSMoveButtonState DPadRightButton;
    PSMoveButtonState DPadDownButton;
    PSMoveButtonState DPadLeftButton;

    unsigned char TriggerValue;
    unsigned char Stick_XAxis;
    unsigned char Stick_YAxis;

public:
    void Clear();
    void ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline bool GetHasUnpublishedState() const
    {
        return false;
    }

    inline PSMoveButtonState GetButtonL1() const
    {
        return IsValid() ? L1Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL2() const
    {
        return IsValid() ? L2Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL3() const
    {
        return IsValid() ? L3Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonCircle() const
    {
        return IsValid() ? CircleButton : PSMoveButton_UP;
    }
    
    inline PSMoveButtonState GetButtonCross() const
    {
        return IsValid() ? CrossButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonPS() const
    {
        return IsValid() ? PSButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTrigger() const
    {
        return IsValid() ? TriggerButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadUp() const
    {
        return IsValid() ? DPadUpButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadRight() const
    {
        return IsValid() ? DPadRightButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadDown() const
    {
        return IsValid() ? DPadDownButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadLeft() const
    {
        return IsValid() ? DPadLeftButton : PSMoveButton_UP;
    }

    inline unsigned char GetTriggerValue() const
    {
        return IsValid() ? TriggerValue : 0;
    }

    inline float GetStickXAxis() const
    {
        float float_value= 0.f;

        if (IsValid())
        {
            int signed_value= static_cast<int>(Stick_XAxis) - 0x80;

            float_value= static_cast<float>(signed_value) / static_cast<float>(0x80);
        }

        return float_value;
    }

    inline float GetStickYAxis() const
    {
        float float_value= 0.f;

        if (IsValid())
        {
            int signed_value= static_cast<int>(Stick_YAxis) - 0x80;

            float_value= static_cast<float>(signed_value) / static_cast<float>(0x80);
        }

        return float_value;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSDualShock4RawSensorData
{
    PSMoveIntVector3 Accelerometer;
    PSMoveIntVector3 Gyroscope;

    inline void Clear()
    {
        Accelerometer = *k_psmove_int_vector3_zero;
        Gyroscope = *k_psmove_int_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS PSDualShock4CalibratedSensorData
{
    PSMoveFloatVector3 Accelerometer;
    PSMoveFloatVector3 Gyroscope;

    inline void Clear()
    {
        Accelerometer = *k_psmove_float_vector3_zero;
        Gyroscope = *k_psmove_float_vector3_zero;
    }
};

struct PSM_CPP_PUBLIC_CLASS ClientPSDualShock4View
{
private:
    bool bValid;
    bool bHasValidHardwareCalibration;
    bool bIsTrackingEnabled;
    bool bIsCurrentlyTracking;
    bool bIsOrientationValid;
    bool bIsPositionValid;
    bool bHasUnpublishedState;

    PSMovePose Pose;
    PSMovePhysicsData PhysicsData;
    PSDualShock4RawSensorData RawSensorData;
    PSDualShock4CalibratedSensorData CalibratedSensorData;
    PSMoveRawTrackerData RawTrackerData;

    PSMoveButtonState DPadUpButton;
    PSMoveButtonState DPadDownButton;
    PSMoveButtonState DPadLeftButton;
    PSMoveButtonState DPadRightButton;

    PSMoveButtonState SquareButton;
    PSMoveButtonState CrossButton;
    PSMoveButtonState CircleButton;
    PSMoveButtonState TriangleButton;

    PSMoveButtonState L1Button;
    PSMoveButtonState R1Button;
    PSMoveButtonState L2Button;
    PSMoveButtonState R2Button;
    PSMoveButtonState L3Button;
    PSMoveButtonState R3Button;

    PSMoveButtonState ShareButton;
    PSMoveButtonState OptionsButton;

    PSMoveButtonState PSButton;
    PSMoveButtonState TrackPadButton;

    float LeftAnalogX;
    float LeftAnalogY;
    float RightAnalogX;
    float RightAnalogY;
    float LeftTriggerValue;
    float RightTriggerValue;

    unsigned char BigRumble, SmallRumble;
    unsigned char LED_r, LED_g, LED_b;

	long long ResetPoseButtonPressTime;
	bool bResetPoseRequestSent;
	bool bPoseResetButtonEnabled;

public:
    void Clear();
    void ApplyControllerDataFrame(class ClientControllerView *parentView, const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish(PSMoveProtocol::DeviceInputDataFrame_ControllerDataPacket *data_frame);

    void SetBigRumble(float rumbleFraction);
    void SetSmallRumble(float rumbleFraction);
    void SetLEDOverride(unsigned char red, unsigned char g, unsigned char b);

    inline bool IsValid() const
    {
        return bValid;
    }

    inline void SetValid(bool flag)
    {
        bValid = flag;
    }

    inline bool GetHasValidHardwareCalibration() const
    {
        return IsValid() ? bHasValidHardwareCalibration : false;
    }

    inline bool GetIsCurrentlyTracking() const
    {
        return IsValid() ? bIsCurrentlyTracking : false;
    }

    inline bool GetIsTrackingEnabled() const
    {
        return IsValid() ? bIsTrackingEnabled : false;
    }

    inline bool GetIsOrientationValid() const
    {
        return IsValid() ? bIsOrientationValid : false;
    }

    inline bool GetIsPositionValid() const
    {
        return IsValid() ? bIsPositionValid : false;
    }

    inline bool GetHasUnpublishedState() const
    {
        return IsValid() ? bHasUnpublishedState : false;
    }

    inline const PSMovePose &GetPose() const
    {
        return IsValid() ? Pose : *k_psmove_pose_identity;
    }

    inline const PSMovePosition &GetPosition() const
    {
        return IsValid() ? Pose.Position : *k_psmove_position_origin;
    }

    inline const PSMoveQuaternion &GetOrientation() const
    {
        return IsValid() ? Pose.Orientation : *k_psmove_quaternion_identity;
    }

    inline PSMoveButtonState GetButtonDPadUp() const
    {
        return IsValid() ? DPadUpButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadDown() const
    {
        return IsValid() ? DPadDownButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadLeft() const
    {
        return IsValid() ? DPadLeftButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonDPadRight() const
    {
        return IsValid() ? DPadRightButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonSquare() const
    {
        return IsValid() ? SquareButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonCross() const
    {
        return IsValid() ? CrossButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonCircle() const
    {
        return IsValid() ? CircleButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTriangle() const
    {
        return IsValid() ? TriangleButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonOptions() const
    {
        return IsValid() ? OptionsButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonShare() const
    {
        return IsValid() ? ShareButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonPS() const
    {
        return IsValid() ? PSButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonTrackpad() const
    {
        return IsValid() ? TrackPadButton : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL1() const
    {
        return IsValid() ? L1Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonR1() const
    {
        return IsValid() ? R1Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL2() const
    {
        return IsValid() ? L2Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonR2() const
    {
        return IsValid() ? R2Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonL3() const
    {
        return IsValid() ? L3Button : PSMoveButton_UP;
    }

    inline PSMoveButtonState GetButtonR3() const
    {
        return IsValid() ? R3Button : PSMoveButton_UP;
    }

    inline float GetLeftAnalogX() const
    {
        return IsValid() ? LeftAnalogX : 0.f;
    }

    inline float GetLeftAnalogY() const
    {
        return IsValid() ? LeftAnalogY : 0.f;
    }

    inline float GetRightAnalogX() const
    {
        return IsValid() ? RightAnalogX : 0.f;
    }

    inline float GetRightAnalogY() const
    {
        return IsValid() ? RightAnalogY : 0.f;
    }

    inline float GetLeftTriggerValue() const
    {
        return IsValid() ? LeftTriggerValue : 0.f;
    }

    inline float GetRightTriggerValue() const
    {
        return IsValid() ? RightTriggerValue : 0.f;
    }

    inline float GetBigRumble() const
    {
        return IsValid() ? static_cast<float>(BigRumble) / 255.f : 0.f;
    }

    inline float GetSmallRumble() const
    {
        return IsValid() ? static_cast<float>(SmallRumble) / 255.f : 0.f;
    }

    const PSMovePhysicsData &GetPhysicsData() const;
    const PSDualShock4RawSensorData &GetRawSensorData() const;
    const PSDualShock4CalibratedSensorData &GetCalibratedSensorData() const;
    bool GetIsStable() const;
	bool GetIsGyroStable() const;
    const PSMoveRawTrackerData &GetRawTrackerData() const;

	inline void SetPoseResetButtonEnabled(bool bEnabled)
	{
		bPoseResetButtonEnabled = bEnabled;
	}

protected:
	void ProcessRecenterAction(class ClientControllerView *parentView);
};

class PSM_CPP_PUBLIC_CLASS ClientControllerView
{
public:
    enum eControllerType
    {
        None= -1,
        PSMove,
        PSNavi,
        PSDualShock4
    };

private:
    union
    {
        ClientPSMoveView PSMoveView;
        ClientPSNaviView PSNaviView;
        ClientPSDualShock4View PSDualShock4View;
    } ViewState;
    eControllerType ControllerViewType;

    int ControllerID;
    int OutputSequenceNum;
    int InputSequenceNum;
    int ListenerCount;

    bool IsConnected;

    long long data_frame_last_received_time;
    float data_frame_average_fps;

public:
    ClientControllerView(int ControllerID);

    void Clear();
    void ApplyControllerDataFrame(const PSMoveProtocol::DeviceOutputDataFrame_ControllerDataPacket *data_frame);
    void Publish();

    // Listener State
    inline void IncListenerCount()
    {
        ++ListenerCount;
    }

    inline void DecListenerCount()
    {
        assert(ListenerCount > 0);
        --ListenerCount;
    }

    inline int GetListenerCount() const
    {
        return ListenerCount;
    }

    // Controller Data Accessors
    inline int GetControllerID() const
    {
        return ControllerID;
    }

    inline int GetOutputSequenceNum() const
    {
        return IsValid() ? OutputSequenceNum : -1;
    }

    inline int GetInputSequenceNum() const
    {
        return IsValid() ? InputSequenceNum : -1;
    }

    bool GetHasUnpublishedState() const;

    inline eControllerType GetControllerViewType() const
    {
        return ControllerViewType;
    }

    inline const ClientPSMoveView &GetPSMoveView() const
    {
        assert(ControllerViewType == PSMove);
        return ViewState.PSMoveView;
    }

    inline ClientPSMoveView &GetPSMoveViewMutable()
    {
        assert(ControllerViewType == PSMove);
        return ViewState.PSMoveView;
    }

    inline const ClientPSNaviView &GetPSNaviView() const
    {
        assert(ControllerViewType == PSNavi);
        return ViewState.PSNaviView;
    }

    inline const ClientPSDualShock4View &GetPSDualShock4View() const
    {
        assert(ControllerViewType == PSDualShock4);
        return ViewState.PSDualShock4View;
    }

    inline ClientPSDualShock4View &GetPSDualShock4ViewMutable()
    {
        assert(ControllerViewType == PSDualShock4);
        return ViewState.PSDualShock4View;
    }

    inline bool IsValid() const
    {
        return ControllerID != -1;
    }

    inline bool GetIsConnected() const
    {
        return (IsValid() && IsConnected);
    }

    const PSMovePose &GetPose() const;
    const PSMovePosition &GetPosition() const;
    const PSMoveQuaternion &GetOrientation() const;
    const PSMovePhysicsData &GetPhysicsData() const;
    const PSMoveRawTrackerData &GetRawTrackerData() const;

    bool GetIsCurrentlyTracking() const;
    bool GetIsPoseValid() const;
	bool GetIsGyroStable() const;
    bool GetIsStable() const;

    void SetLEDOverride(unsigned char r, unsigned char g, unsigned char b);
    
    // Statistics
    inline float GetDataFrameFPS() const
    {
        return data_frame_average_fps;
    }
};

#endif
