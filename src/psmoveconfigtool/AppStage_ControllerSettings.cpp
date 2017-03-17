//-- inludes -----
#include "AppStage_ControllerSettings.h"
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_OpticalCalibration.h"
#include "AppStage_GyroscopeCalibration.h"
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_MainMenu.h"
#include "AppStage_PairController.h"
#include "AppStage_TestRumble.h"
#include "App.h"
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <sstream>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_ControllerSettings::APP_STAGE_NAME= "ControllerSettings";

//-- constants -----
const int k_default_position_filter_index = 3; // LowPassExponential
const int k_default_psmove_orientation_filter_index = 3; // ComplementaryMARG
const int k_default_ds4_position_filter_index = 5; // PositionKalman
const int k_default_ds4_orientation_filter_index = 3; // OrientationKalman
const int k_default_ds4_gyro_gain_index = 4; // 2000deg/s

const char* k_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "PositionKalman" };
const char* k_psmove_orientation_filter_names[] = { "PassThru", "MadgwickARG", "MadgwickMARG", "ComplementaryMARG", "OrientationKalman" };
const char* k_ds4_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "OrientationKalman" };
const char* k_ds4_gyro_gain_setting_labels[] = { "125deg/s", "250deg/s", "500deg/s", "1000deg/s", "2000deg/s", "custom"};

const float k_max_prediction_time = 0.15f; // About 150ms seems to be about the point where you start to get really bad over-prediction 

inline int find_string_entry(const char *string_entry, const char* string_list[], size_t list_size)
{
	int found_index = -1;
	for (size_t test_index = 0; test_index < list_size; ++test_index)
	{
		if (strncmp(string_entry, string_list[test_index], 32) == 0)
		{
			found_index = static_cast<int>(test_index);
			break;
		}
	}

	return found_index;
}

//-- public methods -----
AppStage_ControllerSettings::AppStage_ControllerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_ControllerSettings::inactive)
    , m_selectedControllerIndex(-1)
{ }

void AppStage_ControllerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

    request_controller_list();
}

void AppStage_ControllerSettings::exit()
{
    m_menuState= AppStage_ControllerSettings::inactive;
}

void AppStage_ControllerSettings::update()
{
}
    
void AppStage_ControllerSettings::render()
{
    glm::mat4 scale2RotateX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f)), 
            90.f, glm::vec3(1.f, 0.f, 0.f));    

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            if (m_selectedControllerIndex >= 0)
            {
                const ControllerInfo &controllerInfo= m_usableControllerInfos[m_selectedControllerIndex];

                switch(controllerInfo.ControllerType)
                {
                    case PSMController_Move:
                    case PSMController_DualShock4:
                        {
                            const ControllerInfo &controllerInfo = m_usableControllerInfos[m_selectedControllerIndex];

                            // Display the tracking color being used for the controller
                            glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

                            switch (controllerInfo.TrackingColorType)
                            {
                            case PSMTrackingColorType_Magenta:
                                bulb_color = glm::vec3(1.f, 0.f, 1.f);
                                break;
                            case PSMTrackingColorType_Cyan:
                                bulb_color = glm::vec3(0.f, 1.f, 1.f);
                                break;
                            case PSMTrackingColorType_Yellow:
                                bulb_color = glm::vec3(1.f, 1.f, 0.f);
                                break;
                            case PSMTrackingColorType_Red:
                                bulb_color = glm::vec3(1.f, 0.f, 0.f);
                                break;
                            case PSMTrackingColorType_Green:
                                bulb_color = glm::vec3(0.f, 1.f, 0.f);
                                break;
                            case PSMTrackingColorType_Blue:
                                bulb_color = glm::vec3(0.f, 0.f, 1.f);
                                break;
                            default:
                                break;
                            }

                            if (controllerInfo.ControllerType == PSMController_Move)
                            {
                                drawPSMoveModel(scale2RotateX90, bulb_color);
                            }
                            else
                            {
                                drawPSDualShock4Model(scale2RotateX90, bulb_color);
                            }
                        } break;
                    case PSMController_Navi:
                        {
                            drawPSNaviModel(scale2RotateX90);
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }        
            }
        } break;

    case eControllerMenuState::pendingControllerListRequest:
    case eControllerMenuState::failedControllerListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ControllerSettings::renderUI()
{
    const char *k_window_title= "Controller Settings";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(350, 490));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_hostSerial.length() > 1 && m_hostSerial != "00:00:00:00:00:00")
            {
                ImGui::Text("Host Serial: %s", m_hostSerial.c_str());
            }
            else
            {
                ImGui::Text("No bluetooth adapter detected!");
            }
            
            ImGui::Separator();

            if (m_usableControllerInfos.size() > 0)
            {
                ControllerInfo &controllerInfo= m_usableControllerInfos[m_selectedControllerIndex];

                if (m_selectedControllerIndex > 0)
                {
                    if (ImGui::Button("<##ControllerIndex"))
                    {
                        --m_selectedControllerIndex;
                    }
                    ImGui::SameLine();
                }
                ImGui::Text("Controller: %d", m_selectedControllerIndex);
                if (m_selectedControllerIndex + 1 < static_cast<int>(m_usableControllerInfos.size()))
                {
                    ImGui::SameLine();
                    if (ImGui::Button(">##ControllerIndex"))
                    {
                        ++m_selectedControllerIndex;
                    }
                }

				// Combo box selection for controller tracking color
				if (controllerInfo.ControllerType != PSMController_Navi)
				{
					int newTrackingColorType = controllerInfo.TrackingColorType;

					if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0\0"))
					{
						controllerInfo.TrackingColorType = static_cast<PSMTrackingColorType>(newTrackingColorType);

						request_set_controller_tracking_color_id(controllerInfo.ControllerID, controllerInfo.TrackingColorType);

						// Re-request the controller list since the tracking colors could changed for other controllers
						request_controller_list();
					}
				}

                ImGui::BulletText("Controller ID: %d", controllerInfo.ControllerID);

                switch(controllerInfo.ControllerType)
                {
                    case PSMController_Move:
                        {
							//###HipsterSloth $TODO - The HID report for fetching the firmware revision doesn't appear to work
                            //ImGui::BulletText("Controller Type: PSMove (v%d.%d)", controllerInfo.FirmwareVersion, controllerInfo.FirmwareRevision);
							ImGui::BulletText("Controller Type: PSMove");
                        } break;
                    case PSMController_Navi:
                        {
                            ImGui::BulletText("Controller Type: PSNavi");
                        } break;
                    case PSMController_DualShock4:
                        {
                            ImGui::BulletText("Controller Type: PSDualShock4");
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }

                ImGui::BulletText("Device Serial: %s", controllerInfo.DeviceSerial.c_str());
                ImGui::BulletText("Assigned Host Serial: %s", controllerInfo.AssignedHostSerial.c_str());

                if (controllerInfo.ControllerType == PSMController_Move)
                {
					if (controllerInfo.HasMagnetometer)
					{
						if (ImGui::Button("Calibrate Magnetometer"))
						{
							m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(false);
							m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
						}
					}
					else
					{
						ImGui::TextDisabled("Magnetometer Disabled");
					}

                    if (ImGui::Button("Calibrate Gyroscope"))
                    {
                        m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(false);
                        m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
                    }

					if (ImGui::Button("Calibrate Optical Noise"))
					{
						m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(false);
						m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
					}

                    if (ImGui::Button("Test Orientation"))
                    {
                        m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(true);
                        m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
                    }
                }

                if (controllerInfo.ControllerType == PSMController_DualShock4)
                {

					if (ImGui::Button("Calibrate Optical Noise"))
					{
						m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(false);
						m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
					}

                    if (ImGui::Button("Test Orientation"))
                    {
                        m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(true);
                        m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
                    }
                }

                if (controllerInfo.ControllerType == PSMController_Move || 
                    controllerInfo.ControllerType == PSMController_DualShock4)
                {
                    if (ImGui::Button("Test Accelerometer"))
                    {
                        m_app->getAppStage<AppStage_AccelerometerCalibration>()->setBypassCalibrationFlag(true);
                        m_app->setAppStage(AppStage_AccelerometerCalibration::APP_STAGE_NAME);
                    }

                    if (ImGui::Button("Test Rumble"))
                    {
                        m_app->setAppStage(AppStage_TestRumble::APP_STAGE_NAME);
                    }
                }

				if (controllerInfo.ControllerType == PSMController_Move)
				{		
					ImGui::PushItemWidth(195);
					if (ImGui::Combo("Position Filter", &controllerInfo.PositionFilterIndex, k_position_filter_names, UI_ARRAYSIZE(k_position_filter_names)))
					{
						controllerInfo.PositionFilterName = k_position_filter_names[controllerInfo.PositionFilterIndex];
						request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
					}
					if (ImGui::Combo("Orientation Filter", &controllerInfo.OrientationFilterIndex, k_psmove_orientation_filter_names, UI_ARRAYSIZE(k_psmove_orientation_filter_names)))
					{
						controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[controllerInfo.OrientationFilterIndex];
						request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
					}
					if (ImGui::SliderFloat("Prediction Time", &controllerInfo.PredictionTime, 0.f, k_max_prediction_time))
					{
						request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
					}
					if (ImGui::Button("Reset Filter Defaults"))
					{
						controllerInfo.PositionFilterIndex = k_default_position_filter_index;
						controllerInfo.OrientationFilterIndex = k_default_psmove_orientation_filter_index;
						controllerInfo.PositionFilterName = k_position_filter_names[k_default_position_filter_index];
						controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[k_default_psmove_orientation_filter_index];
						request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
						request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
					}
					ImGui::PopItemWidth();
				}				
				else if (controllerInfo.ControllerType == PSMController_DualShock4)
				{
					ImGui::PushItemWidth(195);
					if (ImGui::Combo("Position Filter", &controllerInfo.PositionFilterIndex, k_position_filter_names, UI_ARRAYSIZE(k_position_filter_names)))
					{
						controllerInfo.PositionFilterName = k_position_filter_names[controllerInfo.PositionFilterIndex];
						request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
					}
					if (ImGui::Combo("Orientation Filter", &controllerInfo.OrientationFilterIndex, k_ds4_orientation_filter_names, UI_ARRAYSIZE(k_ds4_orientation_filter_names)))
					{
						controllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[controllerInfo.OrientationFilterIndex];
						request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
					}
					if (ImGui::Combo("Gyro Gain", &controllerInfo.GyroGainIndex, k_ds4_gyro_gain_setting_labels, UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels)))
					{
						controllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[controllerInfo.GyroGainIndex];
						request_set_gyroscope_gain_setting(controllerInfo.ControllerID, controllerInfo.GyroGainSetting);
					}
					if (ImGui::SliderFloat("Prediction Time", &controllerInfo.PredictionTime, 0.f, k_max_prediction_time))
					{
						request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
					}
					if (ImGui::Button("Reset Filter Defaults"))
					{
						controllerInfo.PositionFilterIndex = k_default_ds4_position_filter_index;
						controllerInfo.OrientationFilterIndex = k_default_ds4_orientation_filter_index;
						controllerInfo.GyroGainIndex = k_default_ds4_gyro_gain_index;
						controllerInfo.PositionFilterName = k_position_filter_names[k_default_ds4_position_filter_index];
						controllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[k_default_ds4_orientation_filter_index];
						controllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[k_default_ds4_gyro_gain_index];
						request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
						request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
						request_set_gyroscope_gain_setting(controllerInfo.ControllerID, controllerInfo.GyroGainSetting);
					}
					ImGui::PopItemWidth();
				}

				if (controllerInfo.ControllerType == PSMController_Navi && 
					controllerInfo.PotentialParentControllerSerials.size() > 0)
				{
					ImGui::PushItemWidth(195);					
					if (ImGui::Combo(
							"Parent Controller", 
							&controllerInfo.AssignedParentControllerIndex, 
							ControllerInfo::ParentControllerComboItemGetter, 
							&controllerInfo, 
							static_cast<int>(controllerInfo.PotentialParentControllerSerials.size())))
					{
						std::string parentControllerSerial= controllerInfo.PotentialParentControllerSerials[controllerInfo.AssignedParentControllerIndex];

						controllerInfo.AssignedParentControllerSerial = parentControllerSerial;

						request_set_parent_controller_id(controllerInfo.ControllerID, find_controller_id_by_serial(parentControllerSerial));
					}
					ImGui::PopItemWidth();
				}
            }
            else
            {
                ImGui::Text("No connected usable controllers");
            }

            ImGui::Separator();

            // If there are any controllers waiting to be paired, 
            // just present the first one as an option
            if (m_awaitingPairingControllerInfos.size() > 0)
            {
                // Only consider the first controller connected via usb
                ControllerInfo &controllerInfo = m_awaitingPairingControllerInfos[0];

                // We can only unpair controllers connected via usb
                if (controllerInfo.PairedToHost)
                {
                    if (ImGui::Button("Unpair USB Controller"))
                    {
                        m_app->getAppStage<AppStage_PairController>()->request_controller_unpair(controllerInfo.ControllerID, controllerInfo.ControllerType);
                        m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
                    }
                }
                else
                {
                    if (ImGui::Button("Pair USB Controller"))
                    {
                        m_app->getAppStage<AppStage_PairController>()->request_controller_pair(controllerInfo.ControllerID, controllerInfo.ControllerType);
                        m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
                    }
                }
            }
            else
            {
                ImGui::Text("No controllers awaiting pairing");
            }

            ImGui::Separator();

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(300, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller list response...");

            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(300, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to get controller list!");

            if (ImGui::Button("Retry"))
            {
                request_controller_list();
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_ControllerSettings::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled= false;

    switch(event)
    {
    case PSMEventMessage::PSMEvent_controllerListUpdated:
        {
            bHandled= true;
            request_controller_list();
        } break;
    }

    return bHandled;
}

void AppStage_ControllerSettings::request_controller_list()
{
    if (m_menuState != AppStage_ControllerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_ControllerSettings::pendingControllerListRequest;

        // Tell the psmove service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Do get controllers connected bia USB in this menu since we need the info for pairing/unpairing
        request->mutable_request_get_controller_list()->set_include_usb_controllers(true);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_ControllerSettings::handle_controller_list_response, this);
    }
}

void AppStage_ControllerSettings::request_set_orientation_filter(
	const int controller_id,
	const std::string &filter_name)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_ORIENTATION_FILTER);

	request->mutable_request_set_orientation_filter()->set_controller_id(controller_id);
	request->mutable_request_set_orientation_filter()->set_orientation_filter(filter_name);

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ControllerSettings::request_set_position_filter(
	const int controller_id,
	const std::string &filter_name)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_POSITION_FILTER);

	request->mutable_request_set_position_filter()->set_controller_id(controller_id);
	request->mutable_request_set_position_filter()->set_position_filter(filter_name);

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ControllerSettings::request_set_gyroscope_gain_setting(
	const int controller_id,
	const std::string& gain_setting)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_GYROSCOPE_CALIBRATION);

	PSMoveProtocol::Request_RequestSetControllerGyroscopeCalibration *calibration =
		request->mutable_set_controller_gyroscope_calibration_request();

	calibration->set_controller_id(controller_id);
	calibration->set_drift(-1.f); // keep existing drift
	calibration->set_variance(-1.f); // keep existing variance
	calibration->set_gyro_gain_setting(gain_setting);

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ControllerSettings::request_set_controller_prediction(
	const int controller_id,
	const float prediction_time)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_PREDICTION_TIME);

	PSMoveProtocol::Request_RequestSetControllerPredictionTime *calibration =
		request->mutable_request_set_controller_prediction_time();

	calibration->set_controller_id(controller_id);
	calibration->set_prediction_time(prediction_time); // keep existing drift

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ControllerSettings::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
    const PSMResponseHandle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
			int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;

            thisPtr->m_hostSerial = response->result_controller_list().host_serial();
			thisPtr->m_selectedControllerIndex= -1;
			thisPtr->m_usableControllerInfos.clear();
			thisPtr->m_awaitingPairingControllerInfos.clear();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_ControllerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType = PSMController_Move;
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType = PSMController_Navi;
                    break;
                case PSMoveProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = PSMController_DualShock4;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                ControllerInfo.TrackingColorType = 
                    static_cast<PSMTrackingColorType>(ControllerResponse.tracking_color_type());
                ControllerInfo.DevicePath= ControllerResponse.device_path();
                ControllerInfo.DeviceSerial= ControllerResponse.device_serial();
                ControllerInfo.AssignedHostSerial= ControllerResponse.assigned_host_serial();
				ControllerInfo.AssignedParentControllerSerial= ControllerResponse.parent_controller_serial();
				ControllerInfo.AssignedParentControllerIndex= -1;
				ControllerInfo.PotentialParentControllerSerials.clear();
                ControllerInfo.PairedToHost=
                    ControllerResponse.assigned_host_serial().length() > 0 && 
                    ControllerResponse.assigned_host_serial() == thisPtr->m_hostSerial;
				ControllerInfo.FirmwareVersion = ControllerResponse.firmware_version();
				ControllerInfo.FirmwareRevision = ControllerResponse.firmware_revision();
				ControllerInfo.HasMagnetometer = ControllerResponse.has_magnetometer();
				ControllerInfo.OrientationFilterName= ControllerResponse.orientation_filter();
				ControllerInfo.PositionFilterName = ControllerResponse.position_filter();
				ControllerInfo.GyroGainSetting = ControllerResponse.gyro_gain_setting();
				ControllerInfo.PredictionTime = ControllerResponse.prediction_time();

				if (ControllerInfo.ControllerType == PSMController_Move)
				{
					ControllerInfo.OrientationFilterIndex =
						find_string_entry(
							ControllerInfo.OrientationFilterName.c_str(),
							k_psmove_orientation_filter_names,
							UI_ARRAYSIZE(k_psmove_orientation_filter_names));
					if (ControllerInfo.OrientationFilterIndex == -1)
					{
						ControllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[0];
						ControllerInfo.OrientationFilterIndex = 0;
					}
				}
				else if (ControllerInfo.ControllerType == PSMController_DualShock4)
				{
					ControllerInfo.OrientationFilterIndex =
						find_string_entry(
							ControllerInfo.OrientationFilterName.c_str(),
							k_ds4_orientation_filter_names,
							UI_ARRAYSIZE(k_ds4_orientation_filter_names));
					if (ControllerInfo.OrientationFilterIndex == -1)
					{
						ControllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[0];
						ControllerInfo.OrientationFilterIndex = 0;
					}
				}
				else
				{
					ControllerInfo.OrientationFilterName = "";
					ControllerInfo.OrientationFilterIndex = -1;
				}

				if (ControllerInfo.ControllerType == PSMController_Move ||
					ControllerInfo.ControllerType == PSMController_DualShock4)
				{
					ControllerInfo.PositionFilterIndex =
						find_string_entry(
							ControllerInfo.PositionFilterName.c_str(),
							k_position_filter_names,
							UI_ARRAYSIZE(k_position_filter_names));
					if (ControllerInfo.PositionFilterIndex == -1)
					{
						ControllerInfo.PositionFilterName = k_position_filter_names[0];
						ControllerInfo.PositionFilterIndex = 0;
					}
				}
				else
				{
					ControllerInfo.PositionFilterName = "";
					ControllerInfo.PositionFilterIndex = -1;
				}

				if (ControllerInfo.ControllerType == PSMController_DualShock4)
				{
					ControllerInfo.GyroGainIndex =
						find_string_entry(
							ControllerInfo.GyroGainSetting.c_str(),
							k_ds4_gyro_gain_setting_labels,
							UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels));
					if (ControllerInfo.GyroGainIndex == -1)
					{
						ControllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[0];
						ControllerInfo.GyroGainIndex = 0;
					}
				}
				else
				{
					ControllerInfo.GyroGainSetting = "";
					ControllerInfo.GyroGainIndex = -1;
				}

                // Add the controller to the appropriate connection list
				
				if (ControllerResponse.connection_type() == PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_BLUETOOTH ||
					ControllerResponse.controller_type() == PSMoveProtocol::PSNAVI) // Don't currently attempt to pair navi controller
				{
					thisPtr->m_usableControllerInfos.push_back(ControllerInfo);
				}
				else
				{
					thisPtr->m_awaitingPairingControllerInfos.push_back(ControllerInfo);
				}
            }

			// Build a list of potential parent controllers for each navi controller
			// and assign the index current parent controller serial
			for (ControllerInfo &navi_info : thisPtr->m_usableControllerInfos)
			{
				if (navi_info.ControllerType != PSMController_Navi)
					continue;

				for (ControllerInfo &psmove_info : thisPtr->m_usableControllerInfos)
				{
					if (psmove_info.ControllerType != PSMController_Move)
						continue;

					if (navi_info.AssignedParentControllerSerial.length() > 0 &&
						navi_info.AssignedParentControllerSerial == psmove_info.DeviceSerial)
					{
						navi_info.AssignedParentControllerIndex= static_cast<int>(navi_info.PotentialParentControllerSerials.size());
					}

					navi_info.PotentialParentControllerSerials.push_back(psmove_info.DeviceSerial);
				}
			}

			if (oldSelectedControllerIndex != -1)
			{
				// Maintain the same position in the list if possible
				thisPtr->m_selectedControllerIndex= 
					(oldSelectedControllerIndex < static_cast<int>(thisPtr->m_usableControllerInfos.size())) 
					? oldSelectedControllerIndex
					: 0;
			}
			else
			{
	            thisPtr->m_selectedControllerIndex= (thisPtr->m_usableControllerInfos.size() > 0) ? 0 : -1;
			}

            thisPtr->m_menuState= AppStage_ControllerSettings::idle;
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
		case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_ControllerSettings::failedControllerListRequest;
        } break;
    }
}

int AppStage_ControllerSettings::find_controller_id_by_serial(std::string controller_serial) const
{
	int ControllerID= -1;

	for (const ControllerInfo &info : m_usableControllerInfos)
	{
		if (info.DeviceSerial == controller_serial)
		{
			ControllerID= info.ControllerID;
			break;
		}
	}

	return ControllerID;
}

void AppStage_ControllerSettings::request_set_controller_tracking_color_id(
	int ControllerID,
	PSMTrackingColorType tracking_color_type)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_LED_TRACKING_COLOR);
	request->mutable_set_led_tracking_color_request()->set_controller_id(ControllerID);
	request->mutable_set_led_tracking_color_request()->set_color_type(
		static_cast<PSMoveProtocol::TrackingColorType>(tracking_color_type));

	PSM_SendOpaqueRequest(&request, nullptr);
}

void AppStage_ControllerSettings::request_set_parent_controller_id(
	int ControllerID,
	int ParentControllerID)
{
	if (ControllerID != -1 && ParentControllerID != -1)
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_ATTACHED_CONTROLLER);
		request->mutable_request_set_attached_controller()->set_child_controller_id(ControllerID);
		request->mutable_request_set_attached_controller()->set_parent_controller_id(ParentControllerID);

		PSM_SendOpaqueRequest(&request, nullptr);
	}
}