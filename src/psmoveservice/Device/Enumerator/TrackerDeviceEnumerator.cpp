// -- includes -----
#include "TrackerDeviceEnumerator.h"
#include "ServerUtility.h"
#include "USBAsyncRequestManager.h"
#include "assert.h"
#include "libusb.h"
#include "string.h"

// -- private definitions -----
#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

// -- macros ----
#define MAX_CAMERA_TYPE_INDEX               GET_DEVICE_TYPE_INDEX(CommonDeviceState::SUPPORTED_CAMERA_TYPE_COUNT)

// -- globals -----
// NOTE: This list must match the tracker order in CommonDeviceState::eDeviceType
USBDeviceInfo k_supported_tracker_infos[MAX_CAMERA_TYPE_INDEX] = {
    { 0x1415, 0x2000 }, // PS3Eye
    //{ 0x05a9, 0x058a }, // PS4 Camera - TODO
    //{ 0x045e, 0x02ae }, // V1 Kinect - TODO
};

// -- methods -----
TrackerDeviceEnumerator::TrackerDeviceEnumerator()
    : DeviceEnumerator(CommonDeviceState::PS3EYE)
    , dev_index(0)
    , dev_count(0)
    , camera_index(-1)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);

    dev_count= USBAsyncRequestManager::getInstance()->getFilteredDeviceCount();    
    dev_index= -1;
    camera_index = -1;
    next();
}

TrackerDeviceEnumerator::TrackerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType)
    : DeviceEnumerator(deviceType)
    , dev_index(0)
    , dev_count(0)
    , camera_index(-1)
    , dev_valid(false)
{
    assert(m_deviceType >= 0 && GET_DEVICE_TYPE_INDEX(m_deviceType) < MAX_CAMERA_TYPE_INDEX);

    dev_count = USBAsyncRequestManager::getInstance()->getFilteredDeviceCount();
    dev_index = -1;
    camera_index = -1;
    next();
}

const char *TrackerDeviceEnumerator::get_path() const
{
    const char *result = nullptr;

    if (is_valid())
    {
        // Return a pointer to our member variable that has the path cached
        result= cur_path;
    }

    return result;
}

bool TrackerDeviceEnumerator::is_valid() const
{
    return dev_index < dev_count;
}

bool TrackerDeviceEnumerator::next()
{
    bool foundValid = false;

    while (dev_index < dev_count && !foundValid)
    {
        USBDeviceInfo devInfo;

        ++dev_index;
        if (USBAsyncRequestManager::getInstance()->getFilteredDeviceInfo(dev_index, devInfo))
        {
            // See if the next filtered device is a camera that we care about
            for (int tracker_type_index = 0; tracker_type_index < MAX_CAMERA_TYPE_INDEX; ++tracker_type_index)
            {
                const USBDeviceInfo &supported_type= k_supported_tracker_infos[tracker_type_index];

                if (devInfo.product_id == supported_type.product_id &&
                    devInfo.vendor_id == supported_type.vendor_id)
                {
                    // Cache the path to the device
                    USBAsyncRequestManager::getInstance()->getFilteredDevicePath(dev_index, cur_path, sizeof(cur_path));

                    m_deviceType = static_cast<CommonDeviceState::eDeviceType>(CommonDeviceState::TrackingCamera + tracker_type_index);
                    foundValid= true;
                    break;
                }
            }
        }
    }

    if (foundValid)
    {
        ++camera_index;
    }

    return foundValid;
}