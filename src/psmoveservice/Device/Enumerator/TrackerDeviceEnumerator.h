#ifndef TRACKER_DEVICE_ENUMERATOR_H
#define TRACKER_DEVICE_ENUMERATOR_H

//-- includes -----
#include "DeviceEnumerator.h"
#include "USBDeviceInfo.h" // for MAX_USB_DEVICE_PORT_PATH

//-- definitions -----
class TrackerDeviceEnumerator : public DeviceEnumerator
{
public:
    TrackerDeviceEnumerator();
    TrackerDeviceEnumerator(CommonDeviceState::eDeviceType deviceType);

    bool is_valid() const override;
    bool next() override;
    const char *get_path() const override;
    inline int get_camera_index() const { return camera_index; }

private:
    char cur_path[256];
    int dev_index, dev_count;
    int camera_index;
    bool dev_valid;
};

#endif // TRACKER_DEVICE_ENUMERATOR_H