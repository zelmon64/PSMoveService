#ifndef USB_ASYNC_REQUEST_MANAGER_H
#define USB_ASYNC_REQUEST_MANAGER_H

//-- includes -----
#include "USBDeviceInfo.h"

//-- definitions -----
/// Manages async control and bulk transfer requests to usb devices via libusb.
class USBAsyncRequestManager
{
public:
    USBAsyncRequestManager(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length);
    virtual ~USBAsyncRequestManager();

    bool startup(); /**< Initialize the libusb thread. */
    void update();  /**< Process events from the libusb thread. */
    void shutdown();/**< Shutdown the libusb thread. */

    static inline USBAsyncRequestManager *getInstance()
    {
        return m_instance;
    }

    int getFilteredDeviceCount() const;
    bool getFilteredDeviceInfo(int filteredDeviceIndex, USBDeviceInfo &outDeviceInfo) const;
    bool getFilteredDevicePath(int filteredDeviceIndex, char *outBuffer, size_t bufferSize) const;

private:
    // Always use the overloaded constructor
    USBAsyncRequestManager();

    /// private implementation
    class USBAsyncRequestManagerImpl *implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static USBAsyncRequestManager *m_instance;
};

#endif  // USB_ASYNC_REQUEST_MANAGER_H