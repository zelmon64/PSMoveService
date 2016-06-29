#ifndef USB_DEVICE_MANAGER_H
#define USB_DEVICE_MANAGER_H

//-- includes -----
#include "USBDeviceInfo.h"

//-- definitions -----
/// Manages async control and bulk transfer requests to usb devices via libusb.
class USBDeviceManager
{
public:
    USBDeviceManager(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length);
    virtual ~USBDeviceManager();

    static inline USBDeviceManager *getInstance()
    {
        return m_instance;
    }

    // -- System ----
    bool startup(); /**< Initialize the libusb thread. */
    void update();  /**< Process events from the libusb thread. */
    void shutdown();/**< Shutdown the libusb thread. */

    // -- Device Actions ----
    bool openUSBDevice(t_usb_device_handle handle);
    void closeUSBDevice(t_usb_device_handle handle);

    // -- Device Queries ----
    int getUSBDeviceCount() const;
    t_usb_device_handle getFirstUSBDeviceHandle() const;
    t_usb_device_handle getNextUSBDeviceHandle(t_usb_device_handle handle) const;
    bool getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceInfo &outDeviceInfo) const;
    bool getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const;
    bool getIsUSBDeviceOpen(t_usb_device_handle handle) const;

    // -- Request Queue ----
    enum eUSBTransferRequestType
    {
        _USBRequestType_ControlTransfer,
        _USBRequestType_StartBulkTransfer,
        _USBRequestType_StopBulkTransfer,
    };

    struct RequestPayload_ControlTransfer
    {
        t_usb_device_handle usb_device_handle;
        unsigned int timeout;
        unsigned short wValue;
        unsigned short wIndex;
        unsigned short wLength;
        unsigned char data[1];
        unsigned char bmRequestType;
        unsigned char bRequest;
    };

    typedef void(*bulk_transfer_cb_fn)(unsigned char *packet_data, int packet_length, void *userdata);
    struct RequestPayload_StartBulkTransfer
    {
        t_usb_device_handle usb_device_handle;
        int transfer_packet_size;
        int in_flight_transfer_packet_count;
        bulk_transfer_cb_fn on_data_callback;
        void *transfer_callback_userdata;
        bool bAutoResubmit;
    };

    struct RequestPayload_StopBulkTransfer
    {
        t_usb_device_handle usb_device_handle;
    };

    struct USBTransferRequest
    {
        union
        {
            RequestPayload_ControlTransfer control_transfer;
            RequestPayload_StartBulkTransfer start_bulk_transfer;
            RequestPayload_StopBulkTransfer stop_bulk_transfer;
        } payload;
        eUSBTransferRequestType request_type;
    };

    // Send the transfer request to the worker thread
    bool submitTransferRequest(const USBTransferRequest &request);

private:
    // Always use the overloaded constructor
    USBDeviceManager();

    /// private implementation
    class USBDeviceManagerImpl *implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static USBDeviceManager *m_instance;
};

#endif  // USB_DEVICE_MANAGER_H