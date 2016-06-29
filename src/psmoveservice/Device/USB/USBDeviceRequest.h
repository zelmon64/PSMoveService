#ifndef USB_DEVICE_REQUEST_H
#define USB_DEVICE_REQUEST_H

//-- includes -----
#include "USBDeviceInfo.h"

//-- constants -----
enum eUSBTransferRequestType
{
    _USBRequestType_ControlTransfer,
    _USBRequestType_StartBulkTransfer,
    _USBRequestType_CancelBulkTransfer,
};

//-- typedefs -----
typedef void(*usb_bulk_transfer_cb_fn)(unsigned char *packet_data, int packet_length, void *userdata);

//-- definitions -----
struct USBRequestPayload_ControlTransfer
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

struct USBRequestPayload_BulkTransfer
{
    t_usb_device_handle usb_device_handle;
    int transfer_packet_size;
    int in_flight_transfer_packet_count;
    usb_bulk_transfer_cb_fn on_data_callback;
    void *transfer_callback_userdata;
    bool bAutoResubmit;
};

struct USBRequestPayload_CancelBulkTransfer
{
    t_usb_device_handle usb_device_handle;
};

struct USBTransferRequest
{
    union
    {
        USBRequestPayload_ControlTransfer control_transfer;
        USBRequestPayload_BulkTransfer start_bulk_transfer;
        USBRequestPayload_CancelBulkTransfer cancel_bulk_transfer;
    } payload;
    eUSBTransferRequestType request_type;
};

#endif // USB_DEVICE_REQUEST_H