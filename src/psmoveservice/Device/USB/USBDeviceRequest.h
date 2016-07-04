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

enum eUSBTransferResultType
{
    _USBResultType_ControlTransfer,
    _USBResultType_BulkTransfer
};

enum eUSBResultCode
{
    // Success Codes
    _USBResultCode_Started,
    _USBResultCode_Canceled,
    _USBResultCode_Completed,

    // Failure Codes
    _USBResultCode_GeneralError,
    _USBResultCode_BadHandle,
    _USBResultCode_NoMemory,
    _USBResultCode_SubmitFailed,
    _USBResultCode_DeviceNotOpen,
    _USBResultCode_TransferNotActive,
    _USBResultCode_TransferAlreadyStarted,
    _USBResultCode_Overflow,
    _USBResultCode_Pipe,
    _USBResultCode_TimedOut
};

#define MAX_CONTROL_TRANSFER_PAYLOAD 32

//-- typedefs -----
typedef void(*usb_bulk_transfer_cb_fn)(unsigned char *packet_data, int packet_length, void *userdata);

//-- definitions -----

//-- Request Structures --
struct USBRequestPayload_ControlTransfer
{
    t_usb_device_handle usb_device_handle;
    unsigned int timeout;
    unsigned short wValue;
    unsigned short wIndex;
    unsigned short wLength;
    unsigned char data[MAX_CONTROL_TRANSFER_PAYLOAD];
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

//-- Result Structures --
struct USBResultPayload_BulkTransfer
{
    t_usb_device_handle usb_device_handle;
    eUSBResultCode result_code;
};

struct USBResultPayload_ControlTransfer
{
    t_usb_device_handle usb_device_handle;
    eUSBResultCode result_code;
    unsigned char data[MAX_CONTROL_TRANSFER_PAYLOAD];
    int dataLength;
};

struct USBTransferResult
{
    union 
    {
        USBResultPayload_ControlTransfer control_transfer;
        USBResultPayload_BulkTransfer bulk_transfer;
    } payload;
    eUSBTransferResultType result_type;
};

#endif // USB_DEVICE_REQUEST_H