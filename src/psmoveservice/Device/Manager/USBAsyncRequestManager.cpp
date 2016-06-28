//-- includes -----
#include "USBAsyncRequestManager.h"
#include "USBDeviceInfo.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "libusb.h"

#include <atomic>
#include <thread>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

//-- pre-declarations -----

//-- private implementation -----

// -USBAsyncRequestManagerImpl-
/// Internal implementation of the USB async request manager.
class USBAsyncRequestManagerImpl
{
public:
    USBAsyncRequestManagerImpl(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length)
        : m_usb_context(nullptr)
        , m_exit_signaled({ false })
        , m_active_bulk_transfers(0)
        , m_active_control_transfers(0)
        , m_thread_started(false)
    {
        for (size_t list_index = 0; list_index < device_whitelist_length; ++list_index)
        {
            m_device_whitelist.push_back(device_whitelist[list_index]);
        }
    }

    virtual ~USBAsyncRequestManagerImpl()
    {
    }

    bool startup()
    {
        bool bSuccess= true;

        SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Initializing libusb context";
        libusb_init(&m_usb_context);
        libusb_set_debug(m_usb_context, 1);

        // Get a list of all of the available USB devices that are on the white-list
        rebuildFilteredDeviceList();

        // Start the worker thread to process async requests
        startWorkerThread();

        return bSuccess;
    }

    void update()
    {
    }

    void shutdown()
    {
        // Shutdown any async transfers
        stopWorkerThread();

        if (m_usb_context != nullptr)
        {
            // Unref any libusb devices
            freeFilteredDeviceList();

            // Free the libusb context
            libusb_exit(m_usb_context);
            m_usb_context= nullptr;
        }
    }

    // -- Device Queries ----
    int getUSBDeviceCount() const
    {
        return static_cast<int>(m_filtered_device_list.size());
    }

    t_usb_device_handle getFirstUSBDeviceHandle() const
    {
        return (m_filtered_device_list.size() > 0) ? static_cast<t_usb_device_handle>(0) : k_invalid_usb_device_handle;
    }

    t_usb_device_handle getNextUSBDeviceHandle(t_usb_device_handle handle) const
    {
        int device_index= static_cast<int>(handle);

        return (device_index + 1 < getUSBDeviceCount()) ? static_cast<t_usb_device_handle>(device_index + 1) : k_invalid_usb_device_handle;
    }

    bool getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceInfo &outDeviceInfo) const
    {
        bool bSuccess= false;
        int device_index = static_cast<int>(handle);

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            libusb_device *dev = m_filtered_device_list[device_index];

            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(dev, &dev_desc);

            outDeviceInfo.product_id= dev_desc.idProduct;
            outDeviceInfo.vendor_id= dev_desc.idVendor;
            bSuccess= true;
        }

        return bSuccess;
    }

    bool getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
    {
        bool bSuccess = false;
        int device_index = static_cast<int>(handle);

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            libusb_device *dev = m_filtered_device_list[device_index];

            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(dev, &dev_desc);

            //###HipsterSloth $TODO Put bus/port numbers here
            int nCharsWritten= 
                ServerUtility::format_string(
                    outBuffer, bufferSize,
                    "USB\\VID_%04X&PID_%04X\\%d",
                    dev_desc.idVendor, dev_desc.idProduct, device_index);

            bSuccess = (nCharsWritten > 0);
        }

        return bSuccess;
    }

    bool submitTransferRequest(const USBAsyncRequestManager::USBTransferRequest &request)
    {
        return request_queue.push(request);
    }

protected:
    void startWorkerThread()
    {
        if (!m_thread_started)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Starting USB event thread";
            m_worker_thread = std::thread(&USBAsyncRequestManagerImpl::workerThreadFunc, this);
            m_thread_started = true;
        }
    }

    void workerThreadFunc()
    {
        ServerUtility::set_current_thread_name("USB Async Worker Thread");

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000; // ms

        while (!m_exit_signaled)
        {
            // Process incoming USB transfer requests
            USBAsyncRequestManager::USBTransferRequest request;
            while (request_queue.pop(request))
            {
                switch (request.request_type)
                {
                case USBAsyncRequestManager::_USBRequestType_ControlTransfer:
                    handleControlTransferRequest(request.payload.control_transfer);
                    break;
                case USBAsyncRequestManager::_USBRequestType_StartBulkTransfer:
                    handleStartBulkTransferRequest(request.payload.start_bulk_transfer);
                    break;
                case USBAsyncRequestManager::_USBRequestType_StopBulkTransfer:
                    handleStopBulkTransferRequest(request.payload.stop_bulk_transfer);
                    break;
                }
            }

            if (m_active_bulk_transfers > 0 || m_active_control_transfers > 0)
            {
                libusb_handle_events_timeout_completed(m_usb_context, &tv, NULL);
            }
            else
            {
                ServerUtility::sleep_ms(100);
            }
        }
    }

    void handleControlTransferRequest(USBAsyncRequestManager::RequestPayload_ControlTransfer &control_transfer)
    {
        //TODO
    }

    void handleStartBulkTransferRequest(USBAsyncRequestManager::RequestPayload_StartBulkTransfer &start_bulk_transfer)
    {
        //TODO
    }

    void handleStopBulkTransferRequest(USBAsyncRequestManager::RequestPayload_StopBulkTransfer &stop_bulk_transfer)
    {
        //TODO
    }

    void stopWorkerThread()
    {
        if (m_thread_started)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Stopping USB event thread...";
            m_exit_signaled = true;
            m_worker_thread.join();

            SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "USB event thread stopped";
            m_thread_started = false;
            m_exit_signaled = false;
        }
    }

    void rebuildFilteredDeviceList()
    {
        libusb_device **device_list;
        if (libusb_get_device_list(m_usb_context, &device_list) < 0)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::rebuildFilteredDeviceList") << "Unable to fetch device list.";
        }

        unsigned char dev_port_numbers[MAX_USB_DEVICE_PORT_PATH] = { 0 };
        for (int i= 0; device_list[i] != NULL; ++i)
        {
            libusb_device *dev= device_list[i];

            if (isDeviceInWhitelist(dev))
            {
                uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];
                memset(port_numbers, 0, sizeof(port_numbers));
                int elements_filled = libusb_get_port_numbers(dev, port_numbers, MAX_USB_DEVICE_PORT_PATH);

                if (elements_filled > 0)
                {
                    // Make sure this device is actually different from the last device we looked at
                    // (i.e. has a different device port path)
                    if (memcmp(port_numbers, dev_port_numbers, sizeof(port_numbers)) != 0)
                    {
                        libusb_device_handle *devhandle;
                        int libusb_result = libusb_open(dev, &devhandle);

                        if (libusb_result == LIBUSB_SUCCESS || libusb_result == LIBUSB_ERROR_ACCESS)
                        {
                            if (libusb_result == LIBUSB_SUCCESS)
                            {
                                libusb_close(devhandle);
                                m_filtered_device_list.push_back(dev);
                                libusb_ref_device(dev);
                            }

                            // Cache the port number for the last valid device found
                            memcpy(dev_port_numbers, port_numbers, sizeof(port_numbers));
                        }
                    }
                }
            }
        }

        libusb_free_device_list(device_list, 1);
    }

    void freeFilteredDeviceList()
    {
        std::for_each(
            m_filtered_device_list.begin(), 
            m_filtered_device_list.end(), [](libusb_device *dev) {
                libusb_unref_device(dev);
            });
        m_filtered_device_list.clear();
    }

    bool isDeviceInWhitelist(libusb_device *dev)
    {
        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(dev, &desc);

        auto iter= std::find_if(
            m_device_whitelist.begin(), 
            m_device_whitelist.end(), 
            [&desc](const USBDeviceInfo &entry)->bool 
            {
                return desc.idVendor == entry.vendor_id && desc.idProduct == entry.product_id;
            });
        
        return iter != m_device_whitelist.end();
    }

private:
    // Multithreaded state
    libusb_context* m_usb_context;
    std::atomic_bool m_exit_signaled;
    boost::lockfree::spsc_queue<USBAsyncRequestManager::USBTransferRequest, boost::lockfree::capacity<128> > request_queue;

    // Worker thread state
    int m_active_bulk_transfers;
    int m_active_control_transfers;

    // Main thread state
    bool m_thread_started;
    std::thread m_worker_thread;
    std::vector<USBDeviceInfo> m_device_whitelist;
    std::vector<libusb_device *> m_filtered_device_list;
};

//-- public interface -----
USBAsyncRequestManager *USBAsyncRequestManager::m_instance = NULL;

USBAsyncRequestManager::USBAsyncRequestManager(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length)
    : implementation_ptr(new USBAsyncRequestManagerImpl(device_whitelist, device_whitelist_length))
{
}

USBAsyncRequestManager::~USBAsyncRequestManager()
{
    if (m_instance != NULL)
    {
        SERVER_LOG_ERROR("~USBAsyncRequestManager()") << "USB Async Request Manager deleted without shutdown() getting called first";
    }

    if (implementation_ptr != nullptr)
    {
        delete implementation_ptr;
        implementation_ptr = nullptr;
    }
}

bool USBAsyncRequestManager::startup()
{
    m_instance = this;
    return implementation_ptr->startup();
}

void USBAsyncRequestManager::update()
{
    implementation_ptr->update();
}

void USBAsyncRequestManager::shutdown()
{
    implementation_ptr->shutdown();
    m_instance = NULL;
}

int USBAsyncRequestManager::getUSBDeviceCount() const
{
    return implementation_ptr->getUSBDeviceCount();
}

t_usb_device_handle USBAsyncRequestManager::getFirstUSBDeviceHandle() const
{
    return implementation_ptr->getFirstUSBDeviceHandle();
}

t_usb_device_handle USBAsyncRequestManager::getNextUSBDeviceHandle(t_usb_device_handle handle) const
{
    return implementation_ptr->getNextUSBDeviceHandle(handle);
}

bool USBAsyncRequestManager::getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceInfo &outDeviceInfo) const
{
    return implementation_ptr->getUSBDeviceInfo(handle, outDeviceInfo);
}

bool USBAsyncRequestManager::getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
{
    return implementation_ptr->getUSBDevicePath(handle, outBuffer, bufferSize);
}

bool USBAsyncRequestManager::submitTransferRequest(const USBTransferRequest &request)
{
    return implementation_ptr->submitTransferRequest(request);
}