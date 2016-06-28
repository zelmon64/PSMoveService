//-- includes -----
#include "USBAsyncRequestManager.h"
#include "USBDeviceInfo.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "libusb.h"

#include <atomic>
#include <thread>
#include <vector>

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

    int getFilteredDeviceCount() const
    {
        return static_cast<int>(m_filtered_device_list.size());
    }

    bool getFilteredDeviceInfo(int filteredDeviceIndex, USBDeviceInfo &outDeviceInfo) const
    {
        bool bSuccess= false;

        if (filteredDeviceIndex >= 0 && filteredDeviceIndex < getFilteredDeviceCount())
        {
            libusb_device *dev= m_filtered_device_list[filteredDeviceIndex];

            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(dev, &dev_desc);

            outDeviceInfo.product_id= dev_desc.idProduct;
            outDeviceInfo.vendor_id= dev_desc.idVendor;
            bSuccess= true;
        }

        return bSuccess;
    }

    bool getFilteredDevicePath(int filteredDeviceIndex, char *outBuffer, size_t bufferSize) const
    {
        bool bSuccess = false;

        if (filteredDeviceIndex >= 0 && filteredDeviceIndex < getFilteredDeviceCount())
        {
            libusb_device *dev = m_filtered_device_list[filteredDeviceIndex];

            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(dev, &dev_desc);

            //###HipsterSloth $TODO Put bus/port numbers here
            int nCharsWritten= 
                ServerUtility::format_string(
                    outBuffer, bufferSize,
                    "USB\\VID_%04X&PID_%04X\\%d",
                    dev_desc.idVendor, dev_desc.idProduct, filteredDeviceIndex);

            bSuccess = (nCharsWritten > 0);
        }

        return bSuccess;
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

int USBAsyncRequestManager::getFilteredDeviceCount() const
{
    return implementation_ptr->getFilteredDeviceCount();
}

bool USBAsyncRequestManager::getFilteredDeviceInfo(int filteredDeviceIndex, USBDeviceInfo &outDeviceInfo) const
{
    return implementation_ptr->getFilteredDeviceInfo(filteredDeviceIndex, outDeviceInfo);
}

bool USBAsyncRequestManager::getFilteredDevicePath(int filteredDeviceIndex, char *outBuffer, size_t bufferSize) const
{
    return implementation_ptr->getFilteredDevicePath(filteredDeviceIndex, outBuffer, bufferSize);
}