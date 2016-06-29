//-- includes -----
#include "USBDeviceManager.h"
#include "USBDeviceInfo.h"
#include "ServerLog.h"
#include "ServerUtility.h"

#include "libusb.h"

#include <atomic>
#include <thread>
#include <vector>

#include <boost/lockfree/spsc_queue.hpp>

//-- private implementation -----
// -USBBulkTransferBundle-
/// Internal class used to manage a set of libusb bulk transfer packets.
class USBBulkTransferBundle
{
public:
    USBBulkTransferBundle(
        USBDeviceManager::RequestPayload_StartBulkTransfer &request,
        libusb_device *dev,
        libusb_device_handle *dev_handle)
        : m_request(request)
        , m_device(dev)
        , m_device_handle(dev_handle)
        , m_active_transfer_count(0)
        , m_is_canceled(false)
        , bulk_transfer_requests(nullptr)
        , transfer_buffer(nullptr)
    {
    }

    virtual ~USBBulkTransferBundle()
    {
        dispose();

        if (m_active_transfer_count > 0)
        {
            SERVER_MT_LOG_INFO("USBBulkTransferBundle::destructor") << "active transfer count non-zero!";
        }
    }

    inline t_usb_device_handle getUSBDeviceHandle() const
    {
        return m_request.usb_device_handle;
    }

    inline int getActiveTransferCount() const
    {
        return m_active_transfer_count;
    }

    bool initialize()
    {
        bool bSuccess = (m_active_transfer_count == 0);
        uint8_t bulk_endpoint = 0;

        // Find the bulk transfer endpoint          
        if (bSuccess)
        {
            if (find_bulk_transfer_endpoint(m_device, bulk_endpoint))
            {
                libusb_clear_halt(m_device_handle, bulk_endpoint);
            }
            else
            {
                bSuccess = false;
            }
        }

        // Allocate the libusb transfer request array
        if (bSuccess)
        {
            size_t xfer_array_byte_size = m_request.in_flight_transfer_packet_count * sizeof(libusb_transfer *);
            bulk_transfer_requests = (libusb_transfer **)malloc(xfer_array_byte_size);

            if (bulk_transfer_requests != nullptr)
            {
                memset(bulk_transfer_requests, 0, xfer_array_byte_size);
            }
            else
            {
                bSuccess = false;
            }
        }

        // Allocate the transfer buffer
        if (bSuccess)
        {
            // Allocate the transfer buffer that the requests write data into
            size_t xfer_buffer_size = m_request.in_flight_transfer_packet_count * m_request.transfer_packet_size;
            transfer_buffer = (uint8_t *)malloc(xfer_buffer_size);

            if (transfer_buffer != nullptr)
            {
                memset(transfer_buffer, 0, xfer_buffer_size);
            }
            else
            {
                bSuccess = false;
            }
        }

        // Allocate and initialize the transfers
        if (bSuccess)
        {
            for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
            {
                bulk_transfer_requests[transfer_index] = libusb_alloc_transfer(0);

                if (bulk_transfer_requests[transfer_index] != nullptr)
                {
                    libusb_fill_bulk_transfer(
                        bulk_transfer_requests[transfer_index],
                        m_device_handle,
                        bulk_endpoint,
                        transfer_buffer + transfer_index*m_request.transfer_packet_size,
                        m_request.transfer_packet_size,
                        transfer_callback_function,
                        reinterpret_cast<void*>(this),
                        0);
                }
                else
                {
                    bSuccess = false;
                }
            }
        }

        return bSuccess;
    }

    bool startTransfers()
    {
        bool bSuccess = (m_active_transfer_count == 0 && !m_is_canceled);

        // Start the transfers
        if (bSuccess)
        {
            for (int transfer_index = 0; transfer_index < m_request.in_flight_transfer_packet_count; ++transfer_index)
            {
                libusb_transfer *bulk_transfer= bulk_transfer_requests[transfer_index];

                if (libusb_submit_transfer(bulk_transfer) == 0)
                {
                    ++m_active_transfer_count;
                }
                else
                {
                    bSuccess= false;
                    break;
                }
            }
        }

        return bSuccess;
    }

    static void LIBUSB_CALL transfer_callback_function(struct libusb_transfer *bulk_transfer)
    {
        USBBulkTransferBundle *bundle = reinterpret_cast<USBBulkTransferBundle*>(bulk_transfer->user_data);
        enum libusb_transfer_status status = bulk_transfer->status;

        if (status == LIBUSB_TRANSFER_COMPLETED)
        {
            // NOTE: This callback is getting executed on the worker thread!
            // It should not:
            // 1) Do any expensive work
            // 2) Call any blocking functions
            // 3) Access data on the main thread, unless it can do so in an atomic way
            bundle->m_request.on_data_callback(
                bulk_transfer->buffer,
                bulk_transfer->actual_length,
                bundle->m_request.transfer_callback_userdata);
        }

        // See if the request wants to resubmitted the moment it completes.
        // If the transfer was canceled, this overrides the auto-resubmit.
        bool bRestartedTransfer= false;
        if (status != LIBUSB_TRANSFER_CANCELLED && bundle->m_request.bAutoResubmit)
        {
            // Start the transfer over with the same properties
            if (libusb_submit_transfer(bulk_transfer) == 0)
            {
                bRestartedTransfer= true;
            }
        }
        
        // If the transfer didn't restart update the active transfer count
        if (!bRestartedTransfer)
        {
            assert(bundle->m_active_transfer_count > 0);
            --bundle->m_active_transfer_count;
        }
    }

    void cancelTransfers()
    {
        assert(bulk_transfer_requests != nullptr);

        if (!m_is_canceled)
        {
            for (int transfer_index = 0;
                transfer_index < m_request.in_flight_transfer_packet_count;
                ++transfer_index)
            {
                libusb_transfer* bulk_transfer = bulk_transfer_requests[transfer_index];

                assert(bulk_transfer != nullptr);
                libusb_cancel_transfer(bulk_transfer);
            }

            m_is_canceled= true;
        }
    }

    // Search for an input transfer endpoint in the endpoint descriptor
    // of the device interfaces alt_settings
    static bool find_bulk_transfer_endpoint(struct libusb_device *device, uint8_t &out_endpoint_addr)
    {
        bool bSuccess= false;
        libusb_config_descriptor *config = nullptr;

        libusb_get_active_config_descriptor(device, &config);

        if (config != nullptr)
        {
            const libusb_interface_descriptor *altsetting = nullptr;

            for (int i = 0; i < config->bNumInterfaces; i++)
            {
                const libusb_interface_descriptor *test_altsetting = config->interface[i].altsetting;

                if (test_altsetting[0].bInterfaceNumber == 0)
                {
                    altsetting = test_altsetting;
                    break;
                }
            }

            if (altsetting != nullptr)
            {
                for (int i = 0; i < altsetting->bNumEndpoints; i++)
                {
                    const libusb_endpoint_descriptor *endpoint_desc = &altsetting->endpoint[i];

                    if ((endpoint_desc->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK
                        && endpoint_desc->wMaxPacketSize != 0)
                    {
                        out_endpoint_addr = endpoint_desc->bEndpointAddress;
                        bSuccess = true;
                        break;
                    }
                }
            }

            libusb_free_config_descriptor(config);
        }

        return bSuccess;
    }

protected:
    void dispose()
    {
        assert(m_active_transfer_count == 0);

        for (int transfer_index = 0;
            transfer_index < m_request.in_flight_transfer_packet_count;
            ++transfer_index)
        {
            if (bulk_transfer_requests[transfer_index] != nullptr)
            {
                libusb_free_transfer(bulk_transfer_requests[transfer_index]);
            }
        }

        if (transfer_buffer != nullptr)
        {
            free(transfer_buffer);
            transfer_buffer = nullptr;
        }

        if (bulk_transfer_requests != nullptr)
        {
            free(bulk_transfer_requests);
            bulk_transfer_requests = nullptr;
        }
    }

private:
    USBDeviceManager::RequestPayload_StartBulkTransfer m_request;
    struct libusb_device *m_device;
    struct libusb_device_handle *m_device_handle;

    int m_active_transfer_count;
    bool m_is_canceled;
    struct libusb_transfer** bulk_transfer_requests;
    unsigned char* transfer_buffer;
};

// -USBAsyncRequestManagerImpl-
/// Internal implementation of the USB async request manager.
class USBDeviceManagerImpl
{
protected:
    struct LibUSBDeviceState
    {
        t_usb_device_handle handle;
        libusb_device *device;
        libusb_device_handle *device_handle;
        bool is_interface_claimed;
    };

public:
    USBDeviceManagerImpl(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length)
        : m_usb_context(nullptr)
        , m_exit_signaled({ false })
        , m_active_control_transfers(0)
        , m_thread_started(false)
    {
        for (size_t list_index = 0; list_index < device_whitelist_length; ++list_index)
        {
            m_device_whitelist.push_back(device_whitelist[list_index]);
        }
    }

    virtual ~USBDeviceManagerImpl()
    {
    }

    // -- System ----
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
            freeDeviceStateList();

            // Free the libusb context
            libusb_exit(m_usb_context);
            m_usb_context= nullptr;
        }
    }

    // -- Device Actions ----
    bool openUSBDevice(t_usb_device_handle handle)
    {
        bool bOpened= false;

        if (getIsUSBDeviceOpen(handle))
        {
            LibUSBDeviceState *state= get_libusb_state_from_handle(handle);

            int res = libusb_open(state->device, &state->device_handle);
            if (res == 0)
            {
                res = libusb_claim_interface(state->device_handle, 0);
                if (res == 0)
                {
                    state->is_interface_claimed= true;
                    bOpened= true;

                    SERVER_LOG_INFO("USBAsyncRequestManager::openUSBDevice") << "Successfully opened device " << handle;
                }
                else
                {
                    SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to claim USB device: " << res;
                }
            }
            else
            {
                SERVER_LOG_ERROR("USBAsyncRequestManager::openUSBDevice") << "Failed to open USB device: " << res;
            }

            if (!bOpened)
            {
                closeUSBDevice(handle);
            }
        }

        return bOpened;
    }

    void closeUSBDevice(t_usb_device_handle handle)
    {
        LibUSBDeviceState *state = get_libusb_state_from_handle(handle);

        if (state->is_interface_claimed)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Released USB interface on handle " << handle;
            libusb_release_interface(state->device_handle, 0);
            state->is_interface_claimed= false;
        }

        if (state->device_handle != nullptr)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::closeUSBDevice") << "Close USB device on handle " << handle;
            libusb_close(state->device_handle);
            state->device_handle= nullptr;
        }
    }

    // -- Device Queries ----
    int getUSBDeviceCount() const
    {
        return static_cast<int>(m_device_state_list.size());
    }

    t_usb_device_handle getFirstUSBDeviceHandle() const
    {
        return (m_device_state_list.size() > 0) ? static_cast<t_usb_device_handle>(0) : k_invalid_usb_device_handle;
    }

    t_usb_device_handle getNextUSBDeviceHandle(t_usb_device_handle handle) const
    {
        int device_index= static_cast<int>(handle);

        return (device_index + 1 < getUSBDeviceCount()) ? static_cast<t_usb_device_handle>(device_index + 1) : k_invalid_usb_device_handle;
    }

    bool getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceInfo &outDeviceInfo) const
    {
        bool bSuccess= false;
        const libusb_device *dev = get_libusb_device_from_handle_const(handle);

        if (dev != nullptr)
        {
            struct libusb_device_descriptor dev_desc;
            libusb_get_device_descriptor(const_cast<libusb_device *>(dev), &dev_desc);

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
            libusb_device *dev = m_device_state_list[device_index].device;

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

    bool getIsUSBDeviceOpen(t_usb_device_handle handle) const
    {
        bool bIsOpen= false;
        const LibUSBDeviceState *state= get_libusb_state_from_handle_const(handle);

        if (state != nullptr)
        {
            bIsOpen= (state->device_handle != nullptr);
        }

        return bIsOpen;
    }

    // -- Request Queue ----
    bool submitTransferRequest(const USBDeviceManager::USBTransferRequest &request)
    {
        return request_queue.push(request);
    }

protected:
    void startWorkerThread()
    {
        if (!m_thread_started)
        {
            SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Starting USB event thread";
            m_worker_thread = std::thread(&USBDeviceManagerImpl::workerThreadFunc, this);
            m_thread_started = true;
        }
    }

    void workerThreadFunc()
    {
        ServerUtility::set_current_thread_name("USB Async Worker Thread");

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50 * 1000; // ms

        // Stay in the message loop until asked to exit by the main thread
        while (!m_exit_signaled)
        {
            // Process incoming USB transfer requests
            USBDeviceManager::USBTransferRequest request;
            while (request_queue.pop(request))
            {
                switch (request.request_type)
                {
                case USBDeviceManager::_USBRequestType_ControlTransfer:
                    handleControlTransferRequest(request.payload.control_transfer);
                    break;
                case USBDeviceManager::_USBRequestType_StartBulkTransfer:
                    handleStartBulkTransferRequest(request.payload.start_bulk_transfer);
                    break;
                case USBDeviceManager::_USBRequestType_StopBulkTransfer:
                    handleCancelBulkTransferRequest(request.payload.stop_bulk_transfer);
                    break;
                }
            }

            if (m_active_bulk_transfer_bundles.size() > 0 || 
                m_canceled_bulk_transfer_bundles.size() > 0 ||
                m_active_control_transfers > 0)
            {
                // Give libusb a change to process transfer requests and post events
                libusb_handle_events_timeout_completed(m_usb_context, &tv, NULL);

                // Cleanup any requests that no longer have any pending cancellations
                cleanupCanceledRequests();
            }
            else
            {
                ServerUtility::sleep_ms(100);
            }
        }

        // Drain the request queue
        while (request_queue.pop());

        // Cancel all active transfers
        while (m_active_bulk_transfer_bundles.size() > 0)
        {
            USBBulkTransferBundle *bundle= m_active_bulk_transfer_bundles.back();
            m_active_bulk_transfer_bundles.pop_back();
            bundle->cancelTransfers();
            m_canceled_bulk_transfer_bundles.push_back(bundle);
        }

        // Wait for the canceled transfers to exit
        while (m_canceled_bulk_transfer_bundles.size() > 0)
        {
            // Give libusb a change to process the cancellation requests
            libusb_handle_events_timeout_completed(m_usb_context, &tv, NULL);

            // Cleanup any requests that no longer have any pending cancellations
            cleanupCanceledRequests();
        }
    }

    void cleanupCanceledRequests()
    {
        for (auto it = m_canceled_bulk_transfer_bundles.begin(); it != m_canceled_bulk_transfer_bundles.end(); ++it)
        {
            USBBulkTransferBundle *bundle = *it;

            //###HipsterSloth $TODO Timeout the cancellation?
            if (bundle->getActiveTransferCount() == 0)
            {
                m_canceled_bulk_transfer_bundles.erase(it);
                delete bundle;
            }
        }
    }

    void handleControlTransferRequest(USBDeviceManager::RequestPayload_ControlTransfer &request)
    {
        //TODO
    }

    void handleStartBulkTransferRequest(USBDeviceManager::RequestPayload_StartBulkTransfer &request)
    {
        LibUSBDeviceState *state = get_libusb_state_from_handle(request.usb_device_handle);

        if (state != nullptr)
        {
            if (state->device_handle != nullptr)
            {
                // Only start a bulk transfer if the device doesn't have one going already
                auto it = std::find_if(
                    m_active_bulk_transfer_bundles.begin(),
                    m_active_bulk_transfer_bundles.end(),
                    [this, &request](const USBBulkTransferBundle *bundle) {
                    return bundle->getUSBDeviceHandle() == request.usb_device_handle;
                });

                if (it == m_active_bulk_transfer_bundles.end())
                {
                    USBBulkTransferBundle *bundle = 
                        new USBBulkTransferBundle(request, state->device, state->device_handle);

                    // Allocate and initialize the bulk transfers
                    if (bundle->initialize())
                    {
                        // Attempt to start all the transfers
                        if (bundle->startTransfers())
                        {
                            // Success! Add the bundle to the list of active bundles
                            m_active_bulk_transfer_bundles.push_back(bundle);
                        }
                        else
                        {                            
                            // Unable to start all of the transfers in the bundle
                            if (bundle->getActiveTransferCount() > 0)
                            {
                                // If any transfers started we have to cancel the ones that started
                                // and wait for the cancellation request to complete.
                                bundle->cancelTransfers();
                                m_canceled_bulk_transfer_bundles.push_back(bundle);
                            }
                            else
                            {
                                // No transfer requests started.
                                // Delete the bundle right away.
                                delete bundle;
                            }

                            SERVER_MT_LOG_ERROR("USBAsyncRequestManager::handleStartBulkTransferRequest")
                                << "Failed to start all bulk transfer bundle for device " << request.usb_device_handle;
                        }
                    }
                    else
                    {
                        SERVER_MT_LOG_ERROR("USBAsyncRequestManager::handleStartBulkTransferRequest")
                            << "Failed to initialize bulk transfer bundle for device " << request.usb_device_handle;
                        delete bundle;
                    }
                }
                else
                {
                    SERVER_MT_LOG_WARNING("USBAsyncRequestManager::handleStartBulkTransferRequest")
                        << "Already started bulk transfer request for device " << request.usb_device_handle;
                }
            }
            else
            {
                SERVER_MT_LOG_WARNING("USBAsyncRequestManager::handleStartBulkTransferRequest")
                    << "Already started bulk transfer request for device " << request.usb_device_handle;
            }
        }
        else
        {
            SERVER_MT_LOG_WARNING("USBAsyncRequestManager::handleStartBulkTransferRequest")
                << "Invalid device handle " << request.usb_device_handle;
        }
    }

    void handleCancelBulkTransferRequest(USBDeviceManager::RequestPayload_StopBulkTransfer &request)
    {
        libusb_device *dev = get_libusb_device_from_handle(request.usb_device_handle);

        if (dev != nullptr)
        {
            auto it = std::find_if(
                m_active_bulk_transfer_bundles.begin(),
                m_active_bulk_transfer_bundles.end(),
                [this, &request](const USBBulkTransferBundle *bundle) {
                return bundle->getUSBDeviceHandle() == request.usb_device_handle;
            });

            if (it != m_active_bulk_transfer_bundles.end())
            {
                USBBulkTransferBundle *bundle = *it;

                // Tell the bundle to cancel all active transfers.
                // This is an asynchronous operation.
                bundle->cancelTransfers();

                // Remove the bundle from the list of active transfers
                m_active_bulk_transfer_bundles.erase(it);

                // Put the bundle on the list of canceled transfers.
                // The bundle will get cleaned up once all active transfers are done.
                m_canceled_bulk_transfer_bundles.push_back(bundle);
            }
            else
            {
                SERVER_MT_LOG_WARNING("USBAsyncRequestManager::handleStopBulkTransferRequest")
                    << "Bulk transfer bundle not active for device " << request.usb_device_handle;
            }
        }
        else
        {
            SERVER_MT_LOG_ERROR("USBAsyncRequestManager::handleStartBulkTransferRequest")
                << "Invalid device handle " << request.usb_device_handle;
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

    inline const LibUSBDeviceState *get_libusb_state_from_handle_const(t_usb_device_handle handle) const
    {
        const LibUSBDeviceState *state = nullptr;
        int device_index = static_cast<int>(handle);

        if (device_index >= 0 && device_index < getUSBDeviceCount())
        {
            state = &m_device_state_list[device_index];
        }

        return state;
    }

    inline LibUSBDeviceState *get_libusb_state_from_handle(t_usb_device_handle handle)
    {
        return const_cast<LibUSBDeviceState *>(get_libusb_state_from_handle_const(handle));
    }

    inline const libusb_device *get_libusb_device_from_handle_const(t_usb_device_handle handle) const
    {
        const LibUSBDeviceState *state= get_libusb_state_from_handle_const(handle);
        const libusb_device *device= nullptr;

        if (state != nullptr)
        {
            device = state->device;
        }

        return device;
    }

    inline libusb_device *get_libusb_device_from_handle(t_usb_device_handle handle)
    {
        return const_cast<libusb_device *>(get_libusb_device_from_handle_const(handle));
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

                                // Add a device state entry to the list
                                {
                                    LibUSBDeviceState device_state;

                                    // The "handle" is really just an index into the device state list
                                    device_state.handle= static_cast<t_usb_device_handle>(m_device_state_list.size());
                                    device_state.device = dev;
                                    device_state.device_handle = nullptr;
                                    device_state.is_interface_claimed = false;

                                    m_device_state_list.push_back(device_state);
                                }

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

    void freeDeviceStateList()
    {
        std::for_each(
            m_device_state_list.begin(), 
            m_device_state_list.end(), [this](LibUSBDeviceState &device_state) {
                closeUSBDevice(device_state.handle);
                libusb_unref_device(device_state.device);
            });
        m_device_state_list.clear();
    }

    bool isDeviceInWhitelist(libusb_device *dev)
    {
        libusb_device_descriptor desc;
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
    boost::lockfree::spsc_queue<USBDeviceManager::USBTransferRequest, boost::lockfree::capacity<128> > request_queue;

    // Worker thread state
    std::vector<USBBulkTransferBundle *> m_active_bulk_transfer_bundles;
    std::vector<USBBulkTransferBundle *> m_canceled_bulk_transfer_bundles;
    int m_active_control_transfers;

    // Main thread state
    bool m_thread_started;
    std::thread m_worker_thread;
    std::vector<USBDeviceInfo> m_device_whitelist;
    std::vector<LibUSBDeviceState> m_device_state_list;
};

//-- public interface -----
USBDeviceManager *USBDeviceManager::m_instance = NULL;

USBDeviceManager::USBDeviceManager(struct USBDeviceInfo *device_whitelist, size_t device_whitelist_length)
    : implementation_ptr(new USBDeviceManagerImpl(device_whitelist, device_whitelist_length))
{
}

USBDeviceManager::~USBDeviceManager()
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

bool USBDeviceManager::startup()
{
    m_instance = this;
    return implementation_ptr->startup();
}

void USBDeviceManager::update()
{
    implementation_ptr->update();
}

void USBDeviceManager::shutdown()
{
    implementation_ptr->shutdown();
    m_instance = NULL;
}

bool USBDeviceManager::openUSBDevice(t_usb_device_handle handle)
{
    return implementation_ptr->openUSBDevice(handle);
}

void USBDeviceManager::closeUSBDevice(t_usb_device_handle handle)
{
    implementation_ptr->closeUSBDevice(handle);
}

int USBDeviceManager::getUSBDeviceCount() const
{
    return implementation_ptr->getUSBDeviceCount();
}

t_usb_device_handle USBDeviceManager::getFirstUSBDeviceHandle() const
{
    return implementation_ptr->getFirstUSBDeviceHandle();
}

t_usb_device_handle USBDeviceManager::getNextUSBDeviceHandle(t_usb_device_handle handle) const
{
    return implementation_ptr->getNextUSBDeviceHandle(handle);
}

bool USBDeviceManager::getUSBDeviceInfo(t_usb_device_handle handle, USBDeviceInfo &outDeviceInfo) const
{
    return implementation_ptr->getUSBDeviceInfo(handle, outDeviceInfo);
}

bool USBDeviceManager::getUSBDevicePath(t_usb_device_handle handle, char *outBuffer, size_t bufferSize) const
{
    return implementation_ptr->getUSBDevicePath(handle, outBuffer, bufferSize);
}

bool USBDeviceManager::getIsUSBDeviceOpen(t_usb_device_handle handle) const
{
    return implementation_ptr->getIsUSBDeviceOpen(handle);
}

bool USBDeviceManager::submitTransferRequest(const USBTransferRequest &request)
{
    return implementation_ptr->submitTransferRequest(request);
}