#ifndef USB_ASYNC_REQUEST_MANAGER_H
#define USB_ASYNC_REQUEST_MANAGER_H

//-- includes -----

//-- definitions -----
/// Manages async control and bulk transfer requests to usb devices via libusb.
class USBAsyncRequestManager
{
public:
    USBAsyncRequestManager();
    virtual ~USBAsyncRequestManager();

    bool startup(); /**< Initialize the libusb thread. */
    void update();  /**< Process events from the libusb thread. */
    void shutdown();/**< Shutdown the libusb thread. */

    static inline USBAsyncRequestManager *getInstance()
    {
        return m_instance;
    }

private:
    /// private implementation
    class USBAsyncRequestManagerImpl *implementation_ptr;

    /// Singleton instance of the class
    /// Assigned in startup, cleared in teardown
    static USBAsyncRequestManager *m_instance;
};

#endif  // USB_ASYNC_REQUEST_MANAGER_H