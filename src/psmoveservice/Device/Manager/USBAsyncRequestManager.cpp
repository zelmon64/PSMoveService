//-- includes -----
#include "USBAsyncRequestManager.h"
#include "ServerLog.h"

//-- pre-declarations -----

//-- private implementation -----

// -USBAsyncRequestManagerImpl-
/// Internal implementation of the USB async request manager.
class USBAsyncRequestManagerImpl
{
public:
    USBAsyncRequestManagerImpl()
    {
    }

    virtual ~USBAsyncRequestManagerImpl()
    {
    }

    bool startup()
    {
        bool bSuccess= true;

        SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Starting USB event thread";

        return bSuccess;
    }

    void update()
    {
    }

    void shutdown()
    {
        SERVER_LOG_INFO("USBAsyncRequestManager::startup") << "Stopping USB event thread";
    }
};

//-- public interface -----
USBAsyncRequestManager *USBAsyncRequestManager::m_instance = NULL;

USBAsyncRequestManager::USBAsyncRequestManager()
    : implementation_ptr(new USBAsyncRequestManagerImpl())
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