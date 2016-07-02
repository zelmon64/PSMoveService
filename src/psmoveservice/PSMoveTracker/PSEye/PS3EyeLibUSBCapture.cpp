/*
 This file is largely reproduced from PS3EYEDriver(https://github.com/inspirit/PS3EYEDriver).
 We adapted the library to work with a shared USBManager in the service.
 The license for PS3EYEDriver is reproduced below:

 <license>
 License information for PS3EYEDriver
 ------------------------------------
 
 The license of the PS3EYEDriver is MIT (for newly-written code) and GPLv2 for
 all code derived from the Linux Kernel Driver (ov534) sources.
 
 In https://github.com/inspirit/PS3EYEDriver/pull/3, Eugene Zatepyakin writes:
 
 "all of my code is MIT licensed and to tell the truth i didnt check Linux
 p3eye version license. as far as i know it was contributed to Linux by some
 devs who decided to do it on their own..."
 
 The code is based on the Linux driver for the PSEye, which can be found here:
 
 http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/drivers/media/usb/gspca/ov534.c
  
 ov534-ov7xxx gspca driver
 
 Copyright (C) 2008 Antonio Ospite <ospite@studenti.unina.it>
 Copyright (C) 2008 Jim Paris <jim@jtan.com>
 Copyright (C) 2009 Jean-Francois Moine http://moinejf.free.fr
 
 Based on a prototype written by Mark Ferrell <majortrips@gmail.com>
 USB protocol reverse engineered by Jim Paris <jim@jtan.com>
 https://jim.sh/svn/jim/devl/playstation/ps3/eye/test/
 
 PS3 Eye camera enhanced by Richard Kaswy http://kaswy.free.fr
 PS3 Eye camera - brightness, contrast, awb, agc, aec controls
                  added by Max Thrun <bear24rw@gmail.com>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 </license>
*/

//-- includes -----
#include "PS3EyeLibUSBCapture.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include "USBDeviceManager.h"

#include <algorithm>
#include <atomic>
#include <iomanip>
#include <string>
#include <deque>

#include "async/async.hpp"

//-- constants -----
#define TRANSFER_SIZE		16384
#define NUM_TRANSFERS		8

#define OV534_REG_ADDRESS	0xf1	/* sensor address */
#define OV534_REG_SUBADDR	0xf2
#define OV534_REG_WRITE		0xf3
#define OV534_REG_READ		0xf4
#define OV534_REG_OPERATION	0xf5
#define OV534_REG_STATUS	0xf6

#define OV534_OP_WRITE_3	0x37
#define OV534_OP_WRITE_2	0x33
#define OV534_OP_READ_2		0xf9

#define CTRL_TIMEOUT 500
#define VGA	 0
#define QVGA 1

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type {
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

// From: libusb.h
//###HipsterSloth $TODO This should be hidden in USBDeviceManager.cpp
enum libusb_request_type {
    /** Standard */
    LIBUSB_REQUEST_TYPE_STANDARD = (0x00 << 5),

    /** Class */
    LIBUSB_REQUEST_TYPE_CLASS = (0x01 << 5),

    /** Vendor */
    LIBUSB_REQUEST_TYPE_VENDOR = (0x02 << 5),

    /** Reserved */
    LIBUSB_REQUEST_TYPE_RESERVED = (0x03 << 5)
};

// From: libusb.h
//###HipsterSloth $TODO This should be hidden in USBDeviceManager.cpp
enum libusb_endpoint_direction {
    /** In: device-to-host */
    LIBUSB_ENDPOINT_IN = 0x80,

    /** Out: host-to-device */
    LIBUSB_ENDPOINT_OUT = 0x00
};

// From: libusb.h
//###HipsterSloth $TODO This should be hidden in USBDeviceManager.cpp
enum libusb_request_recipient {
    /** Device */
    LIBUSB_RECIPIENT_DEVICE = 0x00,

    /** Interface */
    LIBUSB_RECIPIENT_INTERFACE = 0x01,

    /** Endpoint */
    LIBUSB_RECIPIENT_ENDPOINT = 0x02,

    /** Other */
    LIBUSB_RECIPIENT_OTHER = 0x03,
};

//-- typedefs -----
// From: libusb.h
/* stdint.h is not available on older MSVC */
#if defined(_MSC_VER) && (_MSC_VER < 1600) && (!defined(_STDINT)) && (!defined(_STDINT_H))
    typedef unsigned __int8   uint8_t;
    typedef unsigned __int16  uint16_t;
    typedef unsigned __int32  uint32_t;
#else
    #include <stdint.h>
#endif

//-- macros -----
//#define DEBUG_PS3EYE
#if defined(DEBUG_PS3EYE)
#define debug(...) fprintf(stdout, __VA_ARGS__)
#else
#define debug(...) 
#endif

//-- data -----
static const uint8_t ov534_reg_initdata[][2] = {
    { 0xe7, 0x3a },

    { OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

    { 0xc2, 0x0c },
    { 0x88, 0xf8 },
    { 0xc3, 0x69 },
    { 0x89, 0xff },
    { 0x76, 0x03 },
    { 0x92, 0x01 },
    { 0x93, 0x18 },
    { 0x94, 0x10 },
    { 0x95, 0x10 },
    { 0xe2, 0x00 },
    { 0xe7, 0x3e },

    { 0x96, 0x00 },

    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x0a },
    { 0x97, 0x3f },
    { 0x97, 0x4a },
    { 0x97, 0x20 },
    { 0x97, 0x15 },
    { 0x97, 0x0b },

    { 0x8e, 0x40 },
    { 0x1f, 0x81 },
    { 0x34, 0x05 },
    { 0xe3, 0x04 },
    { 0x88, 0x00 },
    { 0x89, 0x00 },
    { 0x76, 0x00 },
    { 0xe7, 0x2e },
    { 0x31, 0xf9 },
    { 0x25, 0x42 },
    { 0x21, 0xf0 },

    { 0x1c, 0x00 },
    { 0x1d, 0x40 },
    { 0x1d, 0x02 }, /* payload size 0x0200 * 4 = 2048 bytes */
    { 0x1d, 0x00 }, /* payload size */

    // -------------

    //	{ 0x1d, 0x01 },/* frame size */		// kwasy
    //	{ 0x1d, 0x4b },/* frame size */
    //	{ 0x1d, 0x00 }, /* frame size */


    //	{ 0x1d, 0x02 },/* frame size */		// macam
    //	{ 0x1d, 0x57 },/* frame size */
    //	{ 0x1d, 0xff }, /* frame size */

    { 0x1d, 0x02 },/* frame size */		// jfrancois / linuxtv.org/hg/v4l-dvb
    { 0x1d, 0x58 },/* frame size */
    { 0x1d, 0x00 }, /* frame size */

    // ---------

    { 0x1c, 0x0a },
    { 0x1d, 0x08 }, /* turn on UVC header */
    { 0x1d, 0x0e }, /* .. */

    { 0x8d, 0x1c },
    { 0x8e, 0x80 },
    { 0xe5, 0x04 },

    // ----------------
    //	{ 0xc0, 0x28 },//	kwasy / macam
    //	{ 0xc1, 0x1e },//

    { 0xc0, 0x50 },		// jfrancois
    { 0xc1, 0x3c },
    { 0xc2, 0x0c },
};

static const uint8_t ov772x_reg_initdata[][2] = {
    { 0x12, 0x80 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },
    { 0x11, 0x01 },

    { 0x3d, 0x03 },
    { 0x17, 0x26 },
    { 0x18, 0xa0 },
    { 0x19, 0x07 },
    { 0x1a, 0xf0 },
    { 0x32, 0x00 },
    { 0x29, 0xa0 },
    { 0x2c, 0xf0 },
    { 0x65, 0x20 },
    { 0x11, 0x01 },
    { 0x42, 0x7f },
    { 0x63, 0xAA }, 	// AWB
    { 0x64, 0xff },
    { 0x66, 0x00 },
    { 0x13, 0xf0 },	// COM8  - jfrancois 0xf0	orig x0f7
    { 0x0d, 0x41 },
    { 0x0f, 0xc5 },
    { 0x14, 0x11 },

    { 0x22, 0x7f },
    { 0x23, 0x03 },
    { 0x24, 0x40 },
    { 0x25, 0x30 },
    { 0x26, 0xa1 },
    { 0x2a, 0x00 },
    { 0x2b, 0x00 },
    { 0x6b, 0xaa },
    { 0x13, 0xff },	// COM8 - jfrancois 0xff orig 0xf7

    { 0x90, 0x05 },
    { 0x91, 0x01 },
    { 0x92, 0x03 },
    { 0x93, 0x00 },
    { 0x94, 0x60 },
    { 0x95, 0x3c },
    { 0x96, 0x24 },
    { 0x97, 0x1e },
    { 0x98, 0x62 },
    { 0x99, 0x80 },
    { 0x9a, 0x1e },
    { 0x9b, 0x08 },
    { 0x9c, 0x20 },
    { 0x9e, 0x81 },

    { 0xa6, 0x04 },
    { 0x7e, 0x0c },
    { 0x7f, 0x16 },
    { 0x80, 0x2a },
    { 0x81, 0x4e },
    { 0x82, 0x61 },
    { 0x83, 0x6f },
    { 0x84, 0x7b },
    { 0x85, 0x86 },
    { 0x86, 0x8e },
    { 0x87, 0x97 },
    { 0x88, 0xa4 },
    { 0x89, 0xaf },
    { 0x8a, 0xc5 },
    { 0x8b, 0xd7 },
    { 0x8c, 0xe8 },
    { 0x8d, 0x20 },

    { 0x0c, 0x90 },

    { 0x2b, 0x00 },
    { 0x22, 0x7f },
    { 0x23, 0x03 },
    { 0x11, 0x01 },
    { 0x0c, 0xd0 },
    { 0x64, 0xff },
    { 0x0d, 0x41 },

    { 0x14, 0x41 },
    { 0x0e, 0xcd },
    { 0xac, 0xbf },
    { 0x8e, 0x00 },	// De-noise threshold - jfrancois 0x00 - orig 0x04
    { 0x0c, 0xd0 }

};

static const uint8_t bridge_start_vga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x40 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x02 },
    { 0x1d, 0x58 },
    { 0x1d, 0x00 },
    { 0xc0, 0x50 },
    { 0xc1, 0x3c },
};
static const uint8_t sensor_start_vga[][2] = {
    { 0x12, 0x00 },
    { 0x17, 0x26 },
    { 0x18, 0xa0 },
    { 0x19, 0x07 },
    { 0x1a, 0xf0 },
    { 0x29, 0xa0 },
    { 0x2c, 0xf0 },
    { 0x65, 0x20 },
};
static const uint8_t bridge_start_qvga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x40 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x01 },
    { 0x1d, 0x4b },
    { 0x1d, 0x00 },
    { 0xc0, 0x28 },
    { 0xc1, 0x1e },
};
static const uint8_t sensor_start_qvga[][2] = {
    { 0x12, 0x40 },
    { 0x17, 0x3f },
    { 0x18, 0x50 },
    { 0x19, 0x03 },
    { 0x1a, 0x78 },
    { 0x29, 0x50 },
    { 0x2c, 0x78 },
    { 0x65, 0x2f },
};

//-- private methods -----
static void async_init_camera(t_usb_device_handle device_handle, uint32_t width, uint32_t height, uint8_t frameRate, async::TaskCallback<int> outCallback);
static void async_start_camera(const t_usb_device_handle device_handle, const PS3EyeProperties &properties, class PS3EyeVideoPacketProcessor* processor, async::TaskCallback<int> outCallback);
static void async_stop_camera(const t_usb_device_handle device_handle, async::TaskCallback<int> outCallback);

static void async_set_autogain(t_usb_device_handle device_handle, bool bAutoGain, uint8_t gain, uint8_t exposure, async::TaskCallback<int> outCallback);
static void async_set_auto_white_balance(t_usb_device_handle device_handle, bool bAutoWhiteBalance, async::TaskCallback<int> outCallback);
static void async_set_gain(t_usb_device_handle handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_exposure(t_usb_device_handle handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_sharpness(t_usb_device_handle device_handle,unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_contrast(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_brightness(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_hue(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_red_balance(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_green_balance(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_blue_balance(t_usb_device_handle device_handle, unsigned char val, async::TaskCallback<int> outCallback);
static void async_set_flip(t_usb_device_handle device_handle, bool horizontal, bool vertical, async::TaskCallback<int> outCallback);
static void async_set_frame_rate(t_usb_device_handle device_handle, uint32_t frame_width, uint8_t frame_rate, async::TaskCallback<int> outCallback);

static void async_sccb_reg_write(t_usb_device_handle device_handle, uint8_t reg, uint8_t val, async::TaskCallback<int> outCallback);
static void async_sccb_reg_write_array(t_usb_device_handle device_handle, const uint8_t (*sequence)[2], int sequenceLength, async::TaskCallback<int> outCallback);
static void async_sccb_reg_read(t_usb_device_handle device_handle, uint16_t reg, async::TaskCallback<int> outCallback);
static void async_sccb_check_status(t_usb_device_handle device_handle, async::TaskCallback<int> checkStatusCallback);

static void async_ov534_reg_write(t_usb_device_handle device_handle, uint16_t reg, uint8_t val, async::TaskCallback<int> callback);
static void async_ov534_reg_write_array(t_usb_device_handle device_handle, const uint8_t (*sequence)[2], int sequenceLength, async::TaskCallback<int> outCallback);
static void async_ov534_reg_read(t_usb_device_handle device_handle, uint16_t reg, async::TaskCallback<int> callback);
static void async_ov534_set_led(t_usb_device_handle device_handle, bool bLedOn, async::TaskCallback<int> outCallback);

static uint8_t compute_frame_rate_settings(const uint32_t frame_width, const uint8_t frame_rate, uint8_t *out_r11=nullptr, uint8_t *out_r0d=nullptr, uint8_t *out_re5=nullptr);
static void log_usb_result_code(const char *function_name, eUSBResultCode result_code);

//-- public interface -----
struct USBAsyncTask
{
    std::string task_name;
    async::Task<int> task_func;
};

//-- USBAsyncTaskQueue -----
class USBAsyncTaskQueue
{
public:
    USBAsyncTaskQueue() 
        : m_taskQueue()
    {}

    void addAsyncTask(const std::string &task_name, async::Task<int> task_func)
    {
        USBAsyncTask task= {task_name, task_func};
         
        m_taskQueue.push_back(task);

        // If this is the only task, go ahead and start it because there
        // won't be an older task completing to start this new one
        if (m_taskQueue.size() == 1)
        {
            startNextTask();
        }
    }

    void startNextTask()
    {
        if (m_taskQueue.size() > 0)
        {
            // Get the next task in the list
            USBAsyncTask task= m_taskQueue.front();

            // Once the task completes, start the next task (if any).
            // NOTE: The [&] capture on this lambda means it gets a reference to 
            // this objects task queue that it can use remove the task from the queue.
            async::TaskCallback<int> onTaskCompleted = 
                [&](async::ErrorCode error, int result) {
                    if (error == async::FAIL)
                    {
                        SERVER_LOG_ERROR("PSEYECaptureCAM_LibUSB::startNextTask") 
                            << "Camera Async Task(" << task.task_name 
                            << ") failed with code: " << result;
                    }

                    m_taskQueue.pop_front();
                    startNextTask();
                };

            // Start the next task
            task.task_func(onTaskCompleted);
        }
    }

    std::deque<USBAsyncTask> m_taskQueue;
};

//-- PS3EyeVideoPacketProcessor -----
/// Decodes incoming "USB Video Class" (UVC) packets from the USBDeviceManager worker thread
/// and 
class PS3EyeVideoPacketProcessor
{
public:
    PS3EyeVideoPacketProcessor()
        : m_last_packet_type(DISCARD_PACKET)
        , m_lastPresentationTimestamp(0)
        , m_lastFieldID(0)
        , m_currentFrameStart(nullptr)
        , m_currentFrameBytesWritten(0)
        , m_frameCounter({0})
        , m_frameTotalSizeBytes(0)
        , m_frameCounter_mainThread(0)
    {
        m_frameBuffers[0] = nullptr;
        m_frameBuffers[1] = nullptr;
        m_frameBuffer_mainThread= nullptr;
    }

    virtual ~PS3EyeVideoPacketProcessor()
    {
        release_mainThread();
    }

    void setFrameSize_mainThread(uint32_t frame_size)
    {
        m_frameTotalSizeBytes= frame_size;
        reallocate_mainThread();
    }

    const uint8_t *getFrameBuffer_mainThread()
    {
        const int k_max_refetch_attempts= 2;
        int fetchAttempt= 0;

        // Pull data from the worker thread if it changed
        int worker_thread_frame_counter= static_cast<int>(m_frameCounter);
        while (worker_thread_frame_counter != m_frameCounter_mainThread && fetchAttempt < k_max_refetch_attempts)
        {
            // The frame being written to is indexed by the frame counter.
            // That means the completed frame we want to read is the other frame index.
            int read_frame_index= (worker_thread_frame_counter + 1) % 2;

            // Copy the completed frame from the worker thread
            memcpy(m_frameBuffer_mainThread, m_frameBuffers[read_frame_index], m_frameTotalSizeBytes);

            // Update the frame counter for the main thread
            m_frameCounter_mainThread= worker_thread_frame_counter;

            // Pull the frame counter from the worker thread again,
            // in case it changed while we were in the middle of copying the buffer
            worker_thread_frame_counter= static_cast<int>(m_frameCounter);

            ++fetchAttempt;
        }

        return m_frameBuffer_mainThread;
    }

    static void usbBulkTransferCallback_workerThread(unsigned char *packet_data, int packet_length, void *userdata)
    {
        PS3EyeVideoPacketProcessor *processor= reinterpret_cast<PS3EyeVideoPacketProcessor *>(userdata);

        processor->packetScan_workerThread(packet_data, packet_length);
    }

protected:
    void reallocate_mainThread()
    {
        release_mainThread();

        if (m_frameTotalSizeBytes > 0)
        {
            m_frameBuffers[0] = new uint8_t[m_frameTotalSizeBytes];
            m_frameBuffers[1] = new uint8_t[m_frameTotalSizeBytes];
            m_frameBuffer_mainThread= new uint8_t[m_frameTotalSizeBytes];

            memset(m_frameBuffers[0], 0, m_frameTotalSizeBytes);
            memset(m_frameBuffers[1], 0, m_frameTotalSizeBytes);
            memset(m_frameBuffer_mainThread, 0, m_frameTotalSizeBytes);
        }

        // Point the write pointer at the first buffer
        m_frameCounter= 0;
        m_frameCounter_mainThread= 0;
        m_currentFrameStart= m_frameBuffers[0];
    }

    void release_mainThread()
    {
        if (m_frameBuffers[0] != nullptr)
        {
            delete[] m_frameBuffers[0];
            m_frameBuffers[0]= nullptr;
        } 

        if (m_frameBuffers[1] != nullptr)
        {
            delete[] m_frameBuffers[1];
            m_frameBuffers[1]= nullptr;
        } 

        if (m_frameBuffer_mainThread != nullptr)
        {
            delete[] m_frameBuffer_mainThread;
            m_frameBuffer_mainThread= nullptr;
        }
    }

    void packetScan_workerThread(unsigned char *data, int len)
    {
        uint32_t presentationTimeStamp; // a.k.a. "PTS"
        uint16_t fieldID; // a.k.a. "FID"
        int remaining_len = len;
        int payload_len;

        payload_len = 2048; // bulk type
        do {
            len = std::min(remaining_len, payload_len);

            // Payloads are prefixed with a "USB Video Class" (UVC) style header. 
            // We consider a frame to start when the "Field ID Bit" (FID) toggles 
            // or the "Presentation Time Stamp" (PTS) changes.
            // A frame ends when EOF is set, and we've received
            // the correct number of bytes.

            // Verify UVC header.  Header length is always 12 
            if (data[0] != 12 || len < 12) 
            {
                debug("bad header\n");
                goto discard;
            }

            // Check errors
            if (data[1] & UVC_STREAM_ERR) 
            {
                debug("payload error\n");
                goto discard;
            }

            // Extract PTS and FID
            if (!(data[1] & UVC_STREAM_PTS))
            {
                debug("PTS not present\n");
                goto discard;
            }

            presentationTimeStamp = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
            fieldID = (data[1] & UVC_STREAM_FID) ? 1 : 0;

            // If PTS or FID has changed, start a new frame.
            if (presentationTimeStamp != m_lastPresentationTimestamp || fieldID != m_lastFieldID) 
            {
                if (m_last_packet_type == INTER_PACKET)
                {
                    // The last frame was incomplete, so don't keep it or we will glitch
                    frameAddData_workerThread(DISCARD_PACKET, NULL, 0);
                }

                m_lastPresentationTimestamp = presentationTimeStamp;
                m_lastFieldID = fieldID;
                frameAddData_workerThread(FIRST_PACKET, data + 12, len - 12);
            } 
            // If this packet is marked as EOF, end the frame
            else if (data[1] & UVC_STREAM_EOF) 
            {
                m_lastPresentationTimestamp = 0;
                if(m_currentFrameBytesWritten + len - 12 != m_frameTotalSizeBytes)
                {
                    goto discard;
                }
                frameAddData_workerThread(LAST_PACKET, data + 12, len - 12);
            }
            else 
            {
                // Add the data from this payload
                frameAddData_workerThread(INTER_PACKET, data + 12, len - 12);
            }


            // Done this payload
            goto scan_next;

        discard:
            // Discard data until a new frame starts.
            frameAddData_workerThread(DISCARD_PACKET, NULL, 0);

        scan_next:
            remaining_len -= len;
            data += len;
        } while (remaining_len > 0);
    }

    void frameAddData_workerThread(enum gspca_packet_type packet_type, const uint8_t *data, int len)
    {
        if (packet_type == FIRST_PACKET) 
        {
            m_currentFrameBytesWritten = 0;
        } 
        else
        {
            switch(m_last_packet_type)  // ignore warning.
            {
            case DISCARD_PACKET:
                if (packet_type == LAST_PACKET) 
                {
                    m_last_packet_type = packet_type;
                    m_currentFrameBytesWritten = 0;
                }
                return;
            case LAST_PACKET:
                return;
            default:
                break;
            }
        }

        /* append the packet to the frame buffer */
        if (len > 0)
        {
            if(m_currentFrameBytesWritten + len > m_frameTotalSizeBytes)
            {
                packet_type = DISCARD_PACKET;
                m_currentFrameBytesWritten = 0;
            }
            else
            {
                memcpy(m_currentFrameStart+m_currentFrameBytesWritten, data, len);
                m_currentFrameBytesWritten += len;
            }
        }

        m_last_packet_type = packet_type;

        if (packet_type == LAST_PACKET)
        {
            m_currentFrameBytesWritten = 0;
            m_currentFrameStart = frameAdvance_workerThread();
        }
    }

    uint8_t* frameAdvance_workerThread()
    {
        // Atomically increment the write frame counter.
        // The main thread checks this to see if a new frame has been posted.
        m_frameCounter++;

        // Return the write pointer to the start of the appropriate buffer
        return m_frameBuffers[static_cast<int>(m_frameCounter) % 2];
    }

private:
    // Worker Thread Data
    enum gspca_packet_type m_last_packet_type;
    uint32_t m_lastPresentationTimestamp;
    uint16_t m_lastFieldID;
    uint8_t* m_currentFrameStart;
    uint32_t m_currentFrameBytesWritten;

    // Shared Data
    uint8_t *m_frameBuffers[2];
    std::atomic_int m_frameCounter;
    uint32_t m_frameTotalSizeBytes;

    // Main Thread Data
    uint8_t *m_frameBuffer_mainThread;
    int m_frameCounter_mainThread;
};

//-- PS3EyeLibUSBCapture -----
PS3EyeProperties::PS3EyeProperties()
    : autogain(false)
    , gain(20)
    , exposure(120)
    , sharpness(0)
    , hue(143)
    , awb(false)
    , brightness(20)
    , contrast(37)
    , blueBalance(128)
    , redBalance(128)
    , greenBalance(128)
    , flip_h(false)
    , flip_v(false)
    , frame_width(0)
    , frame_height(0)
    , frame_stride(0)
    , frame_rate(0)
{
}

PS3EyeLibUSBCapture::PS3EyeLibUSBCapture(t_usb_device_handle usb_device_handle)
    : m_properties()
    , m_is_streaming(false)
    , m_last_qued_frame_time(0.0)
    , m_usb_device_handle(usb_device_handle)
    , m_video_packet_processor(new PS3EyeVideoPacketProcessor())
    , m_task_queue(new USBAsyncTaskQueue())
{

}

PS3EyeLibUSBCapture::~PS3EyeLibUSBCapture()
{
    stop();
    release();

    delete m_task_queue;
}

bool PS3EyeLibUSBCapture::init(
    unsigned int width, 
    unsigned int height,
    unsigned int desiredFrameRate)
{
    if (m_usb_device_handle != k_invalid_usb_device_handle)
    {
        if (!USBDeviceManager::getInstance()->openUSBDevice(m_usb_device_handle))
        {
            return false;
        }
    }

    if ((width == 0 && height == 0) || width > 320 || height > 240)
    {
        m_properties.frame_width = 640;
        m_properties.frame_height = 480;
    }
    else
    {
        m_properties.frame_width = 320;
        m_properties.frame_height = 240;
    }
    m_properties.frame_stride = m_properties.frame_width * 2; //(YUV Format = 2bytes per pixel)
    m_properties.frame_rate = compute_frame_rate_settings(width, desiredFrameRate);

    // Allocate the video frame buffers 
    m_video_packet_processor->setFrameSize_mainThread(m_properties.frame_stride*m_properties.frame_height);

    // Add an async task to initialize the camera
    {
        const t_usb_device_handle handle= m_usb_device_handle;
        const unsigned int width= m_properties.frame_width;
        const unsigned int height= m_properties.frame_height;
        const unsigned char fps= m_properties.frame_rate;

        m_task_queue->addAsyncTask(
            "init",
            [handle, width, height, fps](async::TaskCallback<int> callback) {
                async_init_camera(handle, width, height, fps, callback);
        });
    }

    return true;
}

void PS3EyeLibUSBCapture::release()
{
    if (m_video_packet_processor != nullptr)
    {
        delete m_video_packet_processor;
        m_video_packet_processor= nullptr;
    }

    if(m_usb_device_handle != k_invalid_usb_device_handle)
    {
        USBDeviceManager::getInstance()->closeUSBDevice(m_usb_device_handle);
    }
}

void PS3EyeLibUSBCapture::start()
{
    if (m_is_streaming) return;

    const t_usb_device_handle device_handle= m_usb_device_handle;
    const PS3EyeProperties &properties= m_properties;
    PS3EyeVideoPacketProcessor* processor= m_video_packet_processor;
    
    // Add an async task to initialize the camera
    m_task_queue->addAsyncTask(
        "start",
        [device_handle, properties, processor](async::TaskCallback<int> callback) {
            async_start_camera(device_handle, properties, processor, callback);
    });

    m_is_streaming = true;
}

void PS3EyeLibUSBCapture::stop()
{
    if(!m_is_streaming) return;

    const t_usb_device_handle device_handle= m_usb_device_handle;

    // Add an async task to initialize the camera
    m_task_queue->addAsyncTask(
        "stop",
        [device_handle](async::TaskCallback<int> callback) {
            async_stop_camera(device_handle, callback);
    });

    m_is_streaming = false;
}

void PS3EyeLibUSBCapture::setAutogain(bool bAutoGain)
{
    const t_usb_device_handle handle= m_usb_device_handle;
    const int currentGain= m_properties.gain;
    const int currentExposure= m_properties.exposure;

    // Cache the new auto-gain flag (the camera will converge to this)
    m_properties.autogain = bAutoGain;

    // Add an async task to set the autogain on the camera
    m_task_queue->addAsyncTask(
        "setAutogain", 
        [handle, bAutoGain, currentGain, currentExposure](async::TaskCallback<int> callback) {
            async_set_autogain(handle, bAutoGain, currentGain, currentExposure, callback);
        });
}

void PS3EyeLibUSBCapture::setAutoWhiteBalance(bool val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new AWB value (the camera will converge to this)
    m_properties.awb = val;

    // Add an async task to set the awb on the camera
    m_task_queue->addAsyncTask(
        "setAutoWhiteBalance", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_auto_white_balance(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setGain(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new gain value (the camera will converge to this)
    m_properties.gain = val;

    // Add an async task to set the gain on the camera
    m_task_queue->addAsyncTask(
        "setGain", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_gain(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setExposure(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new exposure value (the camera will converge to this)
    m_properties.exposure = val;

    // Add an async task to set the exposure on the camera
    m_task_queue->addAsyncTask(
        "setExposure", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_exposure(handle, val, callback);
        });
}


void PS3EyeLibUSBCapture::setSharpness(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new sharpness value (the camera will converge to this)
    m_properties.sharpness = val;

    // Add an async task to set the sharpness on the camera
    m_task_queue->addAsyncTask(
        "setSharpness", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_sharpness(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setContrast(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new contrast value (the camera will converge to this)
    m_properties.contrast = val;

    // Add an async task to set the sharpness on the camera
    m_task_queue->addAsyncTask(
        "setContrast", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_contrast(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setBrightness(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new brightness value (the camera will converge to this)
    m_properties.brightness = val;

    // Add an async task to set the sharpness on the camera
    m_task_queue->addAsyncTask(
        "setBrightness", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_brightness(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setHue(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new hue value (the camera will converge to this)
    m_properties.hue = val;

    // Add an async task to set the sharpness on the camera
    m_task_queue->addAsyncTask(
        "setHue", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_hue(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setRedBalance(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new red balance value (the camera will converge to this)
    m_properties.redBalance = val;

    // Add an async task to set the red balance on the camera
    m_task_queue->addAsyncTask(
        "setRedBalance", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_red_balance(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setGreenBalance(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new green balance value (the camera will converge to this)
    m_properties.greenBalance = val;

    // Add an async task to set the green balance on the camera
    m_task_queue->addAsyncTask(
        "setGreenBalance", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_green_balance(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setBlueBalance(unsigned char val)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new blue balance value (the camera will converge to this)
    m_properties.blueBalance = val;

    // Add an async task to set the red balance on the camera
    m_task_queue->addAsyncTask(
        "setBlueBalance", 
        [handle, val](async::TaskCallback<int> callback) {
            async_set_blue_balance(handle, val, callback);
        });
}

void PS3EyeLibUSBCapture::setFlip(bool horizontal, bool vertical)
{
    const t_usb_device_handle handle= m_usb_device_handle;

    // Cache the new camera flip flags (the camera will converge to this)
    m_properties.flip_h= horizontal;
    m_properties.flip_v= vertical;

    // Add an async task to set the red balance on the camera
    m_task_queue->addAsyncTask(
        "setFlip", 
        [handle, horizontal, vertical](async::TaskCallback<int> callback) {
            async_set_flip(handle, horizontal, vertical, callback);
        });
}

bool PS3EyeLibUSBCapture::getUSBPortPath(char *out_identifier, size_t max_identifier_length) const
{
    return USBDeviceManager::getInstance()->getUSBDevicePortPath(m_usb_device_handle, out_identifier, max_identifier_length);   
}

const unsigned char* PS3EyeLibUSBCapture::getFrame()
{
    return m_video_packet_processor->getFrameBuffer_mainThread();
}

//-- private helpers ----
static void async_init_camera(
    t_usb_device_handle device_handle,
    uint32_t width,
    uint32_t height,
    uint8_t frameRate,
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        uint8_t sensor_id;

        TaskChainState() : sensor_id(0) {}
    };
    TaskChainState *taskChainState = new TaskChainState;

    async::TaskVector<int> tasks{
        // reset bridge
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0xe7, 0x3a, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0xe0, 0x08, taskDoneCallback);
        },
        [](async::TaskCallback<int> taskDoneCallback) {
            ServerUtility::sleep_ms(10);
            taskDoneCallback(async::OK, 0);
        },
        // initialize the sensor address
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_ADDRESS, 0x42, taskDoneCallback);
        },
        // reset sensor
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x12, 0x80, taskDoneCallback);
        },
        [](async::TaskCallback<int> taskDoneCallback) {
            ServerUtility::sleep_ms(10);
            taskDoneCallback(async::OK, 0);
        },
        // probe the sensor
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_read(device_handle, 0x0a, taskDoneCallback);
        },
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_read(device_handle, 0x0a,
                [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                taskChainState->sensor_id = result << 8;
                taskDoneCallback(error, result);
            });
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_read(device_handle, 0x0b, taskDoneCallback);
        },
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_read(device_handle, 0x0b,
                [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                    taskChainState->sensor_id |= result;
                    SERVER_LOG_DEBUG("async_init_camera") << "PS3EYE Sensor ID: " << taskChainState->sensor_id;
                    taskDoneCallback(error, result);
            });
        },
        // initialize 
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write_array(device_handle, ov534_reg_initdata, ARRAY_SIZE(ov534_reg_initdata), taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_set_led(device_handle, true, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write_array(device_handle, ov772x_reg_initdata, ARRAY_SIZE(ov772x_reg_initdata), taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0xe0, 0x09, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_set_led(device_handle, false, taskDoneCallback);
        }
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [taskChainState, outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
    {
        int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

        delete taskChainState;
        outCallback(outErrorCode, outResult);
    });
}

static void async_start_camera(
    const t_usb_device_handle device_handle, 
    const PS3EyeProperties &properties, 
    PS3EyeVideoPacketProcessor* processor, 
    async::TaskCallback<int> outCallback)
{
    const uint32_t frame_width = properties.frame_width;
    const uint8_t frame_rate = properties.frame_rate;
    const bool bAutoGain = properties.autogain;
    const bool bAWB = properties.awb;
    const uint8_t gain = properties.gain;
    const uint8_t hue = properties.hue;
    const uint8_t exposure = properties.exposure;
    const uint8_t brightness = properties.brightness;
    const uint8_t contrast = properties.contrast;
    const uint8_t sharpness = properties.sharpness;
    const uint8_t redBalance = properties.redBalance;
    const uint8_t greenBalance = properties.greenBalance;
    const uint8_t blueBalance = properties.blueBalance;
    const bool bFlipH = properties.flip_h;
    const bool bFlipV = properties.flip_v;

    async::TaskVector<int> tasks{
        [device_handle, frame_width](async::TaskCallback<int> taskDoneCallback) {
            if (frame_width == 320) // 320x240
                async_ov534_reg_write_array(device_handle, bridge_start_qvga, ARRAY_SIZE(bridge_start_qvga), taskDoneCallback);
            else // 640x480 
                async_ov534_reg_write_array(device_handle, bridge_start_qvga, ARRAY_SIZE(bridge_start_qvga), taskDoneCallback);
        },
        [device_handle, frame_width](async::TaskCallback<int> taskDoneCallback) {
            if (frame_width == 320) // 320x240
                async_sccb_reg_write_array(device_handle, sensor_start_qvga, ARRAY_SIZE(sensor_start_qvga), taskDoneCallback);
            else // 640x480
                async_sccb_reg_write_array(device_handle, sensor_start_vga, ARRAY_SIZE(sensor_start_vga), taskDoneCallback);
        },
        [device_handle, frame_width, frame_rate](async::TaskCallback<int> taskDoneCallback) {
            async_set_frame_rate(device_handle, frame_width, frame_rate, taskDoneCallback);
        },
        [device_handle, bAutoGain, gain, exposure](async::TaskCallback<int> taskDoneCallback) {
            async_set_autogain(device_handle, bAutoGain, gain, exposure, taskDoneCallback);
        },
        [device_handle, bAWB](async::TaskCallback<int> taskDoneCallback) {
            async_set_auto_white_balance(device_handle, bAWB, taskDoneCallback);
        },
        [device_handle, gain](async::TaskCallback<int> taskDoneCallback) {
            async_set_gain(device_handle, gain, taskDoneCallback);
        },
        [device_handle, hue](async::TaskCallback<int> taskDoneCallback) {
            async_set_hue(device_handle, hue, taskDoneCallback);
        },
        [device_handle, exposure](async::TaskCallback<int> taskDoneCallback) {
            async_set_exposure(device_handle, exposure, taskDoneCallback);
        },
        [device_handle, brightness](async::TaskCallback<int> taskDoneCallback) {
            async_set_brightness(device_handle, brightness, taskDoneCallback);
        },
        [device_handle, contrast](async::TaskCallback<int> taskDoneCallback) {
            async_set_contrast(device_handle, contrast, taskDoneCallback);
        },
        [device_handle, sharpness](async::TaskCallback<int> taskDoneCallback) {
            async_set_sharpness(device_handle, sharpness, taskDoneCallback);
        },
        [device_handle, redBalance](async::TaskCallback<int> taskDoneCallback) {
            async_set_red_balance(device_handle, redBalance, taskDoneCallback);
        },
        [device_handle, blueBalance](async::TaskCallback<int> taskDoneCallback) {
            async_set_blue_balance(device_handle, blueBalance, taskDoneCallback);
        },
        [device_handle, greenBalance](async::TaskCallback<int> taskDoneCallback) {
            async_set_green_balance(device_handle, greenBalance, taskDoneCallback);
        },
        [device_handle, bFlipH, bFlipV](async::TaskCallback<int> taskDoneCallback) {
            async_set_flip(device_handle, bFlipH, bFlipV, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_set_led(device_handle, true, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0xe0, 0x00, taskDoneCallback); // start stream
        },
        [device_handle, processor](async::TaskCallback<int> taskDoneCallback) {
            USBTransferRequest request;
            memset(&request, 0, sizeof(USBTransferRequest));
            request.request_type= _USBRequestType_StartBulkTransfer;
            request.payload.start_bulk_transfer.usb_device_handle= device_handle;
            request.payload.start_bulk_transfer.bAutoResubmit= true;
            request.payload.start_bulk_transfer.in_flight_transfer_packet_count= NUM_TRANSFERS;
            request.payload.start_bulk_transfer.transfer_packet_size= TRANSFER_SIZE;
            request.payload.start_bulk_transfer.on_data_callback= PS3EyeVideoPacketProcessor::usbBulkTransferCallback_workerThread;
            request.payload.start_bulk_transfer.transfer_callback_userdata= processor;

            // Send a request to the USBDeviceManager to start a bulk transfer stream for video frames
            USBDeviceManager::getInstance()->submitTransferRequest(
                request, 
                [taskDoneCallback](USBTransferResult &result) {
                    assert(result.result_type == _USBResultType_BulkTransfer);
                    log_usb_result_code("async_start_camera", result.payload.bulk_transfer.result_code); 
                    taskDoneCallback(async::OK, result.payload.bulk_transfer.result_code);
                });
        }
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
    {
        int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

        outCallback(outErrorCode, outResult);
    });
}

static void async_stop_camera(
    const t_usb_device_handle device_handle,
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks{
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0xe0, 0x09, taskDoneCallback); // stop stream
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_set_led(device_handle, false, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            USBTransferRequest request;
            memset(&request, 0, sizeof(USBTransferRequest));
            request.request_type= _USBRequestType_CancelBulkTransfer;
            request.payload.cancel_bulk_transfer.usb_device_handle= device_handle;

            // Send a request to the USBDeviceManager to stop the bulk transfer stream for video frames
            USBDeviceManager::getInstance()->submitTransferRequest(
                request, 
                [taskDoneCallback](USBTransferResult &result) {
                    assert(result.result_type == _USBResultType_BulkTransfer);
                    log_usb_result_code("async_stop_camera", result.payload.bulk_transfer.result_code);
                    taskDoneCallback(async::OK, result.payload.bulk_transfer.result_code);
                });
        }
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
    {
        int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

        outCallback(outErrorCode, outResult);
    });
}

static void async_set_autogain(
    t_usb_device_handle device_handle, 
    bool bAutoGain, 
    uint8_t gain,
    uint8_t exposure,
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks;

    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        uint8_t read_reg_0x64_result;

        TaskChainState() : read_reg_0x64_result(0) {}
    };
    TaskChainState *taskChainState = new TaskChainState;

    if (bAutoGain) 
    {
        tasks.push_back(
            [device_handle](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_write(device_handle, 0x13, 0xf7, taskDoneCallback); //AGC,AEC,AWB ON
            });
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_read(device_handle, 0x64, 
                    [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                        taskChainState->read_reg_0x64_result= result;
                        taskDoneCallback(error, result);
                    });
            });
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_write(device_handle, 0x64, taskChainState->read_reg_0x64_result | 0x03, taskDoneCallback);
            });
    }
    else 
    {
        tasks.push_back(
            [device_handle](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_write(device_handle, 0x13, 0xf0, taskDoneCallback); //AGC,AEC,AWB OFF
            });
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_read(device_handle, 0x64, 
                    [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                        taskChainState->read_reg_0x64_result= result;
                        taskDoneCallback(error, result);
                    });
            });
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_sccb_reg_write(device_handle, 0x64, taskChainState->read_reg_0x64_result & 0xFC, taskDoneCallback);
            });
        tasks.push_back(
            [device_handle, gain](async::TaskCallback<int> taskDoneCallback) {
                async_set_gain(device_handle, gain, taskDoneCallback);
            });
        tasks.push_back(
            [device_handle, exposure](async::TaskCallback<int> taskDoneCallback) {
                async_set_exposure(device_handle, exposure, taskDoneCallback);
            });
    }

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [taskChainState, outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
    {
        int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

        delete taskChainState;
        outCallback(outErrorCode, outResult);
    });
}

static void async_set_auto_white_balance(
    t_usb_device_handle device_handle, 
    bool bAutoWhiteBalance, 
    async::TaskCallback<int> outCallback)
{
    if (bAutoWhiteBalance)
    {
        async_sccb_reg_write(device_handle, 0x63, 0xe0, outCallback); //AWB ON
    }
    else
    {
        async_sccb_reg_write(device_handle, 0x63, 0xAA, outCallback); //AWB OFF
    }
}

static void async_set_gain(
    t_usb_device_handle device_handle, 
    unsigned char val, 
    async::TaskCallback<int> outCallback)
{
    switch (val & 0x30)
    {
    case 0x00:
        val &= 0x0F;
        break;
    case 0x10:
        val &= 0x0F;
        val |= 0x30;
        break;
    case 0x20:
        val &= 0x0F;
        val |= 0x70;
        break;
    case 0x30:
        val &= 0x0F;
        val |= 0xF0;
        break;
    }

    async_sccb_reg_write(device_handle, 0x00, val, outCallback);
}

static void async_set_exposure(
    t_usb_device_handle device_handle, 
    unsigned char val, 
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks {
        [device_handle, val](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x08, val >> 7, taskDoneCallback);
        },
        [device_handle, val](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x10, val << 1, taskDoneCallback);
        },
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            outCallback(outErrorCode, outResult);
        });
}

static void async_set_sharpness(
    t_usb_device_handle device_handle,
    unsigned char val, 
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks {
        [device_handle, val](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x91, val, taskDoneCallback);
        },
        [device_handle, val](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x8E, val, taskDoneCallback);
        },
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            outCallback(outErrorCode, outResult);
        });
}

static void async_set_contrast(
    t_usb_device_handle device_handle, 
    unsigned char val, 
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x9C, val, outCallback);
}

static void async_set_brightness(
    t_usb_device_handle device_handle, 
    unsigned char val, 
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x9B, val, outCallback);
}

static void async_set_hue(
    t_usb_device_handle device_handle,
    unsigned char val,
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x01, val, outCallback);
}

static void async_set_red_balance(
    t_usb_device_handle device_handle,
    unsigned char val,
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x43, val, outCallback);
}

static void async_set_green_balance(
    t_usb_device_handle device_handle,
    unsigned char val,
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x44, val, outCallback);
}

static void async_set_blue_balance(
    t_usb_device_handle device_handle,
    unsigned char val,
    async::TaskCallback<int> outCallback)
{
    async_sccb_reg_write(device_handle, 0x42, val, outCallback);
}

static void async_set_flip(
    t_usb_device_handle device_handle, 
    bool horizontal, 
    bool vertical, 
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        uint8_t read_reg_0x0C_result;

        TaskChainState() : read_reg_0x0C_result(0) {}
    };
    TaskChainState *taskChainState = new TaskChainState;

    async::TaskVector<int> tasks {
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_read(device_handle, 0x0C, 
                [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                    taskChainState->read_reg_0x0C_result= result & ~0xC0;
                    taskDoneCallback(error, result);
                });
        },
        [device_handle, taskChainState, horizontal, vertical](async::TaskCallback<int> taskDoneCallback) {
            uint8_t val= taskChainState->read_reg_0x0C_result;
            if (!horizontal) val |= 0x40;
            if (!vertical) val |= 0x80;
            async_sccb_reg_write(device_handle, 0x0C, val, taskDoneCallback);
        }
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [taskChainState, outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            delete taskChainState;
            outCallback(outErrorCode, outResult);
        });
}

static void async_set_frame_rate(
    t_usb_device_handle device_handle, 
    uint32_t frame_width,
    uint8_t frame_rate, 
    async::TaskCallback<int> outCallback)
{
    uint8_t r11, r0d, re5;
    compute_frame_rate_settings(frame_width, frame_rate, &r11, &r0d, &re5);

    async::TaskVector<int> tasks {
        [device_handle, r11](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x11, r11, taskDoneCallback);
        },
        [device_handle, r0d](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_reg_write(device_handle, 0x0d, r0d, taskDoneCallback);
        },
        [device_handle, re5](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0xe5, re5, taskDoneCallback);
        }
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks, 
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            outCallback(outErrorCode, outResult);
        });
}

static void async_sccb_reg_write(
    t_usb_device_handle device_handle, 
    uint8_t reg, 
    uint8_t val, 
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks {
        [device_handle, reg](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_SUBADDR, reg, taskDoneCallback);
        },
        [device_handle, val](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_WRITE, val, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_WRITE_3, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_check_status(device_handle, [taskDoneCallback](async::ErrorCode statusCheckErrorCode, int statusCheckResult)
            {
                taskDoneCallback((statusCheckResult == 1) ? async::OK : async::FAIL, statusCheckResult);
            });
        },
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks, 
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            outCallback(outErrorCode, outResult);
        });
}

static void async_sccb_reg_write_array(
    t_usb_device_handle device_handle, 
    const uint8_t (*sequence)[2], 
    int sequenceLength, 
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        const uint8_t (*data)[2];
        int len;

        TaskChainState(const uint8_t (*sequence)[2], int sequenceLength)
            : data(sequence)
            , len(sequenceLength)
        {}
    };
    TaskChainState *taskChainState = new TaskChainState(sequence, sequenceLength);

    // Loop-Test function
    // Keep running Work function while Loop-Test return true
    auto loop_test = [taskChainState]()
    {
        --taskChainState->len;

        return taskChainState->len >= 0;
    };

    // Work function
    // This gets run each iteration
    auto work_func = [device_handle, taskChainState](std::function<void(async::ErrorCode error)> work_done_callback)
    {
        if ((*taskChainState->data)[0] != 0xff) 
        {
            // Write to the register...
            async_sccb_reg_write(
                device_handle,
                (*taskChainState->data)[0], // register
                (*taskChainState->data)[1], // value
                [taskChainState, work_done_callback](async::ErrorCode errorCode, int result)
                {
                    //... then increment the sequence pointer
                    taskChainState->data++;

                    // Move on to the next iteration
                    work_done_callback(errorCode);
                });
        }
        else
        {
            // Read the register first..
            async_sccb_reg_read(
                device_handle,
                (*taskChainState->data)[1], // register
                [device_handle, taskChainState, work_done_callback](async::ErrorCode errorCode, int result) 
                {
                    // ... then write 0x00 to reg 0xff ...
                    async_sccb_reg_write(
                        device_handle, 0xff, 0x00, 
                        [taskChainState, work_done_callback](async::ErrorCode errorCode, int result) 
                        {
                            // ... then increment the sequence pointer
                            taskChainState->data++;

                            // Move on to the next iteration
                            work_done_callback(errorCode);
                        });
                });
        }
    };

    // Final-Callback function
    // This gets run after the Loop-Test function returns false
    auto final_callback = [taskChainState, outCallback](async::ErrorCode errorCode)
    {
        delete taskChainState;
        outCallback(errorCode, 0);
    };

    // Write out the (reg - val) pair array
    async::whilst(loop_test, work_func, final_callback);
}

static void async_sccb_reg_read(
    t_usb_device_handle device_handle, 
    uint16_t reg, 
    async::TaskCallback<int> outCallback)
{
    async::TaskVector<int> tasks{
        [device_handle, reg](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_SUBADDR, (uint8_t)reg, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_WRITE_2, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_check_status(device_handle, [taskDoneCallback](async::ErrorCode statusCheckErrorCode, int statusCheckResult)
            {
                taskDoneCallback((statusCheckResult == 1) ? async::OK : async::FAIL, statusCheckResult);
            });
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_READ_2, taskDoneCallback);
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_sccb_check_status(device_handle, [taskDoneCallback](async::ErrorCode statusCheckErrorCode, int statusCheckResult)
            {
                taskDoneCallback((statusCheckResult == 1) ? async::OK : async::FAIL, statusCheckResult);
            });
        },
        [device_handle](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_read(device_handle, OV534_REG_READ, taskDoneCallback);
        },
    };

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            // NOTE: If the task chain succeeded, 
            // the final result value will be the value read from camera register
            // via the ov534_reg_read task.
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            outCallback(outErrorCode, outResult);
        });
}

static void async_sccb_check_status(
    t_usb_device_handle device_handle,
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        int sccbStatusResult;
        bool bQuerySuccess;
        int queryAttempCount;

        TaskChainState()
            : sccbStatusResult(0)
            , bQuerySuccess(false)
            , queryAttempCount(0)
        {}
    };
    TaskChainState *taskChainState = new TaskChainState();

    // Loop-Test function
    // Keep running Work function while Loop-Test return true
    auto loop_test = [taskChainState]()
    {
        bool keep_going = !taskChainState->bQuerySuccess && taskChainState->queryAttempCount < 5;
        taskChainState->queryAttempCount++;
        return keep_going;
    };

    // Work function
    // This gets run each iteration
    auto work_func = [device_handle, taskChainState](
        std::function<void(async::ErrorCode error)> work_done_callback)
    {
        async_ov534_reg_read(
            device_handle,
            OV534_REG_STATUS,
            [taskChainState, work_done_callback](async::ErrorCode errorCode, int data)
            {
                if (errorCode == async::OK)
                {
                    taskChainState->bQuerySuccess = true;

                    switch (data)
                    {
                    case 0x00:
                        taskChainState->sccbStatusResult = 1;
                    case 0x04:
                        taskChainState->sccbStatusResult = 0;
                    case 0x03:
                        break;
                    default:
                        SERVER_LOG_WARNING("sccb_check_status") << "unknown sccb status 0x"
                            << std::hex << std::setfill('0') << std::setw(2) << data;
                    }
                }

                work_done_callback(async::OK);
            });
    };

    // Final-Callback function
    // This gets run after the Loop-Test function returns false
    auto final_callback = [taskChainState, outCallback](async::ErrorCode errorCode)
    {        
        // Send the final query result
        outCallback(errorCode, taskChainState->sccbStatusResult);
        delete taskChainState;
    };

    // Try to poll status up to 5 time max from the OV534_REG_STATUS register
    async::whilst(loop_test, work_func, final_callback);
}

static void async_ov534_reg_write(
    t_usb_device_handle device_handle, 
    uint16_t reg, 
    uint8_t val, 
    async::TaskCallback<int> outCallback)
{
    USBTransferRequest request;
    memset(&request, 0, sizeof(USBTransferRequest));
    request.request_type= _USBRequestType_ControlTransfer;
    request.payload.control_transfer.usb_device_handle= device_handle;
    request.payload.control_transfer.bmRequestType = 
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    request.payload.control_transfer.bRequest= 0x01;
    request.payload.control_transfer.wValue= 0x00;
    request.payload.control_transfer.wIndex = reg;
    request.payload.control_transfer.data[0]= val;
    request.payload.control_transfer.wLength= 1;
    request.payload.control_transfer.timeout= 500;

    // Submit the async usb control transfer request...
    USBDeviceManager::getInstance()->submitTransferRequest(
        request, 
        [outCallback](USBTransferResult &result) 
        {
            assert(result.result_type == _USBResultType_ControlTransfer);

            //... whose result we get notified of here
            if (result.payload.control_transfer.result_code == _USBResultCode_Completed)
            {
                // Tell the callback the transfer completed ok :)
                // The callback parameter value will be _USBResultCode_Completed
                outCallback(async::OK, result.payload.control_transfer.result_code);
            }
            else
            {
                // Tell the callback the transfer failed :(
                // The callback parameter value will be the error code
                log_usb_result_code("ov534_reg_write", result.payload.control_transfer.result_code);
                outCallback(async::FAIL, result.payload.control_transfer.result_code);
            }
        }
    );
}

static void async_ov534_reg_write_array(
    t_usb_device_handle device_handle,
    const uint8_t (*sequence)[2], 
    int sequenceLength,
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        const uint8_t (*data)[2];
        int len;

        TaskChainState(const uint8_t (*sequence)[2], int sequenceLength)
            : data(sequence)
            , len(sequenceLength)
        {}
    };
    TaskChainState *taskChainState = new TaskChainState(sequence, sequenceLength);

    // Loop-Test function
    // Keep running Work function while Loop-Test return true
    auto loop_test = [taskChainState]()
    {
        --taskChainState->len;
        return taskChainState->len >= 0;
    };

    // Work function
    // This gets run each iteration
    auto work_func = [device_handle, taskChainState](
        std::function<void(async::ErrorCode error)> work_done_callback)
    {
        async_ov534_reg_write(
            device_handle,
            (*taskChainState->data)[0], // register
            (*taskChainState->data)[1], // value
            [taskChainState, work_done_callback](async::ErrorCode errorCode, int result)
            {
                taskChainState->data++;
                work_done_callback(errorCode);
            });
    };

    // Final-Callback function
    // This gets run after the Loop-Test function returns false
    auto final_callback = [taskChainState, outCallback](async::ErrorCode errorCode)
    {        
        outCallback(errorCode, 0);
        delete taskChainState;
    };

    // Write out the (reg - val) pair array
    async::whilst(loop_test, work_func, final_callback);
}

static void async_ov534_reg_read(
    t_usb_device_handle device_handle, 
    uint16_t reg, 
    async::TaskCallback<int> outCallback)
{
    USBTransferRequest request;
    memset(&request, 0, sizeof(USBTransferRequest));
    request.request_type = _USBRequestType_ControlTransfer;
    request.payload.control_transfer.usb_device_handle= device_handle;
    request.payload.control_transfer.bmRequestType =
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    request.payload.control_transfer.bRequest = 0x01;
    request.payload.control_transfer.wValue = 0x00;
    request.payload.control_transfer.wIndex = reg;
    request.payload.control_transfer.wLength = 1;
    request.payload.control_transfer.timeout = 500;

    // Submit the async usb control transfer request...
    USBDeviceManager::getInstance()->submitTransferRequest(
        request,
        [outCallback](USBTransferResult &result)
        {
            assert(result.result_type == _USBResultType_ControlTransfer);

            //... whose result we get notified of here
            if (result.payload.control_transfer.result_code == _USBResultCode_Completed)
            {
                assert(result.payload.control_transfer.dataLength == 1);

                // Tell the callback the transfer completed ok :)
                // The callback parameter value will value we read from the camera register.
                outCallback(async::OK, static_cast<int>(result.payload.control_transfer.data[0]));
            }
            else
            {
                // Tell the callback the transfer failed :(
                log_usb_result_code("ov534_reg_read", result.payload.control_transfer.result_code);
                outCallback(async::FAIL, result.payload.control_transfer.result_code);
            }
        }
    );
}

/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
static void async_ov534_set_led(
    t_usb_device_handle device_handle,
    bool bLedOn, 
    async::TaskCallback<int> outCallback)
{
    // Side-Effects valid for the duration of the task chain
    struct TaskChainState
    {
        uint8_t read_reg_result;

        TaskChainState(): read_reg_result(0)
        {}
    };
    TaskChainState *taskChainState = new TaskChainState();

    async::TaskVector<int> tasks {
        // Change register value 0x21
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_read(
                device_handle, 0x21, 
                [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                    taskChainState->read_reg_result= result | 0x80;
                    taskDoneCallback(error, result);
                });
        },
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0x21, taskChainState->read_reg_result, taskDoneCallback);
        },
        // Change register value 0x23
        [device_handle, taskChainState, bLedOn](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_read(
                device_handle, 0x23, 
                [taskChainState, taskDoneCallback, bLedOn](async::ErrorCode error, int result) {
                    if (bLedOn)
                        taskChainState->read_reg_result= result | 0x80;
                    else
                        taskChainState->read_reg_result= result & ~0x80;
                    taskDoneCallback(error, result);
                });
        },
        [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
            async_ov534_reg_write(device_handle, 0x23, taskChainState->read_reg_result, taskDoneCallback);
        },
    };

    if (!bLedOn)
    {
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_ov534_reg_read(
                    device_handle, 0x21, 
                    [taskChainState, taskDoneCallback](async::ErrorCode error, int result) {
                        taskChainState->read_reg_result= result & ~0x80;
                        taskDoneCallback(error, result);
                    });
            });
        tasks.push_back(
            [device_handle, taskChainState](async::TaskCallback<int> taskDoneCallback) {
                async_ov534_reg_write(device_handle, 0x21, taskChainState->read_reg_result, taskDoneCallback);
            });
    }

    // Evaluate tasks in order
    // Pass outCallback the final error code and result of the task chain
    async::series<int>(
        tasks,
        [taskChainState, outCallback](async::ErrorCode outErrorCode, std::vector<int> results)
        {
            // NOTE: If the task chain succeeded, 
            // the final result value will be the value read from camera register
            // via the ov534_reg_read task.
            int outResult = (results.size() > 0) ? results[results.size() - 1] : 0;

            delete taskChainState;
            outCallback(outErrorCode, outResult);
        });
}

static uint8_t compute_frame_rate_settings(
    const uint32_t frame_width,
    const uint8_t desired_frame_rate,
    uint8_t *out_r11,
    uint8_t *out_r0d,
    uint8_t *out_re5)
{
    struct rate_s {
        uint8_t fps;
        uint8_t r11;
        uint8_t r0d;
        uint8_t re5;
    };
    static const struct rate_s rate_0[] = { /* 640x480 */
        { 60, 0x01, 0xc1, 0x04 },
        { 50, 0x01, 0x41, 0x02 },
        { 40, 0x02, 0xc1, 0x04 },
        { 30, 0x04, 0x81, 0x02 },
        { 15, 0x03, 0x41, 0x04 },
    };
    static const struct rate_s rate_1[] = { /* 320x240 */
        { 205, 0x01, 0xc1, 0x02 }, /* 205 FPS: video is partly corrupt */
        { 187, 0x01, 0x81, 0x02 }, /* 187 FPS or below: video is valid */
        { 150, 0x01, 0xc1, 0x04 },
        { 137, 0x02, 0xc1, 0x02 },
        { 125, 0x02, 0x81, 0x02 },
        { 100, 0x02, 0xc1, 0x04 },
        { 75, 0x03, 0xc1, 0x04 },
        { 60, 0x04, 0xc1, 0x04 },
        { 50, 0x02, 0x41, 0x04 },
        { 37, 0x03, 0x41, 0x04 },
        { 30, 0x04, 0x41, 0x04 },
    };

    int rate_index;
    const struct rate_s *rate = nullptr;

    if (frame_width == 640) {
        rate = rate_0;
        rate_index = ARRAY_SIZE(rate_0);
    }
    else {
        rate = rate_1;
        rate_index = ARRAY_SIZE(rate_1);
    }
    while (--rate_index > 0) {
        if (desired_frame_rate >= rate->fps)
            break;
        rate++;
    }

    if (out_r11 != nullptr)
    {
        *out_r11 = rate->r11;
    }

    if (out_r0d != nullptr)
    {
        *out_r0d = rate->r0d;
    }

    if (out_re5 != nullptr)
    {
        *out_re5 = rate->re5;
    }

    return rate->fps;
}

static void log_usb_result_code(const char *function_name, eUSBResultCode result_code)
{
    switch (result_code)
    {
    // Success Codes
    case eUSBResultCode::_USBResultCode_Started:
        SERVER_LOG_INFO(function_name) << "request started";
        break;
    case eUSBResultCode::_USBResultCode_Canceled:
        SERVER_LOG_INFO(function_name) << "request canceled";
        break;
    case eUSBResultCode::_USBResultCode_Completed:
        SERVER_LOG_INFO(function_name) << "request completed";
        break;

    // Failure Codes
    case eUSBResultCode::_USBResultCode_GeneralError:
        SERVER_LOG_ERROR(function_name) << "request failed: general request error";
        break;
    case eUSBResultCode::_USBResultCode_BadHandle:
        SERVER_LOG_ERROR(function_name) << "request failed: bad USB device handle";
        break;
    case eUSBResultCode::_USBResultCode_NoMemory:
        SERVER_LOG_ERROR(function_name) << "request failed: no memory";
        break;
    case eUSBResultCode::_USBResultCode_SubmitFailed:
        SERVER_LOG_ERROR(function_name) << "request failed: submit failed";
        break;
    case eUSBResultCode::_USBResultCode_DeviceNotOpen:
        SERVER_LOG_ERROR(function_name) << "request failed: device not open";
        break;
    case eUSBResultCode::_USBResultCode_TransferNotActive:
        SERVER_LOG_ERROR(function_name) << "request failed: transfer not active";
        break;
    case eUSBResultCode::_USBResultCode_TransferAlreadyStarted:
        SERVER_LOG_ERROR(function_name) << "request failed: transfer already started";
        break;
    case eUSBResultCode::_USBResultCode_Overflow:
        SERVER_LOG_ERROR(function_name) << "request failed: transfer overflow";
        break;
    case eUSBResultCode::_USBResultCode_Pipe:
        SERVER_LOG_ERROR(function_name) << "request failed: transfer pipe error";
        break;
    case eUSBResultCode::_USBResultCode_TimedOut:
        SERVER_LOG_ERROR(function_name) << "request failed: transfer timed out";
        break;
    };
}