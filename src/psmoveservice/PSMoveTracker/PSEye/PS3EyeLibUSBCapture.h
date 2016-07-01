#ifndef PS3EYE_LIBUSB_CAPTURE_H
#define PS3EYE_LIBUSB_CAPTURE_H

//-- includes -----
#include "USBDeviceInfo.h"

//-- definitions -----
class PS3EyeLibUSBCapture
{
public:
    PS3EyeLibUSBCapture(t_usb_device_handle usb_device_handle);
    virtual ~PS3EyeLibUSBCapture();

    bool init(unsigned int width = 0, unsigned int height = 0, unsigned int desiredFrameRate = 30);
    void start();
    void stop();

    // Camera Control Setters
    // Used in when we don't care about an async task result
    void setAutogain(bool val);
    void setAutoWhiteBalance(bool val);
    void setGain(unsigned char val);
    void setExposure(unsigned char val);
    void setSharpness(unsigned char val);
    void setContrast(unsigned char val);
    void setBrightness(unsigned char val);
    void setHue(unsigned char val);
    void setRedBalance(unsigned char val);
    void setGreenBalance(unsigned char val);
    void setBlueBalance(unsigned char val);
    void setFlip(bool horizontal, bool vertical);

    // Camera Control Accessors
    inline bool getAutogain() const { return m_autogain; }
    inline bool getAutoWhiteBalance() const { return m_awb; }
    inline unsigned char getGain() const { return m_gain; }
    inline unsigned char getExposure() const { return m_exposure; }
    inline unsigned char getSharpness() const { return m_sharpness; }
    inline unsigned char getContrast() const { return m_contrast; }
    inline unsigned char getBrightness() const { return m_brightness; }
    inline unsigned char getHue() const { return m_hue; }
    inline unsigned char getRedBalance() const { return m_redBalance; }
    inline unsigned char getBlueBalance() const { return m_blueBalance; }
    inline unsigned char getGreenBalance() const { return m_greenBalance; }
    inline bool getFlipH() const { return m_flip_h; }
    inline bool getFlipV() const { return m_flip_v; }

    // Camera Property Accessors
    inline bool isStreaming() const { return m_is_streaming; }
    inline unsigned int getWidth() const { return m_frame_width; }
    inline unsigned int getHeight() const { return m_frame_height; }
    inline unsigned char getFrameRate() const { return m_frame_rate; }
    inline unsigned int getRowBytes() const { return m_frame_stride; }

    // Get the USB Bus/Port path that this camera is connected to
    bool getUSBPortPath(char *out_identifier, size_t max_identifier_length) const;

    // Get a frame from the camera. Notes:
    // - If there is no frame available, this function will block until one is
    // - The returned frame is a malloc'd copy; you must free() it yourself when done with it
    unsigned char* getFrame();

private:
    PS3EyeLibUSBCapture(const PS3EyeLibUSBCapture&);
    void operator=(const PS3EyeLibUSBCapture&);

    void release();

    // controls
    bool m_autogain;
    unsigned char m_gain; // 0 <-> 63
    unsigned char m_exposure; // 0 <-> 255
    unsigned char m_sharpness; // 0 <-> 63
    unsigned char m_hue; // 0 <-> 255
    bool m_awb;
    unsigned char m_brightness; // 0 <-> 255
    unsigned char m_contrast; // 0 <-> 255
    unsigned char m_blueBalance; // 0 <-> 255
    unsigned char m_redBalance; // 0 <-> 255
    unsigned char m_greenBalance; // 0 <-> 255
    bool m_flip_h;
    bool m_flip_v;

    bool m_is_streaming;
    unsigned int m_frame_width;
    unsigned int m_frame_height;
    unsigned int m_frame_stride;
    unsigned char m_frame_rate;

    double m_last_qued_frame_time;

    // usb stuff
    t_usb_device_handle m_usb_device_handle;
    unsigned char *m_libusb_thread_frame_buffers[2];
    unsigned char *m_main_thread_frame_buffer;
    class USBAsyncTaskQueue *m_task_queue;
};

#endif
