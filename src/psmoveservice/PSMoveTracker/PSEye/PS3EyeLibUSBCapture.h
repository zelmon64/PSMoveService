#ifndef PS3EYE_LIBUSB_CAPTURE_H
#define PS3EYE_LIBUSB_CAPTURE_H

//-- includes -----
#include "USBDeviceInfo.h"

//-- definitions -----
struct PS3EyeProperties
{
    bool autogain;
    unsigned char gain; // 0 <-> 63
    unsigned char exposure; // 0 <-> 255
    unsigned char sharpness; // 0 <-> 63
    unsigned char hue; // 0 <-> 255
    bool awb;
    unsigned char brightness; // 0 <-> 255
    unsigned char contrast; // 0 <-> 255
    unsigned char blueBalance; // 0 <-> 255
    unsigned char redBalance; // 0 <-> 255
    unsigned char greenBalance; // 0 <-> 255
    bool flip_h;
    bool flip_v;

    unsigned int frame_width;
    unsigned int frame_height;
    unsigned int frame_stride;
    unsigned char frame_rate;

    PS3EyeProperties();
};

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
    inline bool getAutogain() const { return m_properties.autogain; }
    inline bool getAutoWhiteBalance() const { return m_properties.awb; }
    inline unsigned char getGain() const { return m_properties.gain; }
    inline unsigned char getExposure() const { return m_properties.exposure; }
    inline unsigned char getSharpness() const { return m_properties.sharpness; }
    inline unsigned char getContrast() const { return m_properties.contrast; }
    inline unsigned char getBrightness() const { return m_properties.brightness; }
    inline unsigned char getHue() const { return m_properties.hue; }
    inline unsigned char getRedBalance() const { return m_properties.redBalance; }
    inline unsigned char getBlueBalance() const { return m_properties.blueBalance; }
    inline unsigned char getGreenBalance() const { return m_properties.greenBalance; }
    inline bool getFlipH() const { return m_properties.flip_h; }
    inline bool getFlipV() const { return m_properties.flip_v; }

    // Camera Property Accessors
    inline bool isStreaming() const { return m_is_streaming; }
    inline unsigned int getWidth() const { return m_properties.frame_width; }
    inline unsigned int getHeight() const { return m_properties.frame_height; }
    inline unsigned char getFrameRate() const { return m_properties.frame_rate; }
    inline unsigned int getRowBytes() const { return m_properties.frame_stride; }

    // Get the USB Bus/Port path that this camera is connected to
    inline t_usb_device_handle getUSBDeviceHandle() const { return m_usb_device_handle; }
    bool getUSBPortPath(char *out_identifier, size_t max_identifier_length) const;

    // Returns a pointer to the last frame copied from the bulk transfer worker thread
    inline class PS3EyeVideoPacketProcessor* getVideoPacketProcessor() { return m_video_packet_processor; }
    const unsigned char* getFrame();

private:
    PS3EyeLibUSBCapture(const PS3EyeLibUSBCapture&);
    void operator=(const PS3EyeLibUSBCapture&);

    void release();

    PS3EyeProperties m_properties;

    bool m_is_streaming;
    double m_last_qued_frame_time;

    // usb stuff
    t_usb_device_handle m_usb_device_handle;
    class PS3EyeVideoPacketProcessor *m_video_packet_processor;
    class USBAsyncTaskQueue *m_task_queue;
};

#endif
