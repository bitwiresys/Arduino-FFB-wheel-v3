

#ifndef __USBAPI__
#define __USBAPI__

#if defined(USBCON)

//================================================================================
//================================================================================
//	Low level API

typedef struct
{
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint8_t wValueL;
  uint8_t wValueH;
  uint16_t wIndex;
  uint16_t wLength;
} Setup;

//================================================================================
//================================================================================
//	USB

class USBDevice_
{
  public:
    USBDevice_();
    bool configured();

    void attach();
    void detach();	// Serial port goes down too...
    void poll();

    b8 (*HID_Setup_Callback) (Setup& setup);
    void (*HID_ReceiveReport_Callback) (uint8_t *data, uint16_t len);
};
extern USBDevice_ USBDevice;

//================================================================================
//================================================================================
//	Serial over CDC (Serial1 is the physical port)

class Serial_ : public Stream
{
  private:
    ring_buffer *_cdc_rx_buffer;
  public:
    void begin(uint16_t baud_count);
    void end(void);

    virtual int available(void);
    virtual void accept(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool();
};
extern Serial_ Serial;

//================================================================================
//================================================================================
// Joystick
// Implemented in HID.cpp

// dustin's rig - only the report layout actually referenced by SendInputReport (USBDesc.h)
// is kept; the dozen legacy send_* packing variants were dead code.
class Joystick_
{
  public:
    Joystick_();
    void send_16_16_12_12_12_28(uint16_t x, uint16_t y, uint16_t z, uint16_t rx, uint16_t ry, uint32_t buttons); // milos, added this one
};
extern Joystick_ Joystick;

// dustin's rig, removed - Mouse_/Keyboard_ class declarations (never in the HID descriptor, see HID.cpp)

//================================================================================
//================================================================================
//	HID 'Driver'

int		HID_GetInterface(uint8_t* interfaceNum);
int		HID_GetDescriptor(int i);
b8		HID_Setup(Setup& setup);
void	HID_SendReport(uint8_t id, const void* data, int len);
u8		HID_ReportAvailable();
s16	 	HID_ReceiveReport(void* data, int len);

//================================================================================
//================================================================================
//	MSC 'Driver'

int		MSC_GetInterface(uint8_t* interfaceNum);
int		MSC_GetDescriptor(int i);
bool	MSC_Setup(Setup& setup);
bool	MSC_Data(uint8_t rx, uint8_t tx);

//================================================================================
//================================================================================
//	CSC 'Driver'

int		CDC_GetInterface(uint8_t* interfaceNum);
int		CDC_GetDescriptor(int i);
bool	CDC_Setup(Setup& setup);

//================================================================================
//================================================================================

#define TRANSFER_PGM		0x80
#define TRANSFER_RELEASE	0x40
#define TRANSFER_ZERO		0x20

int USB_SendControl(uint8_t flags, const void* d, int len);
int USB_RecvControl(void* d, int len);

uint8_t	USB_Available(uint8_t ep);
int USB_Send(uint8_t ep, const void* data, int len);	// blocking
int USB_Recv(uint8_t ep, void* data, int len);		// non-blocking
int USB_Recv(uint8_t ep);							// non-blocking
void USB_Flush(uint8_t ep);

#endif

#endif /* if defined(USBCON) */
