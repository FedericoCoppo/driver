*******************************************************************************

********************************************************************************
	File        : UsbCdcInterface.h
	Date        : 11/02/2016
	Author      : Federico Coppo
	Versione    : 1.00
	Description : Interface with 'Communication Device Class'.
	Revisioni   :
*******************************************************************************/

#ifndef USB_CDC_INTERFACE_
#define USB_CDC_INTERFACE_

/*******************************************************************************
    Includes .
*******************************************************************************/

#include "usbd_cdc.h"
#include "usbd_desc.h"

#include "System.h"
#include "Ose.h"
#include "Utilities.h"
#include "stm32f4xx_hal_pcd.h"

/*******************************************************************************
    Exteranal data structures .
*******************************************************************************/

/// PCD Handle Structure definition for Usb FS .
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/// PCD Handle Structure definition for Usb HS .
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

/*******************************************************************************
	Class declaration .
*******************************************************************************/

/// Interface with 'Communication Device Class'.
class UsbCdcInterface : public Object
{
public:

	/// Max buffer size for Tx/Rx .
	enum
	{
		IfRxBuffMaxSize = 64,
		IfTxBuffMaxSize = 64,
	};

	/// Device supported num .
	enum { NumberOfDevices = 2 };

	/// Identifier of uC internal USB periferal .
	enum UsbIdentifiers
	{
		UsbOtgFs = 0,
		UsbOtgHs = 1,
	};

	/// Enumaration of transfer speeds .
	enum UsbSpeeds
	{
		 UsbSpeedHigh = 0,
		 UsbSpeedFull = 1,
		 UsbSpeedLow = 2,
	};

	/// Usb device info structure .
	struct UsbDeviceInfo
	{
		UsbSpeeds Speed;
		ubyte DeviceState;
		ulong Ep0State;
		ubyte Ep0MaxPacketSize;
		ushort Vid;
		ushort Pid;
		const char *ManufacturerString;
		const char *ProductString;
	};

	/// Enumeration of types of serial communication parities .
	enum class Parity { None, Even, Odd };

	/// Enumeration of control flows (not supported) .
	enum class FlowControl { None, CTS, RTS, CTS_RTS };

	/// Enumeration of communication errors .
	enum Errors { OverrunError = 0x04, TransmissionTimeoutError = 0x20 };

	/// Stack used by transmission task .
	const ushort TransmissionTaskStackSize = 1024;

	/// Transmission timeout .
	const ulong TransmissionTimeout = 100ul;

	/// Constructor .
	UsbCdcInterface(UsbIdentifiers id, const char * p_name = null);

	/// Destructor .
	virtual ~UsbCdcInterface();

	/// Open interface .
	bool Open(ulong baudrate, Parity parity = Parity::None, ubyte dataBits = 8, ubyte stopBits = 1,
				FlowControl flowControl = FlowControl::None, ulong rxBufferSize = 512, ulong txBufferSize = 256);

	/// Close interface .
	void Close();

	/// Indication of device status .
	bool IsOpen() { return this->isOpen; }

	/// Return host connection status .
	bool IsHostConnected();

	/// Write a bytes in the transmit circular buffer .
	int Write(ubyte data);

	/// Write a buffer of bytes in the transmit circular buffer .
	int Write(ubyte *p_data, uint lenght);

	/// Get device identifier .
	ubyte GetId() { return (ubyte) id; }

	/// Read and remove a singolar byte from circular buffer .
	int Read();

	/// Read without remove a singolar byte from circular buffer .
	int Peek();

	/// Clear the receive buffer .
	void Flush();

	/// Get error word .
	ulong GetErrorFlags();

	/// Get byte received number .
	int GetByteToRead();

	/// Get byte to transmit number .
	int GetByteToTransmit();

	/// Get usb device information .
	bool GetDeviceInfo(UsbDeviceInfo *p_info);

	/// Interrupt service routine handler .
	void IRQHandler();

protected:

	/// Callback on connection event: port connect 1st time or cable reconnect after disconnection .
	static EventCallback<void> *p_IHostConnected[NumberOfDevices];

	/// Callback on cable disconnection event .
	static EventCallback<void> *p_IHostDisconnected[NumberOfDevices];

	/// Callback on Rx event .
	static EventCallback<void> *p_IDataReceived[NumberOfDevices];

	/// Callback on Rx event error .
	static EventCallback<void> * p_IErrorReceived[NumberOfDevices];

	/// Callback on Tx end event .
	EventCallback<void> *p_ITransmissionEnded;

private:

	/* --- Structures --- */

	/// Configuration info struct .
	struct LineCoding
	{
		/// Data terminal rate, in bits per second .
		uint32_t Baudrate;

		/// Stop bits .
		uint8_t StopBits;

		/// Parity .
		uint8_t Parity;

		/// Number Data bits .
		uint8_t DataBits;

		/// Indicates whether the settings have been changed by the host .
		bool Changed;
	};


	/* --- Static callback routines used by USB High-speed --- */

	/// Init the CDC low layer over the USB HS .
	static int8_t cdcInitHs();

	/// Deinit the CDC low layer .
	static int8_t cdcDeInitHs();

	/// Manage the CDC class requests .
	static int8_t cdcControlHs(uint8_t cmd, uint8_t* pbuf, uint16_t length);

	/// Data received over USB OUT endpoint are sent over CDC interface through this function .
	static int8_t cdcReceiveHs(uint8_t* Buf, uint32_t *Len);


	/* --- Static callback routines used by USB Full-speed --- */

	/// Init the CDC low layer over the USB HS .
	static int8_t cdcInitFs();

	/// Deinit the CDC media low layer .
	static int8_t cdcDeInitFs();

	/// Manage the CDC class requests .
	static int8_t cdcControlFs(uint8_t cmd, uint8_t* pbuf, uint16_t length);

	/// Data received over USB OUT endpoint are sent over CDC interface through this function .
	static int8_t cdcReceiveFs(uint8_t* Buf, uint32_t *Len);

	/// Set the word containing word error.
	static void setErrorFlags(ulong mask, UsbIdentifiers id);


	/* --- Non-static methods --- */

	/// Init ST USB Device Library .
	void usbDeviceInit();

	/// DeInit ST USB the Library .
	void usbDeviceDeInit();

	/// Start trasmission process .
	static void runTransmissionTask(void *p);

	/// Trasmission process .
	void transmissionTask();

	/// Stop transmission process .
	void stopTransmissionTask();

	/// Check if a transmission is ongoing .
	bool isTransmissionBusy();

	/// Transmit a maximum of IfTxBuffMaxSize on usb (64) .
	int transmitBuffer(ubyte *p_buffer, ushort length);

	/// Disable usb interrupt .
	void disableUsbInterrupt();

	/// Enable usb interrupt .
	void enableUsbInterrupt();


	/* --- Static members --- */

	/// Buffer used to share Rx byte with USB library .
	static ubyte ifRxBuff[IfRxBuffMaxSize];

	/// Buffer used to share Tx byte with USB library .
	static ubyte ifTxBuff[IfTxBuffMaxSize];


	/// Rx circular buffer array .
	static CharRingBuffer *p_rxRingBufferVector[NumberOfDevices];

	/// USB device table pointer array .
	static USBD_HandleTypeDef *p_hUsbDeviceVector[NumberOfDevices];

	/// Error mask array.
	static ulong errorFlags[NumberOfDevices];

	/// Usb Serial configuration for high speed and full speed .
	static LineCoding configInfos[NumberOfDevices];


	/* --- Non-static members --- */

	/// Usb process name .
	const char *p_name;

	/// Device identifier .
	UsbIdentifiers id;

	/// Flag indicating port status .
	bool isOpen;

	/// Run process status flag .
	bool isRunning;

	/// Stop process status flag .
	bool isStopped;

	/// Transmission complete event .
	event_t transmitEvent;

	/// PCD handle structure pointer .
	PCD_HandleTypeDef *p_hpcd;

	/// USB device table pointer's object .
	USBD_HandleTypeDef *p_hUsbDevice;

	/// USB Communication device Class interface (CDC) .
	USBD_CDC_ItfTypeDef fops;

	/// Tx circular buffer .
	CharRingBuffer *p_txRingBuffer;

	/// Friend class .
	friend class UsbSerialPort;
};

/******************************************************************************/

#endif // USB_CDC_INTERFACE_

