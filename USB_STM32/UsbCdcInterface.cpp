/*******************************************************************************

 ********************************************************************************
    File        : UsbCdcInterface.cpp
    Date        : 11/02/2016
    Author      : Federico Coppo
    Version     : 1.00
    Description : Interface with 'Communication Device Class'.
    Revision    :
 *******************************************************************************/

#include "UsbCdcInterface.h"

/*******************************************************************************
	Static attribute .
*******************************************************************************/

// Callback on connection event .
EventCallback<void> *UsbCdcInterface::p_IHostConnected[NumberOfDevices] = { null, null };

// Callback on cable disconnection event .
EventCallback<void> *UsbCdcInterface::p_IHostDisconnected[NumberOfDevices] = { null, null };

// Callback on Rx event .
EventCallback<void> *UsbCdcInterface::p_IDataReceived[NumberOfDevices] = { null, null };

// Callback on Rx event error .
EventCallback<void> *UsbCdcInterface::p_IErrorReceived[NumberOfDevices] = { null, null };


// Buffer used to share Rx byte with USB library .
ubyte UsbCdcInterface::ifRxBuff[IfRxBuffMaxSize];

// Buffer used to share Tx byte with USB library .
ubyte UsbCdcInterface::ifTxBuff[IfTxBuffMaxSize];

/// Rx circular buffer array .
CharRingBuffer *UsbCdcInterface::p_rxRingBufferVector[NumberOfDevices] = { null, null };

// USB device table pointer array .
USBD_HandleTypeDef *UsbCdcInterface::p_hUsbDeviceVector[NumberOfDevices] = { null, null };

// Error mask array.
ulong UsbCdcInterface::errorFlags[NumberOfDevices] = { 0ul, 0ul };

// Configuration infos default values .
UsbCdcInterface::LineCoding UsbCdcInterface::configInfos[NumberOfDevices] =
{
	{ 115200, 0,  0,  8, false },
	{ 115200, 0,  0,  8, false }
};

/*******************************************************************************
	Public class method implementation .
*******************************************************************************/

// Constructor .
UsbCdcInterface::UsbCdcInterface(UsbIdentifiers id, const char * p_name)
{
	// Update identifier .
	this->id = id;

	if (p_name != null)
	{
		this->p_name = p_name;
	}
	else
	{
		this->p_name = "Usb 2.0";
	}

	isRunning = false;
	isStopped = true;

	isOpen = false;

	p_hUsbDevice = null;
	p_hpcd = null;

	// Initialize tx buffer .
	p_txRingBuffer = null;

	// Initialize callback pointer .
	p_ITransmissionEnded = null;
}

// Destructor .
UsbCdcInterface::~UsbCdcInterface()
{
	if (isOpen == true)
	{
		Close();
	}
}

// Open interface .
bool UsbCdcInterface::Open(ulong baudrate, Parity parity, ubyte dataBits, ubyte stopBits,
		FlowControl flowControl, ulong rxBufferSize, ulong txBufferSize)
{
	ulong processId;

	// Check if params is valid .
	if (id != UsbOtgHs && id != UsbOtgFs)
	{
		return false;
	}

	if (rxBufferSize == 0 || txBufferSize == 0)
	{
		return false;
	}

	// Check if this object is already open or other instances are using the same port .
	if (isOpen == true || p_hUsbDeviceVector[id] != null)
	{
		return false;
	}

	// Compile line coding structure .
	configInfos[id].Baudrate = baudrate;

	switch(parity)
	{
		case Parity::Even:
		{
			configInfos[id].Parity = 2;
			break;
		}
		case Parity::Odd:
		{
			configInfos[id].Parity = 1;
			break;
		}
		default:
		{
			configInfos[id].Parity = 0;
			break;
		}
	}

	if (stopBits == 2)
	{
		configInfos[id].StopBits = 2;
	}
	else
	{
		configInfos[id].StopBits = 0;
	}

	if (dataBits == 5 || dataBits == 6 || dataBits == 7)
	{
		configInfos[id].DataBits = dataBits;
	}
	else
	{
		configInfos[id].DataBits = 8;
	}

	// Initialize tx event .
	Ose.InitializeSingleEventX(&transmitEvent);

	// Usb library table allocation .
	p_hUsbDeviceVector[id] = new USBD_HandleTypeDef;

	memset (p_hUsbDeviceVector[id], 0u, (size_t)sizeof(USBD_HandleTypeDef));

	p_hUsbDevice = p_hUsbDeviceVector[id];

	// Link to static callback method used by USB library H/F Speed .
	if (id == UsbOtgFs)
	{
		p_hpcd = &hpcd_USB_OTG_FS;

		fops = { cdcInitFs, cdcDeInitFs, cdcControlFs, cdcReceiveFs};
	}
	else
	{
		p_hpcd = &hpcd_USB_OTG_HS;

		fops = { cdcInitHs, cdcDeInitHs, cdcControlHs, cdcReceiveHs};
	}

	// Rx buffer allocation .
	p_rxRingBufferVector[id] = new CharRingBuffer(rxBufferSize);

	// Tx buffer allocation .
	p_txRingBuffer = new CharRingBuffer(txBufferSize);

	// Initialize cdc library .
	usbDeviceInit();

	// Process creation .
	processId = Ose.CreateProcess
	(
		runTransmissionTask,
		(void *) this,
		TransmissionTaskStackSize,
		OSE_PROCESS_PRIORITY_10,
		OSE_PROCESS_READY
	);

	if (processId != PROCESS_ID_NONE)
	{
		// Port is open .
		isOpen = true;

		return true;
	}

	// Deinitialize usb library .
	usbDeviceDeInit();

	// Initialize callback pointer .
	fops = { null, null, null, null};

	// Initialize callback pointer .
	p_IHostConnected[id] = null;
	p_IHostDisconnected[id] = null;

	p_IDataReceived[id] = null;
	p_IErrorReceived[id] = null;
	p_ITransmissionEnded = null;

	// Delete tx ring buffer .
	if (p_txRingBuffer != null)
	{
		delete (p_txRingBuffer);

		p_txRingBuffer = null;
	}

	// Delete rx ring buffer .
	if (p_rxRingBufferVector[id] != null)
	{
		delete (p_rxRingBufferVector[id]);

		p_rxRingBufferVector[id] = null;
	}

	if (p_hUsbDeviceVector[id] != null)
	{
		delete (p_hUsbDeviceVector[id]);

		p_hUsbDeviceVector[id] = null;
	}

	p_hUsbDevice = null;
	p_hpcd = null;

	return false;
}

// Close interface .
void UsbCdcInterface::Close()
{
	if (isOpen == true)
	{
		// Set device as closed: no other tx are triggered .
		isOpen = false;

		// Wait last transmission ending .
		while (isTransmissionBusy() == true)
		{
			Ose.YieldProcess();
		}

		// Close the process .
		stopTransmissionTask();

		// Deinitialize usb library .
		usbDeviceDeInit();

		// Initialize callback pointer .
		fops = { null, null, null, null };

		// Initialize callback pointer .
		p_IHostConnected[id] = null;
		p_IHostDisconnected[id] = null;

		p_IDataReceived[id] = null;
		p_IErrorReceived[id] = null;
		p_ITransmissionEnded = null;

		// Delete tx ring buffer .
		if (p_txRingBuffer != null)
		{
			delete (p_txRingBuffer);

			p_txRingBuffer = null;
		}

		// Delete rx ring buffer .
		if (p_rxRingBufferVector[id] != null)
		{
			delete (p_rxRingBufferVector[id]);

			p_rxRingBufferVector[id] = null;
		}

		if (p_hUsbDeviceVector[id] != null)
		{
			delete (p_hUsbDeviceVector[id]);

			p_hUsbDeviceVector[id] = null;
		}

		p_hUsbDevice = null;
		p_hpcd = null;
	}
}

// Return host connection status .
bool UsbCdcInterface::IsHostConnected()
{
	if (isOpen == true)
	{
		if (/* (p_hUsbDevice->ep0_state != USBD_EP0_STATUS_IN) && */ (p_hUsbDevice->dev_state == USBD_STATE_CONFIGURED) )
		{
			return true;
		}
	}

	return false;
}

// Write a bytes in the transmit circular buffer .
int UsbCdcInterface::Write(ubyte data)
{
	if (IsHostConnected() == true)
	{
		if (p_txRingBuffer->IsFull() == true)
		{
			// Process wake-up .
			Ose.SignalEvent(&transmitEvent);

			// Wait process unload data before insert byte into buffer .
			while (p_txRingBuffer->IsFull() == true)
			{
				Ose.YieldProcess();
			}
		}

		// Insert byte into buffer .
		p_txRingBuffer->Add(data);

		// Process wake-up .
		Ose.SignalEvent(&transmitEvent);

		return 1;
	}

	return 0;
}

// Write a buffer of bytes in the transmit circular buffer .
int UsbCdcInterface::Write(ubyte *p_data, uint length)
{
	uint i;

	if (IsHostConnected() == true && p_data != null && length > 0)
	{
		for (i = 0; i < length; i++)
		{
			if (p_txRingBuffer->IsFull() == true)
			{
				// Process wake-up .
				Ose.SignalEvent(&transmitEvent);

				// Wait process unload data before insert byte into buffer .
				while (p_txRingBuffer->IsFull() == true)
				{
					Ose.YieldProcess();
				}
			}

			// Insert byte into buffer .
			p_txRingBuffer->Add(p_data[i]);
		}

		// Process wake-up .
		Ose.SignalEvent(&transmitEvent);

		return (int)length;
	}

	return 0;
}

// Read and unload one single byte from rx buffer .
int UsbCdcInterface::Read()
{
	int data = EOF;

	if (isOpen == true)
	{
		ubyte t;

		if (p_rxRingBufferVector[id]->Remove(t) == true)
		{
			data = (int)(uint)t;
		}
	}
	return data;
}

// Read data without removing .
int UsbCdcInterface::Peek()
{
	int data = EOF;

	if (isOpen == true)
	{
		ubyte t;

		if (p_rxRingBufferVector[id]->First(t) == true)
		{
			data = (int)(uint)t;
		}
	}
	return data;
}

// Clear the receive buffer .
void UsbCdcInterface::Flush()
{
	if (isOpen == true)
	{
		p_rxRingBufferVector[id]->Clear();
	}
}

// Get the word containing word error.
ulong UsbCdcInterface::GetErrorFlags()
{
	ulong error = 0;

	if (isOpen == true)
	{
		error = errorFlags[id];

		// Flag error healing .
		errorFlags[id] = 0;
	}

	return error;
}

// Get number of received byte .
int UsbCdcInterface::GetByteToRead()
{
	int byteToRead = 0;

	if (isOpen == true)
	{
		byteToRead = p_rxRingBufferVector[id]->Count();
	}

	return byteToRead;
}

// Get number of byte still present intocircular buffer waiting to be transmit .
int UsbCdcInterface::GetByteToTransmit()
{
	int byteToRead = 0;

	if (isOpen == true)
	{
		byteToRead = p_txRingBuffer->Count();
	}

	return byteToRead;
}

// Get usb device information .
bool UsbCdcInterface::GetDeviceInfo(UsbDeviceInfo *p_info)
{
	if (isOpen == true)
	{
		uint16_t length;

		ubyte *p_descriptorData = p_hUsbDevice->pDesc->GetDeviceDescriptor(p_hUsbDevice->dev_speed,  &length);

		// Fill info structure .
		switch (p_hUsbDevice->dev_speed)
		{
			case USBD_SPEED_HIGH:
			{
				p_info->Speed = UsbSpeeds::UsbSpeedHigh;
				break;
			}
			case USBD_SPEED_FULL:
			{
				p_info->Speed = UsbSpeeds::UsbSpeedFull;
				break;
			}
			case USBD_SPEED_LOW:
			{
				p_info->Speed = UsbSpeeds::UsbSpeedLow;
				break;
			}
		}

		p_info->DeviceState = p_hUsbDevice->dev_state;
		p_info->Ep0State = p_hUsbDevice->ep0_state;
		p_info->Ep0MaxPacketSize = p_descriptorData[7];

		p_info->Vid = (ushort) p_descriptorData[8] + ((ushort) p_descriptorData[9] << 8);
		p_info->Pid = (ushort) p_descriptorData[10] + ((ushort) p_descriptorData[11] << 8);

		p_info->ManufacturerString = USBD_GetManufacturerString();
		p_info->ProductString = USBD_GetProductString();

		return true;
	}

	return false;
}

// Interrupt service routine handler .
void UsbCdcInterface::IRQHandler()
{
	HAL_PCD_IRQHandler(p_hpcd);
}

/*******************************************************************************
	Private class static method implementation .
*******************************************************************************/

// Init the CDC media low layer over the USB HS .
int8_t UsbCdcInterface::cdcInitHs()
{
	 // Only if table has been allocated .
	if (p_hUsbDeviceVector[UsbOtgHs] != null)
	{
		USBD_CDC_SetTxBuffer(p_hUsbDeviceVector[UsbOtgHs], ifTxBuff, 0u);
		USBD_CDC_SetRxBuffer(p_hUsbDeviceVector[UsbOtgHs], ifRxBuff);

		// Callback on connection event: port connect 1st time or cable reconnect after disconnection .
		if (p_hUsbDeviceVector[UsbOtgHs]->dev_state == USBD_STATE_CONFIGURED)
		{
			if (p_IHostConnected[UsbOtgHs] != null )
			{
				// New connection callback .
				p_IHostConnected[UsbOtgHs]->Invoke();
			}
		}

		return USBD_OK;
	}

	return USBD_FAIL;
}

// DeInitialize the CDC media low layer .
int8_t UsbCdcInterface::cdcDeInitHs()
{
	// Callback on cable disconnection event .
	if (p_hUsbDeviceVector[UsbOtgHs]->dev_state == USBD_STATE_DEFAULT)
	{
		// Flush all Tx FIFOs to avoid incoherence between Hw FIFO and CDC buffer when disconnecting .
		USB_FlushTxFifo(hpcd_USB_OTG_HS.Instance, 0x10);

		if (p_IHostDisconnected[UsbOtgHs] != null)
		{
			// Disconnection callback .
			p_IHostDisconnected[UsbOtgHs]->Invoke();
		}
	}

	return USBD_OK;
}

// Manage the CDC class requests .
int8_t UsbCdcInterface::cdcControlHs(uint8_t cmd, uint8_t* p_buffer, uint16_t length)
{
	switch (cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:
		{
			break;
		}
		case CDC_GET_ENCAPSULATED_RESPONSE:
		{
			break;
		}
		case CDC_SET_COMM_FEATURE:
		{
			break;
		}
		case CDC_GET_COMM_FEATURE:
		{
			break;
		}
		case CDC_CLEAR_COMM_FEATURE:
		{
			break;
		}

		case CDC_SET_LINE_CODING:     // Keep configuration data from host .
		{
			configInfos[UsbOtgHs].Baudrate = ( ((uint32_t)p_buffer[0] << 0)  |
											   ((uint32_t)p_buffer[1] << 8)  |
											   ((uint32_t)p_buffer[2] << 16) |
											   ((uint32_t)p_buffer[3] << 24) );

			configInfos[UsbOtgHs].StopBits = p_buffer[4];
			configInfos[UsbOtgHs].Parity = p_buffer[5];
			configInfos[UsbOtgHs].DataBits = p_buffer[6];
			configInfos[UsbOtgHs].Changed = true;

			break;
		}
		case CDC_GET_LINE_CODING:     // Send to host  configuration data .
		{
			p_buffer[0] = (uint8_t) (configInfos[UsbOtgHs].Baudrate >> 0);
			p_buffer[1] = (uint8_t) (configInfos[UsbOtgHs].Baudrate >> 8);
			p_buffer[2] = (uint8_t) (configInfos[UsbOtgHs].Baudrate >> 16);
			p_buffer[3] = (uint8_t) (configInfos[UsbOtgHs].Baudrate >> 24);

			p_buffer[4] = configInfos[UsbOtgHs].StopBits;
			p_buffer[5] = configInfos[UsbOtgHs].Parity;
			p_buffer[6] = configInfos[UsbOtgHs].DataBits;

			break;
		}
		case CDC_SET_CONTROL_LINE_STATE:
		{
			break;
		}
		case CDC_SEND_BREAK:
		{
			break;
		}
		default:
		{
			break;
		}
	}

	return USBD_OK;
}

// High Speed Rx callback .
int8_t UsbCdcInterface::cdcReceiveHs(uint8_t *p_buffer, uint32_t *p_length)
{
	uint16_t i;

	bool success = true;

	// Check if port is open .
	if (p_rxRingBufferVector[UsbOtgHs] != null)
	{
		for (i = 0; i < *p_length; i++)
		{
			// Rx buffer management .
			success = p_rxRingBufferVector[UsbOtgHs]->Add(p_buffer[i]);

			if (success == false)
			{
				break;
			}
		}

		// Overload buffer error management .
		if (success == false)
		{
			// Update err mask .
			setErrorFlags(Errors::OverrunError, UsbOtgHs);

			if (p_IErrorReceived[UsbOtgHs] != null)
			{
				// Invokes error callback .
				p_IErrorReceived[UsbOtgHs]->Invoke();
			}
		}

		if (p_IDataReceived[UsbOtgHs] != null)
		{
			// Invokes receive callback .
			p_IDataReceived[UsbOtgHs]->Invoke();
		}
	}

	// Handler for other interrupt .
	USBD_CDC_ReceivePacket(p_hUsbDeviceVector[UsbOtgHs]);

	return USBD_OK;
}

// Initialize the CDC media low layer over the USB FS .
int8_t UsbCdcInterface::cdcInitFs()
{
	// Check if table has been allocated .
	if (p_hUsbDeviceVector[UsbOtgFs] != null)
	{
		USBD_CDC_SetTxBuffer(p_hUsbDeviceVector[UsbOtgFs], ifTxBuff, 0u);
		USBD_CDC_SetRxBuffer(p_hUsbDeviceVector[UsbOtgFs], ifRxBuff);

		// Callback on connection event: port connect 1st time or cable reconnect after disconnection .
		if (p_hUsbDeviceVector[UsbOtgFs]->dev_state == USBD_STATE_CONFIGURED)
		{
			if (p_IHostConnected[UsbOtgFs] != null )
			{
				// New connection callback .
				p_IHostConnected[UsbOtgFs]->Invoke();
			}
		}

		return USBD_OK;
	}

	return USBD_FAIL;
}

// DeInitialize the CDC media low layer .
int8_t UsbCdcInterface::cdcDeInitFs()
{
	// Callback on cable disconnection event .
	if (p_hUsbDeviceVector[UsbOtgFs]->dev_state == USBD_STATE_DEFAULT)
	{
		// Flush all Tx FIFOs to avoid incoherence between Hw FIFO and CDC buffer when disconnecting .
		USB_FlushTxFifo(hpcd_USB_OTG_FS.Instance, 0x10);

		if (p_IHostDisconnected[UsbOtgFs] != null)
		{
			// Disconnection callback .
			p_IHostDisconnected[UsbOtgFs]->Invoke();
		}
	}

	return USBD_OK;
}

// Manage the CDC class requests .
int8_t UsbCdcInterface::cdcControlFs(uint8_t cmd, uint8_t* p_buffer, uint16_t length)
{
	switch (cmd)
	{
		case CDC_SEND_ENCAPSULATED_COMMAND:
		{
			break;
		}
		case CDC_GET_ENCAPSULATED_RESPONSE:
		{
			break;
		}
		case CDC_SET_COMM_FEATURE:
		{
			break;
		}
		case CDC_GET_COMM_FEATURE:
		{
			break;
		}
		case CDC_CLEAR_COMM_FEATURE:
		{
			break;
		}
		case CDC_SET_LINE_CODING:     // Keep configuration data from host .
		{
			configInfos[UsbOtgFs].Baudrate = ( ((uint32_t)p_buffer[0] << 0)  |
											   ((uint32_t)p_buffer[1] << 8)  |
											   ((uint32_t)p_buffer[2] << 16) |
											   ((uint32_t)p_buffer[3] << 24) );

			configInfos[UsbOtgFs].StopBits = p_buffer[4];
			configInfos[UsbOtgFs].Parity = p_buffer[5];
			configInfos[UsbOtgFs].DataBits = p_buffer[6];
			configInfos[UsbOtgFs].Changed = true;

			break;
		}
		case CDC_GET_LINE_CODING:     // Send to host  configuration data .
		{
			p_buffer[0] = (uint8_t) (configInfos[UsbOtgFs].Baudrate >> 0);
			p_buffer[1] = (uint8_t) (configInfos[UsbOtgFs].Baudrate >> 8);
			p_buffer[2] = (uint8_t) (configInfos[UsbOtgFs].Baudrate >> 16);
			p_buffer[3] = (uint8_t) (configInfos[UsbOtgFs].Baudrate >> 24);

			p_buffer[4] = configInfos[UsbOtgFs].StopBits;
			p_buffer[5] = configInfos[UsbOtgFs].Parity;
			p_buffer[6] = configInfos[UsbOtgFs].DataBits;

			break;
		}
		case CDC_SET_CONTROL_LINE_STATE:
		{
			break;
		}
		case CDC_SEND_BREAK:
		{
			break;
		}
		default:
		{
			break;
		}
	}

	return USBD_OK;
}

// Data received over USB OUT endpoint are sent over CDC interface through this function .
int8_t UsbCdcInterface::cdcReceiveFs(uint8_t* p_buffer, uint32_t *p_length)
{
	uint i;

	bool success = true;

	// Check if port is open .
	if (p_rxRingBufferVector[UsbOtgFs] != null)
	{
		for (i = 0; i < *p_length; i++)
		{
			// Rx buffer management .
			success = p_rxRingBufferVector[UsbOtgFs]->Add(p_buffer[i]);

			if (success == false)
			{
				break;
			}
		}

		// Overload buffer error management .
		if (success == false)
		{
			// Update err mask .
			setErrorFlags(Errors::OverrunError, UsbOtgFs);

			if ( (p_IErrorReceived[UsbOtgFs] != null) )
			{
				// Invoke error callback .
				p_IErrorReceived[UsbOtgFs]->Invoke();
			}
		}

		if (p_IDataReceived[UsbOtgFs] != null)
		{
			// Invoke receive callback .
			p_IDataReceived[UsbOtgFs]->Invoke();
		}
	}

	// Handler for other interrupt .
	USBD_CDC_ReceivePacket(p_hUsbDeviceVector[UsbOtgFs]);

	return USBD_OK;
}

// Get the word containing word error.
void UsbCdcInterface::setErrorFlags(ulong mask, UsbIdentifiers id)
{
	if ((int)id < (int)NumberOfDevices)
	{
		if (p_rxRingBufferVector[id] != null)
		{
			// Flag error insertion .
			errorFlags[id] |= mask;
		}
	}
}

/*******************************************************************************
	Private class method implementation .
*******************************************************************************/

// Initialize ST USB Device Library (used by "Open" method) .
void UsbCdcInterface::usbDeviceInit(void)
{
	if(this->id == UsbOtgHs)
	{
		USBD_Init(this->p_hUsbDevice, &FS_Desc, this->id);
	}
	else
	{
		//Initializes  the Low Level portion of the Device driver .
		USBD_Init(this->p_hUsbDevice, &FS_Desc, this->id);
	}

	// CDC class Type config  executing this link: dev->pClass = pclass .
	USBD_RegisterClass(this->p_hUsbDevice, &USBD_CDC);

	// Link to callback user function (cdcInitXs,cdcDeInitXs,cdcControlXs, cdcReceiveXs) .
	USBD_CDC_RegisterInterface(this->p_hUsbDevice, &fops);

	// To avoid Library DeInit of pClassData .
	this->p_hUsbDevice->pClassData = null;

	// Low level routine that allow Hw connecting USB .
	USBD_Start(this->p_hUsbDevice);
}

// Private routine used by close() to switch off the USB Library .
void UsbCdcInterface::usbDeviceDeInit(void)
{
	//USBD_Stop(this->p_hUsbDevice);

	// Not needed .
	USBD_DeInit(this->p_hUsbDevice);
}

// Start transmission process .
void UsbCdcInterface::runTransmissionTask(void *p)
{
	UsbCdcInterface *p_cdcInterface = (UsbCdcInterface *) p;

	p_cdcInterface->transmissionTask();
}

// Transmission process .
void UsbCdcInterface::transmissionTask()
{
	// Circular buffer byte num .
	int dataToTransmitNum;

	// Transmission shared array byte num .
	int txCount;

	// Process is ongoing .
	isRunning = true;
	isStopped = false;

	// Process naming .
	Ose.SetProcessName(p_name);

	while (isRunning == true)
	{
		// Transmit all byte prensent in the circ buffer .
		if (p_txRingBuffer->IsEmpty() == false)
		{
			dataToTransmitNum = p_txRingBuffer->Count();

			if (dataToTransmitNum > IfTxBuffMaxSize)
			{
				txCount = IfTxBuffMaxSize;
			}
			else
			{
				txCount = dataToTransmitNum;
			}

			for (int i = 0; i < txCount; i++)
			{
				p_txRingBuffer->Remove(ifTxBuff[i]);
			}

			// Transmit all byte loaded into interface array.
			if (transmitBuffer(ifTxBuff, txCount) != USBD_OK)
			{
				// If trasmission fails then clears tx ring buffer .
				p_txRingBuffer->Clear();

				// Update err mask .
				setErrorFlags(Errors::TransmissionTimeoutError, id);

				if (p_IErrorReceived[id] != null)
				{
					// Invokes error callback .
					p_IErrorReceived[id]->Invoke();
				}
			}
		}
		else
		{
			if (p_ITransmissionEnded != null)
			{
				p_ITransmissionEnded->Invoke();
			}

			// Transmission endend, wait to be wake up by another tx request .
			Ose.WaitEvent(&transmitEvent);
		}
	}

	isStopped = true;

	Ose.DestroyProcessMyself();
}

// Stop transmission process .
void UsbCdcInterface::stopTransmissionTask()
{
	if (isRunning)
	{
		isRunning = false;

		// Process wake-up to allow isStopped flag equal true .
		Ose.SignalEvent(&transmitEvent);

		// Wait process end .
		while (!isStopped)
		{
			Ose.YieldProcess();
		}
	}
}

// Method to check if transmission is ongoing .
bool UsbCdcInterface::isTransmissionBusy()
{
	return ( ((USBD_CDC_HandleTypeDef*) (p_hUsbDevice->pClassData))->TxState != 0);
}

// Transmit series of byte on usb .
int UsbCdcInterface::transmitBuffer(ubyte *p_buffer, ushort length)
{
	uint8_t result = USBD_OK;

	ulong startTime = Clock.Milliseconds();

	// Ack needed to allow next Tx .
	while (isTransmissionBusy() == true)
	{
		Ose.YieldProcess();

		if ((Clock.Milliseconds() - startTime) > TransmissionTimeout)
		{
			// Time out expired .
			return USBD_BUSY;
		}
	}

	USBD_CDC_SetTxBuffer(p_hUsbDevice, p_buffer, length);

	// Disable usb receipt interrupt before transmission .
	disableUsbInterrupt();

	result = USBD_CDC_TransmitPacket(p_hUsbDevice);

	// Enable usb interrupt after transmission .
	enableUsbInterrupt();

	if (result == USBD_OK)
	{
		//if ((length % 64) == 0)
		if ( (length & (64 - 1)) == 0 )
		{
			startTime = Clock.Milliseconds();

			// Ack needed to allow next Tx .
			while (isTransmissionBusy() == true)
			{
				Ose.YieldProcess();

				if ((Clock.Milliseconds() - startTime) > TransmissionTimeout)
				{
					 // Time out expired .
					return USBD_BUSY;
				}
			}

			USBD_CDC_SetTxBuffer(p_hUsbDevice, p_buffer, 0u);

			// Disable usb receipt interrupt before transmission .
			disableUsbInterrupt();

			result = USBD_CDC_TransmitPacket(p_hUsbDevice);

			// Enable usb interrupt after transmission .
			enableUsbInterrupt();
		}
	}

	return result;
}

// Disable usb interrupt .
void UsbCdcInterface::disableUsbInterrupt()
{
	// Disable usb receipt interrupt during transmission .
	if (id==UsbIdentifiers::UsbOtgFs)
	{
		__HAL_PCD_DISABLE(&hpcd_USB_OTG_FS);
	}
	else
	{
		__HAL_PCD_DISABLE(&hpcd_USB_OTG_HS);
	}
}
// Enable usb interrupt .
void UsbCdcInterface::enableUsbInterrupt()
{
	// Enable usb interrupt during transmission .
	if (id==UsbIdentifiers::UsbOtgFs)
	{
		__HAL_PCD_ENABLE(&hpcd_USB_OTG_FS);
	}
	else
	{
		__HAL_PCD_ENABLE(&hpcd_USB_OTG_HS);
	}
}

/******************************************************************************/

