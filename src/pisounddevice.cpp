//
// pisounddevice.cpp
//
// Support for the Blokas Pisound HAT. See pisounddevice.h for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include "pisounddevice.h"

#include <cassert>
#include <cstdio>
#include <cstring>

#include <circle/logger.h>
#include <circle/timer.h>

LOGMODULE("pisound");

// Pin assignments from pisound-overlay.dts (BCM numbering)
#define PISOUND_PIN_OSR0 13
#define PISOUND_PIN_OSR1 26
#define PISOUND_PIN_OSR2 16
#define PISOUND_PIN_ADC_RESET 12
#define PISOUND_PIN_SPI_RESET 24
#define PISOUND_PIN_DATA_AVAILABLE 25
#define PISOUND_PIN_BUTTON 17

// SPI settings from pisound-overlay.dts / pisound.c
#define PISOUND_SPI_BUS 0
#define PISOUND_SPI_CS 0
#define PISOUND_SPI_SPEED_HZ 150000
#define PISOUND_SPI_DELAY_US 10

// Protocol details from pisound.c (pisnd_work_handler)
#define PISOUND_TRANSFER_SIZE 4
#define PISOUND_CMD_MIDI_BYTE 0x0F
#define PISOUND_CMD_LED_FLASH 0xF0
#define PISOUND_OUTPUT_BUFFER_SIZE_MILLIBYTES (127 * 1000)
// 31250 baud, 10 bits per byte = 3125 bytes/s = 3.125 bytes/ms
#define PISOUND_MIDI_MILLIBYTES_PER_MS 3125

// Upper bound of SPI transfers per Process() call, so that a busy
// MIDI stream cannot block the user interface.
#define PISOUND_MAX_TRANSFERS_PER_PROCESS 64

CPisoundDevice::CPisoundDevice(CMiniDexed *pSynthesizer, CConfig *pConfig, CUserInterface *pUI) :
CMIDIDevice{pSynthesizer, pConfig, pUI},
m_pConfig{pConfig},
m_pSPIMaster{nullptr},
m_bMIDIReady{false},
m_TxFIFO{},
m_nTxIn{0},
m_nTxOut{0},
m_nOutBufferUsedMilliBytes{0},
m_nLastTransferTicks{0},
m_nParserState{0},
m_nSysEx{0},
m_Message{}
{
	// Do not touch any GPIO here: this object also exists
	// when the Pisound is not enabled in minidexed.ini.
}

CPisoundDevice::~CPisoundDevice()
{
	delete m_pSPIMaster;
	m_pSPIMaster = nullptr;
}

bool CPisoundDevice::InitAudio(unsigned nSampleRate)
{
#if RASPPI >= 5
	LOGERR("Pisound is not supported on Raspberry Pi 5");
	return false;
#else
	unsigned nOSR0, nOSR1, nOSR2;
	switch (nSampleRate)
	{
	case 48000:
		nOSR0 = HIGH;
		nOSR1 = LOW;
		nOSR2 = LOW;
		break;
	case 96000:
		nOSR0 = HIGH;
		nOSR1 = LOW;
		nOSR2 = HIGH;
		break;
	case 192000:
		nOSR0 = HIGH;
		nOSR1 = HIGH;
		nOSR2 = HIGH;
		break;
	default:
		LOGERR("Sample rate %u not supported (use 48000, 96000 or 192000)", nSampleRate);
		return false;
	}

	m_ADCReset.AssignPin(PISOUND_PIN_ADC_RESET);
	m_ADCReset.SetMode(GPIOModeOutput, false);
	m_OSR0.AssignPin(PISOUND_PIN_OSR0);
	m_OSR0.SetMode(GPIOModeOutput, false);
	m_OSR1.AssignPin(PISOUND_PIN_OSR1);
	m_OSR1.SetMode(GPIOModeOutput, false);
	m_OSR2.AssignPin(PISOUND_PIN_OSR2);
	m_OSR2.SetMode(GPIOModeOutput, false);

	// Same order as pisnd_hw_params(): hold the ADC in reset,
	// set the oversampling ratio, then release the reset.
	m_ADCReset.Write(LOW);
	m_OSR0.Write(nOSR0);
	m_OSR1.Write(nOSR1);
	m_OSR2.Write(nOSR2);
	CTimer::SimpleMsDelay(1);
	m_ADCReset.Write(HIGH);

	// Give the ADC time to start the I2S clocks
	CTimer::SimpleMsDelay(10);

	LOGNOTE("Audio: I2S slave, %u Hz", nSampleRate);

	return true;
#endif
}

bool CPisoundDevice::Initialize()
{
#if RASPPI >= 5
	return false;
#else
	m_Button.AssignPin(PISOUND_PIN_BUTTON);
	m_Button.SetMode(GPIOModeInputPullUp);

	m_DataAvailable.AssignPin(PISOUND_PIN_DATA_AVAILABLE);
	m_DataAvailable.SetMode(GPIOModeInput);

	m_SPIReset.AssignPin(PISOUND_PIN_SPI_RESET);
	m_SPIReset.SetMode(GPIOModeOutput, false);

	m_pSPIMaster = new CSPIMaster(PISOUND_SPI_SPEED_HZ, 0, 0, PISOUND_SPI_BUS);
	if (!m_pSPIMaster->Initialize())
	{
		LOGERR("Cannot initialize SPI%u", PISOUND_SPI_BUS);
		delete m_pSPIMaster;
		m_pSPIMaster = nullptr;
		return false;
	}

	// Reset the microcontroller (pisnd_spi_gpio_init)
	m_SPIReset.Write(LOW);
	CTimer::SimpleMsDelay(1);
	m_SPIReset.Write(HIGH);

	// Give time for the SPI slave to start
	CTimer::SimpleMsDelay(64);

	if (!ReadInfo())
	{
		LOGERR("No answer from the Pisound MIDI controller");
		return false;
	}

	// Flash the LEDs, like the Linux driver does at start-up
	Transfer16(0xF008);

	m_nLastTransferTicks = CTimer::GetClockTicks();
	m_bMIDIReady = true;

	AddDevice("pisound");

	return true;
#endif
}

void CPisoundDevice::Process()
{
	if (!m_bMIDIReady)
	{
		return;
	}

	uint8_t TxBuffer[PISOUND_TRANSFER_SIZE];
	uint8_t RxBuffer[PISOUND_TRANSFER_SIZE];

	for (unsigned nTransfer = 0; nTransfer < PISOUND_MAX_TRANSFERS_PER_PROCESS; nTransfer++)
	{
		// Space in the HAT's MIDI output buffer becomes available
		// at the MIDI byte rate.
		unsigned nNow = CTimer::GetClockTicks();
		unsigned nElapsedMs = (nNow - m_nLastTransferTicks) / (CLOCKHZ / 1000);
		if (nElapsedMs > 0)
		{
			m_nOutBufferUsedMilliBytes -= static_cast<int>(nElapsedMs) * PISOUND_MIDI_MILLIBYTES_PER_MS;
			if (m_nOutBufferUsedMilliBytes < 0)
			{
				m_nOutBufferUsedMilliBytes = 0;
			}
			m_nLastTransferTicks += nElapsedMs * (CLOCKHZ / 1000);
		}

		bool bTxPending = m_nTxIn != m_nTxOut;
		if (!bTxPending && !HasMore())
		{
			break;
		}

		memset(TxBuffer, 0, sizeof TxBuffer);
		for (unsigned i = 0; i < PISOUND_TRANSFER_SIZE; i += 2)
		{
			if (m_nTxIn == m_nTxOut || m_nOutBufferUsedMilliBytes + 1000 >= PISOUND_OUTPUT_BUFFER_SIZE_MILLIBYTES)
			{
				break;
			}

			TxBuffer[i + 0] = PISOUND_CMD_MIDI_BYTE;
			TxBuffer[i + 1] = m_TxFIFO[m_nTxOut];
			m_nTxOut = (m_nTxOut + 1) % TxFIFOSize;
			m_nOutBufferUsedMilliBytes += 1000;
		}

		Transfer(TxBuffer, RxBuffer, sizeof TxBuffer);

		bool bHadData = false;
		for (unsigned i = 0; i < PISOUND_TRANSFER_SIZE; i += 2)
		{
			if (RxBuffer[i])
			{
				ParseMIDIByte(RxBuffer[i + 1]);
				bHadData = true;
			}
		}

		// Nothing received and the HAT's output buffer is full:
		// wait for the next Process() call.
		if (!bHadData && !HasMore() && m_nOutBufferUsedMilliBytes + 1000 >= PISOUND_OUTPUT_BUFFER_SIZE_MILLIBYTES)
		{
			break;
		}
	}
}

void CPisoundDevice::Send(const uint8_t *pMessage, int nLength, int nCable)
{
	if (!m_bMIDIReady)
	{
		return;
	}

	for (int i = 0; i < nLength; i++)
	{
		unsigned nNext = (m_nTxIn + 1) % TxFIFOSize;
		if (nNext == m_nTxOut)
		{
			LOGWARN("MIDI output buffer overflow");
			return;
		}

		m_TxFIFO[m_nTxIn] = pMessage[i];
		m_nTxIn = nNext;
	}
}

bool CPisoundDevice::IsButtonPressed() const
{
	return m_bMIDIReady && m_Button.Read() == LOW;
}

uint16_t CPisoundDevice::Transfer16(uint16_t nValue)
{
	uint8_t TxBuffer[2] = {static_cast<uint8_t>(nValue >> 8), static_cast<uint8_t>(nValue & 0xFF)};
	uint8_t RxBuffer[2];

	Transfer(TxBuffer, RxBuffer, sizeof TxBuffer);

	return static_cast<uint16_t>((RxBuffer[0] << 8) | RxBuffer[1]);
}

void CPisoundDevice::Transfer(const uint8_t *pTxBuffer, uint8_t *pRxBuffer, unsigned nLength)
{
	assert(m_pSPIMaster);

	memset(pRxBuffer, 0, nLength);

	// The Linux driver keeps CS active for 10 us after each transfer
	m_pSPIMaster->SetCSHoldTime(PISOUND_SPI_DELAY_US);
	if (m_pSPIMaster->WriteRead(PISOUND_SPI_CS, pTxBuffer, pRxBuffer, nLength) != static_cast<int>(nLength))
	{
		LOGERR("SPI transfer failed");
		memset(pRxBuffer, 0, nLength);
	}
}

bool CPisoundDevice::ReadBytes(uint8_t *pBuffer, unsigned nMaxLength, unsigned *pBytesRead)
{
	*pBytesRead = 0;

	uint16_t nRx = Transfer16(0);
	if (!(nRx >> 8))
	{
		return false;
	}

	unsigned nSize = nRx & 0xFF;
	if (nSize > nMaxLength)
	{
		return false;
	}

	for (unsigned i = 0; i < nSize; i++)
	{
		nRx = Transfer16(0);
		if (!(nRx >> 8))
		{
			return false;
		}

		pBuffer[i] = nRx & 0xFF;
	}

	*pBytesRead = nSize;

	return true;
}

bool CPisoundDevice::ReadInfo()
{
	// Same sequence as spi_read_info() in pisound.c
	uint16_t nRx = Transfer16(0);
	if (!(nRx >> 8))
	{
		return false;
	}

	unsigned nCount = nRx & 0xFF;

	char Serial[32] = "";
	char FirmwareVersion[16] = "";
	char HardwareVersion[16] = "1.0";

	for (unsigned i = 0; i < nCount; i++)
	{
		uint8_t Buffer[256];
		unsigned nRead;
		if (!ReadBytes(Buffer, sizeof Buffer, &nRead))
		{
			return false;
		}

		switch (i)
		{
		case 0:
			if (nRead == 2)
			{
				snprintf(FirmwareVersion, sizeof FirmwareVersion, "%x.%02x", Buffer[0], Buffer[1]);
			}
			break;

		case 1:
			if (nRead < sizeof Serial)
			{
				memcpy(Serial, Buffer, nRead);
				Serial[nRead] = '\0';
			}
			break;

		case 3:
			if (nRead == 2)
			{
				snprintf(HardwareVersion, sizeof HardwareVersion, "%x.%x", Buffer[0], Buffer[1]);
			}
			break;

		default:
			break;
		}
	}

	LOGNOTE("MIDI: serial %s, firmware %s, hardware %s", Serial, FirmwareVersion, HardwareVersion);

	return true;
}

bool CPisoundDevice::HasMore() const
{
	return m_DataAvailable.Read() == HIGH;
}

// Incoming MIDI byte parser. This is the same logic as in
// CSerialMIDIDevice::Process(), applied to one byte at a time.
void CPisoundDevice::ParseMIDIByte(uint8_t uchData)
{
	if (uchData == 0xF0)
	{
		// SYSEX found
		m_nSysEx = 0;
		m_Message[m_nSysEx++] = uchData;
		return;
	}

	// System Real Time messages may appear anywhere in the byte stream
	if (uchData == 0xF8 || uchData == 0xFA || uchData == 0xFB || uchData == 0xFC || uchData == 0xFE || uchData == 0xFF)
	{
		MIDIMessageHandler(&uchData, 1);
		return;
	}

	if (m_nSysEx > 0)
	{
		m_Message[m_nSysEx++] = uchData;
		if ((uchData & 0x80) == 0x80 || m_nSysEx >= MAX_MIDI_MESSAGE)
		{
			if (uchData == 0xF7)
			{
				MIDIMessageHandler(m_Message, m_nSysEx);
			}
			m_nSysEx = 0;
		}
		return;
	}

	switch (m_nParserState)
	{
	case 0:
	MIDIRestart:
		if ((uchData & 0x80) == 0x80 // status byte, all channels
		    && (uchData & 0xF0) != 0xF0) // ignore system messages
		{
			m_Message[m_nParserState++] = uchData;
		}
		break;

	case 1:
	case 2:
	DATABytes:
		if (uchData & 0x80) // got status when parameter expected
		{
			m_nParserState = 0;
			goto MIDIRestart;
		}

		m_Message[m_nParserState++] = uchData;

		if ((m_Message[0] & 0xE0) == 0xC0 || m_nParserState == 3 // message is complete
		    || (m_Message[0] & 0xF0) == 0xD0) // channel aftertouch
		{
			MIDIMessageHandler(m_Message, m_nParserState);

			m_nParserState = 4; // test if the next byte is a status byte or a data byte (running status)
		}
		break;

	case 4:
		if ((uchData & 0x80) == 0) // data byte: running status
		{
			m_nParserState = 1;
			goto DATABytes;
		}
		else
		{
			m_nParserState = 0;
			goto MIDIRestart;
		}
		break;

	default:
		assert(0);
		break;
	}
}
