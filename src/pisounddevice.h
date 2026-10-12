//
// pisounddevice.h
//
// Support for the Blokas Pisound HAT (https://blokas.io/pisound/):
//   - Audio: the Pisound ADC is the I2S clock master, so the Raspberry Pi
//     runs I2S in slave mode. The sample rate is selected with three
//     "oversampling ratio" GPIOs (OSR0..2) while the ADC is held in reset.
//   - MIDI: DIN MIDI in/out is handled by a microcontroller on the HAT,
//     which talks to the Raspberry Pi over SPI0 (CE0) at 150 kHz.
//
// Protocol and pin assignments are taken from the Linux driver
// sound/soc/bcm/pisound.c and overlays/pisound-overlay.dts
// (Copyright (C) 2016-2024 Vilniaus Blokas UAB, GPL v2).
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
#pragma once

#include <cstdint>

#include <circle/gpiopin.h>
#include <circle/spimaster.h>

#include "config.h"
#include "mididevice.h"
#include "userinterface.h"

class CMiniDexed;

class CPisoundDevice : public CMIDIDevice
{
public:
	CPisoundDevice(CMiniDexed *pSynthesizer, CConfig *pConfig, CUserInterface *pUI);
	~CPisoundDevice();

	// Selects the sample rate and releases the ADC from reset, so that
	// the I2S clocks run. Call this before the I2S sound device is created.
	// Only 48000, 96000 and 192000 Hz are supported by the hardware.
	bool InitAudio(unsigned nSampleRate);

	// Resets the MIDI microcontroller and checks that it answers.
	bool Initialize();

	// Exchanges MIDI bytes with the HAT. Call this from the main loop.
	void Process();

	void Send(const uint8_t *pMessage, int nLength, int nCable = 0) override;

	// State of the Pisound button (true = pressed).
	bool IsButtonPressed() const;

private:
	uint16_t Transfer16(uint16_t nValue);
	void Transfer(const uint8_t *pTxBuffer, uint8_t *pRxBuffer, unsigned nLength);
	bool ReadBytes(uint8_t *pBuffer, unsigned nMaxLength, unsigned *pBytesRead);
	bool ReadInfo();
	bool HasMore() const;
	void ParseMIDIByte(uint8_t uchData);

private:
	CConfig *m_pConfig;

	// Audio pins (BCM numbering, see pisound-overlay.dts)
	CGPIOPin m_OSR0;	// GPIO 13
	CGPIOPin m_OSR1;	// GPIO 26
	CGPIOPin m_OSR2;	// GPIO 16
	CGPIOPin m_ADCReset;	// GPIO 12

	// MIDI / control pins
	CGPIOPin m_SPIReset;	  // GPIO 24
	CGPIOPin m_DataAvailable; // GPIO 25
	CGPIOPin m_Button;	  // GPIO 17, active low

	CSPIMaster *m_pSPIMaster;
	bool m_bMIDIReady;

	// Outgoing MIDI bytes (ring buffer)
	static const unsigned TxFIFOSize = 1024;
	uint8_t m_TxFIFO[TxFIFOSize];
	volatile unsigned m_nTxIn;
	volatile unsigned m_nTxOut;

	// Estimate of the HAT's MIDI output buffer usage, so that it does not
	// overflow (same method as the Linux driver).
	int m_nOutBufferUsedMilliBytes;
	unsigned m_nLastTransferTicks;

	// Incoming MIDI parser state (same logic as CSerialMIDIDevice)
	int m_nParserState;
	int m_nSysEx;
	uint8_t m_Message[MAX_MIDI_MESSAGE];
};
