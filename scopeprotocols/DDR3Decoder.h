/***********************************************************************************************************************
*                                                                                                                      *
* ANTIKERNEL v0.1                                                                                                      *
*                                                                                                                      *
* Copyright (c) 2012-2020 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief Declaration of DDR3Decoder
 */

#ifndef DDR3Decoder_h
#define DDR3Decoder_h

#include "../scopehal/ProtocolDecoder.h"

class DDR3Symbol
{
public:
	enum stype
	{
		TYPE_MRS,
		TYPE_REF,
		TYPE_PRE,
		TYPE_PREA,
		TYPE_ACT,
		TYPE_WR,
		TYPE_WRA,
		TYPE_RD,
		TYPE_RDA,

		TYPE_ERROR
	};

	DDR3Symbol()
	{}

	DDR3Symbol(stype t, int bank = 0)
	 : m_stype(t)
	 , m_bank(bank)
	{}

	stype m_stype;
	int m_bank;

	bool operator== (const DDR3Symbol& s) const
	{
		return (m_stype == s.m_stype) && (m_bank == s.m_bank);
	}
};

typedef Waveform<DDR3Symbol> DDR3Waveform;

class DDR3Decoder : public ProtocolDecoder
{
public:
	DDR3Decoder(std::string color);

	virtual std::string GetText(int i);
	virtual Gdk::Color GetColor(int i);

	virtual void Refresh();
	virtual bool NeedsConfig();

	static std::string GetProtocolName();
	virtual void SetDefaultName();

	virtual bool ValidateChannel(size_t i, OscilloscopeChannel* channel);

	PROTOCOL_DECODER_INITPROC(DDR3Decoder)

protected:
};

#endif
