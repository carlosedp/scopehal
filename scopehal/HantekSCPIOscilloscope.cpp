/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg and contributors                                                         *
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

/*
 * Current State
 * =============
 *
 * - Basic functionality for analog channels works.
 * - There is no feature detection because the scope does not support *OPT? (Request made)
 * - Digital channels are not implemented (code in here is leftover from LeCroy)
 * - Triggers are untested.
 * - Sampling lengths up to 10MSamples are supported. 50M and 100M need to be batched and will be
 *   horribly slow.
 *
 * SDS2000/5000/6000 port (c) 2021 Dave Marples. Note that this is only tested on SDS2000X+. If someone wants
 * to loan an SDS5000/6000 for testing that can be integrated. This file is derrived from the LeCroy driver.
 *
 */

#include "scopehal.h"
#include "HantekSCPIOscilloscope.h"
#include "base64.h"
#include <locale>
#include <stdarg.h>
#include <omp.h>
#include <thread>
#include <chrono>

#include "EdgeTrigger.h"
#include "PulseWidthTrigger.h"
#include "SlewRateTrigger.h"
#include "UartTrigger.h"
#include "WindowTrigger.h"

using namespace std;

static const struct
{
	const char* name;
	float val;
} c_threshold_table[] = {{"TTL", 1.5F}, {"CMOS", 1.65F}, {"LVCMOS33", 1.65F}, {"LVCMOS25", 1.25F}, {NULL, 0}};

static const std::chrono::milliseconds c_setting_delay(50);		 // Delay required when setting parameters via SCPI
static const std::chrono::milliseconds c_trigger_delay(1000);	 // Delay required when forcing trigger
static const char* c_custom_thresh = "CUSTOM,";					 // Prepend string for custom digital threshold
static const float c_thresh_thresh = 0.01f;						 // Zero equivalence threshold for fp comparisons

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

HantekSCPIOscilloscope::HantekSCPIOscilloscope(SCPITransport* transport)
	: SCPIOscilloscope(transport)
	, m_hasLA(false)
	, m_hasDVM(false)
	, m_hasFunctionGen(false)
	, m_hasFastSampleRate(false)
	, m_memoryDepthOption(0)
	, m_hasI2cTrigger(false)
	, m_hasSpiTrigger(false)
	, m_hasUartTrigger(false)
	, m_maxBandwidth(10000)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
	, m_sampleRateValid(false)
	, m_sampleRate(1)
	, m_memoryDepthValid(false)
	, m_memoryDepth(1)
	, m_triggerOffsetValid(false)
	, m_triggerOffset(0)
	, m_interleaving(false)
	, m_interleavingValid(false)
	, m_highDefinition(false)
{
	// Set a base read time
	next_tx = chrono::system_clock::now();

	//standard initialization
	FlushConfigCache();
	IdentifyHardware();
	DetectAnalogChannels();
	DetectOptions();
}

string HantekSCPIOscilloscope::converse(const char* fmt, ...)

{
	string ret;
	char opString[128];
	va_list va;
	va_start(va, fmt);
	vsnprintf(opString, sizeof(opString), fmt, va);
	va_end(va);

	this_thread::sleep_until(next_tx);
	// m_transport->FlushRXBuffer();
	m_transport->SendCommand(opString);
	ret = m_transport->ReadReply();
	return ret;
}

void HantekSCPIOscilloscope::sendOnly(const char* fmt, ...)

{
	char opString[128];
	va_list va;

	va_start(va, fmt);
	vsnprintf(opString, sizeof(opString), fmt, va);
	va_end(va);

	this_thread::sleep_until(next_tx);
	// m_transport->FlushRXBuffer();
	m_transport->SendCommand(opString);
	next_tx = chrono::system_clock::now() + c_setting_delay;
}

void HantekSCPIOscilloscope::IdentifyHardware()
{
	//Ask for the ID
	string reply = converse("*IDN?");
	char vendor[128] = "";
	char model[128] = "";
	char serial[128] = "";
	char version[128] = "";
	if(4 != sscanf(reply.c_str(), "%127[^,],%127[^,],%127[^,],%127s", vendor, model, serial, version))
	{
		LogError("Bad IDN response %s\n", reply.c_str());
		return;
	}
	m_vendor = vendor;
	m_model = model;
	m_serial = serial;
	m_fwVersion = version;

	//Look up model info
	m_modelid = MODEL_UNKNOWN;
	m_maxBandwidth = 0;

	if(m_vendor.compare("undefined") == 0)
	{
        m_vendor = "Hantek";
		if(m_model.compare(0, 4, "DSO2") == 0)
		{
			m_modelid = MODEL_HANTEK_DSO2000;

			m_maxBandwidth = 100;
			if(m_model.compare(5, 2, "15") == 0)
				m_maxBandwidth = 150;
			return;
		}
	}
	LogWarning("Model \"%s\" is unknown, available sample rates/memory depths may not be properly detected\n",
		m_model.c_str());
}

void HantekSCPIOscilloscope::DetectOptions()
{
	/* It's possible to detect the Signal Generator thru the model number */
	return;
}

/**
	@brief Figures out how many analog channels we have, and add them to the device

 */
void HantekSCPIOscilloscope::DetectAnalogChannels()
{
	int nchans = 1;

	// Char 3 of the model name is the number of channels
	if(m_model.length() > 4)
	{
		switch(m_model[3])
		{
			case '2':
				nchans = 2;
				break;
			case '4':
				nchans = 4;
				break;
		}
	}

	for(int i = 0; i < nchans; i++)
	{
		//Hardware name of the channel
		string chname = string("C1");
		chname[1] += i;

		//Color the channels based on Siglents standard color sequence
		//yellow-pink-cyan-green-lightgreen
		string color = "#ffffff";
		switch(i % 4)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#00c100";
				break;

			case 2:
				color = "#00ffff";
				break;

			case 3:
				color = "#ff6abc";
				break;
		}

		//Create the channel
		m_channels.push_back(
			new OscilloscopeChannel(this, chname, OscilloscopeChannel::CHANNEL_TYPE_ANALOG, color, 1, i, true));
	}
	m_analogChannelCount = nchans;
}

HantekSCPIOscilloscope::~HantekSCPIOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device information

string HantekSCPIOscilloscope::GetDriverNameInternal()
{
	return "hantek";
}

OscilloscopeChannel* HantekSCPIOscilloscope::GetExternalTrigger()
{
	return m_extTrigChannel;
}

void HantekSCPIOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	if(m_trigger)
		delete m_trigger;
	m_trigger = NULL;

	m_channelVoltageRanges.clear();
	m_channelOffsets.clear();
	m_channelsEnabled.clear();
	m_channelDeskew.clear();
	m_channelDisplayNames.clear();
	m_probeIsActive.clear();
	m_sampleRateValid = false;
	m_memoryDepthValid = false;
	m_triggerOffsetValid = false;
	m_interleavingValid = false;
	m_meterModeValid = false;
}

/**
	@brief See what measurement capabilities we have
 */
unsigned int HantekSCPIOscilloscope::GetMeasurementTypes()
{
	unsigned int type = 0;
	return type;
}

/**
	@brief See what features we have
 */
unsigned int HantekSCPIOscilloscope::GetInstrumentTypes()
{
	unsigned int type = INST_OSCILLOSCOPE;
	if(m_hasFunctionGen)
		type |= INST_FUNCTION;
	return type;
}

string HantekSCPIOscilloscope::GetName()
{
	return m_model;
}

string HantekSCPIOscilloscope::GetVendor()
{
	return m_vendor;
}

string HantekSCPIOscilloscope::GetSerial()
{
	return m_serial;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel configuration

bool HantekSCPIOscilloscope::IsChannelEnabled(size_t i)
{
	//ext trigger should never be displayed
	if(i == m_extTrigChannel->GetIndex())
		return false;

	//Early-out if status is in cache
	{
		lock_guard<recursive_mutex> lock2(m_cacheMutex);
		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	//Need to lock the main mutex first to prevent deadlocks
	lock_guard<recursive_mutex> lock(m_mutex);
	lock_guard<recursive_mutex> lock2(m_cacheMutex);

	//Analog
	if(i < m_analogChannelCount)
	{
		//See if the channel is enabled, hide it if not
		string reply = converse(":CHANNEL%d:DISPlay?", i + 1);
		m_channelsEnabled[i] = (reply.find("0") == 0);	//may have a trailing newline, ignore that
	}

	return m_channelsEnabled[i];
}

void HantekSCPIOscilloscope::EnableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//If this is an analog channel, just toggle it
	if(i < m_analogChannelCount)
	{
		sendOnly(":CHANNEL%d:DISPlay 1", i + 1);
	}

	//Trigger can't be enabled
	else if(i == m_extTrigChannel->GetIndex())
	{
	}
}

bool HantekSCPIOscilloscope::CanEnableChannel(size_t i)
{
	// Can enable all channels except trigger
	return !(i == m_extTrigChannel->GetIndex());
}

void HantekSCPIOscilloscope::DisableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	m_channelsEnabled[i] = false;

	//If this is an analog channel, just toggle it
	if(i < m_analogChannelCount)
		sendOnly(":CHANNEL%d:DISPlay 0", i + 1);

	//Trigger can't be enabled
	else if(i == m_extTrigChannel->GetIndex())
	{
	}
}

vector<OscilloscopeChannel::CouplingType> HantekSCPIOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_GND);
	return ret;
}

OscilloscopeChannel::CouplingType HantekSCPIOscilloscope::GetChannelCoupling(size_t i)
{
	if(i >= m_analogChannelCount)
		return OscilloscopeChannel::COUPLE_SYNTHETIC;

	string replyType;
	string replyImp;

	lock_guard<recursive_mutex> lock(m_mutex);

	replyType = Trim(converse(":CHANNEL%d:COUPLING?", i + 1).substr(0, 2));

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_probeIsActive[i] = false;

	if(replyType == "AC")
		return OscilloscopeChannel::COUPLE_AC_1M;
	else if(replyType == "DC")
		return OscilloscopeChannel::COUPLE_DC_1M;
	else if(replyType == "GND")
		return OscilloscopeChannel::COUPLE_GND;

	//invalid
	LogWarning("HantekSCPIOscilloscope::GetChannelCoupling got invalid coupling [%s] [%s]\n",
		replyType.c_str(),
		replyImp.c_str());
	return OscilloscopeChannel::COUPLE_SYNTHETIC;
}

void HantekSCPIOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	if(i >= m_analogChannelCount)
		return;

	//Get the old coupling value first.
	//This ensures that m_probeIsActive[i] is valid
	GetChannelCoupling(i);

	//If we have an active probe, don't touch the hardware config
	if(m_probeIsActive[i])
		return;

	lock_guard<recursive_mutex> lock(m_mutex);
	switch(type)
	{
		case OscilloscopeChannel::COUPLE_AC_1M:
			sendOnly(":CHANNEL%d:COUPLING AC", i + 1);
			break;

		case OscilloscopeChannel::COUPLE_DC_1M:
			sendOnly(":CHANNEL%d:COUPLING DC", i + 1);
			break;

		//treat unrecognized as ground
		case OscilloscopeChannel::COUPLE_GND:
		default:
			sendOnly(":CHANNEL%d:COUPLING GND", i + 1);
			break;
	}
}

double HantekSCPIOscilloscope::GetChannelAttenuation(size_t i)
{
	if(i > m_analogChannelCount)
		return 1;

	//TODO: support ext/10
	if(i == m_extTrigChannel->GetIndex())
		return 1;

	lock_guard<recursive_mutex> lock(m_mutex);

	string reply = converse(":CHANNEL%d:PROBE?", i + 1);

	double d;
	sscanf(reply.c_str(), "%lf", &d);
	return d;
}

void HantekSCPIOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	if(i >= m_analogChannelCount)
		return;

	//Get the old coupling value first.
	//This ensures that m_probeIsActive[i] is valid
	GetChannelCoupling(i);

	//Don't allow changing attenuation on active probes
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_probeIsActive[i])
			return;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	sendOnly(":CHANNEL%d:PROBE %lf", i + 1, atten);
}

vector<unsigned int> HantekSCPIOscilloscope::GetChannelBandwidthLimiters(size_t /*i*/)
{
	vector<unsigned int> ret;

	//"no limit"
	ret.push_back(0);

	//Supported by all models
	ret.push_back(20);

	return ret;
}

int HantekSCPIOscilloscope::GetChannelBandwidthLimit(size_t i)
{
	if(i > m_analogChannelCount)
		return 0;

	lock_guard<recursive_mutex> lock(m_mutex);
	string reply = converse(":CHANNEL%d:BWLIMIT?", i + 1);
	if(reply == "0")
		return 0;
	else if(reply == "1")
		return 20;

	LogWarning("HantekSCPIOscilloscope::GetChannelCoupling got invalid bwlimit %s\n", reply.c_str());
	return 0;
}

void HantekSCPIOscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	switch(limit_mhz)
	{
		case 0:
			sendOnly(":CHANNEL%d:BWLIMIT 0", i + 1);
			break;

		case 20:
			sendOnly(":CHANNEL%d:BWLIMIT 1", i + 1);
			break;

		default:
			LogWarning("HantekSCPIOscilloscope::invalid bwlimit set request (%dMhz)\n", limit_mhz);
	}
}

bool HantekSCPIOscilloscope::CanInvert(size_t i)
{
	//All analog channels, and only analog channels, can be inverted
	return (i < m_analogChannelCount);
}

void HantekSCPIOscilloscope::Invert(size_t i, bool invert)
{
	if(i >= m_analogChannelCount)
		return;

	lock_guard<recursive_mutex> lock(m_mutex);
	sendOnly(":CHANNEL%d:INVERT %s", i + 1, invert ? "1" : "0");
}

bool HantekSCPIOscilloscope::IsInverted(size_t i)
{
	if(i >= m_analogChannelCount)
		return false;

	lock_guard<recursive_mutex> lock(m_mutex);
	auto reply = Trim(converse(":CHANNEL%d:INVERT?", i + 1));
	return (reply == "1");
}

void HantekSCPIOscilloscope::SetChannelDisplayName(size_t i, string name)
{
	auto chan = m_channels[i];

	//External trigger cannot be renamed in hardware.
	//TODO: allow clientside renaming?
	if(chan == m_extTrigChannel)
		return;

	//Update cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelDisplayNames[m_channels[i]] = name;
	}
    return;
}

string HantekSCPIOscilloscope::GetChannelDisplayName(size_t i)
{
	auto chan = m_channels[i];

	//External trigger cannot be renamed in hardware.
	//TODO: allow clientside renaming?
	if(chan == m_extTrigChannel)
		return m_extTrigChannel->GetHwname();

	//Check cache first
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelDisplayNames.find(chan) != m_channelDisplayNames.end())
			return m_channelDisplayNames[chan];
	}

	lock_guard<recursive_mutex> lock(m_mutex);

	string name;

	//Default to using hwname if no alias defined
	if(name == "")
		name = chan->GetHwname();

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelDisplayNames[chan] = name;

	return name;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Triggering

bool HantekSCPIOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

Oscilloscope::TriggerMode HantekSCPIOscilloscope::PollTrigger()

{
	//Read the Internal State Change Register
	string sinr;
	lock_guard<recursive_mutex> lock(m_mutex);

	if(m_triggerForced)
	{
		// The force trigger completed, return the sample set
		m_triggerForced = false;
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}

	sinr = converse(":TRIGGER:STATUS?");

	//No waveform, but ready for one?
	if((sinr == "NOTRIG")
	{
		m_triggerArmed = true;
		return TRIGGER_MODE_RUN;
	}

	//Stopped, no data available
	if(sinr == "TRIGed")
	{
		if(m_triggerArmed)
		{
			m_triggerArmed = false;
			return TRIGGER_MODE_TRIGGERED;
		}
		else
			return TRIGGER_MODE_STOP;
	}
	return TRIGGER_MODE_RUN;
}

// TODO!!!
int HantekSCPIOscilloscope::ReadWaveformBlock(uint32_t maxsize, char* data)

{
	char packetSizeSequence[17];
	uint32_t getLength;

	// Get size of this sequence
	m_transport->ReadRawData(7, (unsigned char*)packetSizeSequence);

	// This is an aweful cludge, but the response can be in different formats depending on
	// if this was a direct trigger or a forced trigger. This is the report format for a direct trigger
	if((!strncmp(packetSizeSequence, "DESC,#9", 7)) || (!strncmp(packetSizeSequence, "DAT2,#9", 7)))
	{
		m_transport->ReadRawData(9, (unsigned char*)packetSizeSequence);
	}

	// This is the report format for a forced trigger
	if(!strncmp(&packetSizeSequence[2], ":WF D", 5))
	{
		// Read the front end junk, then the actually number we're looking for
		m_transport->ReadRawData(6, (unsigned char*)packetSizeSequence);
		m_transport->ReadRawData(9, (unsigned char*)packetSizeSequence);
	}

	packetSizeSequence[9] = 0;
	LogTrace("INITIAL PACKET [%s]\n", packetSizeSequence);
	getLength = atoi(packetSizeSequence);

	// Now get the data
	m_transport->ReadRawData((getLength > maxsize) ? maxsize : getLength, (unsigned char*)data);

	return getLength;
}

/**
	@brief Optimized function for checking channel enable status en masse with less round trips to the scope
 */
void HantekSCPIOscilloscope::BulkCheckChannelEnableState()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	//Check enable state in the cache.
	vector<int> uncached;
	for(unsigned int i = 0; i < m_analogChannelCount; i++)
	{
		if(m_channelsEnabled.find(i) == m_channelsEnabled.end())
			uncached.push_back(i);
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	for(auto i : uncached)
	{
		string reply = converse(":CHANNEL%d:DISPlay?", i + 1);
		if(reply == "0")
			m_channelsEnabled[i] = false;
		else if(reply == "1")
			m_channelsEnabled[i] = true;
		else
			LogWarning("BulkCheckChannelEnableState: Unrecognised reply [%s]\n", reply.c_str());
	}
}

// TODO!!!
bool HantekSCPIOscilloscope::ReadWavedescs(
	char wavedescs[MAX_ANALOG][WAVEDESC_SIZE], bool* enabled, unsigned int& firstEnabledChannel, bool& any_enabled)
{
	BulkCheckChannelEnableState();
	for(unsigned int i = 0; i < m_analogChannelCount; i++)
	{
		enabled[i] = IsChannelEnabled(i);
		any_enabled |= enabled[i];
	}

	for(unsigned int i = 0; i < m_analogChannelCount; i++)
	{
		if(enabled[i] || (!any_enabled && i == 0))
		{
			if(firstEnabledChannel == UINT_MAX)
				firstEnabledChannel = i;

			m_transport->SendCommand(":WAVEFORM:SOURCE C" + to_string(i + 1) + ";:WAVEFORM:PREAMBLE?");
			if(WAVEDESC_SIZE != ReadWaveformBlock(WAVEDESC_SIZE, wavedescs[i]))
				LogError("ReadWaveformBlock for wavedesc %u failed\n", i);

			// I have no idea why this is needed, but it certainly is
			m_transport->ReadReply();
		}
	}

	return true;
}

// TODO!!!
time_t HantekSCPIOscilloscope::ExtractTimestamp(unsigned char* wavedesc, double& basetime)
{
	/*
                TIMESTAMP is shown as Reserved In Siglent data format.
                This information is from LeCroy which uses the same wavedesc header.
		Timestamp is a somewhat complex format that needs some shuffling around.
		Timestamp starts at offset 296 bytes in the wavedesc
		(296-303)	double seconds
		(304)		byte minutes
		(305)		byte hours
		(306)		byte days
		(307)		byte months
		(308-309)	uint16 year

		TODO: during startup, query instrument for its current time zone
		since the wavedesc reports instment local time
	 */
	//Yes, this cast is intentional.
	//It assumes you're on a little endian system using IEEE754 64-bit float, but that applies to everything we support.
	//cppcheck-suppress invalidPointerCast
	double fseconds = *reinterpret_cast<const double*>(wavedesc + 296);
	uint8_t seconds = floor(fseconds);
	basetime = fseconds - seconds;
	time_t tnow = time(NULL);
	struct tm tstruc;

#ifdef _WIN32
	localtime_s(&tstruc, &tnow);
#else
	localtime_r(&tnow, &tstruc);
#endif

	//Convert the instrument time to a string, then back to a tm
	//Is there a better way to do this???
	//Naively poking "struct tm" fields gives incorrect results (scopehal-apps:#52)
	//Maybe because tm_yday is inconsistent?
	char tblock[64] = {0};
	snprintf(tblock,
		sizeof(tblock),
		"%d-%d-%d %d:%02d:%02d",
		*reinterpret_cast<uint16_t*>(wavedesc + 308),
		wavedesc[307],
		wavedesc[306],
		wavedesc[305],
		wavedesc[304],
		seconds);
	locale cur_locale;
	auto& tget = use_facet<time_get<char>>(cur_locale);
	istringstream stream(tblock);
	ios::iostate state;
	char format[] = "%F %T";
	tget.get(stream, time_get<char>::iter_type(), stream, state, &tstruc, format, format + strlen(format));
	return mktime(&tstruc);
}

// TODO!!!
vector<WaveformBase*> HantekSCPIOscilloscope::ProcessAnalogWaveform(const char* data,
	size_t datalen,
	char* wavedesc,
	uint32_t num_sequences,
	time_t ttime,
	double basetime,
	double* wavetime,
	int /* ch */)
{
	vector<WaveformBase*> ret;

	//Parse the wavedesc headers
	auto pdesc = wavedesc;

	//cppcheck-suppress invalidPointerCast
	float v_gain = *reinterpret_cast<float*>(pdesc + 156);

	//cppcheck-suppress invalidPointerCast
	float v_off = *reinterpret_cast<float*>(pdesc + 160);

	//cppcheck-suppress invalidPointerCast
	float v_probefactor = *reinterpret_cast<float*>(pdesc + 328);

	//cppcheck-suppress invalidPointerCast
	float interval = *reinterpret_cast<float*>(pdesc + 176) * FS_PER_SECOND;

	//cppcheck-suppress invalidPointerCast
	double h_off = *reinterpret_cast<double*>(pdesc + 180) * FS_PER_SECOND;	   //fs from start of waveform to trigger

	//double h_off_frac = fmodf(h_off, interval);	   //fractional sample position, in fs

	double h_off_frac = 0;	  //((interval*datalen)/2)+h_off;

	if(h_off_frac < 0)
		h_off_frac = h_off;	   //interval + h_off_frac;	   //double h_unit = *reinterpret_cast<double*>(pdesc + 244);

	//Raw waveform data
	size_t num_samples;
	if(m_highDefinition)
		num_samples = datalen / 2;
	else
		num_samples = datalen;
	size_t num_per_segment = num_samples / num_sequences;
	int16_t* wdata = (int16_t*)&data[0];
	int8_t* bdata = (int8_t*)&data[0];

	// SDS2000X+ and SDS5000X have 30 codes per div. Todo; SDS6000X has 425.
	// We also need to accomodate probe attenuation here.
	v_gain = v_gain * v_probefactor / 30;

	// Vertical offset is also scaled by the probefactor
	v_off = v_off * v_probefactor;

	// Update channel voltages and offsets based on what is in this wavedesc
	// m_channelVoltageRanges[ch] = v_gain * v_probefactor * 30 * 8;
	// m_channelOffsets[ch] = v_off;
	// m_triggerOffset = ((interval * datalen) / 2) + h_off;
	// m_triggerOffsetValid = true;

	LogTrace("\nV_Gain=%f, V_Off=%f, interval=%f, h_off=%f, h_off_frac=%f, datalen=%ld\n",
		v_gain,
		v_off,
		interval,
		h_off,
		h_off_frac,
		datalen);

	for(size_t j = 0; j < num_sequences; j++)
	{
		//Set up the capture we're going to store our data into
		AnalogWaveform* cap = new AnalogWaveform;
		cap->m_timescale = round(interval);

		cap->m_triggerPhase = h_off_frac;
		cap->m_startTimestamp = ttime;
		cap->m_densePacked = true;

		//Parse the time
		if(num_sequences > 1)
			cap->m_startFemtoseconds = static_cast<int64_t>((basetime + wavetime[j * 2]) * FS_PER_SECOND);
		else
			cap->m_startFemtoseconds = static_cast<int64_t>(basetime * FS_PER_SECOND);

		cap->Resize(num_per_segment);

		//Convert raw ADC samples to volts
		if(m_highDefinition)
		{
			Convert16BitSamples((int64_t*)&cap->m_offsets[0],
				(int64_t*)&cap->m_durations[0],
				(float*)&cap->m_samples[0],
				wdata + j * num_per_segment,
				v_gain,
				v_off,
				num_per_segment,
				0);
		}
		else
		{
			Convert8BitSamples((int64_t*)&cap->m_offsets[0],
				(int64_t*)&cap->m_durations[0],
				(float*)&cap->m_samples[0],
				bdata + j * num_per_segment,
				v_gain,
				v_off,
				num_per_segment,
				0);
		}

		ret.push_back(cap);
	}

	return ret;
}

// TODO!!!
bool HantekSCPIOscilloscope::AcquireData()
{
	//State for this acquisition (may be more than one waveform)
	uint32_t num_sequences = 1;
	map<int, vector<WaveformBase*>> pending_waveforms;
	double start = GetTime();
	time_t ttime = 0;
	double basetime = 0;
	bool denabled = false;
	string wavetime;
	bool enabled[8] = {false};
	double* pwtime = NULL;
	char tmp[128];

	//Acquire the data (but don't parse it)
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		start = GetTime();
		//Get the wavedescs for all channels
		unsigned int firstEnabledChannel = UINT_MAX;
		bool any_enabled = true;

		if(!ReadWavedescs(m_wavedescs, enabled, firstEnabledChannel, any_enabled))
			return false;

		//Grab the WAVEDESC from the first enabled channel
		unsigned char* pdesc = NULL;
		for(unsigned int i = 0; i < m_analogChannelCount; i++)
		{
			if(enabled[i] || (!any_enabled && i == 0))
			{
				pdesc = (unsigned char*)(&m_wavedescs[i][0]);
				break;
			}
		}

		//See if any digital channels are enabled
		if(m_digitalChannelCount > 0)
		{
			m_cacheMutex.lock();
			for(size_t i = 0; i < m_digitalChannels.size(); i++)
			{
				if(m_channelsEnabled[m_digitalChannels[i]->GetIndex()])
				{
					denabled = true;
					break;
				}
			}
			m_cacheMutex.unlock();
		}

		//Pull sequence count out of the WAVEDESC if we have analog channels active
		if(pdesc)
		{
			uint32_t trigtime_len = *reinterpret_cast<uint32_t*>(pdesc + 48);
			if(trigtime_len > 0)
				num_sequences = trigtime_len / 16;
		}

		//No WAVEDESCs, look at digital channels
		else
		{
			//TODO: support sequence capture of digital channels if the instrument supports this
			//(need to look into it)
			if(denabled)
				num_sequences = 1;

			//no enabled channels. abort
			else
				return false;
		}

		if(pdesc)
		{
			// THIS SECTION IS UNTESTED
			//Figure out when the first trigger happened.
			//Read the timestamps if we're doing segmented capture
			ttime = ExtractTimestamp(pdesc, basetime);
			if(num_sequences > 1)
				wavetime = m_transport->ReadReply();
			pwtime = reinterpret_cast<double*>(&wavetime[16]);	  //skip 16-byte SCPI header

			//Read the data from each analog waveform
			for(unsigned int i = 0; i < m_analogChannelCount; i++)
			{
				m_transport->SendCommand(":WAVEFORM:SOURCE C" + to_string(i + 1) + ";:WAVEFORM:DATA?");
				if(enabled[i])
				{
					m_analogWaveformDataSize[i] = ReadWaveformBlock(WAVEFORM_SIZE, m_analogWaveformData[i]);
					// This is the 0x0a0a at the end
					m_transport->ReadRawData(2, (unsigned char*)tmp);
				}
			}
		}

		//Read the data from the digital waveforms, if enabled
		if(denabled)
		{
			if(!ReadWaveformBlock(WAVEFORM_SIZE, m_digitalWaveformDataBytes))
			{
				LogDebug("failed to download digital waveform\n");
				return false;
			}
		}
	}

	//At this point all data has been read so the scope is free to go do its thing while we crunch the results.
	//Re-arm the trigger if not in one-shot mode
	if(!m_triggerOneShot)
	{
		//		lock_guard<recursive_mutex> lock(m_mutex);
		sendOnly(":TRIGger:SWEep SINGLE");
		m_triggerArmed = true;
	}

	//Process analog waveforms
	vector<vector<WaveformBase*>> waveforms;
	waveforms.resize(m_analogChannelCount);
	for(unsigned int i = 0; i < m_analogChannelCount; i++)
	{
		if(enabled[i])
		{
			waveforms[i] = ProcessAnalogWaveform(&m_analogWaveformData[i][0],
				m_analogWaveformDataSize[i],
				&m_wavedescs[i][0],
				num_sequences,
				ttime,
				basetime,
				pwtime,
				i);
		}
	}

	//Save analog waveform data
	for(unsigned int i = 0; i < m_analogChannelCount; i++)
	{
		if(!enabled[i])
			continue;

		//Done, update the data
		for(size_t j = 0; j < num_sequences; j++)
			pending_waveforms[i].push_back(waveforms[i][j]);
	}
	//TODO: proper support for sequenced capture when digital channels are active
	// if(denabled)
	// {
	// 	//This is a weird XML-y format but I can't find any other way to get it :(
	// 	map<int, DigitalWaveform*> digwaves = ProcessDigitalWaveform(m_digitalWaveformData);

	// 	//Done, update the data
	// 	for(auto it : digwaves)
	// 		pending_waveforms[it.first].push_back(it.second);
	// }

	//Now that we have all of the pending waveforms, save them in sets across all channels
	m_pendingWaveformsMutex.lock();
	for(size_t i = 0; i < num_sequences; i++)
	{
		SequenceSet s;
		for(size_t j = 0; j < m_channels.size(); j++)
		{
			if(pending_waveforms.find(j) != pending_waveforms.end())
				s[m_channels[j]] = pending_waveforms[j][i];
		}
		m_pendingWaveforms.push_back(s);
	}
	m_pendingWaveformsMutex.unlock();

	double dt = GetTime() - start;
	LogTrace("Waveform download and processing took %.3f ms\n", dt * 1000);
	return true;
}

void HantekSCPIOscilloscope::Start()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	sendOnly(":running STOP");
	sendOnly(":TRIGger:SWEep SINGLE");	 //always do single captures, just re-trigger
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void HantekSCPIOscilloscope::StartSingleTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	//LogDebug("Start running:MODE STOP");
	sendOnly(":TRIGger:SWEep SINGLE");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void HantekSCPIOscilloscope::Stop()
{
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		sendOnly(":TRIGger:SWEep STOP");
	}

	m_triggerArmed = false;
	m_triggerOneShot = true;

	//Clear out any pending data (the user doesn't want it, and we don't want stale stuff hanging around)
	ClearPendingWaveforms();
}

void HantekSCPIOscilloscope::ForceTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	// Don't allow more than one force at a time
	if(m_triggerForced)
		return;

	m_triggerForced = true;
	sendOnly(":TRIGger:SWEep SINGLE");
	if(!m_triggerArmed)
		sendOnly(":TRIGger:SWEep SINGLE");

	m_triggerArmed = true;
	this_thread::sleep_for(c_trigger_delay);
}

double HantekSCPIOscilloscope::GetChannelOffset(size_t i)
{
	//not meaningful for trigger or digital channels
	if(i > m_analogChannelCount)
		return 0;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	string reply = converse(":CHANNEL%ld:OFFSET?", i + 1);
	double offset;
	sscanf(reply.c_str(), "%lf", &offset);

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void HantekSCPIOscilloscope::SetChannelOffset(size_t i, double offset)
{
	//not meaningful for trigger or digital channels
	if(i > m_analogChannelCount)
		return;

	{
		lock_guard<recursive_mutex> lock2(m_mutex);
		sendOnly(":CHANNEL%ld:OFFSET %1.2E", i + 1, offset);
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
}

double HantekSCPIOscilloscope::GetChannelVoltageRange(size_t i)
{
	//not meaningful for trigger or digital channels
	if(i > m_analogChannelCount)
		return 1;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	string reply = converse(":CHANNEL%d:SCALE?", i + 1);
	double volts_per_div;
	sscanf(reply.c_str(), "%lf", &volts_per_div);

	double v = volts_per_div * 8;	 //plot is 8 divisions high
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = v;
	return v;
}

void HantekSCPIOscilloscope::SetChannelVoltageRange(size_t i, double range)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	double vdiv = range / 8;
	m_channelVoltageRanges[i] = range;

	sendOnly(":CHANNEL%ld:SCALE %.4f", i + 1, vdiv);
}

vector<uint64_t> HantekSCPIOscilloscope::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> ret;
	ret = {2 * 1000,
        5 * 1000,
        10 * 1000,
		20 * 1000,
		50 * 1000,
		100 * 1000,
		200 * 1000,
		500 * 1000,
		1 * 1000 * 1000,
		2 * 1000 * 1000,
		5 * 1000 * 1000,
		10 * 1000 * 1000,
		20 * 1000 * 1000,
		50 * 1000 * 1000,
		100 * 1000 * 1000,
		200 * 1000 * 1000,
		500 * 1000 * 1000,
		1 * 1000 * 1000 * 1000,
		2 * 1000 * 1000 * 1000
		5 * 1000 * 1000 * 1000
		10 * 1000 * 1000 * 1000
        };
	return ret;
}

vector<uint64_t> HantekSCPIOscilloscope::GetSampleRatesInterleaved()
{
	vector<uint64_t> ret = {};
	GetSampleRatesNonInterleaved();
	return ret;
}

vector<uint64_t> HantekSCPIOscilloscope::GetSampleDepthsNonInterleaved()
{
	vector<uint64_t> ret = {};
	return ret;
}

vector<uint64_t> HantekSCPIOscilloscope::GetSampleDepthsInterleaved()
{
	vector<uint64_t> ret = {};
	return ret;
}

set<HantekSCPIOscilloscope::InterleaveConflict> HantekSCPIOscilloscope::GetInterleaveConflicts()
{
	set<InterleaveConflict> ret;

	//All scopes normally interleave channels 1/2 and 3/4.
	//If both channels in either pair is in use, that's a problem.
	ret.emplace(InterleaveConflict(m_channels[0], m_channels[1]));
	if(m_analogChannelCount > 2)
		ret.emplace(InterleaveConflict(m_channels[2], m_channels[3]));

	return ret;
}

uint64_t HantekSCPIOscilloscope::GetSampleRate()
{
	double f;
	if(!m_sampleRateValid)
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		string reply = converse(":ACQuire:SRATe?");
		sscanf(reply.c_str(), "%lf", &f);
		m_sampleRate = static_cast<int64_t>(f);
		m_sampleRateValid = true;
	}

	return m_sampleRate;
}

uint64_t HantekSCPIOscilloscope::GetSampleDepth()
{
	double f;
	if(!m_memoryDepthValid)
	{
		//:AQUIRE:MDEPTH can sometimes return incorrect values! It returns the *cap* on memory depth,
		// not the *actual* memory depth....we don't know that until we've collected samples

		// What you see below is the only observed method that seems to reliably get the *actual* memory depth.
		lock_guard<recursive_mutex> lock(m_mutex);
		string reply = converse(":ACQuire:POINts?");
		f = Unit(Unit::UNIT_SAMPLEDEPTH).ParseString(reply);
		m_memoryDepth = static_cast<int64_t>(f);
		m_memoryDepthValid = true;
	}

	return m_memoryDepth;
}

void HantekSCPIOscilloscope::SetSampleDepth(uint64_t depth)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	switch(depth)
	{
		case 4000:
			sendOnly("ACQUIRE:MDEPTH 4k");
			break;
		case 40000:
			sendOnly("ACQUIRE:MDEPTH 40k");
			break;
		case 400000:
			sendOnly("ACQUIRE:MDEPTH 400k");
			break;
		case 4000000:
			sendOnly("ACQUIRE:MDEPTH 4M");
			break;
		case 8000000:
			sendOnly("ACQUIRE:MDEPTH 8M");
			break;
		default:
			LogError("Invalid memory depth for channel: %lu\n", depth);
	}

	m_memoryDepthValid = false;
}

void HantekSCPIOscilloscope::SetSampleRate(uint64_t rate)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_sampleRate = rate;
	m_sampleRateValid = false;

	m_memoryDepthValid = false;
	double sampletime = GetSampleDepth() / (double)rate;
	sendOnly(":TIMEBASE:SCALE %1.2E", sampletime / 10);
	m_memoryDepthValid = false;
}

void HantekSCPIOscilloscope::EnableTriggerOutput()
{
	LogWarning("EnableTriggerOutput not implemented\n");
}

void HantekSCPIOscilloscope::SetUseExternalRefclk(bool /*external*/)
{
	LogWarning("SetUseExternalRefclk not implemented\n");
}

void HantekSCPIOscilloscope::SetTriggerOffset(int64_t offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Siglents standard has the offset being from the midpoint of the capture.
	//Scopehal has offset from the start.
	int64_t rate = GetSampleRate();
	int64_t halfdepth = GetSampleDepth() / 2;
	int64_t halfwidth = static_cast<int64_t>(round(FS_PER_SECOND * halfdepth / rate));

	sendOnly(":TIMEBASE:POSition %1.2E", (offset - halfwidth) * SECONDS_PER_FS);

	//Don't update the cache because the scope is likely to round the offset we ask for.
	//If we query the instrument later, the cache will be updated then.
	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_triggerOffsetValid = false;
}

int64_t HantekSCPIOscilloscope::GetTriggerOffset()
{
	//Early out if the value is in cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_triggerOffsetValid)
			return m_triggerOffset;
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		reply = converse(":TIMEBASE:POSition?");
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);

	//Result comes back in scientific notation
	double sec;
	sscanf(reply.c_str(), "%le", &sec);
	m_triggerOffset = static_cast<int64_t>(round(sec * FS_PER_SECOND));

	//Convert from midpoint to start point
	int64_t rate = GetSampleRate();
	int64_t halfdepth = GetSampleDepth() / 2;
	int64_t halfwidth = static_cast<int64_t>(round(FS_PER_SECOND * halfdepth / rate));
	m_triggerOffset += halfwidth;

	m_triggerOffsetValid = true;

	return m_triggerOffset;
}

void HantekSCPIOscilloscope::SetDeskewForChannel(size_t channel, int64_t skew)
{
	//Cannot deskew digital/trigger channels
	if(channel >= m_analogChannelCount)
		return;

    return;
    // Apparantly Hantek doesn't have SKEW
	// lock_guard<recursive_mutex> lock(m_mutex);

	// sendOnly(":CHANNEL%ld:SKEW %1.2E", channel, skew * SECONDS_PER_FS);

	// //Update cache
	// lock_guard<recursive_mutex> lock2(m_cacheMutex);
	// m_channelDeskew[channel] = skew;
}

int64_t HantekSCPIOscilloscope::GetDeskewForChannel(size_t channel)
{
    return 0;
    // Apparantly Hantek doesn't have SKEW
	// //Cannot deskew digital/trigger channels
	// if(channel >= m_analogChannelCount)
	// 	return 0;

	// //Early out if the value is in cache
	// {
	// 	lock_guard<recursive_mutex> lock(m_cacheMutex);
	// 	if(m_channelDeskew.find(channel) != m_channelDeskew.end())
	// 		return m_channelDeskew[channel];
	// }

	// //Read the deskew
	// lock_guard<recursive_mutex> lock(m_mutex);
	// string reply = converse(":CHANNEL%ld:SKEW?", channel + 1);

	// //Value comes back as floating point ps
	// float skew;
	// sscanf(reply.c_str(), "%f", &skew);
	// int64_t skew_ps = round(skew * FS_PER_SECOND);

	// lock_guard<recursive_mutex> lock2(m_cacheMutex);
	// m_channelDeskew[channel] = skew_ps;

	// return skew_ps;
}

bool HantekSCPIOscilloscope::IsInterleaving()
{
	LogWarning("IsInterleaving is not implemented\n");
	return false;
}

bool HantekSCPIOscilloscope::SetInterleaving(bool /* combine*/)
{
	LogWarning("SetInterleaving is not implemented\n");
	return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Analog bank configuration

bool HantekSCPIOscilloscope::IsADCModeConfigurable()
{
	return false;
}

vector<string> HantekSCPIOscilloscope::GetADCModeNames(size_t /*channel*/)
{
	vector<string> v;
	LogWarning("GetADCModeNames is not implemented\n");
	return v;
}

size_t HantekSCPIOscilloscope::GetADCMode(size_t /*channel*/)
{
	return 0;
}

void HantekSCPIOscilloscope::SetADCMode(size_t /*channel*/, size_t /*mode*/)
{
	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Triggering

void HantekSCPIOscilloscope::PullTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Figure out what kind of trigger is active.
	string reply = Trim(converse(":TRIGGER:MODE?"));
	if(reply == "EDGE")
		PullEdgeTrigger();
	else if(reply == "UART")
		PullUartTrigger();
	else if(reply == "INTerval")
		PullPulseWidthTrigger();
	else if(reply == "WINDow")
		PullWindowTrigger();

	//Unrecognized trigger type
	else
	{
		LogWarning("Unknown trigger type \"%s\"\n", reply.c_str());
		m_trigger = NULL;
		return;
	}

	//Pull the source (same for all types of trigger)
	PullTriggerSource(m_trigger, reply);

	//TODO: holdoff
    //string holdoff = Trim(converse(":TRIGger:HOLDoff?"));

}

/**
	@brief Reads the source of a trigger from the instrument
 */
void HantekSCPIOscilloscope::PullTriggerSource(Trigger* trig, string triggerModeName)
{
	string reply = Trim(converse(":TRIGGER:%s:SOURCE?", triggerModeName.c_str()));
	auto chan = GetChannelByHwName(reply);
	trig->SetInput(0, StreamDescriptor(chan, 0), true);
	if(!chan)
		LogWarning("Unknown trigger source \"%s\"\n", reply.c_str());
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void HantekSCPIOscilloscope::PullEdgeTrigger()
{
	//Clear out any triggers of the wrong type
	if((m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	//Level
	et->SetLevel(stof(converse(":TRIGGER:EDGE:LEVEL?")));

	//TODO: OptimizeForHF (changes hysteresis for fast signals)

	//Slope
	GetTriggerSlope(et, Trim(converse(":TRIGGER:EDGE:SLOPE?")));
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void HantekSCPIOscilloscope::PullPulseWidthTrigger()
{
	//Clear out any triggers of the wrong type
	if((m_trigger != NULL) && (dynamic_cast<PulseWidthTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new PulseWidthTrigger(this);
	auto pt = dynamic_cast<PulseWidthTrigger*>(m_trigger);

	//Level
	pt->SetLevel(stof(converse(":TRIGGER:INTERVAL:LEVEL?")));

	//Condition
	pt->SetCondition(GetCondition(converse(":TRIGGER:INTERVAL:LIMIT?")));

	//Min range
	Unit fs(Unit::UNIT_FS);
	pt->SetLowerBound(fs.ParseString(converse(":TRIGGER:INTERVAL:TLOWER?")));

	//Max range
	pt->SetUpperBound(fs.ParseString(converse(":TRIGGER:INTERVAL:TUPPER?")));

	//Slope
	GetTriggerSlope(pt, Trim(converse(":TRIGGER:INTERVAL:SLOPE?")));
}

/**
	@brief Reads settings for a UART trigger from the instrument
 */
void HantekSCPIOscilloscope::PullUartTrigger()
{
	//Clear out any triggers of the wrong type
	if((m_trigger != NULL) && (dynamic_cast<UartTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new UartTrigger(this);
	UartTrigger* ut = dynamic_cast<UartTrigger*>(m_trigger);

	//Bit rate
	ut->SetBitRate(stoi(converse(":TRIGger:UART:BAUd?")));

	//Level
	ut->SetLevel(stof(converse(":TRIGger:UART:ALEVel?")));

	//Parity
	auto reply = Trim(converse(":TRIGger:UART:PARIty?"));
	if(reply == "NONE")
		ut->SetParityType(UartTrigger::PARITY_NONE);
	else if(reply == "EVEN")
		ut->SetParityType(UartTrigger::PARITY_EVEN);
	else if(reply == "ODD")
		ut->SetParityType(UartTrigger::PARITY_ODD);

	//Operator
	//bool ignore_p2 = true;

	// It seems this scope only copes with equivalence
	ut->SetCondition(Trigger::CONDITION_EQUAL);

	//Idle polarity
	reply = Trim(converse(":TRIGGER:UART:IDLE?"));
	if(reply == "HIGH")
		ut->SetPolarity(UartTrigger::IDLE_HIGH);
	else if(reply == "LOW")
		ut->SetPolarity(UartTrigger::IDLE_LOW);

	//Stop bits
	ut->SetStopBits(stof(Trim(converse(":TRIGGER:UART:STOP?"))));

	//Trigger type
	reply = Trim(converse(":TRIGger:UART:CONdition?"));
	if(reply == "STARt")
		ut->SetMatchType(UartTrigger::TYPE_START);
	else if(reply == "STOP")
		ut->SetMatchType(UartTrigger::TYPE_STOP);
	else if(reply == "PARITY_ERR")
		ut->SetMatchType(UartTrigger::TYPE_PARITY_ERR);
	else
		ut->SetMatchType(UartTrigger::TYPE_DATA);

	// Data to match (there is no pattern2 on sds)
	string p1 = Trim(converse(":TRIGger:UART:DATA?"));
	ut->SetPatterns(p1, "", true);
}

/**
	@brief Reads settings for a window trigger from the instrument
 */
void HantekSCPIOscilloscope::PullWindowTrigger()
{
	//Clear out any triggers of the wrong type
	if((m_trigger != NULL) && (dynamic_cast<WindowTrigger*>(m_trigger) != NULL))
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new WindowTrigger(this);
	WindowTrigger* wt = dynamic_cast<WindowTrigger*>(m_trigger);

	//Lower bound
	Unit v(Unit::UNIT_VOLTS);
	wt->SetLowerBound(v.ParseString(converse(":TRIGger:WINDOw:ALEVel?")));

	//Upper bound
	wt->SetUpperBound(v.ParseString(converse(":TRIGger:WINDOw:BLEVel?")));
}

/**
	@brief Processes the slope for an edge or edge-derived trigger
 */
void HantekSCPIOscilloscope::GetTriggerSlope(EdgeTrigger* trig, string reply)

{
	reply = Trim(reply);

	if(reply == "RISing")
		trig->SetType(EdgeTrigger::EDGE_RISING);
	else if(reply == "FALLing")
		trig->SetType(EdgeTrigger::EDGE_FALLING);
	else if(reply == "EITHer")
		trig->SetType(EdgeTrigger::EDGE_ANY);
	else
		LogWarning("Unknown trigger slope %s\n", reply.c_str());
}

/**
	@brief Parses a trigger condition
 */
Trigger::Condition HantekSCPIOscilloscope::GetCondition(string reply)
{
	reply = Trim(reply);

	if(reply == "LESS")
		return Trigger::CONDITION_LESS;
	else if(reply == "GREAT")
		return Trigger::CONDITION_GREATER;
	else if(reply == "EQUAl")
		return Trigger::CONDITION_EQUAL;
	else if(reply == "NEQUal")
		return Trigger::CONDITION_NOT_EQUAL;

	//unknown
	LogWarning("Unknown trigger condition [%s]\n", reply.c_str());
	return Trigger::CONDITION_LESS;
}

void HantekSCPIOscilloscope::PushTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	auto pt = dynamic_cast<PulseWidthTrigger*>(m_trigger);
	auto ut = dynamic_cast<UartTrigger*>(m_trigger);
	auto wt = dynamic_cast<WindowTrigger*>(m_trigger);

	if(pt)
	{
		sendOnly(":TRIGGER:MODE INTERVAL");
		sendOnly(":TRIGGER:INTERVAL:SOURCE CHANnel%d", m_trigger->GetInput(0).m_channel->GetIndex() + 1);
		PushPulseWidthTrigger(pt);
	}
	else if(ut)
	{
		sendOnly(":TRIGGER:MODE UART");
		// TODO: Validate these trigger allocations
		sendOnly(":TRIGGER:UART:SOURCE CHANnel%d", m_trigger->GetInput(0).m_channel->GetIndex() + 1);
		PushUartTrigger(ut);
	}
	else if(wt)
	{
		sendOnly(":TRIGGER:MODE WINDOW");
		sendOnly(":TRIGGER:WINDOW:SOURCE CHANnel%d", m_trigger->GetInput(0).m_channel->GetIndex() + 1);
		PushWindowTrigger(wt);
	}

	// TODO: Add in PULSE, VIDEO, PATTERN, QUALITFIED, SPI, IIC, CAN, LIN, FLEXRAY and CANFD Triggers

	else if(et)	   //must be last
	{
		sendOnly(":TRIGGER:TYPE EDGE");
		sendOnly(":TRIGGER:EDGE:SOURCE CHANnel%d", m_trigger->GetInput(0).m_channel->GetIndex() + 1);
		PushEdgeTrigger(et, "EDGE");
	}

	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void HantekSCPIOscilloscope::PushEdgeTrigger(EdgeTrigger* trig, const std::string trigType)
{
	//Slope
	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			sendOnly(":TRIGGER:%s:SLOPE RISING", trigType.c_str());
			break;

		case EdgeTrigger::EDGE_FALLING:
			sendOnly(":TRIGGER:%s:SLOPE FALLING", trigType.c_str());
			break;

		case EdgeTrigger::EDGE_ANY:
			sendOnly(":TRIGGER:%s:SLOPE EITHer", trigType.c_str());
			break;

		default:
			LogWarning("Invalid trigger type %d\n", trig->GetType());
			break;
	}
	//Level
	sendOnly(":TRIGGER:%s:LEVEL %1.2E", trigType.c_str(), trig->GetLevel());
}

// TODO
/**
	@brief Pushes settings for a pulse width trigger to the instrument
 */
void HantekSCPIOscilloscope::PushPulseWidthTrigger(PulseWidthTrigger* trig)
{
	PushEdgeTrigger(trig, "INTERVAL");
	PushCondition(":TRIGGER:INTERVAL", trig->GetCondition());
	PushFloat(":TRIGGER:INTERVAL:TUPPER", trig->GetUpperBound() * SECONDS_PER_FS);
	PushFloat(":TRIGGER:INTERVAL:TLOWER", trig->GetLowerBound() * SECONDS_PER_FS);
}

/**
	@brief Pushes settings for a UART trigger to the instrument
 */
void HantekSCPIOscilloscope::PushUartTrigger(UartTrigger* trig)
{
	PushFloat(":TRIGger:UART:ALEVel", trig->GetLevel());
	PushFloat(":TRIGGER:UART:BAUD", trig->GetBitRate());

	sendOnly(":TRIGger:UART:WIDTh 8");

	switch(trig->GetParityType())
	{
		case UartTrigger::PARITY_NONE:
			sendOnly(":TRIGGER:UART:PARITY NONE");
			break;

		case UartTrigger::PARITY_ODD:
			sendOnly(":TRIGGER:UART:PARITY ODD");
			break;

		case UartTrigger::PARITY_EVEN:
			sendOnly(":TRIGGER:UART:PARITY EVEN");
			break;
	}

	//Pattern length depends on the current format.
	//Note that the pattern length is in bytes, not bits, even though patterns are in binary.
	auto pattern1 = trig->GetPattern1();
	sendOnly(":TRIGger:UART:DATA \"%d\"", (int)pattern1.length() / 8);

	PushCondition(":TRIGGER:UART", trig->GetCondition());

	//Polarity
	sendOnly(":TRIGGER:UART:IDLE %s", (trig->GetPolarity() == UartTrigger::IDLE_HIGH) ? "HIGH" : "LOW");

	auto nstop = trig->GetStopBits();
	if(nstop == 1)
		sendOnly(":TRIGGER:UART:STOP 1");
	else if(nstop == 2)
		sendOnly(":TRIGGER:UART:STOP 2");
	else
		sendOnly(":TRIGGER:UART:STOP 1.5");

	//Match type
	switch(trig->GetMatchType())
	{
		case UartTrigger::TYPE_START:
			sendOnly(":TRIGGER:UART:CONDITION START");
			break;
		case UartTrigger::TYPE_STOP:
			sendOnly(":TRIGGER:UART:CONDITION STOP");
			break;
		case UartTrigger::TYPE_PARITY_ERR:
			sendOnly(":TRIGGER:UART:CONDITION PARITY_ERR");
			break;
		default:
		case UartTrigger::TYPE_DATA:
			sendOnly(":TRIGGER:UART:CONDITION DATA");
			break;
	}

	//UARTCondition
	//ViewingMode
}

/**
	@brief Pushes settings for a window trigger to the instrument
 */
void HantekSCPIOscilloscope::PushWindowTrigger(WindowTrigger* trig)
{
	PushFloat(":TRIGGER:WINDOW:ALEVEL", trig->GetLowerBound());
	PushFloat(":TRIGGER:WINDOW:BLEVEL", trig->GetUpperBound());
}

/**
	@brief Pushes settings for a trigger condition under a .Condition field
 */
void HantekSCPIOscilloscope::PushCondition(const string& path, Trigger::Condition cond)
{
	switch(cond)
	{
		case Trigger::CONDITION_LESS:
			sendOnly("%s:LIMIT LESSTHAN", path.c_str());
			break;

		case Trigger::CONDITION_GREATER:
			sendOnly("%s:LIMIT GREATERTHAN", path.c_str());
			break;

		case Trigger::CONDITION_BETWEEN:
			sendOnly("%s:LIMIT INNER", path.c_str());
			break;

		case Trigger::CONDITION_NOT_BETWEEN:
			sendOnly("%s:LIMIT OUTER", path.c_str());
			break;

		//Other values are not legal here, it seems
		default:
			break;
	}
}

void HantekSCPIOscilloscope::PushFloat(string path, float f)
{
	sendOnly("%s %1.2E", path.c_str(), f);
}

vector<string> HantekSCPIOscilloscope::GetTriggerTypes()
{
	vector<string> ret;
	ret.push_back(EdgeTrigger::GetTriggerName());
	ret.push_back(PulseWidthTrigger::GetTriggerName());
	if(m_hasUartTrigger)
		ret.push_back(UartTrigger::GetTriggerName());
	ret.push_back(WindowTrigger::GetTriggerName());

	// TODO: Add in PULSE, VIDEO, PATTERN, QUALITFIED, SPI, IIC, CAN, LIN, FLEXRAY and CANFD Triggers
	return ret;
}