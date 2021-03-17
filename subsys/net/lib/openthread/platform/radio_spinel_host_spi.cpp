/*
 * Copyright (c) 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction
 *   for radio communication.
 *
 */

#ifdef CONFIG_OPENTHREAD_FTD
#define OPENTHREAD_FTD 1
#else
#define OPENTHREAD_FTD 0
#endif

#ifdef CONFIG_OPENTHREAD_MTD
#define OPENTHREAD_MTD 1
#else
#define OPENTHREAD_MTD 0
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(radio_spinel, CONFIG_OPENTHREAD_L2_LOG_LEVEL);

#include <openthread-system.h>
#include <openthread/platform/radio.h>

#include "lib/spinel/radio_spinel.hpp"

#include <ncp/ncp_spi.hpp>

#define CONFIG_OPENTHREAD_SPINEL_HOST_SPI_ALIGN_ALLOWANCE 16
#define CONFIG_OPENTHREAD_SPINEL_HOST_SPI_SMALL_PACKET_SIZE 32

using ot::Spinel::SpinelInterface;

extern "C" void otPlatSpiHostInit(void);
extern "C" bool otPlatSpiHostCheckInterrupt(void);
extern "C" bool otPlatSpiHostWaitForFrame(uint64_t aTimeoutUs);
extern "C" bool otPlatSpiHostProcess(otInstance *aInstance);
extern "C" int otPlatSpiHostTransfer(uint8_t *aSpiTxFrameBuffer,
				     uint8_t *aSpiRxFrameBuffer,
				     uint32_t aTransferLength);

/**
 * This class defines an SPI interface to the Radio Co-processor (RCP).
 *
 */
class PlatformSpiInterface
{
public:
	/**
	 * This constructor initializes the object.
	 *
	 * @param[in] aCallback         Callback on frame received
	 * @param[in] aCallbackContext  Callback context
	 * @param[in] aFrameBuffer      A reference to a `RxFrameBuffer` object.
	 *
	 */
	PlatformSpiInterface(SpinelInterface::ReceiveFrameCallback aCallback,
			     void *aCallbackContext,
			     SpinelInterface::RxFrameBuffer &aFrameBuffer);

	/**
	 * This destructor deinitializes the object.
	 *
	 */
	~PlatformSpiInterface(void);

	/**
	 * This method initializes the HDLC interface.
	 *
	 */
	otError Init(void);

	/**
	 * This method deinitializes the HDLC interface.
	 *
	 */
	void Deinit(void);

	/**
	 * This method encodes and sends a spinel frame to Radio Co-processor (RCP) over the socket.
	 *
	 * This is blocking call, i.e., if the socket is not writable, this method waits for it to become writable for
	 * up to `kMaxWaitTime` interval.
	 *
	 * @param[in] aFrame     A pointer to buffer containing the spinel frame to send.
	 * @param[in] aLength    The length (number of bytes) in the frame.
	 *
	 * @retval OT_ERROR_NONE     Successfully encoded and sent the spinel frame.
	 * @retval OT_ERROR_NO_BUFS  Insufficient buffer space available to encode the frame.
	 * @retval OT_ERROR_FAILED   Failed to send due to socket not becoming writable within `kMaxWaitTime`.
	 *
	 */
	otError SendFrame(const uint8_t *aFrame, uint16_t aLength);

	/**
	 * This method waits for receiving part or all of spinel frame within specified timeout.
	 *
	 * @param[in]  aTimeoutUs  The timeout value in microseconds.
	 *
	 * @retval OT_ERROR_NONE             Part or all of spinel frame is received.
	 * @retval OT_ERROR_RESPONSE_TIMEOUT No spinel frame is received within @p aTimeoutUs.
	 *
	 */
	otError WaitForFrame(uint64_t aTimeoutUs);

	/**
	 * This method performs radio driver processing.
	 *
	 * @param[in]  aContext  The context containing no thing, never used.
	 *
	 */
	void Process(const otInstance& aInstance);

	void OnRcpReset(void);

private:
	uint8_t *GetRealRxFrameStart(uint8_t *aSpiRxFrameBuffer, uint8_t aAlignAllowance, uint16_t &aSkipLength);
	otError  DoSpiTransfer(uint8_t *aSpiRxFrameBuffer, uint32_t aTransferLength);
	otError  PushPullSpi(void);

	enum {
		kSpiAlignAllowanceMax   = 16,
		kSpiFrameHeaderSize     = 5,
	};

	enum {
		kMsecPerSec             = 1000,
		kUsecPerMsec            = 1000,
		kSpiPollPeriodUs        = kMsecPerSec * kUsecPerMsec / 30,
		kSecPerDay              = 60 * 60 * 24,
		kResetHoldOnUsec        = 10 * kUsecPerMsec,
	};

	enum {
		kMaxFrameSize = SpinelInterface::kMaxFrameSize,
	};

	SpinelInterface::ReceiveFrameCallback mReceiveFrameCallback;
	void *mReceiveFrameContext;
	SpinelInterface::RxFrameBuffer &      mRxFrameBuffer;

	uint8_t mSpiAlignAllowance;
	uint16_t mSpiSmallPacketSize;

	uint64_t mSlaveResetCount;
	uint64_t mSpiFrameCount;
	uint64_t mSpiValidFrameCount;
	uint64_t mSpiGarbageFrameCount;
	uint64_t mSpiDuplexFrameCount;
	uint64_t mSpiUnresponsiveFrameCount;
	uint64_t mSpiRxFrameCount;
	uint64_t mSpiRxFrameByteCount;
	uint64_t mSpiTxFrameCount;
	uint64_t mSpiTxFrameByteCount;

	bool mSpiTxIsReady;
	uint16_t mSpiTxRefusedCount;
	uint16_t mSpiTxPayloadSize;
	uint8_t mSpiTxFrameBuffer[kMaxFrameSize + kSpiAlignAllowanceMax];

	bool mDidPrintRateLimitLog;
	uint16_t mSpiSlaveDataLen;

	// Non-copyable, intentionally not implemented.
	PlatformSpiInterface(const PlatformSpiInterface &);
	PlatformSpiInterface &operator = (const PlatformSpiInterface &);
};

typedef ot::Spinel::RadioSpinel < PlatformSpiInterface, otInstance > PlatformSpiRadioSpinel;

OT_DEFINE_ALIGNED_VAR(gRadioSpinalInstanceRaw, sizeof(PlatformSpiRadioSpinel), uint64_t);

static PlatformSpiRadioSpinel& RadioSpinelInstance(void)
{
	void *instance = &gRadioSpinalInstanceRaw;

	return *static_cast < PlatformSpiRadioSpinel * > (instance);
}

static void InitPlatformRadioSpinelInstance(void)
{
	new(&gRadioSpinalInstanceRaw) PlatformSpiRadioSpinel();
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().GetIeeeEui64(aIeeeEui64));
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().SetPanId(panid));
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++) {
		addr.m8[i] = aAddress->m8[sizeof(addr) - 1 - i];
	}

	SuccessOrDie(RadioSpinelInstance().SetExtendedAddress(addr));
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().SetShortAddress(aAddress));
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().SetPromiscuous(aEnable));
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().IsEnabled();
}

otError otPlatRadioEnable(otInstance *aInstance)
{
	return RadioSpinelInstance().Enable(aInstance);
}

otError otPlatRadioDisable(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().Disable();
}

otError otPlatRadioSleep(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().Sleep();
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().Receive(aChannel);
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().Transmit(*aFrame);
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return &RadioSpinelInstance().GetTransmitFrame();
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetRssi();
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetRadioCaps();
}

const char *otPlatRadioGetVersionString(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetVersion();
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().IsPromiscuous();
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().EnableSrcMatch(aEnable));
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().AddSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++) {
		addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
	}

	return RadioSpinelInstance().AddSrcMatchExtEntry(addr);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().ClearSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
	OT_UNUSED_VARIABLE(aInstance);
	otExtAddress addr;

	for (size_t i = 0; i < sizeof(addr); i++) {
		addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
	}

	return RadioSpinelInstance().ClearSrcMatchExtEntry(addr);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().ClearSrcMatchShortEntries());
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	SuccessOrDie(RadioSpinelInstance().ClearSrcMatchExtEntries());
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().EnergyScan(aScanChannel, aScanDuration);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
	OT_UNUSED_VARIABLE(aInstance);

	if (aPower == NULL) {
		return OT_ERROR_INVALID_ARGS;
	}

	return RadioSpinelInstance().GetTransmitPower(*aPower);
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().SetTransmitPower(aPower);
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
	OT_UNUSED_VARIABLE(aInstance);

	if (aThreshold == NULL) {
		return OT_ERROR_INVALID_ARGS;
	}

	return RadioSpinelInstance().GetCcaEnergyDetectThreshold(*aThreshold);
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().SetCcaEnergyDetectThreshold(aThreshold);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetReceiveSensitivity();
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().SetCoexEnabled(aEnabled);
}

bool otPlatRadioIsCoexEnabled(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().IsCoexEnabled();
}

otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
	OT_UNUSED_VARIABLE(aInstance);

	if (aCoexMetrics == NULL) {
		return OT_ERROR_INVALID_ARGS;
	}

	return RadioSpinelInstance().GetCoexMetrics(*aCoexMetrics);
}
#endif

#if OPENTHREAD_CONFIG_DIAG_ENABLE
otError otPlatDiagProcess(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
	// deliver the platform specific diags commands to radio only ncp.
	OT_UNUSED_VARIABLE(aInstance);
	char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE] = { '\0' };
	char *cur = cmd;
	char *end = cmd + sizeof(cmd);

	for (int index = 0; index < argc; index++) {
		cur += snprintf(cur, static_cast < size_t > (end - cur), "%s ", argv[index]);
	}

	return RadioSpinelInstance().PlatDiagProcess(cmd, aOutput, aOutputMaxLen);
}

void otPlatDiagModeSet(bool aMode)
{
	if (RadioSpinelInstance().PlatDiagProcess(aMode ? "start" : "stop", NULL, 0) == OT_ERROR_NONE) {
		RadioSpinelInstance().SetDiagEnabled(aMode);
	}
}

bool otPlatDiagModeGet(void)
{
	return RadioSpinelInstance().IsDiagEnabled();
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
	char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

	snprintf(cmd, sizeof(cmd), "power %d", aTxPower);

	RadioSpinelInstance().PlatDiagProcess(cmd, NULL, 0);
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
	char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

	snprintf(cmd, sizeof(cmd), "channel %d", aChannel);
	RadioSpinelInstance().PlatDiagProcess(cmd, NULL, 0);
}

void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
	OT_UNUSED_VARIABLE(aInstance);
	OT_UNUSED_VARIABLE(aFrame);
	OT_UNUSED_VARIABLE(aError);
}

void otPlatDiagAlarmCallback(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
}
#endif // OPENTHREAD_CONFIG_DIAG_ENABLE

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetRadioChannelMask(false);
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetRadioChannelMask(true);
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);
	return RadioSpinelInstance().GetState();
}

extern "C" void platformRadioInit()
{
	InitPlatformRadioSpinelInstance();
	RadioSpinelInstance().GetSpinelInterface().Init();
	RadioSpinelInstance().Init(false, false, false);
}

extern "C" void platformRadioDeinit(void)
{
	RadioSpinelInstance().Deinit();
}

extern "C" void platformRadioProcess(otInstance *context)
{
	otPlatSpiHostProcess(context);

	RadioSpinelInstance().Process(*context);
}

PlatformSpiInterface::PlatformSpiInterface(SpinelInterface::ReceiveFrameCallback aCallback,
					   void *aCallbackContext,
					   SpinelInterface::RxFrameBuffer &aFrameBuffer)
	: mReceiveFrameCallback(aCallback)
	, mReceiveFrameContext(aCallbackContext)
	, mRxFrameBuffer(aFrameBuffer)
	, mSlaveResetCount(0)
	, mSpiFrameCount(0)
	, mSpiValidFrameCount(0)
	, mSpiGarbageFrameCount(0)
	, mSpiDuplexFrameCount(0)
	, mSpiUnresponsiveFrameCount(0)
	, mSpiRxFrameCount(0)
	, mSpiRxFrameByteCount(0)
	, mSpiTxFrameCount(0)
	, mSpiTxFrameByteCount(0)
	, mSpiTxIsReady(false)
	, mSpiTxRefusedCount(0)
	, mSpiTxPayloadSize(0)
	, mDidPrintRateLimitLog(false)
	, mSpiSlaveDataLen(0)
{
	// empty
}

otError PlatformSpiInterface::Init(void)
{
	mSpiSmallPacketSize = CONFIG_OPENTHREAD_SPINEL_HOST_SPI_SMALL_PACKET_SIZE;
	mSpiAlignAllowance = CONFIG_OPENTHREAD_SPINEL_HOST_SPI_ALIGN_ALLOWANCE;

	// Reset RCP chip.
	otPlatSpiHostInit();

	return OT_ERROR_NONE;
}

PlatformSpiInterface::~PlatformSpiInterface(void)
{
	Deinit();
}

void PlatformSpiInterface::Deinit(void)
{
	// empty
}

uint8_t *PlatformSpiInterface::GetRealRxFrameStart(uint8_t *aSpiRxFrameBuffer, uint8_t aAlignAllowance, uint16_t &aSkipLength)
{
	uint8_t *start = aSpiRxFrameBuffer;
	const uint8_t *end = aSpiRxFrameBuffer + aAlignAllowance;

	for (; start != end && start[0] == 0xff; start++)
		;

	aSkipLength = static_cast < uint16_t > (start - aSpiRxFrameBuffer);

	return start;
}

otError PlatformSpiInterface::DoSpiTransfer(uint8_t *aSpiRxFrameBuffer, uint32_t aTransferLength)
{
	#define LENGTH_ALIGN 4
	int ret = 0;

	size_t remainder = aTransferLength % LENGTH_ALIGN;
	if (remainder) {
		aTransferLength += LENGTH_ALIGN - remainder;
	}

	ret = otPlatSpiHostTransfer(mSpiTxFrameBuffer, aSpiRxFrameBuffer, aTransferLength);

	if (ret == 0) {
		mSpiFrameCount++;
	}

	return (ret < 0) ? OT_ERROR_FAILED : OT_ERROR_NONE;
}

otError PlatformSpiInterface::PushPullSpi(void)
{
	otError error = OT_ERROR_FAILED;
	uint16_t spiTransferBytes = 0;
	uint8_t successfulExchanges = 0;
	bool discardRxFrame = true;
	uint8_t *spiRxFrameBuffer;
	uint8_t *spiRxFrame;
	uint8_t slaveHeader;
	uint16_t slaveAcceptLen;
	ot::Ncp::SpiFrame txFrame(mSpiTxFrameBuffer);
	uint16_t skipAlignAllowanceLength;

	// Set the reset flag to indicate to our slave that we are coming up from scratch.
	txFrame.SetHeaderFlagByte(mSpiValidFrameCount == 0);

	// Zero out our rx_accept and our data_len for now.
	txFrame.SetHeaderAcceptLen(0);
	txFrame.SetHeaderDataLen(0);

	// Sanity check.
	if (mSpiSlaveDataLen > kMaxFrameSize) {
		mSpiSlaveDataLen = 0;
	}

	if (mSpiTxIsReady) {
		// Go ahead and try to immediately send a frame if we have it queued up.
		txFrame.SetHeaderDataLen(mSpiTxPayloadSize);

		if (mSpiTxPayloadSize > spiTransferBytes) {
			spiTransferBytes = mSpiTxPayloadSize;
		}
	}

	if (mSpiSlaveDataLen != 0) {
		// In a previous transaction the slave indicated it had something to send us. Make sure our transaction
		// is large enough to handle it.
		if (mSpiSlaveDataLen > spiTransferBytes) {
			spiTransferBytes = mSpiSlaveDataLen;
		}
	} else {
		// Set up a minimum transfer size to allow small frames the slave wants to send us to be handled in a
		// single transaction.
		if (spiTransferBytes < mSpiSmallPacketSize) {
			spiTransferBytes = mSpiSmallPacketSize;
		}
	}

	txFrame.SetHeaderAcceptLen(spiTransferBytes);

	// Set skip length to make MultiFrameBuffer to reserve a space in front of the frame buffer.
	SuccessOrExit(error = mRxFrameBuffer.SetSkipLength(kSpiFrameHeaderSize));

	// Check whether the remaining frame buffer has enough space to store the data to be received.
	VerifyOrExit(mRxFrameBuffer.GetFrameMaxLength() >= spiTransferBytes + mSpiAlignAllowance);

	// Point to the start of the reserved buffer.
	spiRxFrameBuffer = mRxFrameBuffer.GetFrame() - kSpiFrameHeaderSize;

	// Set the total number of bytes to be transmitted.
	spiTransferBytes += kSpiFrameHeaderSize + mSpiAlignAllowance;

	// Perform the SPI transaction.
	error = DoSpiTransfer(spiRxFrameBuffer, spiTransferBytes);

	if (error != OT_ERROR_NONE) {
		LOG_ERR("PushPullSpi:DoSpiTransfer: error=%d", error);
		ExitNow();
	}

	// Account for misalignment (0xFF bytes at the start)
	spiRxFrame = GetRealRxFrameStart(spiRxFrameBuffer, mSpiAlignAllowance, skipAlignAllowanceLength);

	{
		ot::Ncp::SpiFrame rxFrame(spiRxFrame);

		LOG_DBG("spi_transfer TX: H:%02X ACCEPT:%d DATA:%d", txFrame.GetHeaderFlagByte(),
			txFrame.GetHeaderAcceptLen(), txFrame.GetHeaderDataLen());
		LOG_DBG("spi_transfer RX: H:%02X ACCEPT:%d DATA:%d", rxFrame.GetHeaderFlagByte(),
			rxFrame.GetHeaderAcceptLen(), rxFrame.GetHeaderDataLen());

		slaveHeader = rxFrame.GetHeaderFlagByte();
		if ((slaveHeader == 0xFF) || (slaveHeader == 0x00)) {
			if ((slaveHeader == spiRxFrame[1]) && (slaveHeader == spiRxFrame[2]) && (slaveHeader == spiRxFrame[3]) &&
			    (slaveHeader == spiRxFrame[4])) {
				// Device is off or in a bad state. In some cases may be induced by flow control.
				if (mSpiSlaveDataLen == 0) {
					LOG_DBG("Slave did not respond to frame. (Header was all 0x%02X)", slaveHeader);
				} else {
					LOG_WRN("Slave did not respond to frame. (Header was all 0x%02X)", slaveHeader);
				}

				mSpiUnresponsiveFrameCount++;
			} else {
				// Header is full of garbage
				mSpiGarbageFrameCount++;

				LOG_WRN("Garbage in header : %02X %02X %02X %02X %02X", spiRxFrame[0], spiRxFrame[1],
					spiRxFrame[2], spiRxFrame[3], spiRxFrame[4]);
			}

			mSpiTxRefusedCount++;
			ExitNow();
		}

		slaveAcceptLen = rxFrame.GetHeaderAcceptLen();
		mSpiSlaveDataLen = rxFrame.GetHeaderDataLen();

		if (!rxFrame.IsValid() || (slaveAcceptLen > kMaxFrameSize) || (mSpiSlaveDataLen > kMaxFrameSize)) {
			mSpiGarbageFrameCount++;
			mSpiTxRefusedCount++;
			mSpiSlaveDataLen = 0;

			LOG_WRN("Garbage in header : %02X %02X %02X %02X %02X", spiRxFrame[0], spiRxFrame[1], spiRxFrame[2],
				spiRxFrame[3], spiRxFrame[4]);

			ExitNow();
		}

		mSpiValidFrameCount++;

		if (rxFrame.IsResetFlagSet()) {
			mSlaveResetCount++;
		}

		// Handle received packet, if any.
		if ((mSpiSlaveDataLen != 0) && (mSpiSlaveDataLen <= txFrame.GetHeaderAcceptLen())) {
			mSpiRxFrameByteCount += mSpiSlaveDataLen;
			mSpiSlaveDataLen = 0;
			mSpiRxFrameCount++;
			successfulExchanges++;

			// Set the skip length to skip align bytes and SPI frame header.
			SuccessOrExit(mRxFrameBuffer.SetSkipLength(skipAlignAllowanceLength + kSpiFrameHeaderSize));
			// Set the received frame length.
			SuccessOrExit(mRxFrameBuffer.SetLength(rxFrame.GetHeaderDataLen()));

			// Upper layer will free the frame buffer.
			discardRxFrame = false;

			mReceiveFrameCallback(mReceiveFrameContext);
		}
	}

	// Handle transmitted packet, if any.
	if (mSpiTxIsReady && (mSpiTxPayloadSize == txFrame.GetHeaderDataLen())) {
		if (txFrame.GetHeaderDataLen() <= slaveAcceptLen) {
			// Our outbound packet has been successfully transmitted. Clear mSpiTxPayloadSize and mSpiTxIsReady so
			// that uplayer can pull another packet for us to send.
			successfulExchanges++;

			mSpiTxFrameCount++;
			mSpiTxFrameByteCount += mSpiTxPayloadSize;

			mSpiTxIsReady = false;
			mSpiTxPayloadSize = 0;
			mSpiTxRefusedCount = 0;
		} else {
			// The slave wasn't ready for what we had to send them. Incrementing this counter will turn on rate
			// limiting so that we don't waste a ton of CPU bombarding them with useless SPI transfers.
			mSpiTxRefusedCount++;
		}
	}

	if (!mSpiTxIsReady) {
		mSpiTxRefusedCount = 0;
	}

	if (successfulExchanges == 2) {
		mSpiDuplexFrameCount++;
	}

exit:
	if (discardRxFrame) {
		mRxFrameBuffer.DiscardFrame();
	}

	return error;
}

otError PlatformSpiInterface::WaitForFrame(uint64_t aTimeoutUs)
{
	if (!otPlatSpiHostWaitForFrame(aTimeoutUs)) {
		return OT_ERROR_RESPONSE_TIMEOUT;
	}

	IgnoreError(PushPullSpi());

	return OT_ERROR_NONE;
}

otError PlatformSpiInterface::SendFrame(const uint8_t *aFrame, uint16_t aLength)
{
	if (aLength >= (kMaxFrameSize - kSpiFrameHeaderSize)) {
		return OT_ERROR_NO_BUFS;
	}

	if (mSpiTxIsReady) {
		return OT_ERROR_BUSY;
	}

	memcpy(&mSpiTxFrameBuffer[kSpiFrameHeaderSize], aFrame, aLength);

	mSpiTxIsReady = true;
	mSpiTxPayloadSize = aLength;

	IgnoreError(PushPullSpi());

	return OT_ERROR_NONE;
}

void PlatformSpiInterface::Process(const otInstance& aInstance)
{
	OT_UNUSED_VARIABLE(aInstance);

	// Service the SPI port if we can receive a packet or we have a packet to be sent.
	if (mSpiTxIsReady || otPlatSpiHostCheckInterrupt()) {
		// We guard this with the above check because we don't want to overwrite any previously received frames.
		PushPullSpi();
	}
}

void PlatformSpiInterface::OnRcpReset(void)
{
	LOG_WRN("OnRcpReset");
}
