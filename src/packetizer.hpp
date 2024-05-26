/*!
  * @file libxpack/src/packetizer.hpp
 *
  * This source is part of the "libxpack" project
  * 2023-2024  Sandro Kalatozishvili (s.kalatoz@gmail.com)
 *
  *@brief Packetize and unpacketize H264
 */

#ifndef __XH624_PACKETIZER__
#define __XH624_PACKETIZER__

#include <stdint.h>

class H264Packetizer {
public:
    // H.264 packetizer status
    enum Status {
        Success = 0,
        Unsupported,
        TooSmall,
        Invalid,
        Ignored,
    };

    // Enumeration of H.264 packetization modes
    enum Mode {
        SingleNAL,
        NonInterleaved,
        Interleaved
    };

    struct NAL {
        std::vector<uint8_t> data;
    };

    // H.264 packetizer settings
    struct Config {
        int nMTU;
        Mode mode;
        unsigned nUnpackNalStart; // Default is 3 (0, 0, 1)
        Config() : nMTU(1500), mode(NonInterleaved), nUnpackNalStart(3) {}
    };

    // Constructor
    H264Packetizer(const Config& cfg = Config()) : m_cfg(cfg) {}

    // Generate an RTP payload from H.264 frame bitstream, in-place processing
    Status Packetize(uint8_t *pBuffer, size_t nBufLen, unsigned *pPosit, const uint8_t **ppPayload, size_t *nPayloadLen);

    // Append RTP payload to a H.264 picture bitstream
    Status Unpacketize(const uint8_t *pPayload, size_t nPayloadLen, uint8_t *pBits, size_t nBitsLen, unsigned *pBitsPos);

    // Repacketize RTP payload from H624 packetization mode from 1 to 0 and get each separated packet
    std::vector<NAL> Repacketize(const uint8_t* pPayload, size_t nPayloadLength);

    // Scan RTP payload and parse h264 NAL units
    std::vector<NAL> NALParse(const uint8_t *pBuffer, size_t nLength);

private:
    // Enumeration of H.264 NAL unit types
    enum NALType {
        SINGLE_NAL_MIN = 1,
        SINGLE_NAL_MAX = 23,
        STAP_A = 24,
        FU_A = 28
    };

    // Find next NAL unit from the specified H.264 bitstream data
    static uint8_t *FindNextNAL(uint8_t *pStart, uint8_t *pEnd);

    Config m_cfg;
    bool m_unpackPrevLost = false;
    unsigned m_unpackLastSyncPos = 0;
};

#endif // __XH624_PACKETIZER__
