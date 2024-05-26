/*!
  * @file libxpack/src/packetizer.cpp
 *
  * This source is part of the "libxpack" project
  * 2023-2024  Sandro Kalatozishvili (s.kalatoz@gmail.com)
 *
  *@brief Packetize and unpacketize H264
 */

#include <iostream>
#include <cstring>
#include <vector>
#include "packetizer.hpp"

uint8_t* H264Packetizer::FindNextNAL(uint8_t *pStart, uint8_t *pEnd)
{
    uint8_t *ptr = pStart;
    while ((ptr <= pEnd - 3) && (ptr[0] || ptr[1] || ptr[2] != 1)) ++ptr;
    if (ptr > pEnd - 3) return nullptr;
    if (ptr > pStart && *(ptr - 1) == 0) return (ptr - 1);
    return ptr;
}

H264Packetizer::Status H264Packetizer::Packetize(uint8_t *pBuffer, size_t nBufLen, unsigned *pPos, const uint8_t **ppPayload, size_t *nPayloadLen)
{
    uint8_t *pOffset = pBuffer + *pPos;
    uint8_t *pEnd = pBuffer + nBufLen;
    uint8_t *pNALStart = nullptr;
    uint8_t *pNALEnd = nullptr;
    uint8_t *pNALOctet = nullptr;

    if (pEnd - pOffset >= 4)
        pNALStart = FindNextNAL(pOffset, pOffset + 4);

    if (!pNALStart)
    {
        // NAL start not found
        pNALStart = pOffset;
    }
    else
    {
        while (*pNALStart++ == 0);
        pNALOctet = pNALStart;
    }

    pOffset = pNALStart + m_cfg.nMTU + 1;
    if (pOffset > pEnd || m_cfg.mode == SingleNAL) pOffset = pEnd;

    pNALEnd = FindNextNAL(pNALStart, pOffset);
    if (!pNALEnd) pNALEnd = pOffset;

    if (m_cfg.mode == SingleNAL && pNALEnd - pNALStart > m_cfg.nMTU)
    {
        std::cerr << "MTU too small for H.264 (required=" << (pNALEnd - pNALStart) << ", MTU=" << m_cfg.nMTU << ")\n";
        return Small;
    }

    if ((m_cfg.mode != SingleNAL) && (!pNALOctet || pNALEnd - pNALStart > m_cfg.nMTU))
    {
        uint8_t nNRI, nType;

        if (pNALOctet)
        {
            nNRI = (*pNALOctet & 0x60) >> 5;
            nType = *pNALOctet & 0x1F;
            ++pNALStart;
        }
        else
        {
            pOffset = pNALStart - m_cfg.nMTU;
            nNRI = (*pOffset & 0x60) >> 5;
            nType = *(pOffset + 1) & 0x1F;
        }

        pOffset = pNALStart - 2;
        *pOffset = (nNRI << 5) | FU_A;
        ++pOffset;

        *pOffset = nType;
        if (pNALOctet) *pOffset |= (1 << 7);
        if (pNALEnd - pNALStart + 2 <= m_cfg.nMTU) *pOffset |= (1 << 6);

        *ppPayload = pNALStart - 2;
        if (pNALEnd - pNALStart + 2 > m_cfg.nMTU) *nPayloadLen = m_cfg.nMTU;
        else *nPayloadLen = pNALEnd - pNALStart + 2;

        *pPos = static_cast<unsigned>(*ppPayload + *nPayloadLen - pBuffer);
        return Success;
    }

    if ((m_cfg.mode != SingleNAL) && (pNALEnd != pEnd) && (pNALEnd - pNALStart + 3) < m_cfg.nMTU)
    {
        unsigned nNALCount = 1;
        size_t nNALSize[32];
        uint8_t *pNAL[32];

        pNAL[0] = pNALStart;
        nNALSize[0] = pNALEnd - pNALStart;

        int nTotalSize = static_cast<int>(nNALSize[0]) + 3;
        uint8_t nNRI = (*pNALOctet & 0x60) >> 5;

        while (nNALCount < 32)
        {
            pOffset = pNAL[nNALCount - 1] + nNALSize[nNALCount - 1];
            while (*pOffset++ == 0);
            pNAL[nNALCount] = pOffset;

            uint8_t *pTmpEnd = pOffset + (m_cfg.nMTU - nTotalSize);
            if (pTmpEnd > pEnd) pTmpEnd = pEnd;

            pOffset = FindNextNAL(pOffset + 1, pTmpEnd);
            if (pOffset) nNALSize[nNALCount] = pOffset - pNAL[nNALCount];
            else break;

            nTotalSize += (2 + static_cast<int>(nNALSize[nNALCount]));
            if (nTotalSize <= m_cfg.nMTU)
            {
                uint8_t nTmpNRI = (*(pNAL[nNALCount] - 1) & 0x60) >> 5;
                if (nTmpNRI > nNRI) nNRI = nTmpNRI;
            }
            else
            {
                // Total size is bigger than MTU
                break;
            }

            ++nNALCount;
        }

        if (nNALCount > 1)
        {
            pOffset = pNAL[0] - 3;
            *pOffset++ = (nNRI << 5) | STAP_A;

            for (unsigned i = 0; i < nNALCount; ++i)
            {
                *pOffset++ = static_cast<uint8_t>(nNALSize[i] >> 8);
                *pOffset++ = static_cast<uint8_t>(nNALSize[i] & 0xFF);

                if (pOffset != pNAL[i]) std::memmove(pOffset, pNAL[i], nNALSize[i]);
                pOffset += nNALSize[i];
            }

            *ppPayload = pNAL[0] - 3;
            if (*ppPayload < pBuffer + *pPos) return Invalid;

            *nPayloadLen = pOffset - *ppPayload;
            *pPos = static_cast<unsigned>(pNAL[nNALCount - 1] + nNALSize[nNALCount - 1] - pBuffer);

            return Success;
        }
    }

    *ppPayload = pNALStart;
    *nPayloadLen = pNALEnd - pNALStart;
    *pPos = static_cast<unsigned>(pNALEnd - pBuffer);

    return Success;
}

H264Packetizer::Status H264Packetizer::Unpacketize(const uint8_t *pPayload, size_t nPayloadLen, uint8_t *pBits, size_t nBitsLen, unsigned *pBitsPos)
{
    const uint8_t pNALStart[4] = {0, 0, 0, 1};
    const uint8_t *nStartCode = pNALStart + 4 - m_cfg.nUnpackNalStart;

    if (pPayload == nullptr)
    {
        m_unpackPrevLost = true;
        return Success;
    }

    if (nPayloadLen < 2)
    {
        m_unpackPrevLost = true;
        return Invalid;
    }

    if (*pBitsPos == 0) m_unpackLastSyncPos = 0;
    uint8_t nNALType = *pPayload & 0x1F;

    if (nNALType >= SINGLE_NAL_MIN && nNALType <= SINGLE_NAL_MAX)
    {
        uint8_t *pOffset = pBits + *pBitsPos;
        if (nBitsLen - *pBitsPos < nPayloadLen + m_cfg.nUnpackNalStart) return Small;

        std::memcpy(pOffset, nStartCode, m_cfg.nUnpackNalStart);
        pOffset += m_cfg.nUnpackNalStart;

        std::memcpy(pOffset, pPayload, nPayloadLen);
        pOffset += nPayloadLen;

        *pBitsPos = static_cast<unsigned>(pOffset - pBits);
        m_unpackLastSyncPos = *pBitsPos;

    }
    else if (nNALType == STAP_A)
    {
        if (nBitsLen - *pBitsPos < nPayloadLen + 32) return Small;
        uint8_t *pOffset = pBits + *pBitsPos;
        uint8_t *pEnd = pBits + nBitsLen;

        const uint8_t *pDataStart = pPayload + 1;
        const uint8_t *pDataEnd = pPayload + nPayloadLen;
        unsigned nCount = 0;

        while (pDataStart < pDataEnd && pOffset < pEnd)
        {
            if (pOffset + m_cfg.nUnpackNalStart > pEnd) return Invalid;
            std::memcpy(pOffset, nStartCode, m_cfg.nUnpackNalStart);
            pOffset += m_cfg.nUnpackNalStart;

            uint16_t nTmpNALSize = (*pDataStart << 8) | *(pDataStart + 1);
            pDataStart += 2;

            if (pOffset + nTmpNALSize > pEnd ||
                pDataStart + nTmpNALSize > pDataEnd)
                    return Invalid;

            std::memcpy(pOffset, pDataStart, nTmpNALSize);
            pOffset += nTmpNALSize;
            pDataStart += nTmpNALSize;
            ++nCount;

            *pBitsPos = static_cast<unsigned>(pOffset - pBits);
            m_unpackLastSyncPos = *pBitsPos;
        }

    }
    else if (nNALType == FU_A)
    {
        const uint8_t *pDataStart = pPayload;
        uint8_t *pOffset = pBits + *pBitsPos;

        if (nBitsLen - *pBitsPos < nPayloadLen + m_cfg.nUnpackNalStart)
        {
            m_unpackPrevLost = true;
            return Small;
        }

        uint8_t nStart = *(pDataStart + 1) & 0x80;
        uint8_t nEnd = *(pDataStart + 1) & 0x40;
        uint8_t nType = *(pDataStart + 1) & 0x1f;
        uint8_t nNRI = (*pDataStart & 0x60) >> 5;

        if (nStart)
        {
            std::memcpy(pOffset, nStartCode, m_cfg.nUnpackNalStart);
            pOffset += m_cfg.nUnpackNalStart;
            *pOffset++ = (nNRI << 5) | nType;
        }
        else if (m_unpackPrevLost)
        {
            if (m_unpackLastSyncPos > *pBitsPos) return Invalid;
            *pBitsPos = m_unpackLastSyncPos;
            return Ignored;
        }

        pDataStart += 2;
        std::memcpy(pOffset, pDataStart, nPayloadLen - 2);

        pOffset += (nPayloadLen - 2);
        *pBitsPos = static_cast<unsigned>(pOffset - pBits);
        if (nEnd) m_unpackLastSyncPos = *pBitsPos;
    }
    else
    {
        *pBitsPos = 0;
        return H264Packetizer::Unsupported;
    }

    m_unpackPrevLost = false;
    return H264Packetizer::Success;
}

std::vector<H264Packetizer::NALUnit> H264Packetizer::Repacketize(const uint8_t* pBuffer, size_t nLength)
{
    std::vector<NALUnit> nalUnits;
    size_t nPosit = 0;

    while (nPosit < nLength)
    {
        const uint8_t *pNALStart = nullptr;
        const uint8_t *pNALEnd = nullptr;
        const uint8_t *pPos = pBuffer + nPosit;
        const uint8_t *pEnd = pBuffer + nLength;

        if (pEnd - pPos >= 4)
        {
            uint8_t *pOffset = const_cast<uint8_t*>(pPos);
            pNALStart = FindNextNAL(pOffset, pOffset + 4);
        }

        if (pNALStart) while (*pNALStart++ == 0);
        else pNALStart = pPos;

        pPos = pNALStart + m_cfg.nMTU + 1;
        if (pPos > pEnd || m_cfg.mode == SingleNAL) pPos = pEnd;

        pNALEnd = FindNextNAL(const_cast<uint8_t*>(pNALStart), const_cast<uint8_t*>(pPos));
        if (!pNALEnd) pNALEnd = pPos;

        NALUnit nalUnit;
        nalUnit.data.assign(pNALStart, pNALEnd);
        nalUnits.push_back(nalUnit);

        nPosit = pNALEnd - pBuffer;
    }

    return nalUnits;
}

std::vector<H264Packetizer::NALUnit> H264Packetizer::ConvertMode(const uint8_t* pPayload, size_t nPayloadLength)
{
    std::vector<NALUnit> nalPackets;
    size_t pPos = 0;

    while (pPos < nPayloadLength)
    {
        uint8_t nNALType = pPayload[pPos] & 0x1F;

        if (nNALType == STAP_A)
        {
            // STAP-A: single-time aggregation packet
            pPos++; // Skip the STAP-A header

            while (pPos < nPayloadLength)
            {
                uint16_t nNALSize = (pPayload[pPos] << 8) | pPayload[pPos + 1];
                pPos += 2;

                NALUnit nalUnit;
                nalUnit.data.assign(pPayload + pPos, pPayload + pPos + nNALSize);

                nalPackets.push_back(nalUnit);
                pPos += nNALSize;
            }
        }
        else if (nNALType == FU_A)
        {
            // FU-A: fragmented NAL unit
            uint8_t nNRI = (pPayload[pPos] & 0x60) >> 5;
            uint8_t nOrigNALType = pPayload[pPos + 1] & 0x1F;
            pPos += 2; // Skip the FU-A header

            NALUnit nalUnit;
            nalUnit.data.push_back((nNRI << 5) | nOrigNALType);

            while (pPos < nPayloadLength)
            {
                uint8_t nFUHeader = pPayload[pPos - 1];
                const uint8_t *pStart = pPayload + pPos;
                const uint8_t *pEnd = pStart + m_cfg.nMTU - 2;

                nalUnit.data.insert(nalUnit.data.end(), pPayload + pPos, pEnd);
                pPos += m_cfg.nMTU - 2;

                if (nFUHeader & 0x40)
                {
                    // End of fragmented NAL unit
                    nalPackets.push_back(nalUnit);
                    break;
                }
            }
        }
        else
        {
            // Single NAL unit
            const uint8_t *pNALStart = pPayload + pPos;
            size_t nNALLen = std::min(nPayloadLength - pPos, (size_t)m_cfg.nMTU);

            const uint8_t *pNALEnd = FindNextNAL(const_cast<uint8_t*>(pNALStart), const_cast<uint8_t*>(pNALStart) + nNALLen);
            if (pNALEnd) nNALLen = pNALEnd - pNALStart;

            NALUnit nalUnit;
            nalUnit.data.assign(pNALStart, pNALStart + nNALLen);

            nalPackets.push_back(nalUnit);
            pPos += nNALLen;
        }
    }

    return nalPackets;
}