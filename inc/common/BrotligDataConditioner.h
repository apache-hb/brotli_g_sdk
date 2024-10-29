// Brotli-G SDK 1.1
//
// Copyright(c) 2022 - 2024 Advanced Micro Devices, Inc. All rights reserved.
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include "BrotligConstants.h"
#include "BrotligCommon.h"
#include <memory>

namespace BrotliG
{
    struct BrotligDataconditionParams
    {
        bool precondition = false;
        bool swizzle = false;
        bool delta_encode = false;

        BROTLIG_DATA_FORMAT format = BROTLIG_DATA_FORMAT_UNKNOWN;
        uint32_t widthInPixels = 0;
        uint32_t heightInPixels = 0;
        uint32_t numMipLevels = 1;
        uint32_t rowPitchInBytes = 0;
        bool pitchd3d12aligned = false;

        uint32_t blockSizeBytes = BROTLIG_DEFAULT_BLOCK_SIZE_BYTES;
        uint32_t blockSizePixels = BROTLIG_DEFAULT_BLOCK_SIZE_PIXELS;
        uint32_t colorSizeBits = BROTLIG_DEFAULT_COLOR_SIZE_BITS;
        uint32_t numSubBlocks = 0;

        uint32_t subBlockSizes[BROTLIG_MAX_NUM_SUB_BLOCKS] = { 0 };
        uint32_t subBlockOffsets[BROTLIG_MAX_NUM_SUB_BLOCKS] = { 0 };

        uint32_t colorSubBlocks[BROTLIG_MAX_NUM_SUB_BLOCKS] = { 0 };
        uint32_t numColorSubBlocks = 0;

        uint32_t widthInBlocks[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS] = { 0 };
        uint32_t heightInBlocks[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS] = { 0 };
        uint32_t pitchInBytes[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS] = { 0 };
        uint32_t numBlocks[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS] = { 0 };
        uint32_t subStreamOffsets[BROTLIG_MAX_NUM_SUB_BLOCKS + 1] = { 0 };
        uint32_t mipOffsetsBytes[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS + 1] = { 0 };
        uint32_t mipOffsetBlocks[BROTLIG_PRECON_MAX_NUM_MIP_LEVELS + 1] = { 0 };
        uint32_t tNumBlocks = 0;

        bool isInitialized = false;

        [[nodiscard]]
        BROTLIG_ERROR CheckParams() const;

        bool Initialize(uint32_t inSize);
    };

    std::unique_ptr<uint8_t[]> Condition(uint32_t inSize, const uint8_t* inData, BrotligDataconditionParams& params, uint32_t& outSize);
}
