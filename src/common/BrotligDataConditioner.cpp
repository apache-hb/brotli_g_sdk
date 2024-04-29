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

#include "common/BrotligCommon.h"
#include "common/BrotligConstants.h"

#include "BrotligDataConditioner.h"
#include "BrotligUtils.h"
#include <memory>

static bool Swizzle(uint32_t size, uint8_t* data, uint32_t blockSize, uint32_t widthInBlocks, uint32_t heightInBlocks, uint32_t pitch)
{
    if (widthInBlocks < BROTLIG_PRECON_SWIZZLE_REGION_SIZE || heightInBlocks < BROTLIG_PRECON_SWIZZLE_REGION_SIZE)
        return false;

    std::unique_ptr temp = std::make_unique<uint8_t[]>(size);
    memcpy(temp.get(), data, size);

    uint32_t effWidthInBlocks = widthInBlocks - (widthInBlocks % BROTLIG_PRECON_SWIZZLE_REGION_SIZE);
    uint32_t effHeightInBlocks = heightInBlocks - (heightInBlocks % BROTLIG_PRECON_SWIZZLE_REGION_SIZE);

    uint32_t inIndex = 0, outRow = 0, outCol = 0, outIndex = 0;
    for (uint32_t row = 0; row < effHeightInBlocks; row += BROTLIG_PRECON_SWIZZLE_REGION_SIZE)
    {
        for (uint32_t col = 0; col < effWidthInBlocks; col += BROTLIG_PRECON_SWIZZLE_REGION_SIZE)
        {
            for (uint32_t rowoffset = 0; rowoffset < BROTLIG_PRECON_SWIZZLE_REGION_SIZE; ++rowoffset)
            {
                for (uint32_t coloffset = 0; coloffset < BROTLIG_PRECON_SWIZZLE_REGION_SIZE; ++coloffset)
                {
                    inIndex = ((row + rowoffset) * pitch) + ((col + coloffset) * blockSize);
                    assert(inIndex < size);
                    outIndex = (outRow * pitch) + (outCol * blockSize);
                    assert(outIndex < size);

                    memcpy(&data[outIndex], &temp[inIndex], blockSize);
                    ++outCol;

                    if (outCol == effWidthInBlocks)
                    {
                        outCol = 0; ++outRow;
                    }
                }
            }
        }
    }

    return true;
}

static void ConditionBC1_5(uint32_t inSize, const uint8_t* inData, BrotliG::BrotligDataconditionParams& params, uint32_t& outSize, uint8_t*& outData)
{
    std::unique_ptr temp = std::make_unique<uint8_t[]>(inSize);
    memcpy(temp.get(), inData, inSize);

    outSize = inSize;
    outData = new uint8_t[outSize];
    memset(outData, 0, outSize);

    if (params.swizzle)
    {
        for (uint32_t mip = 0; mip < params.numMipLevels; ++mip)
        {
            Swizzle(
                params.pitchInBytes[mip] * params.heightInBlocks[mip],
                temp.get() + params.mipOffsetsBytes[mip],
                params.blockSizeBytes,
                params.widthInBlocks[mip],
                params.heightInBlocks[mip],
                params.pitchInBytes[mip]
            );
        }
    }

    uint32_t subStreamCopyPtrs[BROTLIG_MAX_NUM_SUB_BLOCKS];
    memcpy(&subStreamCopyPtrs, &params.subStreamOffsets, BROTLIG_MAX_NUM_SUB_BLOCKS * sizeof(uint32_t));
    for (uint32_t mip = 0; mip < params.numMipLevels; ++mip)
    {
        uint32_t mipOffset = params.mipOffsetsBytes[mip], rowOffset = 0, inIndex = 0;

        for (uint32_t row = 0; row < params.heightInBlocks[mip]; ++row)
        {
            rowOffset = params.pitchInBytes[mip] * row;
            for (uint32_t col = 0; col < params.widthInBlocks[mip]; ++col)
            {
                inIndex = mipOffset + rowOffset + (col * params.blockSizeBytes);

                for (uint32_t sub = 0; sub < params.numSubBlocks; ++sub)
                {
                    memcpy(&outData[subStreamCopyPtrs[sub]], &temp[inIndex], params.subBlockSizes[sub]);
                    inIndex += params.subBlockSizes[sub];
                    subStreamCopyPtrs[sub] += params.subBlockSizes[sub];
                }
            }
        }
    }
}

BROTLIG_ERROR BrotliG::BrotligDataconditionParams::CheckParams() const
{
    if (widthInPixels < BROTLIG_PRECON_MIN_TEX_WIDTH_PIXEL)
        return BROTLIG_ERROR_PRECON_MIN_TEX_WIDTH;

    if (widthInPixels > BROTLIG_PRECON_MAX_TEX_WIDTH_PIXEL)
        return BROTLIG_ERROR_PRECON_MAX_TEX_WIDTH;

    if (heightInPixels < BROTLIG_PRECON_MIN_TEX_HEIGHT_PIXEL)
        return BROTLIG_ERROR_PRECON_MIN_TEX_HEIGHT;

    if (heightInPixels > BROTLIG_PRECON_MAX_TEX_HEIGHT_PIXEL)
        return BROTLIG_ERROR_PRECON_MAX_TEX_HEIGHT;

    if (rowPitchInBytes < BROTLIG_PRECON_MIN_TEX_PITCH_BYTES)
        return BROTLIG_ERROR_PRECON_MIN_TEX_PITCH;

    if (rowPitchInBytes > BROTLIG_PRECON_MAX_TEX_PITCH_BYTES)
        return BROTLIG_ERROR_PRECON_MAX_TEX_PITCH;

    if (numMipLevels < BROTLIG_PRECON_MIN_NUM_MIP_LEVELS)
        return BROTLIG_ERROR_PRECON_MIN_TEX_MIPLEVELS;

    if (numMipLevels > BROTLIG_PRECON_MAX_NUM_MIP_LEVELS)
        return BROTLIG_ERROR_PRECON_MAX_TEX_MIPLEVELS;

    return BROTLIG_OK;
}

bool BrotliG::BrotligDataconditionParams::Initialize(uint32_t inSize)
{
    if (isInitialized) return true;

    switch (format)
    {
    case BROTLIG_DATA_FORMAT_BC1:
        blockSizePixels = BROTLIG_BC1_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_BC1_BLOCK_SIZE_BYTES;
        colorSizeBits = BROTLIG_BC1_COLOR_SIZE_BITS;

        numSubBlocks = BROTLIG_BC1_NUM_SUB_BLOCKS;
        subBlockSizes[0] = BROTLIG_BC1_COLOR_TOP_REF_SIZE_BYTES;
        subBlockSizes[1] = BROTLIG_BC1_COLOR_BOT_REF_SIZE_BYTES;
        subBlockSizes[2] = BROTLIG_BC1_COLOR_INDEX_SIZE_BYTES;

        colorSubBlocks[numColorSubBlocks++] = 0;
        colorSubBlocks[numColorSubBlocks++] = 1;
        break;

    case BROTLIG_DATA_FORMAT_BC2:
        blockSizePixels = BROTLIG_BC2_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_BC2_BLOCK_SIZE_BYTES;
        colorSizeBits = BROTLIG_BC2_COLOR_SIZE_BITS;

        numSubBlocks = BROTLIG_BC2_NUM_SUB_BLOCKS;
        subBlockSizes[0] = BROTLIG_BC2_ALPHA_VECTOR_SIZE_BYTES;
        subBlockSizes[1] = BROTLIG_BC2_COLOR_TOP_REF_SIZE_BYTES;
        subBlockSizes[2] = BROTLIG_BC2_COLOR_BOT_REF_SIZE_BYTES;
        subBlockSizes[3] = BROTLIG_BC2_COLOR_INDEX_SIZE_BYTES;

        colorSubBlocks[numColorSubBlocks++] = 1;
        colorSubBlocks[numColorSubBlocks++] = 2;
        break;

    case BROTLIG_DATA_FORMAT_BC3:
        blockSizePixels = BROTLIG_BC3_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_BC3_BLOCK_SIZE_BYTES;
        colorSizeBits = BROTLIG_BC3_COLOR_SIZE_BITS;

        numSubBlocks = BROTLIG_BC3_NUM_SUB_BLOCKS;
        subBlockSizes[0] = BROTLIG_BC3_ALPHA_TOP_REF_SIZE_BYTES;
        subBlockSizes[1] = BROTLIG_BC3_ALPHA_BOT_REF_SIZE_BYTES;
        subBlockSizes[2] = BROTLIG_BC3_ALPHA_INDEX_SIZE_BYTES;
        subBlockSizes[3] = BROTLIG_BC3_COLOR_TOP_REF_SIZE_BYTES;
        subBlockSizes[4] = BROTLIG_BC3_COLOR_BOT_REF_SIZE_BYTES;
        subBlockSizes[5] = BROTLIG_BC3_COLOR_INDEX_SIZE_BYTES;

        colorSubBlocks[numColorSubBlocks++] = 3;
        colorSubBlocks[numColorSubBlocks++] = 4;
        break;

    case BROTLIG_DATA_FORMAT_BC4:
        blockSizePixels = BROTLIG_BC4_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_BC4_BLOCK_SIZE_BYTES;
        colorSizeBits = BROTLIG_BC4_COLOR_SIZE_BITS;

        numSubBlocks = BROTLIG_BC4_NUM_SUB_BLOCKS;
        subBlockSizes[0] = BROTLIG_BC4_COLOR_TOP_REF_SIZE_BYTES;
        subBlockSizes[1] = BROTLIG_BC4_COLOR_BOT_REF_SIZE_BYTES;
        subBlockSizes[2] = BROTLIG_BC4_COLOR_INDEX_SIZE_BYTES;

        colorSubBlocks[numColorSubBlocks++] = 0;
        colorSubBlocks[numColorSubBlocks++] = 1;
        break;

    case BROTLIG_DATA_FORMAT_BC5:
        blockSizePixels = BROTLIG_BC5_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_BC5_BLOCK_SIZE_BYTES;
        colorSizeBits = BROTLIG_BC5_COLOR_SIZE_BITS;

        numSubBlocks = BROTLIG_BC5_NUM_SUB_BLOCKS;
        subBlockSizes[0] = BROTLIG_BC5_RED_TOP_REF_SIZE_BYTES;
        subBlockSizes[1] = BROTLIG_BC5_RED_BOT_REF_SIZE_BYTES;
        subBlockSizes[2] = BROTLIG_BC5_RED_INDEX_SIZE_BYTES;
        subBlockSizes[3] = BROTLIG_BC5_GREEN_TOP_REF_SIZE_BYTES;
        subBlockSizes[4] = BROTLIG_BC5_GREEN_BOT_REF_SIZE_BYTES;
        subBlockSizes[5] = BROTLIG_BC5_GREEN_INDEX_SIZE_BYTES;

        colorSubBlocks[numColorSubBlocks++] = 0;
        colorSubBlocks[numColorSubBlocks++] = 1;
        colorSubBlocks[numColorSubBlocks++] = 3;
        colorSubBlocks[numColorSubBlocks++] = 4;
        break;
    default:
        blockSizePixels = BROTLIG_DEFAULT_BLOCK_SIZE_PIXELS;
        blockSizeBytes = BROTLIG_DEFAULT_BLOCK_SIZE_BYTES;

        numSubBlocks = 1;
        subBlockSizes[0] = BROTLIG_DEFAULT_BLOCK_SIZE_BYTES;
        break;
    }

    if (numMipLevels == 0) numMipLevels = 1;

    if (widthInBlocks[0] == 0) widthInBlocks[0] = (widthInPixels + blockSizePixels - 1) / blockSizePixels;
    if (heightInBlocks[0] == 0) heightInBlocks[0] = (heightInPixels + blockSizePixels - 1) / blockSizePixels;

    if (widthInPixels == 0) widthInPixels = widthInBlocks[0] * blockSizePixels;
    if (heightInPixels == 0) heightInPixels = heightInBlocks[0] * blockSizePixels;

    tNumBlocks = numBlocks[0] = widthInBlocks[0] * heightInBlocks[0];

    if (pitchInBytes[0] == 0)
    {
        if (rowPitchInBytes != 0) pitchInBytes[0] = rowPitchInBytes;
        else pitchInBytes[0] = (pitchd3d12aligned) ? (RoundUp(widthInBlocks[0] * blockSizeBytes, BROTLIG_D3D12_TEXTURE_DATA_PITCH_ALIGNMENT_BTYES)) : (widthInBlocks[0] * blockSizeBytes);
    }

    uint32_t mipwidthpx = (widthInBlocks[0] * blockSizePixels) / 2, mipheightpx = (heightInBlocks[0] * blockSizePixels) / 2, mip = 1;
    for (; mip <= numMipLevels; ++mip)
    {
        if (mip < numMipLevels)
        {
            widthInBlocks[mip] = (mipwidthpx + blockSizePixels - 1) / blockSizePixels;
            heightInBlocks[mip] = (mipheightpx + blockSizePixels - 1) / blockSizePixels;
            numBlocks[mip] = widthInBlocks[mip] * heightInBlocks[mip];
            pitchInBytes[mip] = (pitchd3d12aligned) ? (RoundUp(widthInBlocks[mip] * blockSizeBytes, BROTLIG_D3D12_TEXTURE_DATA_PITCH_ALIGNMENT_BTYES)) : (widthInBlocks[mip] * blockSizeBytes);
            tNumBlocks += numBlocks[mip];
        }
        mipOffsetsBytes[mip] = mipOffsetsBytes[mip - 1] + (pitchInBytes[mip - 1] * heightInBlocks[mip - 1]);
        mipOffsetBlocks[mip] = mipOffsetBlocks[mip - 1] + numBlocks[mip - 1];

        mipwidthpx /= 2;
        mipheightpx /= 2;
    }

    if (mipOffsetsBytes[numMipLevels] != inSize) return false;

    uint32_t sub = 1;
    for (; sub <= numSubBlocks; ++sub)
    {
        if (sub < numSubBlocks) subBlockOffsets[sub] = (sub == 0) ? 0 : subBlockOffsets[sub - 1] + subBlockSizes[sub - 1];
        subStreamOffsets[sub] = subStreamOffsets[sub - 1];
        for (uint32_t mip = 0; mip < numMipLevels; ++mip)
        {
            subStreamOffsets[sub] += (numBlocks[mip] * subBlockSizes[sub - 1]);
        }
    }

    if (subStreamOffsets[numSubBlocks] != tNumBlocks * blockSizeBytes) return false;

    isInitialized = true;

    return true;
}

bool BrotliG::Condition(uint32_t inSize, const uint8_t* inData, BrotligDataconditionParams& params, uint32_t& outSize, uint8_t*& outData)
{
    switch (params.format)
    {
    case BROTLIG_DATA_FORMAT_BC1:
    case BROTLIG_DATA_FORMAT_BC2:
    case BROTLIG_DATA_FORMAT_BC3:
    case BROTLIG_DATA_FORMAT_BC4:
    case BROTLIG_DATA_FORMAT_BC5:
        ConditionBC1_5(inSize, inData, params, outSize, outData);
        return true;
    default:
        return false;
    }
}
