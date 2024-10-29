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


#include <atomic>
#include <thread>
#include <vector>

#include "common/BrotligConstants.h"

#include "encoder/PageEncoder.h"

#include "DataStream.h"

#include "BrotliG.h"
#include "BrotligEncoder.h"

using namespace BrotliG;

uint32_t BROTLIG_API BrotliG::MaxCompressedSize(uint32_t input_size, bool precondition, bool deltaencode)
{
    uint32_t numPages = (input_size + BROTLIG_DEFAULT_PAGE_SIZE - 1) / (BROTLIG_DEFAULT_PAGE_SIZE);
    uint32_t compressedPagesSize = static_cast<uint32_t>(PageEncoder::MaxCompressedSize(BROTLIG_DEFAULT_PAGE_SIZE));
    uint32_t estimatedSize = (numPages * compressedPagesSize) + (numPages * BROTLIG_PAGE_HEADER_SIZE_BYTES) + sizeof(StreamHeader);

    if (precondition) {
        estimatedSize += sizeof(PreconditionHeader);
        if (deltaencode)
            estimatedSize += numPages * BROTLIG_PRECON_DELTA_ENCODING_BASES_SIZE_BYTES;
    }

    return estimatedSize;
}

BROTLIG_ERROR BROTLIG_API BrotliG::CheckParams(uint32_t page_size, BrotligDataconditionParams dcParams)
{
    if (page_size < BROTLIG_MIN_PAGE_SIZE)
        return BROTLIG_ERROR_MIN_PAGE_SIZE;

    if (page_size > BROTLIG_MAX_PAGE_SIZE)
        return BROTLIG_ERROR_MAX_PAGE_SIZE;

    if (dcParams.precondition)
        return dcParams.CheckParams();

    return BROTLIG_OK;
}

static BROTLIG_ERROR EncodeWithPreconSinglethreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BrotligDataconditionParams& dcParams,
    BROTLIG_Feedback_Proc feedbackProc
)
{
    uint32_t srcCondSize = 0;
    std::unique_ptr<uint8_t[]> srcConditioned = BrotliG::Condition(input_size, src, dcParams, srcCondSize);

    if (!srcConditioned)
        return BROTLIG_ERROR_PRECON_INCORRECT_FORMAT;

    uint32_t numPages = (srcCondSize + page_size - 1) / page_size;

    size_t maxOutPageSize = PageEncoder::MaxCompressedSize(page_size);
    std::unique_ptr<uint8_t[]> tOutput = std::make_unique<uint8_t[]>(maxOutPageSize * numPages);
    std::unique_ptr<size_t[]> tOutpageSizes = std::make_unique<size_t[]>(numPages);

    std::vector<size_t> outputPageSize(numPages);

    BrotligEncoderParams params = {
        BROTLI_MAX_QUALITY,
        BROTLI_MAX_WINDOW_BITS,
        page_size
    };

    PageEncoder pEncoder;
    pEncoder.Setup(params, &dcParams);

    uint32_t pageIndex = 0;
    uint32_t sizeLeftToRead = srcCondSize, sizeToRead = 0, curInOffset = 0, curOutOffset = 0;
    uint8_t* srcPtr = srcConditioned.get();
    uint8_t* outPtr = tOutput.get();

    while (pageIndex < numPages) {
        sizeToRead = (sizeLeftToRead > page_size) ? page_size : sizeLeftToRead;

        tOutpageSizes[pageIndex] = maxOutPageSize;
        pEncoder.Run(srcPtr, sizeToRead, curInOffset, outPtr, &tOutpageSizes[pageIndex], curOutOffset, (pageIndex == numPages - 1));

        outputPageSize.at(pageIndex) = tOutpageSizes[pageIndex];

        sizeLeftToRead -= sizeToRead;
        curInOffset += sizeToRead;
        curOutOffset += (uint32_t)maxOutPageSize;
        ++pageIndex;

        if (feedbackProc)
        {
            float progress = 100.f * ((float)(pageIndex) / numPages);
            if (feedbackProc(BROTLIG_MESSAGE_TYPE::BROTLIG_PROGRESS, std::to_string(progress)))
            {
                break;
            }
        }
    }

    // Prepare page stream
    size_t tcompressedSize = 0;
    outPtr = output;

    StreamHeader header = {};
    header.SetId(BROTLIG_STREAM_ID);
    header.SetPageSize(page_size);
    header.SetUncompressedSize(input_size);
    header.SetPreconditioned(dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    if (dcParams.precondition)
    {
        PreconditionHeader preconHeader = {};
        preconHeader.Swizzled = dcParams.swizzle;
        preconHeader.PitchD3D12Aligned = dcParams.pitchd3d12aligned;
        preconHeader.WidthInBlocks = dcParams.widthInBlocks[0] - 1;
        preconHeader.HeightInBlocks = dcParams.heightInBlocks[0] - 1;
        preconHeader.SetDataFormat(dcParams.format);
        preconHeader.NumMips = dcParams.numMipLevels - 1;
        preconHeader.PitchInBytes = dcParams.pitchInBytes[0] - 1;

        size_t preconHeaderSize = sizeof(PreconditionHeader);
        memcpy(outPtr, reinterpret_cast<char*>(&preconHeader), preconHeaderSize);
        outPtr += preconHeaderSize;
        tcompressedSize += preconHeaderSize;
    }

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    srcPtr = tOutput.get();
    curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < numPages; ++pindex)
    {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, tOutpageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += tOutpageSizes[pindex];
        curOutOffset += (uint32_t)tOutpageSizes[pindex];
        curInOffset += (uint32_t)maxOutPageSize;
    }

    pageTable[0] = (uint32_t)tOutpageSizes[numPages - 1];

    *output_size = (uint32_t)tcompressedSize;

    return BROTLIG_OK;
}

static BROTLIG_ERROR EncodeNoPreconSinglethreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BROTLIG_Feedback_Proc feedbackProc
)
{
    const uint8_t* srcPtr = src;
    uint32_t numPages = (input_size + page_size - 1) / page_size;

    size_t maxOutPageSize = PageEncoder::MaxCompressedSize(page_size);
    std::unique_ptr<uint8_t[]> tOutput = std::make_unique<uint8_t[]>(maxOutPageSize * numPages);
    std::unique_ptr<size_t[]> tOutpageSizes = std::make_unique<size_t[]>(numPages);

    std::vector<size_t> outputPageSize(numPages);

    BrotligEncoderParams params = {
        BROTLI_MAX_QUALITY,
        BROTLI_MAX_WINDOW_BITS,
        page_size
    };

    BrotligDataconditionParams dcParams = {};

    PageEncoder pEncoder;
    pEncoder.Setup(params, &dcParams);

    uint32_t pageIndex = 0;
    uint32_t sizeLeftToRead = input_size, sizeToRead = 0, curInOffset = 0, curOutOffset = 0;

    uint8_t* outPtr = tOutput.get();

    while (pageIndex < numPages) {

        sizeToRead = (sizeLeftToRead > page_size) ? page_size : sizeLeftToRead;

        tOutpageSizes[pageIndex] = maxOutPageSize;
        pEncoder.Run(srcPtr, sizeToRead, curInOffset, outPtr, &tOutpageSizes[pageIndex], curOutOffset, (pageIndex == numPages - 1));

        outputPageSize.at(pageIndex) = tOutpageSizes[pageIndex];

        sizeLeftToRead -= sizeToRead;
        curInOffset += sizeToRead;
        curOutOffset += (uint32_t)maxOutPageSize;
        ++pageIndex;

        if (feedbackProc)
        {
            float progress = 100.f * ((float)(pageIndex) / numPages);
            if (feedbackProc(BROTLIG_MESSAGE_TYPE::BROTLIG_PROGRESS, std::to_string(progress)))
            {
                break;
            }
        }
    }

    // Prepare page stream
    size_t tcompressedSize = 0;
    outPtr = output;

    StreamHeader header = {};
    header.SetId(BROTLIG_STREAM_ID);
    header.SetPageSize(page_size);
    header.SetUncompressedSize(input_size);
    header.SetPreconditioned(dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    srcPtr = tOutput.get();
    curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < numPages; ++pindex)
    {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, tOutpageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += tOutpageSizes[pindex];
        curOutOffset += (uint32_t)tOutpageSizes[pindex];
        curInOffset += (uint32_t)maxOutPageSize;
    }

    pageTable[0] = (uint32_t)tOutpageSizes[numPages - 1];

    *output_size = (uint32_t)tcompressedSize;

    return BROTLIG_OK;
}

static BROTLIG_ERROR EncodeSinglethreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BrotligDataconditionParams dcParams,
    BROTLIG_Feedback_Proc feedbackProc)
{
    BROTLIG_ERROR status = BrotliG::CheckParams(page_size, dcParams);

    if (status != BROTLIG_OK) return status;

    if (dcParams.precondition)
    {
        if (!dcParams.Initialize(input_size))
        {
            dcParams.precondition = false;

            if (feedbackProc)
            {
                std::string msg = "Warning: Incorrect texture format. Preconditioning not applied.";
                feedbackProc(BROTLIG_MESSAGE_TYPE::BROTLIG_WARNING, msg);
            }
        }
    }

    if (dcParams.precondition)
    {
        return EncodeWithPreconSinglethreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            dcParams,
            feedbackProc
        );
    }
    else
    {
      return EncodeNoPreconSinglethreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            feedbackProc
        );
    }
}

struct PageEncoderCtx
{
    std::atomic_uint32_t globalIndex = 0;

    uint32_t maxOutPageSize = 0;
    uint32_t numPages = 0;
    uint32_t lastPageSize = 0;

    std::unique_ptr<uint8_t[]> outputPtr;
    std::unique_ptr<size_t[]> outPageSizes;
    const uint8_t* inputPtr = nullptr;

    BROTLIG_Feedback_Proc feedbackProc = nullptr;

    PageEncoderCtx(size_t page_size, uint32_t input_size, const uint8_t *input_ptr, BROTLIG_Feedback_Proc feedback)
        : maxOutPageSize(BrotliG::MaxCompressedSize(page_size))
        , numPages((input_size + page_size - 1) / page_size)
        , lastPageSize(input_size - ((numPages - 1) * page_size))
        , outputPtr(std::make_unique<uint8_t[]>(maxOutPageSize * numPages))
        , outPageSizes(std::make_unique<size_t[]>(numPages))
        , inputPtr(input_ptr)
        , feedbackProc(feedback)
    { }
};

static void PageEncoderJob(PageEncoderCtx& ctx, BrotligEncoderParams& params, BrotligDataconditionParams& dcParams)
{
    PageEncoder pEncoder;
    pEncoder.Setup(params, &dcParams);

    uint32_t curInOffset = 0, curOutOffset = 0;
    size_t inPageSize = 0;
    while (true)
    {
        const uint32_t pageIndex = ctx.globalIndex.fetch_add(1, std::memory_order_relaxed);

        if (pageIndex >= ctx.numPages)
            break;

        curInOffset = pageIndex * (uint32_t)params.page_size;
        inPageSize = (pageIndex < ctx.numPages - 1) ? params.page_size : ctx.lastPageSize;

        curOutOffset = pageIndex * ctx.maxOutPageSize;
        ctx.outPageSizes[pageIndex] = ctx.maxOutPageSize;

        pEncoder.Run(ctx.inputPtr, inPageSize, curInOffset, ctx.outputPtr.get(), &ctx.outPageSizes[pageIndex], curOutOffset, (pageIndex == ctx.numPages - 1));

        if (ctx.feedbackProc)
        {
            float progress = 100.f * ((float)(pageIndex) / ctx.numPages);
            if (ctx.feedbackProc(BROTLIG_MESSAGE_TYPE::BROTLIG_PROGRESS, std::to_string(progress)))
            {
                break;
            }
        }
    }
}

static BROTLIG_ERROR EncodeWithPreconMultithreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BrotligDataconditionParams& dcParams,
    BROTLIG_Feedback_Proc feedbackProc
)
{
    uint32_t srcCondSize = 0;
    std::unique_ptr<uint8_t[]> srcConditioned = BrotliG::Condition(input_size, src, dcParams, srcCondSize);

    if (!srcConditioned)
        return BROTLIG_ERROR_INCORRECT_STREAM_FORMAT;

    PageEncoderCtx ctx{page_size, srcCondSize, srcConditioned.get(), feedbackProc};

    BrotligEncoderParams params = {
        BROTLI_MAX_QUALITY,
        BROTLI_MAX_WINDOW_BITS,
        page_size
    };

    const uint32_t maxWorkers = std::min(static_cast<unsigned int>(BROTLIG_MAX_WORKERS), BrotliG::GetNumberOfProcessorsThreads());
    std::thread workers[BROTLIG_MAX_WORKERS];

    uint32_t numWorkersLeft = (ctx.numPages > 2 * maxWorkers) ? maxWorkers : 1;
    for (auto& worker : workers)
    {
        if (numWorkersLeft == 1)
            break;

        worker = std::thread([&ctx, &params, &dcParams]() {PageEncoderJob(ctx, params, dcParams); });
        --numWorkersLeft;
    }

    PageEncoderJob(ctx, params, dcParams);

    for (auto& worker : workers)
    {
        if (worker.joinable())
            worker.join();
    }

    // Prepare page stream
    size_t tcompressedSize = 0;
    uint8_t* outPtr = output;

    StreamHeader header = {};
    header.SetId (BROTLIG_STREAM_ID);
    header.SetPageSize (page_size);
    header.SetUncompressedSize (input_size);
    header.SetPreconditioned (dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    if (dcParams.precondition)
    {
        PreconditionHeader preconHeader = {};
        preconHeader.Swizzled           = dcParams.swizzle;
        preconHeader.PitchD3D12Aligned  = dcParams.pitchd3d12aligned;
        preconHeader.WidthInBlocks      = dcParams.widthInBlocks[0] - 1;
        preconHeader.HeightInBlocks     = dcParams.heightInBlocks[0] - 1;
        preconHeader.SetDataFormat (dcParams.format);
        preconHeader.NumMips            = dcParams.numMipLevels - 1;
        preconHeader.PitchInBytes       = dcParams.pitchInBytes[0] - 1;

        size_t preconHeaderSize = sizeof(PreconditionHeader);
        memcpy(outPtr, reinterpret_cast<char*>(&preconHeader), preconHeaderSize);
        outPtr += preconHeaderSize;
        tcompressedSize += preconHeaderSize;
    }

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = ctx.numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    const uint8_t* srcPtr = ctx.outputPtr.get();
    uint32_t curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < ctx.numPages; ++pindex)
    {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, ctx.outPageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += ctx.outPageSizes[pindex];
        curOutOffset += (uint32_t)ctx.outPageSizes[pindex];
        curInOffset += (uint32_t)ctx.maxOutPageSize;
    }

    pageTable[0] = (uint32_t)ctx.outPageSizes[ctx.numPages - 1];

    *output_size = (uint32_t)tcompressedSize;

    return BROTLIG_OK;
}

static BROTLIG_ERROR EncodeNoPreconMultithreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BROTLIG_Feedback_Proc feedbackProc
)
{
    PageEncoderCtx ctx{page_size, input_size, src, feedbackProc};

    BrotligEncoderParams params = {
        BROTLI_MAX_QUALITY,
        BROTLI_MAX_WINDOW_BITS,
        page_size
    };

    BrotligDataconditionParams dcParams = {};

    const uint32_t maxWorkers = std::min(static_cast<unsigned int>(BROTLIG_MAX_WORKERS), BrotliG::GetNumberOfProcessorsThreads());

    std::thread workers[BROTLIG_MAX_WORKERS];
    uint32_t numWorkersLeft = (ctx.numPages > 2 * maxWorkers) ? maxWorkers : 1;
    for (auto& worker : workers)
    {
        if (numWorkersLeft == 1)
            break;

        worker = std::thread([&ctx, &params, &dcParams]() {PageEncoderJob(ctx, params, dcParams); });
        --numWorkersLeft;
    }

    PageEncoderJob(ctx, params, dcParams);

    for (auto& worker : workers)
    {
        if (worker.joinable())
            worker.join();
    }

    // Prepare page stream
    size_t tcompressedSize = 0;
    uint8_t* outPtr = output;

    StreamHeader header = {};
    header.SetId(BROTLIG_STREAM_ID);
    header.SetPageSize(page_size);
    header.SetUncompressedSize(input_size);
    header.SetPreconditioned(dcParams.precondition);
    size_t headersize = sizeof(StreamHeader);
    memcpy(outPtr, reinterpret_cast<char*>(&header), headersize);
    outPtr += headersize;
    tcompressedSize += headersize;

    uint32_t* pageTable = reinterpret_cast<uint32_t*>(outPtr);
    size_t tablesize = ctx.numPages * sizeof(uint32_t);
    outPtr += tablesize;
    tcompressedSize += tablesize;
    const uint8_t* srcPtr = ctx.outputPtr.get();
    uint32_t curInOffset = 0, curOutOffset = 0;

    for (size_t pindex = 0; pindex < ctx.numPages; ++pindex)
    {
        memcpy(outPtr + curOutOffset, srcPtr + curInOffset, ctx.outPageSizes[pindex]);
        pageTable[pindex] = curOutOffset;
        tcompressedSize += ctx.outPageSizes[pindex];
        curOutOffset += (uint32_t)ctx.outPageSizes[pindex];
        curInOffset += (uint32_t)ctx.maxOutPageSize;
    }

    pageTable[0] = (uint32_t)ctx.outPageSizes[ctx.numPages - 1];

    *output_size = (uint32_t)tcompressedSize;

    return BROTLIG_OK;
}

static BROTLIG_ERROR EncodeMultithreaded(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BrotligDataconditionParams dcParams,
    BROTLIG_Feedback_Proc feedbackProc)
{
    BROTLIG_ERROR status = BrotliG::CheckParams(page_size, dcParams);

    if (status != BROTLIG_OK) return status;

    if (dcParams.precondition)
    {
        if (!dcParams.Initialize(input_size))
        {
            dcParams.precondition = false;

            if (feedbackProc)
            {
                std::string msg = "Warning: Incorrect texture format. Preconditioning not applied.";
                feedbackProc(BROTLIG_MESSAGE_TYPE::BROTLIG_WARNING, msg);
            }
        }
    }

    if (dcParams.precondition)
    {
        return EncodeWithPreconMultithreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            dcParams,
            feedbackProc
        );
    }
    else
    {
        return EncodeNoPreconMultithreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            feedbackProc
        );
    }
}

BROTLIG_ERROR BROTLIG_API BrotliG::Encode(
    uint32_t input_size,
    const uint8_t* src,
    uint32_t* output_size,
    uint8_t* output,
    uint32_t page_size,
    BrotligDataconditionParams dcParams,
    BROTLIG_Feedback_Proc feedbackProc,
    bool multithreaded)
{
    if (multithreaded)
    {
        return EncodeMultithreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            dcParams,
            feedbackProc
        );
    }
    else
    {
        return EncodeSinglethreaded(
            input_size,
            src,
            output_size,
            output,
            page_size,
            dcParams,
            feedbackProc
        );
    }
}