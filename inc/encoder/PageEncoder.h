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

/* Copyright 2013 Google Inc. All Rights Reserved.

   Distributed under MIT license.
   See file LICENSE for detail or copy at https://opensource.org/licenses/MIT
*/

#pragma once

extern "C" {
#include "c/enc/command.h"
#include "c/enc/hash.h"
#include "brotli/encode.h"
}

#include "common/BrotligDataConditioner.h"
#include "encoder/BrotligSwizzler.h"

#include <memory>

namespace BrotliG
{
    class BrotligSwizzler;

    struct BrotligEncoderParams
    {
        BrotliEncoderMode mode;
        int quality;
        int lgwin;

        size_t page_size;
        size_t num_bitstreams;
        size_t cmd_group_size;
        size_t swizzle_size;

        BrotligEncoderParams();

        BrotligEncoderParams(
            int quality,
            int lgwin,
            size_t p_size
        );

        BrotligEncoderParams& operator=(const BrotligEncoderParams& other);
    };

    struct BrotligEncoderState
    {
        BrotliEncoderParams params;

        MemoryManager memory_manager_;

        uint64_t input_pos_;
        Command* commands_;
        size_t num_commands_;
        size_t num_literals_;
        size_t last_insert_len_;
        int dist_cache_[BROTLI_NUM_DISTANCE_SHORT_CODES];

        Hasher hasher_;

        BROTLI_BOOL is_initialized_;

        BrotligEncoderState(brotli_alloc_func alloc_func, brotli_free_func free_func, void* opaque);

        ~BrotligEncoderState();

        void BrotligInitEncoderDictionary(BrotliEncoderDictionary* dict);

        BROTLI_BOOL SetParameter(BrotliEncoderParameter p, uint32_t value);

        void BrotligChooseDistanceParams();

        bool EnsureInitialized();
    };

    class PageEncoder
    {
    public:
        PageEncoder() = default;

        static inline size_t MaxCompressedSize(size_t inputSize)
        {
            return 2 * BrotliEncoderMaxCompressedSize(inputSize);
        }

        bool Setup(BrotligEncoderParams& params, BrotligDataconditionParams* preconditioner);
        bool Run(const uint8_t* input, size_t inputSize, size_t inputOffset, uint8_t* output, size_t* outputSize, size_t outputOffset, bool isLast);

    private:
        bool DeltaEncode(size_t page_start, size_t page_end, uint8_t* input);
        void DeltaEncodeByte(size_t inSize, uint8_t* inData);

        inline void StoreCommand(Command& cmd);
        inline void StoreLiteral(uint8_t literal);
        inline void StoreDistance(uint16_t dist, uint32_t distextra);

        BrotligEncoderParams m_params;
        BrotligDataconditionParams* m_dcparams = nullptr;

        uint32_t m_histCommands[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE];
        uint32_t m_histLiterals[BROTLI_NUM_LITERAL_SYMBOLS];
        uint32_t m_histDistances[BROTLIG_NUM_DISTANCE_SYMBOLS];

        uint16_t m_cmdCodes[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE];
        uint8_t m_cmdCodelens[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE];

        uint16_t m_litCodes[BROTLI_NUM_LITERAL_SYMBOLS];
        uint8_t m_litCodelens[BROTLI_NUM_LITERAL_SYMBOLS];

        uint16_t m_distCodes[BROTLIG_NUM_DISTANCE_SYMBOLS];
        uint8_t m_distCodelens[BROTLIG_NUM_DISTANCE_SYMBOLS];

        std::unique_ptr<BrotligSwizzler> m_pWriter;
    };
}
