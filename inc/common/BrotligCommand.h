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
}

namespace BrotliG
{
    class BrotligBitWriterLSB;

    struct BrotligCommand
    {
        uint32_t insert_pos;
        uint32_t insert_len;
        uint32_t copy_len;
        uint32_t dist_extra;
        uint16_t cmd_prefix;
        uint16_t dist_prefix;
        uint32_t dist;
        int32_t dist_code;

        BrotligCommand();

        Command* ToBroliCommand();

        void Copy(Command* in);

        uint32_t CopyLen();

        uint32_t DistanceContext();

        uint16_t Distance();

        uint32_t CopyLenCode();

        uint16_t InsertLengthCode();

        uint16_t GetCopyLengthCode(size_t copylen);

        void GetExtra(uint32_t& n_bits, uint64_t& bits);

        void StoreExtra(BrotligBitWriterLSB* bw);

        uint16_t CombineLengthCodes(uint16_t inscode, uint16_t copycode, bool use_last_distance);
    };
}
