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

#include <cassert>
#include <cstdint>

namespace BrotliG
{
    class BrotligBitWriterLSB
    {
    public:
        BrotligBitWriterLSB()
        {
            m_storage = nullptr;
            m_curbitpos = 0;
        }

        ~BrotligBitWriterLSB()
        {
            m_storage = nullptr;
            m_curbitpos = 0;
        }

        void SetStorage(uint8_t* storage)
        {
            m_storage = storage;
        }

        void SetPosition(size_t pos)
        {
            m_curbitpos = pos;
        }

        size_t& GetPosition()
        {
            return m_curbitpos;
        }

        uint8_t* GetStorage()
        {
            return m_storage;
        }

        void Write(size_t n_bits, uint64_t bits);

        void AlignToNextDWord();

        void AlignToNextByte();

        static void StoreVanLenUint8(
            size_t n,
            BrotligBitWriterLSB* bw
        );

    private:
        uint8_t* m_storage;
        size_t m_curbitpos;
    };

    class BrotligBitWriterMSB
    {
    public:
        BrotligBitWriterMSB()
        {
            m_storage = nullptr;
            m_curbitpos = 0;
        }

        ~BrotligBitWriterMSB()
        {
            m_storage = nullptr;
            m_curbitpos = 0;
        }

        void SetStorage(uint8_t* storage)
        {
            m_storage = storage;
        }

        void SetPosition(size_t pos)
        {
            m_curbitpos = pos;
        }

        size_t& GetPosition()
        {
            return m_curbitpos;
        }

        uint8_t* GetStorage()
        {
            return m_storage;
        }

        void Write(size_t n_bits, uint64_t bits);
    private:
        uint8_t* m_storage;
        size_t m_curbitpos;
    };
}
