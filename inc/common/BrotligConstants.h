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

extern "C" {
#include "brotli/c/common/constants.h"
}

#include "common/BrotligFlags.h"

namespace BrotliG
{
 /**Brolti - G symbol limits**/
#define BROTLIG_NUM_COMMAND_SYMBOLS_WITH_SENTINEL BROTLI_NUM_COMMAND_SYMBOLS + 1
#define BROTLIG_NUM_END_LITERAL_SYMBOLS 23
#define BROTLIG_NUM_COMMAND_SYMBOLS_WITH_END_LITERALS BROTLIG_NUM_COMMAND_SYMBOLS_WITH_SENTINEL + BROTLIG_NUM_END_LITERAL_SYMBOLS
#if USE_INSERT_ONLY_COMMANDS
#define BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE BROTLIG_NUM_COMMAND_SYMBOLS_WITH_END_LITERALS
#else
#define BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE BROTLIG_NUM_COMMAND_SYMBOLS_WITH_SENTINEL
#endif // USE_INSERT_ONLY_COMMANDS

//#define BROTLIG_NUM_DISTANCE_SYMBOLS BROTLI_NUM_DISTANCE_SYMBOLS
#define BROTLIG_NUM_DISTANCE_SYMBOLS 544
//#define BROTLIG_NUM_DISTANCE_SYMBOLS 1128

/**Brotli-G stream settings**/
// Brotli-G Stream Header
#define BROTLIG_STREAM_ID_BITS 8
#define BROTLIG_STREAM_MAGIC_BITS 8
#define BROTLIG_STREAM_NUM_PAGES_BITS 16
#define BROTLIG_STREAM_PAGE_SIZE_IDX_BITS 2
#define BROTLIG_STREAM_LASTPAGE_SIZE_BITS 18
#define BROTLIG_STREAM_PRECONDITION_BITS 1
#define BROTLIG_STREAM_RESERVED_BITS 11
#define BROTLIG_STREAM_HEADER_SIZE_BITS ( BROTLIG_STREAM_ID_BITS + \
                                          BROTLIG_STREAM_MAGIC_BITS + \
                                          BROTLIG_STREAM_NUM_PAGES_BITS + \
                                          BROTLIG_STREAM_PAGE_SIZE_IDX_BITS + \
                                          BROTLIG_STREAM_LASTPAGE_SIZE_BITS + \
                                          BROTLIG_STREAM_PRECONDITION_BITS + \
                                          BROTLIG_STREAM_RESERVED_BITS )

#define BROTLIG_STREAM_HEADER_SIZE_BYTES (BROTLIG_STREAM_HEADER_SIZE_BITS / 8)

// Brotli-G Page Header
#define BROTLIG_PAGE_HEADER_NPOSTFIX_BITS 2
#define BROTLIG_PAGE_HEADER_NDIST_BITS 4
#define BROTLIG_PAGE_HEADER_ISDELTAENCODED_BITS 1
#define BROTLIG_PAGE_HEADER_RESERVED_BITS 1
#define BROTLIG_PAGE_HEADER_SIZE_BITS ( BROTLIG_PAGE_HEADER_NPOSTFIX_BITS + \
                                        BROTLIG_PAGE_HEADER_NDIST_BITS + \
                                        BROTLIG_PAGE_HEADER_ISDELTAENCODED_BITS + \
                                        BROTLIG_PAGE_HEADER_RESERVED_BITS )

#define BROTLIG_PAGE_HEADER_SIZE_BYTES (BROTLIG_PAGE_HEADER_SIZE_BITS / 8)

// Brotli-G Parameters
#define BROTLIG_STREAM_ID 5
#define BROTLIG_MAX_NUM_BITSTREAMS 64
#define BROTLIG_DEFAULT_NUM_BITSTREAMS_LOG 5
#define BROLTIG_DEFAULT_NUM_BITSTREAMS 1 << BROTLIG_DEFAULT_NUM_BITSTREAMS_LOG
#define BROTLIG_COMMAND_GROUP_SIZE 1
#define BROTLIG_SWIZZLE_SIZE 4
#define BROTLIG_MIN_PAGE_SIZE 32 * 1024
#define BROTLIG_DEFAULT_PAGE_SIZE 64 * 1024
#define BROTLIG_MAX_PAGE_SIZE 128 * 1024
#define BROTLIG_DATA_ALIGNMENT 4
#define BROTLIG_MAX_NUM_PAGES (1 << BROTLIG_STREAM_NUM_PAGES_BITS) - 1

// Brolti-G Multi-threaded settings
#define BROTLIG_MAX_WORKERS 128

// Brolti-G LZ77 settings
#define BROTLIG_LZ77_PAD_INPUT 1
#define BROTLIG_LZ77_PADDING_SIZE 8

// Brolti-G Huffman settings
#define BROTLIG_HUFFMAN_MAX_DEPTH 15

#define BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH 10
#define BROTLIG_HUFFMAN_MAX_CODE_LENGTH_CODE_LENGTH (BROTLIG_HUFFMAN_NUM_CODE_LENGTH_CODE_LENGTH - 1)
#define BROTLIG_HUFFMAN_CODE_LENGTH_TABLE_SIZE 1 << BROTLIG_HUFFMAN_MAX_CODE_LENGTH_CODE_LENGTH

#define BROTLIG_HUFFMAN_NUM_CODE_LENGTH 16
#define BROTLIG_HUFFMAN_MAX_CODE_LENGTH (BROTLIG_HUFFMAN_NUM_CODE_LENGTH - 1)
#define BROTLIG_HUFFMAN_TABLE_SIZE 1 << BROTLIG_HUFFMAN_MAX_CODE_LENGTH

#define BROTLIG_NUM_HUFFMAN_TREES 3
#define BROTLIG_ICP_TREE_INDEX 0
#define BROTLIG_DIST_TREE_INDEX 1
#define BROTLIG_LIT_TREE_INDEX 2

// Brolti-G Encoder Settings
#define BROTLIG_INPUT_BIT_MASK 262143
#define BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS 1 << BROTLI_DISTANCE_CONTEXT_BITS
#define BROTLIG_MAX_NUM_DIST_HISTOGRAMS 1

// Brolti-G CPU Decoder Settings
#define BROTLIG_DWORD_SIZE_BITS 32
#define BROTLIG_DWORD_SIZE_BYTES BROTLIG_DWORD_SIZE_BITS / 8

// Brolti-G GPU Decoder Settings
#define BROTLIG_GPUD_MIN_D3D_FEATURE_LEVEL 0xc000
#define BROTLIG_GPUD_MIN_D3D_SHADER_MODEL 0x60
#define BROTLIG_GPUD_MAX_D3D_SHADER_MODEL 0x65

#define BROTLIG_GPUD_DEFAULT_NUM_GROUPS 2560
#define BROTLIG_GPUD_DEFAULT_MAX_TEMP_BUFFER_SIZE 1 * 1024 * 1024 * 1024
#define BROTLIG_GPUD_DEFAULT_MAX_STREAMS_PER_LAUNCH 4096
#define BROTLIG_GPUD_DEFAULT_MAX_QUERIES 32

/**Brolti-G preconditioner settings**/
// Brotli-G precondition header
#define BROTLIG_PRECON_SWIZZLING_BITS 1
#define BROTLIG_PRECON_PITCH_D3D12_ALIGNED_FLAG_BITS 1
#define BROTLIG_PRECON_TEX_WIDTH_BLOCK_BITS 15
#define BROTLIG_PRECON_TEX_HEIGHT_BLOCK_BITS 15
#define BROTLIG_PRECON_DATA_FORMAT 8
#define BROTLIG_PRECON_TEX_NUMMIPLEVELS_BITS 5
#define BROTLIG_PRECON_TEX_PITCH_BYTES_BITS 19
#define BROTLIG_PRECON_HEADER_SIZE_BITS ( BROTLIG_PRECON_SWIZZLING_BITS + \
                                          BROTLIG_PRECON_PITCH_D3D12_ALIGNED_FLAG_BITS + \
                                          BROTLIG_PRECON_TEX_WIDTH_BLOCK_BITS + \
                                          BROTLIG_PRECON_TEX_HEIGHT_BLOCK_BITS + \
                                          BROTLIG_PRECON_DATA_FORMAT + \
                                          BROTLIG_PRECON_TEX_NUMMIPLEVELS_BITS + \
                                          BROTLIG_PRECON_TEX_PITCH_BYTES_BITS )

#define BROTLIG_PRECON_HEADER_SIZE_BYTES (BROTLIG_PRECON_HEADER_SIZE_BITS / 8)

// Brotli-G precondition parameters
#define BROTLIG_PRECON_MAX_TEX_WIDTH_BLOCK (1 << BROTLIG_PRECON_TEX_WIDTH_BLOCK_BITS)
#define BROTLIG_PRECON_MAX_TEX_HEIGHT_BLOCK (1 << BROTLIG_PRECON_TEX_HEIGHT_BLOCK_BITS)
#define BROTLIG_PRECON_MIN_TEX_PITCH_BYTES 0
#define BROTLIG_PRECON_MAX_TEX_PITCH_BYTES (1 << BROTLIG_PRECON_TEX_PITCH_BYTES_BITS)
#define BROTLIG_PRECON_MIN_NUM_MIP_LEVELS 0
#define BROTLIG_PRECON_MAX_NUM_MIP_LEVELS (1 << BROTLIG_PRECON_TEX_NUMMIPLEVELS_BITS)

#define BROTLIG_PRECON_MIN_TEX_WIDTH_PIXEL 0
#define BROTLIG_PRECON_MAX_TEX_WIDTH_PIXEL (4 * BROTLIG_PRECON_MAX_TEX_WIDTH_BLOCK)
#define BROTLIG_PRECON_MIN_TEX_HEIGHT_PIXEL 0
#define BROTLIG_PRECON_MAX_TEX_HEIGHT_PIXEL (4 * BROTLIG_PRECON_MAX_TEX_HEIGHT_BLOCK)

// Brotli-G precondition settings
#define BROTLIG_D3D12_TEXTURE_DATA_PITCH_ALIGNMENT_BTYES 256

#define BROTLIG_PRECON_SWIZZLE_REGION_SIZE_IDX 1
#define BROTLIG_PRECON_SWIZZLE_REGION_SIZE (1 << BROTLIG_PRECON_SWIZZLE_REGION_SIZE_IDX)

// Brotli-G page precondition settings
#define BROTLIG_PRECON_DELTA_ENCODING_BASES_SIZE_BITS 32
#define BROTLIG_PRECON_DELTA_ENCODING_BASES_SIZE_BYTES (BROTLIG_PRECON_DELTA_ENCODING_BASES_SIZE_BITS / 8)

// Block compressed format specifications
#define BROTLIG_DEFALUT_NUM_SUB_BLOCKS 1
#define BROTLIG_DEFAULT_BLOCK_SIZE_BYTES 1
#define BROTLIG_DEFAULT_BLOCK_SIZE_PIXELS 1
#define BROTLIG_DEFAULT_COLOR_SIZE_BITS 8

// BC1 format
#define BROTLIG_BC1_NUM_SUB_BLOCKS 3
// subblocks
#define BROTLIG_BC1_COLOR_TOP_REF_SIZE_BYTES 2
#define BROTLIG_BC1_COLOR_BOT_REF_SIZE_BYTES 2
#define BROTLIG_BC1_COLOR_INDEX_SIZE_BYTES 4
// sizes
#define BROTLIG_BC1_BLOCK_SIZE_BYTES 8
#define BROTLIG_BC1_BLOCK_SIZE_PIXELS 4
#define BROTLIG_BC1_COLOR_SIZE_BITS 16

// BC2 format
#define BROTLIG_BC2_NUM_SUB_BLOCKS 4
// subblocks
#define BROTLIG_BC2_ALPHA_VECTOR_SIZE_BYTES 8
#define BROTLIG_BC2_COLOR_TOP_REF_SIZE_BYTES 2
#define BROTLIG_BC2_COLOR_BOT_REF_SIZE_BYTES 2
#define BROTLIG_BC2_COLOR_INDEX_SIZE_BYTES 4
// sizes
#define BROTLIG_BC2_BLOCK_SIZE_BYTES 16
#define BROTLIG_BC2_BLOCK_SIZE_PIXELS 4
#define BROTLIG_BC2_COLOR_SIZE_BITS 16

// BC3 format
#define BROTLIG_BC3_NUM_SUB_BLOCKS 6
// subblocks
#define BROTLIG_BC3_ALPHA_TOP_REF_SIZE_BYTES 1
#define BROTLIG_BC3_ALPHA_BOT_REF_SIZE_BYTES 1
#define BROTLIG_BC3_ALPHA_INDEX_SIZE_BYTES 6
#define BROTLIG_BC3_COLOR_TOP_REF_SIZE_BYTES 2
#define BROTLIG_BC3_COLOR_BOT_REF_SIZE_BYTES 2
#define BROTLIG_BC3_COLOR_INDEX_SIZE_BYTES 4
// sizes
#define BROTLIG_BC3_BLOCK_SIZE_BYTES 16
#define BROTLIG_BC3_BLOCK_SIZE_PIXELS 4
#define BROTLIG_BC3_COLOR_SIZE_BITS 8

// BC4 format
#define BROTLIG_BC4_NUM_SUB_BLOCKS 3
// subblocks
#define BROTLIG_BC4_COLOR_TOP_REF_SIZE_BYTES 1
#define BROTLIG_BC4_COLOR_BOT_REF_SIZE_BYTES 1
#define BROTLIG_BC4_COLOR_INDEX_SIZE_BYTES 6
//sizes
#define BROTLIG_BC4_BLOCK_SIZE_BYTES 8
#define BROTLIG_BC4_BLOCK_SIZE_PIXELS 4
#define BROTLIG_BC4_COLOR_SIZE_BITS 8

// BC5 format
#define BROTLIG_BC5_NUM_SUB_BLOCKS 6
// subblocks
#define BROTLIG_BC5_RED_TOP_REF_SIZE_BYTES 1
#define BROTLIG_BC5_RED_BOT_REF_SIZE_BYTES 1
#define BROTLIG_BC5_RED_INDEX_SIZE_BYTES 6
#define BROTLIG_BC5_GREEN_TOP_REF_SIZE_BYTES 1
#define BROTLIG_BC5_GREEN_BOT_REF_SIZE_BYTES 1
#define BROTLIG_BC5_GREEN_INDEX_SIZE_BYTES 6
// sizes
#define BROTLIG_BC5_BLOCK_SIZE_BYTES 16
#define BROTLIG_BC5_BLOCK_SIZE_PIXELS 4
#define BROTLIG_BC5_COLOR_SIZE_BITS 8

// BCn max
#define BROTLIG_MAX_NUM_SUB_BLOCKS BROTLIG_BC5_NUM_SUB_BLOCKS
#define BROTLIG_MAX_BLOCK_SIZE_BYTES BROTLIG_BC5_BLOCK_SIZE_BYTES
}


