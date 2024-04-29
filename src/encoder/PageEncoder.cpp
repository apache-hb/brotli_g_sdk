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

extern "C" {
#include "c/common/context.h"
#include "c/common/transform.h"
#include "c/enc/command.h"
#include "c/enc/hash.h"
#include "c/enc/quality.h"
#include "c/enc/metablock.h"

#include "c/enc/utf8_util.h"
#include "c/enc/backward_references_hq.h"
#include "c/enc/cluster.h"
#include "c/enc/entropy_encode.h"
#include "c/enc/bit_cost.h"

#include "brotli/encode.h"
}

#include "BrotligSwizzler.h"
#include "common/BrotligDataConditioner.h"

#include "common/BrotligBitWriter.h"
#include "common/BrotligDataConditioner.h"

#include "encoder/BrotligHuffman.h"

#include "PageEncoder.h"

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)
#define OVERLAP(x1, x2, y1, y2) (x1 < y2 && y1 < x2)

using namespace BrotliG;

/* Chooses the literal context mode for a metablock */
static ContextType ChooseContextMode(const BrotliEncoderParams* params,
    const uint8_t* data, const size_t pos, const size_t mask,
    const size_t length) {
    /* We only do the computation for the option of something else than
       CONTEXT_UTF8 for the highest qualities */
    if (params->quality >= MIN_QUALITY_FOR_HQ_BLOCK_SPLITTING &&
        !BrotliIsMostlyUTF8(data, pos, mask, length, kMinUTF8Ratio)) {
        return CONTEXT_SIGNED;
    }
    return CONTEXT_UTF8;
}

static BROTLI_BOOL ShouldCompress(
    const uint8_t* data, const size_t mask, const uint64_t last_flush_pos,
    const size_t bytes, const size_t num_literals, const size_t num_commands) {
    /* TODO: find more precise minimal block overhead. */
    if (bytes <= 2) return BROTLI_FALSE;
    if (num_commands < (bytes >> 8) + 2) {
        if ((double)num_literals > 0.99 * (double)bytes) {
            uint32_t literal_histo[256] = { 0 };
            static const uint32_t kSampleRate = 13;
            static const double kMinEntropy = 7.92;
            const double bit_cost_threshold =
                (double)bytes * kMinEntropy / kSampleRate;
            size_t t = (bytes + kSampleRate - 1) / kSampleRate;
            uint32_t pos = (uint32_t)last_flush_pos;
            size_t i;
            for (i = 0; i < t; i++) {
                ++literal_histo[data[pos & mask]];
                pos += kSampleRate;
            }
            if (BitsEntropy(literal_histo, 256) > bit_cost_threshold) {
                return BROTLI_FALSE;
            }
        }
    }
    return BROTLI_TRUE;
}

static void BrotligCreateHqZopfliBackwardReferences(
    BrotligEncoderState* state,
    const uint8_t* input,
    size_t input_size,
    ContextLut literal_context_lut,
    bool isLast
)
{
    state->commands_ = BROTLI_ALLOC(&state->memory_manager_, Command, input_size / 2 + 5);

    InitOrStitchToPreviousBlock(
        &state->memory_manager_,
        &state->hasher_,
        input,
        BROTLIG_INPUT_BIT_MASK,
        &state->params,
        0,
        input_size,
        isLast
    );

    BrotliCreateHqZopfliBackwardReferences(
        &state->memory_manager_,
        input_size,
        0,
        input,
        BROTLIG_INPUT_BIT_MASK,
        literal_context_lut,
        &state->params,
        &state->hasher_,
        state->dist_cache_,
        &state->last_insert_len_,
        state->commands_,
        &state->num_commands_,
        &state->num_literals_
    );

    size_t endPos = 0, cmdIndex = 0;
    while (cmdIndex < state->num_commands_) {
        assert((state->commands_[cmdIndex].copy_len_ & 0x1FFFFFF) < input_size);
        endPos += state->commands_[cmdIndex].insert_len_ + (state->commands_[cmdIndex].copy_len_ & 0x1FFFFFF);
        cmdIndex += 1;
    }

    if (endPos < input_size)
    {
        size_t litsLeft = input_size - endPos;

        Command extra = {};
        extra.insert_len_ = (uint32_t)litsLeft;
        extra.cmd_prefix_ = (uint16_t)(BROTLI_NUM_COMMAND_SYMBOLS) + GetInsertLengthCode(extra.insert_len_);

        assert(extra.cmd_prefix_ < BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE);

        state->commands_[state->num_commands_++] = extra;
        state->num_literals_ += litsLeft;
    }

    Command sentinel = {};
    sentinel.cmd_prefix_ = BROTLIG_NUM_COMMAND_SYMBOLS_WITH_SENTINEL - 1;
    state->commands_[state->num_commands_++] = sentinel;
}

static bool StoreUncompressed(size_t inputSize, const uint8_t* input, size_t* outputSize, uint8_t* output)
{
    memset(output, 0, *outputSize);
    memcpy(output, input, inputSize);
    *outputSize = inputSize;
    return true;
}

static bool BrotligComputeDistanceCost(const Command* cmds,
    size_t num_commands,
    const BrotliDistanceParams* orig_params,
    const BrotliDistanceParams* new_params,
    double* cost) {
    size_t i;
    BROTLI_BOOL equal_params = BROTLI_FALSE;
    uint16_t dist_prefix;
    uint32_t dist_extra;
    HistogramDistance histo;
    HistogramClearDistance(&histo);

    if (orig_params->distance_postfix_bits == new_params->distance_postfix_bits &&
        orig_params->num_direct_distance_codes ==
        new_params->num_direct_distance_codes) {
        equal_params = BROTLI_TRUE;
    }

    for (i = 0; i < num_commands; i++) {
        const Command* cmd = &cmds[i];
        if (CommandCopyLen(cmd) && cmd->cmd_prefix_ >= 128 /*&& cmd->cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS*/) {
            if (equal_params) {
                dist_prefix = cmd->dist_prefix_;
            }
            else {
                uint32_t distance = CommandRestoreDistanceCode(cmd, orig_params);
                if (distance > new_params->max_distance) {
                    return BROTLI_FALSE;
                }
                PrefixEncodeCopyDistance(distance,
                    new_params->num_direct_distance_codes,
                    new_params->distance_postfix_bits,
                    &dist_prefix,
                    &dist_extra);
            }
            HistogramAddDistance(&histo, dist_prefix & 0x3FF);
        }
    }

    *cost = BrotliPopulationCostDistance(&histo); // +extra_bits;
    return BROTLI_TRUE;
}

static void BrotligRecomputeDistancePrefixes(
    Command* cmds,
    size_t num_commands,
    const BrotliDistanceParams* orig_params,
    const BrotliDistanceParams* new_params
)
{
    size_t i;

    if (orig_params->distance_postfix_bits == new_params->distance_postfix_bits &&
        orig_params->num_direct_distance_codes ==
        new_params->num_direct_distance_codes) {
        return;
    }

    for (i = 0; i < num_commands; ++i) {
        Command* cmd = &cmds[i];
        if (CommandCopyLen(cmd) && cmd->cmd_prefix_ >= 128) {
            PrefixEncodeCopyDistance(CommandRestoreDistanceCode(cmd, orig_params),
                new_params->num_direct_distance_codes,
                new_params->distance_postfix_bits,
                &cmd->dist_prefix_,
                &cmd->dist_extra_);
        }
    }
}

BrotligEncoderParams::BrotligEncoderParams()
{
    mode = BROTLI_DEFAULT_MODE;
    quality = BROTLI_DEFAULT_QUALITY;
    lgwin = BROTLI_DEFAULT_WINDOW;

    page_size = BROTLIG_DEFAULT_PAGE_SIZE;
    num_bitstreams = BROLTIG_DEFAULT_NUM_BITSTREAMS;
    cmd_group_size = BROTLIG_COMMAND_GROUP_SIZE;
    swizzle_size = BROTLIG_SWIZZLE_SIZE;
}

BrotligEncoderParams::BrotligEncoderParams(
    int quality,
    int lgwin,
    size_t p_size
)
{
    this->mode = BROTLI_DEFAULT_MODE;
    this->quality = quality;
    this->lgwin = lgwin;
    this->page_size = p_size;
    this->num_bitstreams = BROLTIG_DEFAULT_NUM_BITSTREAMS;
    this->cmd_group_size = BROTLIG_COMMAND_GROUP_SIZE;
    this->swizzle_size = BROTLIG_SWIZZLE_SIZE;
}

BrotligEncoderParams& BrotligEncoderParams::operator=(const BrotligEncoderParams& other)
{
    this->mode = other.mode;
    this->lgwin = other.lgwin;
    this->quality = other.quality;
    this->page_size = other.page_size;
    this->num_bitstreams = other.num_bitstreams;
    this->cmd_group_size = other.cmd_group_size;
    this->swizzle_size = other.swizzle_size;

    return *this;
}

BrotligEncoderState::BrotligEncoderState(brotli_alloc_func alloc_func, brotli_free_func free_func, void* opaque)
{
    BrotliInitMemoryManager(&memory_manager_, alloc_func, free_func, opaque);

    params.mode = BROTLI_DEFAULT_MODE;
    params.large_window = BROTLI_FALSE;
    params.quality = BROTLI_DEFAULT_QUALITY;
    params.lgwin = BROTLI_DEFAULT_WINDOW;
    params.lgblock = 0;
    params.stream_offset = 0;
    params.size_hint = 0;
    params.disable_literal_context_modeling = BROTLI_FALSE;
    BrotliInitSharedEncoderDictionary(&params.dictionary);
    params.dist.distance_postfix_bits = 0;
    params.dist.num_direct_distance_codes = 0;
    params.dist.alphabet_size_max =
        BROTLI_DISTANCE_ALPHABET_SIZE(0, 0, BROTLI_MAX_DISTANCE_BITS);
    params.dist.alphabet_size_limit = params.dist.alphabet_size_max;
    params.dist.max_distance = BROTLI_MAX_DISTANCE;

    input_pos_ = 0;
    num_commands_ = 0;
    num_literals_ = 0;
    last_insert_len_ = 0;
    HasherInit(&hasher_);
    is_initialized_ = BROTLI_FALSE;

    commands_ = 0;

    /* Initialize distance cache. */
    dist_cache_[0] = 4;
    dist_cache_[1] = 11;
    dist_cache_[2] = 15;
    dist_cache_[3] = 16;
}

BrotligEncoderState::~BrotligEncoderState()
{
    MemoryManager* m = &memory_manager_;
    if (BROTLI_IS_OOM(m))
    {
        BrotliWipeOutMemoryManager(m);
        return;
    }

    BROTLI_FREE(m, commands_);
    DestroyHasher(m, &hasher_);
}

static const uint16_t gStaticDictionaryHashWords[32768] = { 0 };
static const uint8_t gStaticDictionaryHashLengths[32768] = { 0 };
static const uint16_t gStaticDictionaryBuckets[32768] = { 0 };

void BrotligEncoderState::BrotligInitEncoderDictionary(BrotliEncoderDictionary* dict)
{
    dict->words = BrotliGetDictionary();
    dict->num_transforms = (uint32_t)BrotliGetTransforms()->num_transforms;

    dict->hash_table_words = gStaticDictionaryHashWords;
    dict->hash_table_lengths = gStaticDictionaryHashLengths;
    dict->buckets = gStaticDictionaryBuckets;
    dict->dict_words = kStaticDictionaryWords;

    dict->cutoffTransformsCount = kCutoffTransformsCount;
    dict->cutoffTransforms = kCutoffTransforms;
}

BROTLI_BOOL BrotligEncoderState::SetParameter(BrotliEncoderParameter p, uint32_t value)
{
    if (is_initialized_) return BROTLI_FALSE;
    /* TODO: Validate/clamp parameters here. */
    switch (p) {
    case BROTLI_PARAM_MODE:
        params.mode = (BrotliEncoderMode)value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_QUALITY:
        params.quality = (int)value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_LGWIN:
        params.lgwin = (int)value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_LGBLOCK:
        params.lgblock = (int)value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_DISABLE_LITERAL_CONTEXT_MODELING:
        if ((value != 0) && (value != 1)) return BROTLI_FALSE;
        params.disable_literal_context_modeling = TO_BROTLI_BOOL(!!value);
        return BROTLI_TRUE;

    case BROTLI_PARAM_SIZE_HINT:
        params.size_hint = value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_LARGE_WINDOW:
        params.large_window = TO_BROTLI_BOOL(!!value);
        return BROTLI_TRUE;

    case BROTLI_PARAM_NPOSTFIX:
        params.dist.distance_postfix_bits = value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_NDIRECT:
        params.dist.num_direct_distance_codes = value;
        return BROTLI_TRUE;

    case BROTLI_PARAM_STREAM_OFFSET:
        if (value > (1u << 30)) return BROTLI_FALSE;
        params.stream_offset = value;
        return BROTLI_TRUE;

    default: return BROTLI_FALSE;
    }
}

void BrotligEncoderState::BrotligChooseDistanceParams()
{
    uint32_t distance_postfix_bits = 0;
    uint32_t num_direct_distance_codes = 0;

    if (params.quality >= MIN_QUALITY_FOR_NONZERO_DISTANCE_PARAMS) {
        uint32_t ndirect_msb;
        if (params.mode == BROTLI_MODE_FONT) {
            distance_postfix_bits = 1;
            num_direct_distance_codes = 12;
        }
        else {
            distance_postfix_bits = params.dist.distance_postfix_bits;
            num_direct_distance_codes = params.dist.num_direct_distance_codes;
        }
        ndirect_msb = (num_direct_distance_codes >> distance_postfix_bits) & 0x0F;
        if (distance_postfix_bits > BROTLI_MAX_NPOSTFIX ||
            num_direct_distance_codes > BROTLI_MAX_NDIRECT ||
            (ndirect_msb << distance_postfix_bits) != num_direct_distance_codes) {
            distance_postfix_bits = 0;
            num_direct_distance_codes = 0;
        }
    }

    BrotliInitDistanceParams(
        &params.dist, distance_postfix_bits, num_direct_distance_codes, params.large_window);
}

bool BrotligEncoderState::EnsureInitialized()
{
    if (BROTLI_IS_OOM(&memory_manager_)) return BROTLI_FALSE;
    if (is_initialized_) return BROTLI_TRUE;

    uint32_t tlgwin = BROTLI_MIN_WINDOW_BITS;
    while (BROTLI_MAX_BACKWARD_LIMIT(tlgwin) < (uint64_t)params.size_hint - 16) {
        tlgwin++;
        if (tlgwin == BROTLI_MAX_WINDOW_BITS) break;
    }
    params.lgwin = tlgwin;

    SanitizeParams(&params);
    params.lgblock = ComputeLgBlock(&params);
    BrotligChooseDistanceParams();

    return BROTLI_TRUE;
}

bool PageEncoder::Setup(BrotligEncoderParams& params, BrotligDataconditionParams* dcparams)
{
    m_params = params;
    m_dcparams = dcparams;
    return true;
}

bool PageEncoder::Run(const uint8_t* input, size_t inputSize, size_t inputOffset, uint8_t* output, size_t* outputSize, size_t outputOffset, bool isLast)
{
    bool Isdeltaencoded = false;

    const uint8_t* p_inPtr = input + inputOffset;
    size_t inSize = inputSize;
    if (m_dcparams->precondition && m_dcparams->delta_encode)
    {
        uint8_t* pageCopy = new uint8_t[inSize];
        memcpy(pageCopy, input + inputOffset, inSize);

        Isdeltaencoded = DeltaEncode(inputOffset, inputOffset + inputSize, pageCopy);

        if (Isdeltaencoded)
            p_inPtr = pageCopy;
        else
            delete[] pageCopy;
    }

    uint8_t* p_outPtr = output + outputOffset;

    // Create encoder instance
    std::unique_ptr state = std::make_unique<BrotligEncoderState>(nullptr, nullptr, nullptr);

    state->SetParameter(BROTLI_PARAM_QUALITY, (uint32_t)m_params.quality);
    state->SetParameter(BROTLI_PARAM_LGWIN, (uint32_t)m_params.lgwin);
    state->SetParameter(BROTLI_PARAM_MODE, (uint32_t)m_params.mode);
    state->SetParameter(BROTLI_PARAM_SIZE_HINT, (uint32_t)inSize);

    if (m_params.lgwin > BROTLI_MAX_WINDOW_BITS) {
        state->SetParameter(BROTLI_PARAM_LARGE_WINDOW, BROTLI_TRUE);
    }

    if (!state->EnsureInitialized())
        return false;

    // Generate LZ77 commands
    ContextType literal_context_mode = ChooseContextMode(
        &state->params,
        p_inPtr,
        0,
        BROTLIG_INPUT_BIT_MASK,
        inSize
    );

    ContextLut literal_context_lut = BROTLI_CONTEXT_LUT(literal_context_mode);

    BrotligCreateHqZopfliBackwardReferences(
        state.get(),
        p_inPtr,
        inSize,
        literal_context_lut,
        isLast
    );

    // Check if input is compressible
    if (!ShouldCompress(
        p_inPtr,
        BROTLIG_INPUT_BIT_MASK,
        0,
        inSize,
        state->num_literals_,
        state->num_commands_
    ))
    {
        // If not compressible, copy input directly to output and return
        if (Isdeltaencoded)
            delete[] p_inPtr;

        return StoreUncompressed(inputSize, input + inputOffset, outputSize, p_outPtr);
    }

    // Optimize distance prefixes
    uint32_t ndirect_msb = 0, ndirect = 0;
    bool skip = false, check_orig = true;
    double dist_cost = 0.0, best_dist_cost = 1e99;
    BrotliEncoderParams orig_params = state->params;
    BrotliEncoderParams new_params = state->params;
    for (uint32_t npostfix = 0; npostfix <= BROTLI_MAX_NPOSTFIX; ++npostfix)
    {
        for (; ndirect_msb < 16; ++ndirect_msb)
        {
            ndirect = ndirect_msb << npostfix;
            BrotliInitDistanceParams(&new_params.dist, npostfix, ndirect, new_params.large_window);
            if (npostfix == orig_params.dist.distance_postfix_bits
                && ndirect == orig_params.dist.num_direct_distance_codes)
                check_orig = false;

            skip = !BrotligComputeDistanceCost(
                state->commands_,
                state->num_commands_,
                &orig_params.dist,
                &new_params.dist,
                &dist_cost
            );

            if (skip || (dist_cost > best_dist_cost))
                break;

            best_dist_cost = dist_cost;
            state->params.dist = new_params.dist;
        }

        if (ndirect_msb > 0) --ndirect_msb;
        ndirect_msb /= 2;
    }

    if (check_orig)
    {
        BrotligComputeDistanceCost(
            state->commands_,
            state->num_commands_,
            &orig_params.dist,
            &orig_params.dist,
            &dist_cost
        );

        if (dist_cost < best_dist_cost) state->params.dist = orig_params.dist;
    }

    BrotligRecomputeDistancePrefixes(
        state->commands_,
        state->num_commands_,
        &orig_params.dist,
        &state->params.dist
    );

    // Compute Histograms
    memset(m_histDistances, 0, sizeof(m_histDistances));
    memset(m_histCommands, 0, sizeof(m_histCommands));
    memset(m_histLiterals, 0, sizeof(m_histLiterals));

    size_t cmdIndex = 0, pos = 0, numbitstreams = m_params.num_bitstreams;
    Command cmd;
    uint32_t insertLen = 0, distContext = 0;

    std::unique_ptr litqueue = std::make_unique<uint8_t[]>(inSize);
    uint8_t* litqfront = litqueue.get();
    uint8_t* litqback = litqueue.get();

    HistogramDistance distCtxHists[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    ClearHistogramsDistance(distCtxHists, BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS);

    while (cmdIndex < state->num_commands_)
    {
        cmd = state->commands_[cmdIndex++];
        ++m_histCommands[cmd.cmd_prefix_];
        if ((cmd.copy_len_ & 0x1FFFFFF) &&
            (cmd.cmd_prefix_ >= 128 && cmd.cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS))
        {
            distContext = CommandDistanceContext(&cmd);
            HistogramAddDistance(&distCtxHists[distContext], cmd.dist_prefix_ & 0x3FF);
        }
        insertLen = cmd.insert_len_;
        while (insertLen--) {
            ++m_histLiterals[p_inPtr[pos]];
            *litqback++ = p_inPtr[pos++];
        }
        pos += (cmd.copy_len_ & 0x1FFFFFF);
    }

    // Cluster distance histograms
    HistogramDistance out[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    size_t num_out = 0;
    uint32_t dist_context_map[BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS];
    BrotliClusterHistogramsDistance(
        &state->memory_manager_,
        distCtxHists,
        BROTLIG_NUM_DIST_CONTEXT_HISTOGRAMS,
        BROTLIG_MAX_NUM_DIST_HISTOGRAMS,
        out,
        &num_out,
        dist_context_map
    );

    memcpy(m_histDistances, out[0].data_, sizeof(out[0].data_));

    // Optimize Histograms
    if (state->params.quality >= MIN_QUALITY_FOR_OPTIMIZE_HISTOGRAMS)
    {
        uint8_t good_for_rle[BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE];
        BrotliOptimizeHuffmanCountsForRle(BROTLI_NUM_LITERAL_SYMBOLS, m_histLiterals, good_for_rle);
        BrotliOptimizeHuffmanCountsForRle(BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE, m_histCommands, good_for_rle);
        BrotliOptimizeHuffmanCountsForRle(BROTLIG_NUM_DISTANCE_SYMBOLS, m_histDistances, good_for_rle);
    }

    uint8_t mostFreqLit = (uint8_t)(std::max_element(m_histLiterals, m_histLiterals + BROTLI_NUM_LITERAL_SYMBOLS) - m_histLiterals);

    // Store compressed
    memset(p_outPtr, 0, *outputSize);
    BrotligBitWriterLSB bw;
    bw.SetStorage(p_outPtr);
    bw.SetPosition(0);

    m_pWriter = std::make_unique<BrotligSwizzler>(m_params.num_bitstreams, m_params.page_size);
    m_pWriter->SetOutWriter(&bw, *outputSize);

    {
        BROTLIG_ERROR result;

        // Build and Store Huffman tables
        result = BuildStoreHuffmanTable(
            m_histCommands,
            BROLTIG_NUM_COMMAND_SYMBOLS_EFFECTIVE,
            *m_pWriter,
            m_cmdCodes,
            m_cmdCodelens
        );

        if (result != BROTLIG_OK)
            return false;

        result = BuildStoreHuffmanTable(
            m_histDistances,
            BROTLIG_NUM_DISTANCE_SYMBOLS,
            *m_pWriter,
            m_distCodes,
            m_distCodelens
        );

        if (result != BROTLIG_OK)
            return false;

        result = BuildStoreHuffmanTable(
            m_histLiterals,
            BROTLI_NUM_LITERAL_SYMBOLS,
            *m_pWriter,
            m_litCodes,
            m_litCodelens
        );

        if (result != BROTLIG_OK)
            return false;
    }

    // Encode and store commands and literals
    cmdIndex = 0;
    size_t bsindex = 0;

    size_t nRounds = (state->num_commands_ + numbitstreams - 1) / numbitstreams;
    Command* cqfront = state->commands_;
    size_t litcount = 0, aclitcount = 0, mult = 0, rlitcount = 0, prev_tail = 0;
    size_t effNumbitstreams = (state->num_commands_ >= numbitstreams) ? numbitstreams : state->num_commands_;

    while (nRounds--)
    {
        bsindex = 0;
        litcount = 0;

        while (bsindex < numbitstreams)
        {
            cmd = *cqfront++;
            litcount += cmd.insert_len_;
            StoreCommand(cmd);

            if (cmd.insert_len_ == 0 && (cmd.copy_len_ & 0x1FFFFFF) == 0)
            {
                break;
            }

            if ((cmd.copy_len_ & 0x1FFFFFF)
                && cmd.cmd_prefix_ >= 128
                && cmd.cmd_prefix_ < BROTLI_NUM_COMMAND_SYMBOLS)
            {
                StoreDistance(cmd.dist_prefix_, cmd.dist_extra_);
            }

            ++bsindex;

            m_pWriter->BSSwitch();
        }

        m_pWriter->BSReset();

        effNumbitstreams = (state->num_commands_ >= numbitstreams) ? numbitstreams : state->num_commands_;
        aclitcount = (litcount > prev_tail) ? litcount - prev_tail : 0;
        mult = (aclitcount + effNumbitstreams - 1) / effNumbitstreams;
        rlitcount = effNumbitstreams * mult;
        prev_tail = rlitcount + prev_tail - litcount;

        while (rlitcount--)
        {
            if (litqfront >= litqback)
            {
                if (nRounds > 0 || isLast)
                    StoreLiteral(mostFreqLit);
                else
                    break;
            }
            else
                StoreLiteral(*litqfront++);

            m_pWriter->BSSwitch();
        }

        m_pWriter->BSReset();
    }

    // Store header
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_NPOSTFIX_BITS, state->params.dist.distance_postfix_bits);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_NDIST_BITS, state->params.dist.num_direct_distance_codes >> state->params.dist.distance_postfix_bits);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_ISDELTAENCODED_BITS, Isdeltaencoded);
    m_pWriter->AppendToHeader(BROTLIG_PAGE_HEADER_RESERVED_BITS, 0);

    m_pWriter->AppendBitstreamSizes();

    // Serialize header
    m_pWriter->SerializeHeader();
    // Serialize bitstreams
    m_pWriter->SerializeBitstreams();

    if (Isdeltaencoded)
        delete[] p_inPtr;

    size_t newsize = (bw.GetPosition() + 8 - 1) / 8;

    if (newsize >= inputSize)
        return StoreUncompressed(inputSize, input + inputOffset, outputSize, p_outPtr);
    else
    {
        *outputSize = newsize;
        return true;
    }
}

bool PageEncoder::DeltaEncode(size_t page_start, size_t page_end, uint8_t* data)
{
    uint32_t sub = 0;
    size_t color_start = 0, color_end = 0, p_sub_start = 0, p_sub_end = 0, p_sub_size = 0;
    bool iseconded = false;
    for (uint32_t i = 0; i < m_dcparams->numColorSubBlocks; ++i)
    {
        sub = m_dcparams->colorSubBlocks[i];
        color_start = (size_t)m_dcparams->subStreamOffsets[sub];
        color_end = (size_t)m_dcparams->subStreamOffsets[sub + 1];

        if (OVERLAP(color_start, color_end, page_start, page_end))
        {
            p_sub_start = (color_start > page_start) ? color_start - page_start : 0;
            p_sub_end = (color_end < page_end) ? color_end - page_start : page_end - page_start;
            p_sub_size = p_sub_end - p_sub_start;

            DeltaEncodeByte(p_sub_size, data + p_sub_start);
            iseconded |= true;
        }
    }

    return iseconded;
}

void PageEncoder::DeltaEncodeByte(size_t inSize, uint8_t* inData)
{
    uint8_t ref = inData[0];

    uint8_t prev = ref, cur = 0;
    for (size_t el = 1; el < inSize; ++el)
    {
        cur = inData[el];
        inData[el] -= prev;
        prev = cur;
    }
}

void PageEncoder::StoreCommand(Command& cmd)
{
    uint16_t nbits = m_cmdCodelens[cmd.cmd_prefix_];
    m_pWriter->Append(nbits, m_cmdCodes[cmd.cmd_prefix_]);
    if (cmd.cmd_prefix_ <= BROTLI_NUM_COMMAND_SYMBOLS)
    {
        uint32_t copylen_code = CommandCopyLenCode(&cmd);
        uint16_t inscode = GetInsertLengthCode(cmd.insert_len_);
        uint16_t copycode = (copylen_code == 0) ? 0 : GetCopyLengthCode(copylen_code);
        uint32_t insnumextra = GetInsertExtra(inscode);
        uint64_t insextraval = cmd.insert_len_ - GetInsertBase(inscode);
        uint64_t copyextraval = (copycode > 1) ? copylen_code - GetCopyBase(copycode) : copylen_code;
        uint64_t bits = (copyextraval << insnumextra) | insextraval;
        m_pWriter->Append(insnumextra + GetCopyExtra(copycode), bits);
    }
    else
    {
        uint16_t inscode = GetInsertLengthCode(cmd.insert_len_);
        uint32_t insnumextra = GetInsertExtra(inscode);
        uint64_t insextraval = cmd.insert_len_ - GetInsertBase(inscode);
        m_pWriter->Append(insnumextra, insextraval);
    }
}

void PageEncoder::StoreLiteral(uint8_t literal)
{
    uint16_t nbits = m_litCodelens[literal];
    m_pWriter->Append(nbits, m_litCodes[literal]);
}

void PageEncoder::StoreDistance(uint16_t dist_prefix, uint32_t distextra)
{
    uint16_t dist_code = dist_prefix & 0x3FF;
    uint16_t nbits = m_distCodelens[dist_code];
    uint32_t distnumextra = dist_prefix >> 10;
    m_pWriter->Append(nbits, m_distCodes[dist_code]);
    m_pWriter->Append(distnumextra, distextra);
}
