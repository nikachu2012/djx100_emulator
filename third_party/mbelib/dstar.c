/* D-STAR 音声デコーダ: mbelib の 3600x2400 処理を 1 つの翻訳単位にまとめ、ambe2-c の mbelib 互換 API
 * （同名のシンボル）とぶつからないよう名前を変えて取り込む */
#define mbe_printVersion dstar_mbe_printVersion
#define mbe_initMbeParms dstar_mbe_initMbeParms
#define mbe_synthesizeSilencef dstar_mbe_synthesizeSilencef
#define mbe_synthesizeSilence dstar_mbe_synthesizeSilence
#define mbe_floattoshort dstar_mbe_floattoshort
#include "mbelib.c"
#include "ecc.c"
#include "ambe3600x2400.c"

#include <string.h>
#include "dstar.h"

/* 空中のビット j -> ambe_fr[W[j]][X[j]]（DSD の dstar_const.h と同じ表、ISC） */
static const uint8_t W[72] = {0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2,
                              0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 2, 1, 1, 0, 0, 3, 2,
                              0, 0, 3, 2, 1, 1, 0, 0, 1, 1, 0, 0, 3, 2, 1, 1, 3, 3, 2, 1, 0, 0, 3, 3};
static const uint8_t X[72] = {10, 22, 11, 9, 10, 22, 11, 23, 8, 20, 9, 21, 10, 8, 9, 21, 8, 6, 7, 19, 8, 20, 9, 7,
                              6, 18, 7, 5, 6, 18, 7, 19, 4, 16, 5, 17, 6, 4, 5, 17, 4, 2, 3, 15, 4, 16, 5, 3,
                              2, 14, 3, 1, 2, 14, 3, 15, 0, 12, 1, 13, 2, 0, 1, 13, 0, 12, 10, 11, 0, 12, 1, 13};

struct dstar_decoder {
    mbe_parms cur, prev, prev_enh;
};

dstar_decoder *dstar_new(void)
{
    dstar_decoder *d = (dstar_decoder *)malloc(sizeof *d);
    if (d) dstar_reset(d);
    return d;
}

void dstar_free(dstar_decoder *d)
{
    free(d);
}

void dstar_reset(dstar_decoder *d)
{
    mbe_initMbeParms(&d->cur, &d->prev, &d->prev_enh);
}

int dstar_decode(dstar_decoder *d, const uint8_t air[9], int16_t pcm[160])
{
    char fr[4][24], ambe_d[49], err_str[64];
    int j, errs = 0, errs2 = 0;
    memset(fr, 0, sizeof fr);
    for (j = 0; j < 72; j++) fr[W[j]][X[j]] = (char)((air[j >> 3] >> (j & 7)) & 1);
    mbe_processAmbe3600x2400Frame(pcm, &errs, &errs2, err_str, fr, ambe_d, &d->cur, &d->prev, &d->prev_enh, 3);
    return errs2;
}
