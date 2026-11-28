/* D-STAR の音声フレーム（AMBE 2400 + FEC 1200、72 ビット）のデコーダ。mbelib の 3600x2400 を使う（dstar.c） */
#ifndef DSTAR_H
#define DSTAR_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dstar_decoder dstar_decoder;

dstar_decoder *dstar_new(void);
void dstar_free(dstar_decoder *d);
void dstar_reset(dstar_decoder *d);
/* air: 空中の 9 バイト（各バイト LSB から送出）。戻り値: FEC で訂正したビット数 */
int dstar_decode(dstar_decoder *d, const uint8_t air[9], int16_t pcm[160]);

#ifdef __cplusplus
}
#endif
#endif
