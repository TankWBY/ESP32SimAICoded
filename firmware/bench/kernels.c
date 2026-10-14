/* CPU-bound workload for measuring emulator speed. Pure C with no hardware
 * access, so the same file builds for the host to produce the expected
 * checksum. The mix follows CoreMark's spirit: pointer chasing, a matrix
 * multiply, a byte-wise state machine, CRC, sorting and calls. */
#include "kernels.h"

#define N_LIST 64
#define N_MAT 12
#define N_SORT 128
#define N_TEXT 256

struct node { struct node *next; int32_t val; };

static struct node nodes[N_LIST];
static int32_t ma[N_MAT][N_MAT], mb[N_MAT][N_MAT], mc[N_MAT][N_MAT];
static int32_t arr[N_SORT];
static uint8_t text[N_TEXT];
static uint32_t rng;

static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng >> 8; }

static uint32_t crc32(const uint8_t *p, int n, uint32_t crc) {
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
    }
    return ~crc;
}

static uint32_t list_kernel(uint32_t seed) {
    for (int i = 0; i < N_LIST; ++i) { nodes[i].val = (int32_t)(rnd() & 0xFFFF); nodes[i].next = 0; }
    /* link in a scrambled order */
    struct node *head = &nodes[0];
    for (int i = 1; i < N_LIST; ++i) {
        int j = (int)((i * 37u + seed) % N_LIST);
        if (j == 0) continue;
        struct node *n = &nodes[j];
        if (n->next || n == head) continue;
        n->next = head;
        head = n;
    }
    uint32_t sum = 0;
    for (int pass = 0; pass < 4; ++pass)
        for (struct node *n = head; n; n = n->next) sum = sum * 31 + (uint32_t)n->val + (uint32_t)pass;
    return sum;
}

static uint32_t matrix_kernel(void) {
    for (int i = 0; i < N_MAT; ++i)
        for (int j = 0; j < N_MAT; ++j) { ma[i][j] = (int32_t)(rnd() & 0xFF) - 128; mb[i][j] = (int32_t)(rnd() & 0xFF) - 128; }
    for (int i = 0; i < N_MAT; ++i)
        for (int j = 0; j < N_MAT; ++j) {
            int32_t s = 0;
            for (int k = 0; k < N_MAT; ++k) s += ma[i][k] * mb[k][j];
            mc[i][j] = s;
        }
    uint32_t h = 0;
    for (int i = 0; i < N_MAT; ++i)
        for (int j = 0; j < N_MAT; ++j) h = (h ^ (uint32_t)mc[i][j]) * 16777619u + (uint32_t)(mc[i][j] / 7);
    return h;
}

static uint32_t sort_kernel(void) {
    for (int i = 0; i < N_SORT; ++i) arr[i] = (int32_t)rnd();
    for (int i = 1; i < N_SORT; ++i) {
        int32_t v = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > v) { arr[j + 1] = arr[j]; --j; }
        arr[j + 1] = v;
    }
    return (uint32_t)arr[0] ^ (uint32_t)arr[N_SORT / 2] ^ (uint32_t)arr[N_SORT - 1];
}

/* Classify a stream of characters into numbers, words and punctuation. */
static uint32_t state_kernel(void) {
    static const char alphabet[] = "0123456789 abcdefxyz.,+-eE";
    for (int i = 0; i < N_TEXT; ++i) text[i] = (uint8_t)alphabet[rnd() % (sizeof alphabet - 1)];
    enum { START, INT, FLOAT, EXP, WORD, PUNCT } st = START;
    uint32_t counts[6] = {0};
    for (int i = 0; i < N_TEXT; ++i) {
        uint8_t c = text[i];
        int digit = c >= '0' && c <= '9', alpha = c >= 'a' && c <= 'z';
        switch (st) {
        case START: st = digit ? INT : alpha ? WORD : c == ' ' ? START : PUNCT; break;
        case INT: st = digit ? INT : c == '.' ? FLOAT : c == 'e' || c == 'E' ? EXP : START; break;
        case FLOAT: st = digit ? FLOAT : c == 'e' || c == 'E' ? EXP : START; break;
        case EXP: st = digit || c == '+' || c == '-' ? EXP : START; break;
        case WORD: st = alpha || digit ? WORD : START; break;
        case PUNCT: st = c == ' ' ? START : PUNCT; break;
        }
        counts[st]++;
    }
    return counts[0] + (counts[1] << 4) + (counts[2] << 8) + (counts[3] << 12) + (counts[4] << 16) + (counts[5] << 20);
}

static uint32_t fib(uint32_t n) { return n < 2 ? n : fib(n - 1) + fib(n - 2); }

uint32_t bench_iteration(uint32_t seed) {
    rng = seed;
    uint32_t h = list_kernel(seed);
    h = h * 3 + matrix_kernel();
    h = h * 3 + sort_kernel();
    h = h * 3 + state_kernel();
    h = h * 3 + crc32(text, N_TEXT, h);
    h = h * 3 + fib(12);
    h = h * 3 + (h / (seed | 1)) + (h % 1000003u);
    return h;
}
