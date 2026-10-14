/* Host build of the kernels: prints the checksum the firmware must report.
 *   cc -O2 host_check.c kernels.c && ./a.out */
#include <stdio.h>
#include "kernels.h"

int main(void) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < 100; ++i) sum += bench_iteration(i);
    printf("%08x\n", (unsigned)sum);
    return 0;
}
