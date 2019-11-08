#include <stdio.h>

#define true 1
#define false 0

int main(){
    uint16_t i = 0x55aa;
    printf("Decimal: \n%u\nBinary: \n0b", i);
    for(uint8_t n = sizeof(i)*8; n > 0; n--){
        printf("%u", (i>>(n-1))&1);
    }
}