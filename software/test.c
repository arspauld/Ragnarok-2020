#include <stdio.h>

#define true 1
#define false 0

int main(){
    int i = 0;
    int done = false;
    while(!done){
        i++;
        done = i > 100 ? true: false;
    }
    printf("Result:\n%d\n", i);
}