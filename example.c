#include <stdio.h>
#include "lds02.h"

int main(int argc, char* argv[]){
    struct LDS02_Package package;
    int lds02_serial_port = lds02_init(argv[1]);
    lds02_read(&package, lds02_serial_port);
    printf("Speed: %udeg/s\n", package.speed);
    printf("Timestamp: %us\n", package.timestamp);
    for(size_t i = 0; i < 12; i++){
        printf("Point%u {distance: %umm, confidence for distance: %u/255, angle %f\n", i+1, package.points[i].distance, package.points[i].confidence, package.points[i].angle);
    }
    printf("\n");
    lds02_close(lds02_serial_port);
}
