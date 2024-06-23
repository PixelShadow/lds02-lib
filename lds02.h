/*
    This is based on the tutorial presented in this blog article:

    https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
*/

#include <stdio.h>
#include <string.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


struct Point{
    unsigned int distance;
    float angle;
    unsigned char confidence;
};
struct LDS02_Package{
    unsigned int timestamp;
    unsigned int speed;
    struct Point points[12];
};

void lds02_close(int serial_port){
    close(serial_port);
}

int lds02_init(const char* serial_port_path){ //returns the serial port as int. returns -1 if error
    int serial_port = open(serial_port_path, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB; //no parity
    tty.c_cflag &= ~CSTOPB; //only one stop bit
    tty.c_cflag |= CS8;     //8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON; //disable cannonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_cc[VTIME] = 0;    // Since the lidar is always sending stuff, we don't need any timeout
    tty.c_cc[VMIN] = 94;     // reading byte by byte
    cfsetspeed(&tty, B115200); // setting baud rate to 115200bits/s

    //no data will be sent, but just in case
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    return serial_port;
}

int lds02_read(struct LDS02_Package *package, int serial_port){//returns 0 if everything is allright, returns 1 if error
    unsigned char read_buf[94]; //a two packet-sized buffer
    int num_bytes;
    float start_angle;
    float end_angle;

    size_t i;
    do{
        i = 0;
        num_bytes = read(serial_port, &read_buf, 94); //reading, num_bytes is negative if error
        if (num_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
            return 1;
        }
        while(!(read_buf[i] == 0x54 && read_buf[i+1] == 0x2c)){i++;}
    }while(read_buf[i+46] == 0x0);


    //convert data into humanly readable stuff and put it into the stuct
    start_angle = (float)((read_buf[i+5] << 8) + read_buf[i+4])/100.0;
    end_angle = (float)((read_buf[i+43] << 8) + read_buf[i+42])/100.0;
    (*package).timestamp = (unsigned int)((read_buf[i+45] << 8) + read_buf[i+44]);
    (*package).speed = (unsigned int)((read_buf[i+3] << 8) + read_buf[i+2]);

    for(size_t k = 0; k<12; k++){
        (*package).points[k].distance = (unsigned int)((read_buf[i+7+(k*3)] << 8) + read_buf[i+6+(k*3)]);
        if((*package).points[k].distance == 0){
            (*package).points[k].confidence = 0;
        }else{
            (*package).points[k].confidence = read_buf[i+8+(k*3)];
        }
        (*package).points[k].angle = start_angle + k*((end_angle - start_angle)/12.0);
    }
    return 0;
}
