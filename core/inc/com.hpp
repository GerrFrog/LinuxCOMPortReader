#pragma once

#ifndef COM_HEADER
#define COM_HEADER

#include <cstring>
#include <iostream>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <sys/ioctl.h> // Used for TCGETS2/TCSETS2, which is required for custom baud rates
#include <unistd.h> // write(), read(), close()
#include <stdexcept>
#include <termios.h>
#include <stdlib.h>

/**
 * @brief COM Port class manager
 */
class COM
{
    private:
        /**
         * @brief COM Device string (e.g. /dev/ttyACM1)
         */
        std::string com_dev = "/dev/ttyACM1";

        /**
         * @brief Termios struct in order to configure the serial port.
         */
        struct termios tty; // struct termios2 tty

        /**
         * @brief Old Termios struct (standard)
         */
        struct termios tty_old;

        /**
         * @brief Serial port descriptor;
         */
        int serial_port = 0;

        /**
         * @brief End char of data. Read/Write while symbol is not met
         */
        char end_char = '\0';

        /**
         * @brief Reading buffer.
         */
        char buf = '\0';

        /**
         * @brief Configure the Serial Port
         */
        void configure_sp()
        {
            if(tcgetattr(this->serial_port, &this->tty) != 0)
                throw std::invalid_argument("Error while configuring Serial Port");

            /* Save old tty parameters */
            this->tty_old = this->tty;

            /* Set Baud Rate */
            cfsetospeed (&this->tty, (speed_t)B9600);
            cfsetispeed (&this->tty, (speed_t)B9600);

            /* Setting other Port Stuff */
            this->tty.c_cflag     &=  ~PARENB;        // Make 8n1
            this->tty.c_cflag     &=  ~CSTOPB;
            this->tty.c_cflag     &=  ~CSIZE;
            this->tty.c_cflag     |=  CS8;

            this->tty.c_cflag     &=  ~CRTSCTS;       // no flow control
            this->tty.c_cc[VMIN]   =  1;              // read doesn't block
            this->tty.c_cc[VTIME]  =  5;              // 0.5 seconds read timeout
            this->tty.c_cflag     |=  CREAD | CLOCAL; // turn on READ & ignore ctrl lines

            /* Make raw */
            cfmakeraw(&this->tty);

            /* Flush Port, then applies attributes */
            tcflush(this->serial_port, TCIFLUSH);
            if (tcsetattr (this->serial_port, TCSANOW, &this->tty) != 0) 
            {
                std::cout << "Error " << errno << " from tcsetattr" << std::endl;
                throw std::invalid_argument("");
            }
        }

        /**
         * @brief Connect Serial port
         */
        void connect()
        {
            this->serial_port = open(this->com_dev.c_str(), O_RDWR);
            if (this->serial_port < 0)
                throw std::invalid_argument(this->com_dev + " - No device found!");
        }

    public:
        /**
         * @brief Construct a new COM object
         */
        COM() { }

        /**
         * @brief Construct a new COM object
         * 
         * @param com_dev COM Device
         */
        COM(std::string com_dev) : com_dev(com_dev)
        {
            memset(&this->tty, 0, sizeof(tty));
        } 

        /**
         * @brief Destroy the COM object
         */
        ~COM() 
        { 
            close(this->serial_port);
        }

        /**
         * @brief Set the end char for char array.
         * 
         * @param end End symbol for received data. Read/Write while symbol is not met
         */
        void set_end(char end) { this->end_char = end; }

        /**
         * @brief Test connection with device
         */
        void test_connection()
        {
            this->serial_port = open(this->com_dev.c_str(), O_RDWR);
            if (this->serial_port < 0)
                throw std::invalid_argument(this->com_dev + " - No device found!");
            std::cout << this->com_dev << " - Device successfully tested and ready to work" << std::endl;
            close(this->serial_port);
        }

        /**
         * @brief Initialize device and Serial port
         */
        void initialize()
        {
            this->connect();
            std::cout << "[+] Serial port successfully connected" << std::endl;
            this->configure_sp();
            std::cout << "[+] Serial port successfully configured" << std::endl;
        }

        /**
         * @brief Read data from serial port
         * 
         * @param read_buf Buffer for reading
         * @return int Number of bytes read
         */
        int read_data(char* response)
        {
            int rec_bytes = 0;
            int tot_bytes = 0;
            memset(response, '\0', sizeof(response));
            do {
                rec_bytes = read(this->serial_port, &this->buf, 1);
                sprintf(&response[tot_bytes], "%c", this->buf);
                tot_bytes += rec_bytes;
            } while (this->buf != this->end_char && rec_bytes > 0);

            if (rec_bytes < 0)
            {
                std::cout << "Error reading: " << strerror(errno) << std::endl;
                throw std::invalid_argument("");
            }

            return tot_bytes;
        }

        /**
         * @brief Write data to Serial port
         * 
         * @param data Data char array
         * @param size Size of data
         * @return int Total send bytes
         */
        int write_data(char data[], int size)
        {
            char arr[size];
            for (int i = 0; i < size; i++)
                arr[i] = data[i];
            return write(this->serial_port, arr, size - 1);
        }
};









































#endif