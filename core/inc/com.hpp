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
 * @brief Virtual Com Port
 */
namespace VCP
{
    /**
     * @brief COM Device object
     */
    class COM_Device
    {
        protected:
            /**
             * @brief COM Device string (e.g. /dev/ttyACM1)
             */
            std::string com_dev;

            /**
             * @brief Termios struct in order to configure the serial port.
             */
            struct termios tty; // struct termios2 tty

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
            virtual void configure_sp() = 0;

            /**
             * @brief Connect to Serial Port
             */
            virtual void connect() = 0;

        public:
            /**
             * @brief Construct a new com device object
             */
            COM_Device() = default;

            /**
             * @brief Destroy the com device object
             */
            virtual ~COM_Device()
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
            char* read_data(int size) const
            {
                // int rec_bytes = 0;
                // int tot_bytes = 0;
                // memset(response, '\0', sizeof(response));
                // do {
                //     rec_bytes = read(this->serial_port, &this->buf, 1);
                //     sprintf(&response[tot_bytes], "%c", this->buf);
                //     tot_bytes += rec_bytes;
                // } while (this->buf != this->end_char && rec_bytes > 0);

                // if (rec_bytes < 0)
                // {
                //     std::cout << "Error reading: " << strerror(errno) << std::endl;
                //     throw std::invalid_argument("");
                // }

                // return tot_bytes;

                char read_buffer[size];

                if (read(this->serial_port, &read_buffer, sizeof(read_buffer)) < 0)
                {
                    std::cout << "Error reading: " << strerror(errno) << std::endl;
                    throw std::invalid_argument("");
                }

                char *p = read_buffer;

                return p;
            }

            /**
             * @brief Write data to Serial port
             * 
             * @param data Data char array
             * @param size Size of data
             * @return int Total send bytes
             */
            int write_data(char data[], int size) const
            {
                char arr[size];
                for (int i = 0; i < size; i++)
                    arr[i] = data[i];
                return write(this->serial_port, arr, size - 1);
            }

            /**
             * @brief Test connection
             */
            virtual void test_connection() = 0;
    };

    /**
     * @brief COM Port class manager
     */
    class USB_Device : public VCP::COM_Device
    {
        private:
            /**
             * @brief Configure the Serial Port
             */
            void configure_sp()
            {
                if(tcgetattr(this->serial_port, &this->tty) != 0)
                    throw std::invalid_argument("Error while configuring Serial Port");

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
             * @brief Connect to Serial port
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
             * 
             * @param com_dev COM Device
             */
            USB_Device(std::string com_dev)
            {
                this->com_dev = com_dev;
                memset(&this->tty, 0, sizeof(tty));
            } 

            /**
             * @brief Destroy the COM object
             */
            virtual ~USB_Device() 
            { 
                close(this->serial_port);
            }

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
    };

    /**
     * @brief USART Devices
     * 
     * @note https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
     * @note https://stackoverflow.com/questions/37944461/linux-reading-data-from-uart
     */
    class USART_Device : public VCP::COM_Device
    {
        private:
            /**
             * @brief Configure the Serial Port
             */
            void configure_sp()
            {
                if(tcgetattr(this->serial_port, &this->tty) != 0)
                    throw std::invalid_argument("Error while configuring Serial Port");

                cfsetispeed(&this->tty, B115200); /* Set Read  Speed as 19200                       */
                cfsetospeed(&this->tty, B115200); /* Set Write Speed as 19200                       */

                this->tty.c_cflag &= ~PARENB; /* Disables the Parity   Enable bit(PARENB),So No Parity   */
                this->tty.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
                this->tty.c_cflag &= ~CSIZE;  /* Clears the mask for setting the data size             */
                this->tty.c_cflag |= CS8;     /* Set the data bits = 8                                 */

                this->tty.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
                this->tty.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

                this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);         /* Disable XON/XOFF flow control both i/p and o/p */
                // this->tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Non Cannonical mode                            */

                this->tty.c_oflag &= ~OPOST; /*No Output Processing*/

                // // 
                // this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                //                 | INLCR | IGNCR | ICRNL | IXON);
                // this->tty.c_oflag &= ~OPOST;
                // this->tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
                // this->tty.c_cflag &= ~(CSIZE | PARENB);
                // this->tty.c_cflag |= CS8;

                /* Setting Time outs */
                this->tty.c_cc[VMIN] = 10; /* Read at least 10 characters */
                this->tty.c_cc[VTIME] = 0; /* Wait indefinetly   */

                if ((tcsetattr(this->serial_port, TCSANOW, &this->tty)) != 0) /* Set the attributes to the termios structure*/
                {
                    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
                    throw std::invalid_argument("");
                }                    
            }

            /**
             * @brief Connect to Serial port
             */
            void connect()
            {
                /* !!blocks the read  */
                /* O_RDWR Read/Write access to serial port */
                /* O_NOCTTY - No terminal will control the process   */
                /* O_NDELAY -Non Blocking Mode,Does not care about-  */
                /* -the status of DCD line,Open() returns immediatly */
                this->serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC); 
                if (this->serial_port < 0)
                    throw std::invalid_argument(this->com_dev + " - No device found!");
            }

        public:
            /**
             * @brief Construct a new COM object
             * 
             * @param com_dev COM Device
             */
            USART_Device(std::string com_dev)
            {
                this->com_dev = com_dev;
                memset(&this->tty, 0, sizeof(tty));
            }

            /**
             * @brief Destroy the usart device object
             */
            virtual ~USART_Device()
            {
                close(this->serial_port);
            }

            /**
             * @brief Test connection with device
             */
            void test_connection()
            {
                this->serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
                if (this->serial_port < 0)
                    throw std::invalid_argument(this->com_dev + " - No device found!");
                std::cout << this->com_dev << " - Device successfully tested and ready to work" << std::endl;
                close(this->serial_port);
            }
    };
}












































#endif