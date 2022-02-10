#include "../inc/main.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    VCP::USB_Device usb_device("/dev/ttyACM1");
    VCP::USART_Device usart_device("/dev/ttyACM0");
    char* read_buf;
    int size = 64;
    char write_data[] = "Written data\n\0";

    // usb_device.test_connection();
    // usb_device.initialize();
    // usb_device.set_end('\n');

    usart_device.test_connection();
    usart_device.initialize();
    usart_device.set_end('\n');

    while (1)
    {
        // usart_device.write_data(write_data, sizeof(write_data));
        read_buf = usart_device.read_data(size);
        cout << "Received data: ";
        for (int i = 0; i < size; i++)
            cout << read_buf[i];
        cout << endl;
    }

    return EXIT_SUCCESS;
}
