#include "../inc/main.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    COM com;
    char read_buf[1024];
    char write_data[] = "Written data\n\0";

    com.test_connection();
    com.initialize();
    com.set_end('\n');

    while (1)
    {
        com.write_data(write_data, sizeof(write_data));
        cout << com.read_data(read_buf) << endl; 
        cout << read_buf << endl;
    }


    return EXIT_SUCCESS;
}







