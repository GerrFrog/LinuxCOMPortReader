# Serial Ports 
- [Link1](https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/)

In typical UNIX style, serial ports are represented by files within the operating system. These files usually pop-up in /dev/, and begin with the name tty*
- /dev/ttyACM0 - ACM stands for the ACM modem on the USB bus. Arduino UNOs (and similar) will appear using this name.
- /dev/ttyPS0 - Xilinx Zynq FPGAs running a Yocto-based Linux build will use this name for the default serial port that Getty connects to.
- /dev/ttyS0 - Standard COM ports will have this name. These are less common these days with newer desktops and laptops not having actual COM ports.
- /dev/ttyUSB0 - Most USB-to-serial cables will show up using a file named like this.
- /dev/pts/0 - A pseudo terminal. These can be generated with socat.

To write to a serial port, you write to the file. To read from a serial port, you read from the file. Of course, this allows you to send/receive data, but how do you set the serial port parameters such as baud rate, parity, e.t.c? This is set by a special tty configuration struct.

Serial port open
```c
int serial_port = open("/dev/ttyUSB0", O_RDWR);

// Check for errors
if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
}
```

After configuration
```c
// Create new termios struct, we call it 'tty' for convention
// No need for "= {0}" at the end as we'll immediately write the existing
// config to this struct
struct termios tty;

// Read in existing settings, and handle any error
// NOTE: This is important! POSIX states that the struct passed to tcsetattr()
// must have been initialized with a call to tcgetattr() overwise behaviour
// is undefined
if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
}
```
```c
struct termios {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
};
```

## Control Modes (c_cflags)
The c_cflag member of the termios struct contains control parameter fields.

### PARENB (Parity)
If this bit is set, generation and detection of the parity bit is enabled. Most serial communications do not use a parity bit, so if you are unsure, clear this bit.
```c
tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
```

### CSTOPB (Num. Stop Bits)
If this bit is set, two stop bits are used. If this is cleared, only one stop bit is used. Most serial communications only use one stop bit.
```c
tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
```

### Number Of Bits Per Byte
The CS<number> fields set how many data bits are transmitted per byte across the serial port. The most common setting here is 8 (CS8). Definitely use this if you are unsure, I have never used a serial port before which didn’t use 8 (but they do exist). You must clear all of the size bits before setting any of them with &= ~CSIZE.
```c
tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
tty.c_cflag |= CS5; // 5 bits per byte
tty.c_cflag |= CS6; // 6 bits per byte
tty.c_cflag |= CS7; // 7 bits per byte
tty.c_cflag |= CS8; // 8 bits per byte (most common)
```

### Flow Control (CRTSCTS)
If the CRTSCTS field is set, hardware RTS/CTS flow control is enabled. The most common setting here is to disable it. Enabling this when it should be disabled can result in your serial port receiving no data, as the sender will buffer it indefinitely, waiting for you to be “ready”.
```c
tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
```

### CREAD and CLOCAL
Setting CLOCAL disables modem-specific signal lines such as carrier detect. It also prevents the controlling process from getting sent a SIGHUP signal when a modem disconnect is detected, which is usually a good thing here. Setting CLOCAL allows us to read data (we definitely want that!).
```c
tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
```

## Local Modes (c_lflag)
### Disabling Canonical Mode
UNIX systems provide two basic modes of input, canonical and non-canonical mode. In canonical mode, input is processed when a new line character is received. The receiving application receives that data line-by-line. This is usually undesirable when dealing with a serial port, and so we normally want to disable canonical mode.

Canonical mode is disabled with:
```c
tty.c_lflag &= ~ICANON;
```

### Echo
If this bit is set, sent characters will be echoed back. Because we disabled canonical mode, I don’t think these bits actually do anything, but it doesn’t harm to disable them just in case!
```c
tty.c_lflag &= ~ECHO; // Disable echo
tty.c_lflag &= ~ECHOE; // Disable erasure
tty.c_lflag &= ~ECHONL; // Disable new-line echo
```

### Disable Signal Chars
When the ISIG bit is set, INTR, QUIT and SUSP characters are interpreted. We don’t want this with a serial port, so clear this bit:
```c
tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
```

## Input Modes (c_iflag)
The c_iflag member of the termios struct contains low-level settings for input processing. The c_iflag member is an int.

### Software Flow Control (IXOFF, IXON, IXANY)
Clearing IXOFF, IXON and IXANY disables software flow control, which we don’t want:
```c
tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
```

### Disabling Special Handling Of Bytes On Receive
Clearing all of the following bits disables any special handling of the bytes as they are received by the serial port, before they are passed to the application. We just want the raw data thanks!
```c
tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
```

## Output Modes (c_oflag)
The c_oflag member of the termios struct contains low-level settings for output processing. When configuring a serial port, we want to disable any special handling of output chars/bytes, so do the following:
```c
tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
```

Both OXTABS and ONOEOT are not defined in Linux. Linux however does have the XTABS field which seems to be related. When compiling for Linux, I just exclude these two fields and the serial port still works fine.

## VMIN and VTIME (c_cc)
VMIN and VTIME are a source of confusion for many programmers when trying to configure a serial port in Linux.

An important point to note is that VTIME means slightly different things depending on what VMIN is. When VMIN is 0, VTIME specifies a time-out from the start of the read() call. But when VMIN is > 0, VTIME specifies the time-out from the start of the first received character.

Let’s explore the different combinations:
- **VMIN = 0, VTIME = 0:** No blocking, return immediately with what is available
- **VMIN > 0, VTIME = 0:** This will make read() always wait for bytes (exactly how many is determined by VMIN), so read() could block indefinitely.
- **VMIN = 0, VTIME > 0:** This is a blocking read of any number of chars with a maximum timeout (given by VTIME). read() will block until either any amount of data is available, or the timeout occurs. This happens to be my favourite mode (and the one I use the most).
- **VMIN > 0, VTIME > 0:** Block until either VMIN characters have been received, or VTIME after first character has elapsed. Note that the timeout for VTIME does not begin until the first character is received.

VMIN and VTIME are both defined as the type cc_t, which I have always seen be an alias for unsigned char (1 byte). This puts an upper limit on the number of VMIN characters to be 255 and the maximum timeout of 25.5 seconds (255 deciseconds).

“Returning as soon as any data is received” does not mean you will only get 1 byte at a time. Depending on the OS latency, serial port speed, hardware buffers and many other things you have no direct control over, you may receive any number of bytes.

For example, if we wanted to wait for up to 1s, returning as soon as any data was received, we could use:

```c
tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
tty.c_cc[VMIN] = 0;
```

## Baud Rate
Rather than use bit fields as with all the other settings, the serial port baud rate is set by calling the functions cfsetispeed() and cfsetospeed(), passing in a pointer to your tty struct and a enum:

```c
// Set in/out baud rate to be 9600
cfsetispeed(&tty, B9600);
cfsetospeed(&tty, B9600);
```

If you want to remain UNIX compliant, the baud rate must be chosen from one of the following:
```c
B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
```

Some implementation of Linux provide a helper function cfsetspeed() which sets both the input and output speeds at the same time:
```c
cfsetspeed(&tty, B9600);
```

### Custom Baud Rates
As you are now fully aware that configuring a Linux serial port is no trivial matter, you’re probably unfazed to learn that setting custom baud rates is just as difficult. There is no portable way of doing this, so be prepared to experiment with the following code examples to find out what works on your target system.

If you are compiling with the GNU C library, you can forgo the standard enumerations above just specify an integer baud rate directly to cfsetispeed() and cfsetospeed(), e.g.:
```c
// Specifying a custom baud rate when using GNU C
cfsetispeed(&tty, 104560);
cfsetospeed(&tty, 104560);
```

#### termios2 Method
This method relied on using a termios2 struct, which is like a termios struct but with sightly more functionality. I’m unsure on exactly what UNIX systems termios2 is defined on, but if it is, it is usually defined in termbits.h (it was on the Xubuntu 18.04 with GCC system I was doing these tests on):
```c
struct termios2 {
	tcflag_t c_iflag;		/* input mode flags */
	tcflag_t c_oflag;		/* output mode flags */
	tcflag_t c_cflag;		/* control mode flags */
	tcflag_t c_lflag;		/* local mode flags */
	cc_t c_line;			/* line discipline */
	cc_t c_cc[NCCS];		/* control characters */
	speed_t c_ispeed;		/* input speed */
	speed_t c_ospeed;		/* output speed */
};
```

Which is very similar to plain old termios, except with the addition of the c_ispeed and c_ospeed. We can use these to directly set a custom baud rate! We can pretty much set everything other than the baud rate in exactly the same manner as we could for termios, except for the reading/writing of the terminal attributes to and from the file descriptor — instead of using tcgetattr() and tcsetattr() we have to use ioctl().

Let’s first update our includes, we have to remove termios.h and add the following:
```c
// #include <termios.h> This must be removed! 
// Otherwise we'll get "redefinition of ‘struct termios’" errors
#include <sys/ioctl.h> // Used for TCGETS2/TCSETS2, which is required for custom baud rates
```





