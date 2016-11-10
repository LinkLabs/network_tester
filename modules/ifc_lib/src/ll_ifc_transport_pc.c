#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#if defined(WIN32) || defined (__MINGW32__)
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "ll_ifc_transport_pc.h"
#include "ll_ifc.h"

// Debug switch - printf's every byte Tx'd and Rx'd (to stdout)
// #define DEBUG_PRINT_EVERY_BYTE_TX_RX

#if defined(WIN32) || defined (__MINGW32__)
static HANDLE g_tty_fd;
#else
static int g_tty_fd = -1;
#endif

#if defined(WIN32) || defined (__MINGW32__)
int ll_tty_open(const char * dev_name, int baudrate)
{
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};

    if (dev_name == NULL)
    {
        dev_name = LL_TTY_DEFAULT_DEVICE;
    }
    printf("Open %s baud %d\n", dev_name, baudrate);
    g_tty_fd = CreateFile(dev_name, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if(g_tty_fd == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "Unable to open %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    /**
     * Set device parameters (115200 baud, 1 start bit,
     * 1 stop bit, no parity)
     */
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(g_tty_fd, &dcbSerialParams) == 0)
    {
        fprintf(stderr, "Error getting device state\n");
        CloseHandle(g_tty_fd);
        exit(EXIT_FAILURE);
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if(SetCommState(g_tty_fd, &dcbSerialParams) == 0)
    {
        fprintf(stderr, "Error setting device parameters\n");
        CloseHandle(g_tty_fd);
        exit(EXIT_FAILURE);
    }

    /* Set COM port timeout settings */
    timeouts.ReadIntervalTimeout = 50;          /* milliseconds */
    timeouts.ReadTotalTimeoutConstant = 50;     /* milliseconds */
    timeouts.ReadTotalTimeoutMultiplier = 10;   /* milliseconds */
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(SetCommTimeouts(g_tty_fd, &timeouts) == 0)
    {
        fprintf(stderr, "Error setting timeouts\n");
        CloseHandle(g_tty_fd);
        return 1;
    }

    return 0;
}

int ll_tty_close()
{
    CloseHandle(g_tty_fd);
	return 0;
}

int32_t transport_write(uint8_t *buff, uint16_t len)
{
    DWORD bytes_written;

    if (!WriteFile(g_tty_fd, buff, len, &bytes_written, NULL))
    {
        fprintf(stderr, "Error writing tty\n");
        return -1;
    }
    if (bytes_written < len)
    {
        return -1;
    }

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < bytes_written; i++)
    {
        printf("W: 0x%02x\n", buff[i]);
    }
#endif

    return 0;
}

int32_t transport_read(uint8_t *buff, uint16_t len)
{
    DWORD bytes_read;

    if (!ReadFile(g_tty_fd, buff, len, &bytes_read, NULL))
    {
        fprintf(stderr, "Error reading tty\n");
        return -1;
    }
    if (bytes_read < len)
    {
        return -1;
    }

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < bytes_read; i++)
    {
        printf("\tR: 0x%02x\n", buff[i]);
    }
#endif

    return 0;
}

#else

int ll_tty_close()
{
    if (g_tty_fd < 0)
    {
		close(g_tty_fd);
		g_tty_fd = -1;
    }
	return 0;
}

#ifdef __APPLE__
int ll_tty_open(const char * dev_name, int baudrate)
{
    struct termios term_io_settings;

    if (dev_name == NULL)
    {
        printf("Open %s baud %d\n", LL_TTY_DEFAULT_DEVICE, baudrate);
        g_tty_fd = open(LL_TTY_DEFAULT_DEVICE, O_RDWR);
    }
    else
    {
        printf("Open %s baud %d\n", dev_name, baudrate);
        g_tty_fd = open(dev_name, O_RDWR);
    }
    if(g_tty_fd == -1)
    {
        fprintf(stderr, "Unable to open %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    memset(&term_io_settings, 0, sizeof(struct termios));
    cfmakeraw(&term_io_settings);
    cfsetspeed(&term_io_settings, baudrate);

    term_io_settings.c_cflag = CREAD | CLOCAL;  /* turn on READ */
    term_io_settings.c_cflag |= CS8;
    term_io_settings.c_cc[VMIN] = 0;
    term_io_settings.c_cc[VTIME] = 10;          /* 1 sec timeout */
    ioctl(g_tty_fd, TIOCSETA, &term_io_settings);

    /* Flush read buffer */
    int i = 0;
    while (1)
    {
        int i32_ret;
        uint8_t ret_byte;

        i32_ret = read(g_tty_fd, &ret_byte, 1);
        if (i32_ret <= 0)
        {
            break;
        }
        else if (i32_ret > 0)
        {
            i++;
        }
    }
    printf("Flushed %d bytes\n", i);

    return 0;
}
#else
static int ll_tty_flush() {
    while (g_tty_fd >= 0)
    {
        ssize_t sst_ret;
        uint8_t ret_byte;

        sst_ret = read(g_tty_fd, &ret_byte, 1);
        if (sst_ret == 0)
		{
			return 0;  // no more data.
		}
		else if (sst_ret < 0)
        {
			perror("Error flushing CF UART");
			return -1;
		}
    }
	return 0;
}


int ll_tty_open(const char *dev_name, int baudrate)
{
    if (g_tty_fd >= 0)
    {
        printf("ll_tty_open: already open.");
		ll_tty_close();
    }
	if (baudrate == 0) 
	{
		baudrate = LL_TTY_DEFAULT_BAUDRATE;
    }
    else if (baudrate != LL_TTY_DEFAULT_BAUDRATE)
    {
        return(-1);
    }
    if (dev_name == NULL)
    {
        dev_name = LL_TTY_DEFAULT_DEVICE;
    }

    g_tty_fd = open(dev_name, O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY);
    if(g_tty_fd < 0)
    {
        perror("ll_tty_open");
        return(-1);
    }
    printf("Open %s baud %d\n", dev_name, baudrate);

    /* Turn off blocking for reads, use (g_tty_fd, F_SETFL, FNDELAY) if you want that */
    struct termios tc;
    tcgetattr(g_tty_fd, &tc);

    /* input flags */
    tc.c_iflag &= ~ IGNBRK; /* enable ignoring break */
    tc.c_iflag &= ~(IGNPAR | PARMRK); /* disable parity checks */
    tc.c_iflag &= ~ INPCK; /* disable parity checking */
    tc.c_iflag &= ~ ISTRIP; /* disable stripping 8th bit */
    tc.c_iflag &= ~(INLCR | ICRNL); /* disable translating NL <-> CR */
    tc.c_iflag &= ~ IGNCR; /* disable ignoring CR */
    tc.c_iflag &= ~(IXON | IXOFF); /* disable XON/XOFF flow control */
    /* output flags */
    tc.c_oflag &= ~ OPOST; /* disable output processing */
    tc.c_oflag &= ~(ONLCR | OCRNL); /* disable translating NL <-> CR */
    /* not for FreeBSD */
    tc.c_oflag &= ~ OFILL; /* disable fill characters */
    /* control flags */
    tc.c_cflag |= CLOCAL; /* prevent changing ownership */
    tc.c_cflag |= CREAD; /* enable reciever */
    tc.c_cflag &= ~ PARENB; /* disable parity */
    tc.c_cflag &= ~ CSTOPB; /* disable 2 stop bits */
    tc.c_cflag &= ~ CSIZE; /* remove size flag... */
    tc.c_cflag |= CS8; /* ...enable 8 bit characters */
    tc.c_cflag |= HUPCL; /* enable lower control lines on close - hang up */
    tc.c_cflag &= ~ CRTSCTS; /* disable hardware CTS/RTS flow control */
    /* local flags */
    tc.c_lflag &= ~ ISIG; /* disable generating signals */
    tc.c_lflag &= ~ ICANON; /* disable canonical mode - line by line */
    tc.c_lflag &= ~ ECHO; /* disable echoing characters */
    tc.c_lflag &= ~ ECHONL; /* ??? */
    tc.c_lflag &= ~ NOFLSH; /* disable flushing on SIGINT */
    tc.c_lflag &= ~ IEXTEN; /* disable input processing */

    /* control characters */
    memset(tc.c_cc,0,sizeof(tc.c_cc));

    /* set i/o baud rate */
    cfsetspeed(&tc, B115200);
    tcsetattr(g_tty_fd, TCSAFLUSH, &tc);

    /* enable input & output transmission */
    tcflow(g_tty_fd, TCOON | TCION);

	ll_tty_flush();
    return(0);
}

#endif

int32_t transport_write(uint8_t *buff, uint16_t len)
{
    ssize_t ret = write(g_tty_fd, buff, len);

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < len; i++)
    {
        printf("W: 0x%02x\n", buff[i]);
    }
#endif

    return ret;
}

static ssize_t read_with_timeout(int fd, void *bf, size_t len, time_t sec)
{
    fd_set set;
    struct timeval timeout;
    ssize_t ret;

    FD_ZERO(&set); // clear the set
    FD_SET(fd, &set); // add our file descriptor to the set

    timeout.tv_sec = sec;
    timeout.tv_usec = 0;

    ret = select(fd + 1, &set, NULL, NULL, &timeout);
    if(ret == -1)
    {
        perror("select"); // an error accured
        return ret;
    }
    else if(ret == 0)
    {
        fprintf(stderr, "read_with_timeout: Timeout %zu, %zu\n", sec, len);
        return -1;
    }
    else
    {
        return read(fd, bf, len); // there was data to read
    }
}

int32_t transport_read(uint8_t *buff, uint16_t len)
{
    ssize_t ret;
    uint16_t bytes_read = 0;

    do
    {
        // Read with a timeout of 3.0 seconds
        ret = read_with_timeout(g_tty_fd, buff + bytes_read, len - bytes_read, 3);
        if (ret < 0)
        {
            fprintf(stderr, "Error(%zi) reading tty: %s\n", ret, strerror(errno));
            return ret;
        }
        else
        {
            bytes_read += ret;
        }
    }
    while (bytes_read < len);

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < bytes_read; i++)
    {
        printf("\tR: 0x%02x\n", buff[i]);
    }
#endif

    return 0;
}
#endif
