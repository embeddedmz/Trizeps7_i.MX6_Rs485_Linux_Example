#include "Rs485SerialPort.h"

#include <fcntl.h>
#include <linux/serial.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>
#include <unistd.h>

namespace
{
    /* This internal function is used by default to set RTS */
    void ioctlRTS(int fd, int on)
    {
        int flags;

        ioctl(fd, TIOCMGET, &flags);
        if (on) {
            flags |= TIOCM_RTS;
        }
        else {
            flags &= ~TIOCM_RTS;
        }
        ioctl(fd, TIOCMSET, &flags);
    }
}

struct Rs485SerialPort::Internals
{
    /* Device: "/dev/ttyS0", "/dev/ttyUSB0" or "/dev/tty.USA19*" on Mac OS X. */
    std::string Device;
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int Baud = 9600;
    /* Data bit */
    uint8_t DataBit = 8;
    /* Stop bit */
    uint8_t StopBit = 1;
    /* Parity: 'N', 'O', 'E' */
    char Parity = 'N';

    /* Save old termios settings */
    struct termios OldTios;

    Rs485SerialPort::RTSMode RTSMode = Rs485SerialPort::RTSMode::None;
    int RTSSoftwareDelay = 0;
    int oneByteTime = 0;
    
    int Fd = -1;

    uint32_t RcvTimeoutMs = 500;

    bool Debug = false;
};

Rs485SerialPort::Rs485SerialPort() :
    m_Internals(new Rs485SerialPort::Internals)
{
}

Rs485SerialPort::~Rs485SerialPort()
{
    close();
    delete m_Internals;
}

bool Rs485SerialPort::open(const std::string& port, const int baudrate, const char parity,
    const int dataBit, const int stopBit)
{
    close();
    
    if (port.empty())
    {
        return false;
    }
    m_Internals->Device = port;

    if (baudrate <= 0)
    {
        return false;
    }
    m_Internals->Baud = baudrate;

    if (parity != 'N' && parity != 'E' && parity != 'O')
    {
        return false;
    }
    m_Internals->Parity = parity;

    m_Internals->DataBit = dataBit;
    m_Internals->StopBit = stopBit;

    m_Internals->RTSMode = RTSMode::None;

    /* Calculate estimated time in micro second to send one byte */
    m_Internals->oneByteTime = 1000000 * (1 + m_Internals->DataBit + (parity == 'N' ? 0 : 1) +
        m_Internals->StopBit) / m_Internals->Baud;

    /* The delay before and after transmission when toggling the RTS pin */
    m_Internals->RTSSoftwareDelay = m_Internals->oneByteTime;

    // Connect
    struct termios tios;
    speed_t speed;
    int flags;

    if (m_Internals->Debug)
    {
        printf("Opening %s at %d bauds (%c, %d, %d)\n",
            m_Internals->Device.c_str(), m_Internals->Baud, m_Internals->Parity,
            m_Internals->DataBit, m_Internals->StopBit);
    }

    /* The O_NOCTTY flag tells UNIX that this program doesn't want
   to be the "controlling terminal" for that port. If you
   don't specify this then any input (such as keyboard abort
   signals and so forth) will affect your process

   Timeouts are ignored in canonical input mode or when the
   NDELAY option is set on the file via open or fcntl */
    flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
#ifdef O_CLOEXEC
    flags |= O_CLOEXEC;
#endif

    m_Internals->Fd = ::open(m_Internals->Device.c_str(), flags);
    if (m_Internals->Fd == -1)
    {
        if (m_Internals->Debug)
        {
            fprintf(stderr, "ERROR Can't open the device %s (%s)\n",
                m_Internals->Device.c_str(), strerror(errno));
        }
        return false;
    }

    /* Save */
    tcgetattr(m_Internals->Fd, &m_Internals->OldTios);

    memset(&tios, 0, sizeof(struct termios));

    /* C_ISPEED     Input baud (new interface)
       C_OSPEED     Output baud (new interface)
    */
    switch (m_Internals->Baud)
    {
    case 110:
        speed = B110;
        break;
    case 300:
        speed = B300;
        break;
    case 600:
        speed = B600;
        break;
    case 1200:
        speed = B1200;
        break;
    case 2400:
        speed = B2400;
        break;
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
#ifdef B57600
    case 57600:
        speed = B57600;
        break;
#endif
#ifdef B115200
    case 115200:
        speed = B115200;
        break;
#endif
#ifdef B230400
    case 230400:
        speed = B230400;
        break;
#endif
#ifdef B460800
    case 460800:
        speed = B460800;
        break;
#endif
#ifdef B500000
    case 500000:
        speed = B500000;
        break;
#endif
#ifdef B576000
    case 576000:
        speed = B576000;
        break;
#endif
#ifdef B921600
    case 921600:
        speed = B921600;
        break;
#endif
#ifdef B1000000
    case 1000000:
        speed = B1000000;
        break;
#endif
#ifdef B1152000
    case 1152000:
        speed = B1152000;
        break;
#endif
#ifdef B1500000
    case 1500000:
        speed = B1500000;
        break;
#endif
#ifdef B2500000
    case 2500000:
        speed = B2500000;
        break;
#endif
#ifdef B3000000
    case 3000000:
        speed = B3000000;
        break;
#endif
#ifdef B3500000
    case 3500000:
        speed = B3500000;
        break;
#endif
#ifdef B4000000
    case 4000000:
        speed = B4000000;
        break;
#endif
    default:
        speed = B9600;
        if (m_Internals->Debug)
        {
            fprintf(stderr,
                "WARNING Unknown baud rate %d for %s (B9600 used)\n",
                m_Internals->Baud, m_Internals->Device.c_str());
        }
    }

    /* Set the baud rate */
    if ((cfsetispeed(&tios, speed) < 0) || (cfsetospeed(&tios, speed) < 0))
    {
        ::close(m_Internals->Fd);
        m_Internals->Fd = -1;
        return false;
    }

    /* C_CFLAG      Control options
       CLOCAL       Local line - do not change "owner" of port
       CREAD        Enable receiver
    */
    tios.c_cflag |= (CREAD | CLOCAL);
    /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

    /* Set data bits (5, 6, 7, 8 bits)
       CSIZE        Bit mask for data bits
    */
    tios.c_cflag &= ~CSIZE;
    switch (m_Internals->DataBit) {
    case 5:
        tios.c_cflag |= CS5;
        break;
    case 6:
        tios.c_cflag |= CS6;
        break;
    case 7:
        tios.c_cflag |= CS7;
        break;
    case 8:
    default:
        tios.c_cflag |= CS8;
        break;
    }

    /* Stop bit (1 or 2) */
    if (m_Internals->StopBit == 1)
        tios.c_cflag &= ~CSTOPB;
    else /* 2 */
        tios.c_cflag |= CSTOPB;

    /* PARENB       Enable parity bit
       PARODD       Use odd parity instead of even */
    if (m_Internals->Parity == 'N')
    {
        /* None */
        tios.c_cflag &= ~PARENB;
    }
    else if (m_Internals->Parity == 'E')
    {
        /* Even */
        tios.c_cflag |= PARENB;
        tios.c_cflag &= ~PARODD;
    }
    else
    {
        /* Odd */
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
    }

    /* Read the man page of termios if you need more information. */

    /* This field isn't used on POSIX systems
       tios.c_line = 0;
    */

    /* C_LFLAG      Line options

       ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
       ICANON       Enable canonical input (else raw)
       XCASE        Map uppercase \lowercase (obsolete)
       ECHO Enable echoing of input characters
       ECHOE        Echo erase character as BS-SP-BS
       ECHOK        Echo NL after kill character
       ECHONL       Echo NL
       NOFLSH       Disable flushing of input buffers after
       interrupt or quit characters
       IEXTEN       Enable extended functions
       ECHOCTL      Echo control characters as ^char and delete as ~?
       ECHOPRT      Echo erased character as character erased
       ECHOKE       BS-SP-BS entire line on line kill
       FLUSHO       Output being flushed
       PENDIN       Retype pending input at next read or input char
       TOSTOP       Send SIGTTOU for background output

       Canonical input is line-oriented. Input characters are put
       into a buffer which can be edited interactively by the user
       until a CR (carriage return) or LF (line feed) character is
       received.

       Raw input is unprocessed. Input characters are passed
       through exactly as they are received, when they are
       received. Generally you'll deselect the ICANON, ECHO,
       ECHOE, and ISIG options when using raw input
    */

    /* Raw input */
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* C_IFLAG      Input options

       Constant     Description
       INPCK        Enable parity check
       IGNPAR       Ignore parity errors
       PARMRK       Mark parity errors
       ISTRIP       Strip parity bits
       IXON Enable software flow control (outgoing)
       IXOFF        Enable software flow control (incoming)
       IXANY        Allow any character to start flow again
       IGNBRK       Ignore break condition
       BRKINT       Send a SIGINT when a break condition is detected
       INLCR        Map NL to CR
       IGNCR        Ignore CR
       ICRNL        Map CR to NL
       IUCLC        Map uppercase to lowercase
       IMAXBEL      Echo BEL on input line too long
    */
    if (m_Internals->Parity == 'N') {
        /* None */
        tios.c_iflag &= ~INPCK;
    }
    else {
        tios.c_iflag |= INPCK;
    }

    /* Software flow control is disabled */
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* C_OFLAG      Output options
       OPOST        Postprocess output (not set = raw output)
       ONLCR        Map NL to CR-NL

       ONCLR ant others needs OPOST to be enabled
    */

    /* Raw ouput */
    tios.c_oflag &= ~OPOST;

    /* C_CC         Control characters
       VMIN         Minimum number of characters to read
       VTIME        Time to wait for data (tenths of seconds)

       UNIX serial interface drivers provide the ability to
       specify character and packet timeouts. Two elements of the
       c_cc array are used for timeouts: VMIN and VTIME. Timeouts
       are ignored in canonical input mode or when the NDELAY
       option is set on the file via open or fcntl.

       VMIN specifies the minimum number of characters to read. If
       it is set to 0, then the VTIME value specifies the time to
       wait for every character read. Note that this does not mean
       that a read call for N bytes will wait for N characters to
       come in. Rather, the timeout will apply to the first
       character and the read call will return the number of
       characters immediately available (up to the number you
       request).

       If VMIN is non-zero, VTIME specifies the time to wait for
       the first character read. If a character is read within the
       time given, any read will block (wait) until all VMIN
       characters are read. That is, once the first character is
       read, the serial interface driver expects to receive an
       entire packet of characters (VMIN bytes total). If no
       character is read within the time allowed, then the call to
       read returns 0. This method allows you to tell the serial
       driver you need exactly N bytes and any read call will
       return 0 or N bytes. However, the timeout only applies to
       the first character read, so if for some reason the driver
       misses one character inside the N byte packet then the read
       call could block forever waiting for additional input
       characters.

       VTIME specifies the amount of time to wait for incoming
       characters in tenths of seconds. If VTIME is set to 0 (the
       default), reads will block (wait) indefinitely unless the
       NDELAY option is set on the port with open or fcntl.
    */
    /* Unused because we use open with the NDELAY option */
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    if (tcsetattr(m_Internals->Fd, TCSANOW, &tios) < 0) {
        ::close(m_Internals->Fd);
        m_Internals->Fd = -1;
        return false;
    }

    // RS485 stuff
    struct serial_rs485 rs485conf;
    // Get
    if (ioctl(m_Internals->Fd, TIOCGRS485, &rs485conf) < 0)
    {
        // Add debug stuff
        return false;
    }
    
    // Set
    rs485conf.flags |= SER_RS485_ENABLED;

    if (ioctl(m_Internals->Fd, TIOCSRS485, &rs485conf) < 0)
    {
        // Add debug stuff
        return false;
    }

    return true;
}

void Rs485SerialPort::flush()
{
    tcflush(m_Internals->Fd, TCIOFLUSH);
}

void Rs485SerialPort::close()
{
    if (m_Internals->Fd != -1)
    {
        tcsetattr(m_Internals->Fd, TCSANOW, &m_Internals->OldTios);
        ::close(m_Internals->Fd);
        m_Internals->Fd = -1;
    }
}

bool Rs485SerialPort::setRtsMode(const Rs485SerialPort::RTSMode rtsMode)
{
    if (m_Internals->Fd < 0)
    {
        if (m_Internals->Debug)
        {
            fprintf(stderr, "Rs485SerialPort::open has not been opened !\n");
        }
        return false;
    }

    if (rtsMode == RTSMode::None || rtsMode == RTSMode::SwUp || rtsMode == RTSMode::SwDown)
    {
        m_Internals->RTSMode = rtsMode;

        /* Set the RTS bit in order to not reserve the RS485 bus */
        ioctlRTS(m_Internals->Fd, m_Internals->RTSMode != RTSMode::SwUp);

        // TODO : Make sure Hardware up/down is disabled !


        return true;
    }
    else
    {
        if (rtsMode == RTSMode::HwUp)
        {
            struct serial_rs485 rs485conf;
            // Get
            if (ioctl(m_Internals->Fd, TIOCGRS485, &rs485conf) < 0)
            {
                // Add debug stuff
                return false;
            }

            // Set
            /* Set logical level for RTS pin equal to 1 when sending: */
            rs485conf.flags |= SER_RS485_RTS_ON_SEND;

            /* Set logical level for RTS pin equal to 0 after sending: */
            rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

            /* Set this flag if you want to receive data even while sending data
             * Enable receiver during sending, required for i.MX devices */
            rs485conf.flags |= SER_RS485_RX_DURING_TX;

            if (ioctl(m_Internals->Fd, TIOCSRS485, &rs485conf) < 0)
            {
                // Add debug stuff
                return false;
            }

            return true;
        }
        else if (rtsMode == RTSMode::HwDown)
        {
            struct serial_rs485 rs485conf;
            // Get
            if (ioctl(m_Internals->Fd, TIOCGRS485, &rs485conf) < 0)
            {
                // Add debug stuff
                return false;
            }

            // Set
            /* Set logical level for RTS pin equal to 0 when sending: */
            rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);

            /* Set logical level for RTS pin equal to 1 after sending: */
            rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;

            /* Set this flag if you want to receive data even while sending data
             * Enable receiver during sending, required for i.MX devices */
            rs485conf.flags |= SER_RS485_RX_DURING_TX;

            /* TODO - Set rts delay before send, if needed: */
            //rs485conf.delay_rts_before_send = ...;

            /* TODO - Set rts delay after send, if needed: */
            //rs485conf.delay_rts_after_send = ...;

            if (ioctl(m_Internals->Fd, TIOCSRS485, &rs485conf) < 0)
            {
                // Add debug stuff/Error handling. See errno.
                return false;
            }

            return true;
        }
    }
    
    return false;
}

void Rs485SerialPort::setDebug(const bool enable)
{
    m_Internals->Debug = enable;
}

ssize_t Rs485SerialPort::send(const uint8_t* pData, const size_t uSize)
{
    if (!pData || !uSize)
        return 0;

    if (m_Internals->RTSMode != RTSMode::None && m_Internals->RTSMode != RTSMode::HwUp
        && m_Internals->RTSMode != RTSMode::HwDown)
    {
        if (m_Internals->Debug)
        {
            fprintf(stderr, "Sending request using RTS signal\n");
        }

        ioctlRTS(m_Internals->Fd, m_Internals->RTSMode == RTSMode::SwUp);
        usleep(m_Internals->RTSSoftwareDelay);

        ssize_t size = write(m_Internals->Fd, pData, uSize);

        usleep(m_Internals->oneByteTime * uSize + m_Internals->RTSSoftwareDelay);
        ioctlRTS(m_Internals->Fd, m_Internals->RTSMode != RTSMode::SwUp);

        return size;
    }
    
    return write(m_Internals->Fd, pData, uSize);

    // to send a big chunk of data, user of this class can do this in his main program :)
    /*
    ssize_t total = 0;
    do
    {
        ssize_t nSent;

        nSent = rs485Com.send(pData + total, uSize - total);
        if (nSent <= 0)
        {
            break; // error !
        }
        total += nSent;
    } while (total < uSize);
    */
}

ssize_t Rs485SerialPort::send(const std::string& strData)
{
    return send(reinterpret_cast<const uint8_t*>(strData.c_str()), strData.length());
}

ssize_t Rs485SerialPort::send(const std::vector<uint8_t>& Data)
{
    return send(Data.data(), Data.size());
}

ssize_t Rs485SerialPort::receive(uint8_t* pData, const int uSize)
{
    if (!pData || !uSize)
        return false;

    int rc;
    fd_set rset;
    struct timeval tv;
    struct timeval* p_tv;
    int length_to_read = uSize;
    int msg_length = 0;

    /* Add a file descriptor to the set */
    FD_ZERO(&rset);
    FD_SET(m_Internals->Fd, &rset);

    tv.tv_sec = m_Internals->RcvTimeoutMs / 1000;
    tv.tv_usec= (m_Internals->RcvTimeoutMs % 1000) * 1000;
    p_tv = &tv;

    while (length_to_read != 0)
    {
        while ((rc = select(m_Internals->Fd + 1, &rset, NULL, NULL, p_tv)) == -1)
        {
            if (errno == EINTR) {
                if (m_Internals->Debug)
                {
                    fprintf(stderr, "A non blocked signal was caught\n");
                }
                /* Necessary after an error */
                FD_ZERO(&rset);
                FD_SET(m_Internals->Fd, &rset);
            }
            else
            {
                if (m_Internals->Debug)
                {
                    fprintf(stderr, "ERROR %s: select\n", strerror(errno));
                }
                return -1;
            }
        }
        if (rc == 0) {
            /* Timeout */
            errno = ETIMEDOUT;

            //if (m_Internals->Debug)
            //{
                //fprintf(stderr, "ERROR %s: select\n", strerror(errno));
            //}

            return msg_length;
        }

        rc = read(m_Internals->Fd, pData + msg_length, length_to_read);
        if (rc == 0) {
            errno = ECONNRESET;
            rc = -1;
        }

        if (rc == -1) {
            if (m_Internals->Debug) {
                fprintf(stderr, "ERROR %s: read\n", strerror(errno));
            }
            return -1;
        }
        

        /* Display the hex code of each character received */
        /*if (m_Internals->Debug) {
            for (int i = 0; i < rc; i++)
                printf("<%.2X>", pData[msg_length + i]);
        }*/

        /* Sums bytes received */
        msg_length += rc;
        /* Computes remaining bytes */
        length_to_read -= rc;
    }

    //if (m_Internals->Debug)
        //printf("\n");

    return msg_length;
}

void Rs485SerialPort::setRcvTimeout(uint32_t msecTimeout)
{
    m_Internals->RcvTimeoutMs = msecTimeout;
}

void Rs485SerialPort::setRTSSoftwareDelay(uint32_t msecTimeout)
{
    m_Internals->RTSSoftwareDelay = msecTimeout * 1000;
}
