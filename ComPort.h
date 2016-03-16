#pragma once

#ifdef WINDOWS
#ifndef NOMINMAX
    #define NOMINMAX //иначе API windows определит макросы min и max, конфликтующие с std::max и std::min в vector
#endif

#include <windows.h>
#endif

#if defined( DEMO ) && not defined( WINDOWS )

#define CBR_110 110
#define CBR_300 300
#define CBR_600 600
#define CBR_1200 1200
#define CBR_2400 2400
#define CBR_4800 4800
#define CBR_9600 9600
#define CBR_14400 14400
#define CBR_19200 19200
#define CBR_38400 38400

#define CBR_56000 56000
#define CBR_57600 57600
#define CBR_115200 115200
#define CBR_128000 128000
#define CBR_256000 256000

#define TRUE 1
#define FALSE 0

#define NOPARITY 0
#define ODDPARITY 1
#define EVENPARITY 2
#define MARKPARITY 3
#define SPACEPARITY 4

#define ONESTOPBIT 1
#define ONE5STOPBITS 2
#define TWOSTOPBITS 3

#define EV_RXCHAR 0 	// Any Character received
#define EV_RXFLAG 1 	// Received certain character
#define EV_TXEMPTY 2 	// Transmitt Queue Empty
#define EV_CTS 3          // CTS changed state
#define EV_DSR 4          // DSR changed state
#define EV_RLSD 5        // RLSD changed state
#define EV_BREAK 6      // BREAK received
#define EV_ERR 7          // Line status error occurred
#define EV_RING 8        // Ring signal detected
#define EV_PERR 9        // Printer error occured
#define EV_RX80FULL 10// Receive buffer is 80 percent full
#define EV_EVENT1 11    // Provider specific event 1
#define EV_EVENT2 12    // Provider specific event 2

#define PURGE_TXABORT 0  // Kill the pending/current writes to the comm port.
#define PURGE_RXABORT 1  // Kill the pending/current reads to the comm port.
#define PURGE_TXCLEAR 2  // Kill the transmit queue if there.
#define PURGE_RXCLEAR 3  // Kill the typeahead buffer if there.

typedef int DWORD;
typedef unsigned char BYTE;

typedef int HANDLE;
#define DTR_CONTROL_DISABLE 0
#define RTS_CONTROL_DISABLE 0

#define GENERIC_READ 0
#define GENERIC_WRITE 1
#define OPEN_EXISTING 0
#define FILE_ATTRIBUTE_NORMAL 0

#define CLRDTR 0
#define SETDTR 1


struct COMMTIMEOUTS
{
    int ReadIntervalTimeout;
    int ReadTotalTimeoutMultiplier;
    int ReadTotalTimeoutConstant;
    int WriteTotalTimeoutMultiplier;
    int WriteTotalTimeoutConstant;
};

struct DCB
{
    int BaudRate;
    int ByteSize;
    int Parity;
    int StopBits;
    int fAbortOnError;
    int fDtrControl;
    int fRtsControl;
    int fBinary;
    int fParity;
    int fInX;
    int fOutX;
    BYTE XonChar;
    BYTE XoffChar;
    int fErrorChar;
    int fNull;
    int fOutxCtsFlow;
    int fOutxDsrFlow;
    int XonLim;
    int XoffLim;
    int fDsrSensitivity;
    int DCBlength;
};

static HANDLE CreateFileA( char const*, int, int, void*, int, int, void* ){ return 1;}
static void SetCommMask( HANDLE, int ){}
static void SetupComm( HANDLE, int, int ){}
static bool SetCommTimeouts( HANDLE, COMMTIMEOUTS* ){ return true; }
static bool CloseHandle( HANDLE ){ return true; }
static bool SetCommState( HANDLE, DCB* ) {return true;}
static bool GetCommState( HANDLE, DCB* ) {return true;}
static bool EscapeCommFunction( HANDLE, DWORD ) {return true;}
static bool PurgeComm( HANDLE, DWORD ) {return true;}
static bool WriteFile( HANDLE, BYTE const*, DWORD len, DWORD* fb, void* ){ if (fb) *fb = len; return true;}
static bool ReadFile( HANDLE, BYTE*, DWORD len, DWORD* fb, void* ){ if (fb) *fb = len; return true;}
#endif

#include <string>
#include <memory>
using namespace std;

class COMPort
{
public:
    COMPort ( std::string const& portName );
    COMPort ( std::string const& portName,
              std::unique_ptr< COMMTIMEOUTS > _Timeouts,
              unsigned int CommMask,
              unsigned int InputBuffer,
              unsigned int Outputbuffer);
    ~COMPort ();

    enum BaudRates	{
        BR_75=75,
        BR_110=CBR_110,
        BR_134=134,
        BR_150=150,
        BR_300=CBR_300,
        BR_600=CBR_600,
        BR_1200=CBR_1200,
        BR_1800=1800,
        BR_2400=CBR_2400,
        BR_4800=CBR_4800,
        BR_7200=7200,
        BR_9600=CBR_9600,
        BR_14400=CBR_14400,
        BR_19200=CBR_19200,
        BR_38400=CBR_38400,
        BR_56000=CBR_56000,
        BR_57600=CBR_57600,
        BR_115200=CBR_115200,
        BR_128000=CBR_128000,
        BR_256000=CBR_256000
    };

    enum Paritys	{
        NOparity=NOPARITY,
        ODDparity=ODDPARITY,
        EVENparity=EVENPARITY,
        MARKparity=MARKPARITY,
        SPACEparity=SPACEPARITY
    };
    enum fParitys	{
        ON=TRUE,
        OFF=FALSE
    };
    enum StopBits	{
        ONE=ONESTOPBIT,
        ONE5=ONE5STOPBITS,
        TWO=TWOSTOPBITS
    };
    enum CommMasks	{
        ev_rxchar=EV_RXCHAR, 	// Any Character received
        ev_rxflag=EV_RXFLAG, 	// Received certain character
        ev_txempty=EV_TXEMPTY, 	// Transmitt Queue Empty
        ev_cts=EV_CTS,          // CTS changed state
        ev_dsr=EV_DSR,          // DSR changed state
        ev_rlsd=EV_RLSD,        // RLSD changed state
        ev_break=EV_BREAK,      // BREAK received
        ev_err=EV_ERR,          // Line status error occurred
        ev_ring=EV_RING,        // Ring signal detected
        ev_perr=EV_PERR,        // Printer error occured
        ev_rx80full=EV_RX80FULL,// Receive buffer is 80 percent full
        ev_event1=EV_EVENT1,    // Provider specific event 1
        ev_event2=EV_EVENT2     // Provider specific event 2
    };
    enum Purge_flags{
        TXABORT=PURGE_TXABORT,  // Kill the pending/current writes to the comm port.
        RXABORT=PURGE_RXABORT,  // Kill the pending/current reads to the comm port.
        TXCLEAR=PURGE_TXCLEAR,  // Kill the transmit queue if there.
        RXCLEAR=PURGE_RXCLEAR   // Kill the typeahead buffer if there.
    };

//информация о порте
    bool IsOK (void) const;
    std::string const& Get_Com_Name (void) const
    {
        return port_name;
    }
    std::unique_ptr< DCB > Get_port_Settings (void);
//настройка порта
    bool Set_DCB_Settings( BaudRates BaudRate,
                           Paritys patity,
                           fParitys f_parity,
                           StopBits stop_bits );
    bool Set_DCB_Settings( std::unique_ptr< DCB >  DCB_Settings );
//буфер приема/передачи
    bool Clear_Com_Buff	(DWORD flags );
    void Write ( const BYTE* buff, size_t len );
    size_t Read( BYTE *buff, size_t len );
//сигнальные выводы порта
    bool DTR_On ( unsigned int w8_before=0,
                  unsigned int w8_after=0 );
    bool DTR_oFF ( unsigned int w8_before=0,
                   unsigned int w8_after=0 );

private:
    const std::string port_name;
    std::unique_ptr< COMMTIMEOUTS > TimeOuts;
    std::unique_ptr< DCB > ComDCM;
    HANDLE PortHandle;
};
