#include "ComPort.h"
#include <stdexcept>
#include <thread>

//конструкторы /деструктор
COMPort::COMPort ( std::string const& portName,
                    std::unique_ptr< COMMTIMEOUTS > _Timeouts,
                    unsigned int CommMask,
                    unsigned int InputBuffer,
                    unsigned int Outputbuffer):
    port_name( portName ),
    TimeOuts( std::move( _Timeouts ) ),
    ComDCM(),
    PortHandle( 0 )
{
    PortHandle = CreateFileA( portName.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              NULL,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL);
    if ( PortHandle == reinterpret_cast<HANDLE>( -1 ) )
    {
        PortHandle = 0;
        throw std::runtime_error("COMPort: failed to open.");
    }
    SetCommMask( PortHandle, CommMask );
    SetupComm( PortHandle, InputBuffer, Outputbuffer );

    if( !SetCommTimeouts( PortHandle, TimeOuts.get() ) )
	{
		PortHandle = 0;
        throw std::runtime_error("Ошибка настройки COMMTIMEOUTS");
	}
}

COMPort::COMPort (const string &portName ):
    port_name( portName ),
    TimeOuts( new COMMTIMEOUTS() ),
    ComDCM(),
    PortHandle( 0 )
{
    PortHandle = CreateFileA( portName.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              NULL,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL);

    if ( PortHandle == reinterpret_cast<HANDLE>( -1 ) )
    {
        PortHandle = 0;
        throw std::runtime_error("COMPort: failed to open.");
    }

    SetCommMask( PortHandle, EV_RXCHAR );
    SetupComm( PortHandle, 1024, 1024 );

	TimeOuts->ReadIntervalTimeout = 0xFFFFFFFF;
	TimeOuts->ReadTotalTimeoutMultiplier = 0;
	TimeOuts->ReadTotalTimeoutConstant = 0;
	TimeOuts->WriteTotalTimeoutMultiplier = 0;
	TimeOuts->WriteTotalTimeoutConstant =0;

    if( !SetCommTimeouts( PortHandle, TimeOuts.get() ) )
	{
		PortHandle = 0;
        throw std::runtime_error("Ошибка настройки COMMTIMEOUTS");
	}
    if ( !Set_DCB_Settings( BR_9600, NOparity, OFF, ONE ) )
	{
        CloseHandle( PortHandle );
		PortHandle = 0;
        throw std::runtime_error("Ошибка настройки DCB");
	}
}

COMPort::~COMPort()
{
	// close serial port device
    if ( !CloseHandle (PortHandle) )
	{
       throw std::runtime_error("COMPort: failed to close.");
	}
}

//настройка порта
bool COMPort::Set_DCB_Settings( BaudRates BaudRate, Paritys patity, fParitys f_parity, StopBits stop_bits )
{
    if ( !ComDCM )
	{
        ComDCM = Get_port_Settings();
	}
	ComDCM->BaudRate =BaudRate;
	ComDCM->ByteSize = 8;
	ComDCM->Parity = patity;
	ComDCM->StopBits = stop_bits;
	ComDCM->fAbortOnError = FALSE;
	ComDCM->fDtrControl = DTR_CONTROL_DISABLE;
	ComDCM->fRtsControl = RTS_CONTROL_DISABLE;
	ComDCM->fBinary = TRUE;
	ComDCM->fParity = f_parity;
	ComDCM->fInX = FALSE;
	ComDCM->fOutX = FALSE;
	ComDCM->XonChar = (unsigned char)0x11;
	ComDCM->XoffChar = (unsigned char)0x13;
	ComDCM->fErrorChar = FALSE;
	ComDCM->fNull = FALSE;
	ComDCM->fOutxCtsFlow = FALSE;
	ComDCM->fOutxDsrFlow = FALSE;
	ComDCM->XonLim = 1024;
	ComDCM->XoffLim = 1024;
	ComDCM->fDsrSensitivity=FALSE;

    return SetCommState( PortHandle, ComDCM.get() );
}
bool COMPort::Set_DCB_Settings( std::unique_ptr< DCB > DCB_Settings )
{	
    ComDCM = std::move( DCB_Settings );
    return SetCommState( PortHandle, ComDCM.get() );
}
//информация о порте
bool COMPort::IsOK() const
{
	return PortHandle != 0;
}
std::unique_ptr<DCB> COMPort::Get_port_Settings(void)
{
    std::unique_ptr< DCB >Result( new DCB() );
    Result->DCBlength = sizeof( DCB );
    GetCommState( PortHandle, Result.get() );
	return Result;
}
 //буфер приема/передачи
void COMPort::Write	( const BYTE* buff, size_t len )
{
	DWORD feedback;
    if( !WriteFile( PortHandle, buff, len, &feedback, 0 ) || feedback !=len )
	{
        throw std::runtime_error("Ошибка записи");
	}
	// In some cases it's worth uncommenting
	//FlushFileBuffers(m_Handle);
 }
size_t COMPort::Read( BYTE *buff, size_t len )
{
	//DWORD begin = GetTickCount();
	DWORD feedback =0;

    if( !ReadFile( PortHandle, buff, len, &feedback, NULL ) )
	{
        throw std::runtime_error("Ошибка чтения");
	}
    return feedback;
}
bool COMPort::Clear_Com_Buff (DWORD flags)
{
    return PurgeComm( PortHandle, flags );
}
//сигнальные выводы порта
bool COMPort::DTR_On( unsigned int w8_before, unsigned int w8_after )
{
	if (w8_before)
	{
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_before ) );
	}
    bool result = EscapeCommFunction( PortHandle, SETDTR );
    if ( w8_after )
	{
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_after ) );
	}
	return result;
}
bool COMPort::DTR_oFF( unsigned int w8_before, unsigned int w8_after )
{
    if (w8_before)
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_before ) );
    }
    bool result = EscapeCommFunction( PortHandle, CLRDTR );
    if ( w8_after )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_after ) );
    }
	return result;
}
//сигнальные выводы порта
bool COMPort::RTS_On( unsigned int w8_before, unsigned int w8_after )
{
    if (w8_before)
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_before ) );
    }
    bool result = EscapeCommFunction( PortHandle, SETRTS );
    if ( w8_after )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_after ) );
    }
    return result;
}
bool COMPort::RTS_oFF( unsigned int w8_before, unsigned int w8_after )
{
    if (w8_before)
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_before ) );
    }
    bool result = EscapeCommFunction( PortHandle, CLRRTS );
    if ( w8_after )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( w8_after ) );
    }
    return result;
}
