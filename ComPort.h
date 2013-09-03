//---------------------------------------------------------------------------

#ifndef ComPortH
#define ComPortH
#include <Classes.hpp>
#define PORT_NAME_LEN 9
//---------------------------------------------------------------------------
 #define NOMINMAX //иначе API windows определит макросы min и max, конфликтующие с std::max и std::min в vector
 #include <windows.h>

 using namespace std;

class COMPort
{
	public:
			COMPort ( const wchar_t * const portName );
			COMPort ( const wchar_t * const portName,
						COMMTIMEOUTS *_Timeouts,
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
		enum Patitys	{
						NOparity=NOPARITY,
						ODDparity=ODDPARITY,
						EVENparity=EVENPARITY,
						MARKparity=MARKPARITY,
						SPACEparity=SPACEPARITY
						};
		enum fPatitys	{
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
		bool 		IsOK				(void) const;
const 	wchar_t * 	Get_Com_Name		(void) const {return port_name;};
		DCB * 		Get_port_Settings	(void);
		//настройка порта
		bool 		Set_DCB_Settings(BaudRates BaudRate,Patitys patity,fPatitys f_parity,StopBits stop_bits);
		bool 		Set_DCB_Settings(DCB * DCB_Settings);
		//буфер приема/передачи
		bool 		Clear_Com_Buff	(Purge_flags flags);
		void 		Write			(const BYTE* buff,	size_t len);
		void 		Read			(BYTE *buff,	size_t &len);
		//сигнальные выводы порта
		bool 		DTR_On			(unsigned int w8_before=0,unsigned int w8_after=0);
		bool 		DTR_oFF			(unsigned int w8_before=0,unsigned int w8_after=0);

	private:
		const wchar_t  *port_name;
		COMMTIMEOUTS *TimeOuts;
		DCB *ComDCM;
		HANDLE PortHandle;
};
#endif
