/*

作者:韩智鸿

han.zhihong@qq.com

*/

#include "serial.h"

#ifdef _WIN32
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+
//+				windows	 system
//+

int OpenDev(TzhSerial* sport, char* szComName, DWORD baudrate)
{
  COMMTIMEOUTS CommTimeOuts;
  HANDLE hCom;
  DCB dcb;

  sport->bOpen=1;
  strcpy(sport->szComName,szComName);
  sport->nbaudrate=baudrate;
  sport->hCom=0;

  //OVERLAPPED m_OverlappedR;
  //OVERLAPPED m_OverlappedW;
 
  hCom = CreateFile( szComName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL );

  if( hCom == NULL || hCom == (HANDLE)0xFFFFFFFF ){
    //printf("%s : Open Error!\n", szComName);
    return 1;
 } 

  memset( &sport->overlappedR, 0, sizeof( OVERLAPPED ) );
  memset( &sport->overlappedW, 0, sizeof( OVERLAPPED ) );
  
  CommTimeOuts.ReadIntervalTimeout          = 0xFFFFFFFF;
  CommTimeOuts.ReadTotalTimeoutMultiplier   = 0;
  CommTimeOuts.ReadTotalTimeoutConstant     = 0;
  CommTimeOuts.WriteTotalTimeoutMultiplier  = 0;
  CommTimeOuts.WriteTotalTimeoutConstant    = 5000;
  SetCommTimeouts( hCom, &CommTimeOuts );

  sport->overlappedR.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
  sport->overlappedW.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

  dcb.DCBlength = sizeof( DCB );
  GetCommState( hCom, &dcb );
  dcb.BaudRate = baudrate;
  dcb.ByteSize = 8;

  if( !SetCommState( hCom, &dcb ) ||
    !SetupComm( hCom, 10000, 10000 ) ||
    sport->overlappedR.hEvent == NULL ||
    sport->overlappedW.hEvent == NULL ){

      DWORD dwError = GetLastError();

      if( sport->overlappedR.hEvent != NULL ) CloseHandle( sport->overlappedR.hEvent );
      if( sport->overlappedW.hEvent != NULL ) CloseHandle( sport->overlappedW.hEvent );
      CloseHandle( hCom );
      return 2;
  }

  sport->hCom = hCom;
  SetDev(sport);
  return 0;
}

int SetDev(TzhSerial* sport)
{
  DWORD dwErrorFlags;
  COMSTAT ComStat;
  if( !sport->hCom ) return 0;
  ClearCommError( sport->hCom, &dwErrorFlags, &ComStat );
  return (int)ComStat.cbInQue;
}

DWORD SendDevBuf( TzhSerial* sport, BYTE *btBuf, DWORD dwBytesWritten)
{
  COMSTAT ComStat;
  BOOL bWriteStat;
  DWORD dwErrorFlags, bytesSent, BytesWritten;

  HANDLE hCom = sport->hCom;
  if( !hCom ) return 0;

  bWriteStat = WriteFile( hCom, btBuf, dwBytesWritten, &bytesSent, &sport->overlappedW );

  if (GetLastError() != ERROR_IO_PENDING) {
    ClearCommError(hCom, &dwErrorFlags, &ComStat);
    if(bWriteStat==TRUE)
	{
		return dwBytesWritten;
	}
	else
	{
		return -1;
	}
  }

  while (1) {
    if (GetOverlappedResult(hCom, &sport->overlappedW, &BytesWritten, TRUE)) break;
    if (GetLastError() != ERROR_IO_INCOMPLETE) {
      ClearCommError(hCom, &dwErrorFlags, &ComStat);
      break;
    }
    bytesSent += BytesWritten;
  }
  bytesSent += BytesWritten;

  return dwBytesWritten;
}


DWORD ReadDevBuf( TzhSerial* sport, BYTE *btBuf, DWORD dwBytesRead)
{

  BOOL bReadStatus;
  DWORD dwErrorFlags;
  COMSTAT ComStat;
  HANDLE hCom = sport->hCom;
  int ret;
  ret=0;
  if( !hCom ) return 0;
  memset(btBuf,0,dwBytesRead);
  memset(&ComStat,0,sizeof(COMSTAT));
  ComStat.cbInQue=-1;
  ClearCommError( hCom, &dwErrorFlags, &ComStat );
  if(dwErrorFlags)return 0;
  if( !ComStat.cbInQue ) return 0;

  dwBytesRead = min(dwBytesRead,(DWORD) ComStat.cbInQue);
  ret =dwBytesRead;
  
  bReadStatus=ReadFile( hCom, btBuf, dwBytesRead, &dwBytesRead, &sport->overlappedR );
  if( !bReadStatus ){
    if( GetLastError() == ERROR_IO_PENDING ){
      WaitForSingleObject( sport->overlappedR.hEvent, 2000 );
      goto _end;
    }
    return -1;
  }
_end:
  return ret;
}

void CloseDev(TzhSerial* sport)
{
	sport->bOpen=0;
	CloseHandle( sport->hCom );
	sport->hCom=NULL;
}

int CheckStatus(TzhSerial* sport)
{
	DWORD err;
	COMMTIMEOUTS CommTimeOuts;
	HANDLE hCom;
	DCB dcb;

	if(0==sport->bOpen){return 0;}

	  hCom = CreateFile( sport->szComName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL );

	  if( hCom != NULL && hCom != (HANDLE)0xFFFFFFFF )
	  {
		  memset( &sport->overlappedR, 0, sizeof( OVERLAPPED ) );
		  memset( &sport->overlappedW, 0, sizeof( OVERLAPPED ) );
  
		  CommTimeOuts.ReadIntervalTimeout          = 0xFFFFFFFF;
		  CommTimeOuts.ReadTotalTimeoutMultiplier   = 0;
		  CommTimeOuts.ReadTotalTimeoutConstant     = 0;
		  CommTimeOuts.WriteTotalTimeoutMultiplier  = 0;
		  CommTimeOuts.WriteTotalTimeoutConstant    = 5000;
		  SetCommTimeouts( hCom, &CommTimeOuts );

		  sport->overlappedR.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
		  sport->overlappedW.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL );

		  dcb.DCBlength = sizeof( DCB );
		  GetCommState( hCom, &dcb );
		  dcb.BaudRate = sport->nbaudrate;
		  dcb.ByteSize = 8;

		  if( !SetCommState( hCom, &dcb ) ||
			!SetupComm( hCom, 10000, 10000 ) ||
			sport->overlappedR.hEvent == NULL ||
			sport->overlappedW.hEvent == NULL ){

			  DWORD dwError = GetLastError();

			  if( sport->overlappedR.hEvent != NULL ) CloseHandle( sport->overlappedR.hEvent );
			  if( sport->overlappedW.hEvent != NULL ) CloseHandle( sport->overlappedW.hEvent );
			  CloseHandle( hCom );
			  return 0;
		  }

		  sport->hCom = hCom;
		  SetDev(sport);
	} 

  //--------------------------------------------------
	err=GetLastError();
	if(err==ERROR_ACCESS_DENIED)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

#else
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+
//+				linux/unix	 system
//+

//serial baudrate info
#define ZH_SERIAL_SPEED_COUNT	19
int g_zh_serial_speed_arr[] = { B921600,	B460800,	B230400,	B115200,
								B57600,		B38400,		B19200,		B9600,
								B4800,		B2400,		B1200,		B300, 
								B38400,		B19200,		B9600,		B4800,
								B2400,		B1200,		B300};
int g_zh_serial_speed[] =	  { 921600,		460800,		230400,		115200,
								57600,		38400,		19200,		9600,
								4800,		2400,		1200,		300,
								38400,		19200,		9600,		4800,
								2400,		1200,		300};

int init_serial (int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	int i;
	int bFindSerialSpeed;
	struct termios newtio,oldtio;	
	
	if ( tcgetattr( fd,&oldtio)  !=  0) {
		perror("Setup Serial Fail");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch( nBits ) {
	case 7:
	 	newtio.c_cflag |= CS7;
		break;
	case 8:
 		newtio.c_cflag |= CS8;
  		break;
	}	

	switch( nEvent ) {
 	case 'O':
  		newtio.c_cflag |= PARENB;
  		newtio.c_cflag |= PARODD;
 		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
 	case 'E':
  		newtio.c_iflag |= (INPCK | ISTRIP);
  		newtio.c_cflag |= PARENB;
  		newtio.c_cflag &= ~PARODD;
  		break;
 	case 'N': 
  		newtio.c_cflag &= ~PARENB;
  		break;
	 }

	bFindSerialSpeed=0;
	for ( i= 0;  i < ZH_SERIAL_SPEED_COUNT;  i++) 
	{ 
		if  (nSpeed == g_zh_serial_speed[i]) 
		{
			bFindSerialSpeed=1;
			tcflush(fd,TCIOFLUSH);
			cfsetispeed(&newtio, g_zh_serial_speed_arr[i]);
			cfsetospeed(&newtio, g_zh_serial_speed_arr[i]);
			break;
		} 
	}

	if(0==bFindSerialSpeed)
	{
		//not found speed
		return -1;
	}

 	if( nStop == 1 )
  		newtio.c_cflag &=  ~CSTOPB;
 	else if ( nStop == 2 ) {
 		newtio.c_cflag |=  CSTOPB;
 		newtio.c_cc[VTIME]  = 0;
 		newtio.c_cc[VMIN] = 0;
	}

 	if((tcsetattr(fd,TCSANOW,&newtio))!=0) {
  		//perror("com set error");
  		return -1;
 	}
	
 	tcflush(fd,TCIOFLUSH);
	//printf("Serial Init Finish!\n");

	return 0;
 
}

/**********************************************************************
*
*/
int OpenDev(TzhSerial* sport,char *Dev ,int nbaudrate)
{
	sport->bOpen =1;
	strcpy(sport->szComName,Dev);
	
	sport->nbaudrate=nbaudrate;
	sport->hCom = open(Dev,O_RDWR|O_NOCTTY|O_NDELAY);
	
	if (-1 == sport->hCom)
		{
			//perror("Can't Open Serial Port");
			return 1;
		}
	else
	{
		//NoBlocking
		fcntl(sport->hCom,F_SETFL,FNDELAY);
		//Blocking
		//fcntl(sport->hCom,F_SETFL,0);

		if (init_serial(sport->hCom,nbaudrate,8,'N',1))
		{
			//printf("Set Parity Error\n");
			return 2;
		}
		return 0;
	}
}

int ReadDev(TzhSerial* sport,unsigned char *szBuf,int nBufSize)
{
	//int readlen;
	bzero(szBuf,nBufSize);
	return read(sport->hCom, szBuf, nBufSize);
	//return readlen;
}
int WriteDev(TzhSerial* sport,unsigned char *szBuf,int nBufSize)
{
	int len=0;
	len = write(sport->hCom, szBuf, nBufSize);
	if(len == nBufSize)
	{
		return len;
	}
	else
	{
		tcflush(sport->hCom, TCOFLUSH);
		return -1;
	}
}
void CloseDev(TzhSerial* sport)
{
	sport->bOpen =0;
	close(sport->hCom);
}

int CheckStatus(TzhSerial* sport)
{
	int hCom;
	if(0==sport->bOpen){return 0;}

	hCom = open(sport->szComName,O_RDWR|O_NOCTTY|O_NDELAY);	
	if (-1 != sport->hCom)
	{
		//NoBlocking
		fcntl(sport->hCom,F_SETFL,FNDELAY);
		//Blocking
		//fcntl(sport->hCom,F_SETFL,0);
		if (!init_serial(hCom,sport->nbaudrate,8,'N',1))
		{
			sport->hCom=hCom;
		}
	}

	//printf("error=%s\n",strerror(errno));
	if(errno==EAGAIN)
	{return 1;}
	else
	{return 0;}
}
#endif
