/********************************************************

2008/10
作者:韩智鸿

han.zhihong@qq.com


sample:::::::::: 例子:::::::::::::::::

SERIAL sp;
int nLen;
BYTE btRecv[20];

//打开串口
if(0==zhSerialOpen(&sp,dev,115200))
{ printf("open serial success"); }
else
{ printf("open serial fail"); }

//发送
nLen=zhSerialSend(&sp,start,sizeof(start))
if(-1!=nLen)
{ printf("send ok,len=%d",nLen); }

//接收
nLen=zhSerialRead(&sp,btRecv,sizeof(btRecv));
if(nLen>0)
{
	printf("recvd->%d ",nLen);
}else if(-1==nLen)
{
	printf("disconnect");
}

//关闭
zhSerialClose(&sp);

*/

#ifndef __ZH_SERIAL_MAGE_H__
#define __ZH_SERIAL_MAGE_H__
/*////////////////////////////////////////////////////////////*/
#ifdef  __cplusplus
extern "C" {
#endif


#ifdef _WIN32
	#include <windows.h> 
	#include <tchar.h>
#else
	#include     <stdio.h>      /*标准输入输出定义*/
	#include     <stdlib.h>     /*标准函数库定义*/
	#include     <unistd.h>     /*Unix标准函数定义*/
	#include     <sys/types.h>  /**/
	#include     <sys/stat.h>   /**/
	#include     <fcntl.h>      /*文件控制定义*/
	#include     <termios.h>    /*PPSIX终端控制定义*/
	#include     <errno.h>      /*错误号定义*/
	#include     <sys/ioctl.h> //ioctl
#endif
#include <stdio.h>
#include <memory.h>

#ifndef BYTE
#define BYTE unsigned char
#endif

#ifdef _WIN32
	typedef struct _TzhSerial{
	  char bOpen;
	  char szComName[8];      // 端口名称
	  int nbaudrate;
	  HANDLE hCom;            // 端口打开句柄
	  OVERLAPPED overlappedR; // 读取结构体
	  OVERLAPPED overlappedW; // 发送结构体
	} TzhSerial;
	
	int SetDev(TzhSerial* sport);
	int OpenDev(TzhSerial* sport, char* szComName, DWORD baudrate);
	DWORD SendDevBuf( TzhSerial* sport, BYTE *btBuf, DWORD dwBytesWritten);
	DWORD ReadDevBuf( TzhSerial* sport, BYTE *btBuf, DWORD dwBytesRead);
	void CloseDev(TzhSerial* sport);
	int CheckStatus(TzhSerial* sport);

	#define SERIAL		TzhSerial
	/* 返回0连接成功 */
	#define zhSerialOpen	OpenDev
	#define zhSerialSend	SendDevBuf
	#define zhSerialRead	ReadDevBuf
	#define zhSerialClose	CloseDev
	/* 返回1串口正常 0串口不正常 */
	#define zhSerialCheck	CheckStatus
#else

	#define TIMEOUT_SEC(buflen,baud) (buflen*20/baud+2)
	#define TIMEOUT_USEC 0

	typedef struct _TzhSerial{
	  char bOpen;
	  char szComName[26]; //serial info
	  int hCom;   //handle
	  int nbaudrate;
	} TzhSerial;
	int init_serial (int fd,int nSpeed, int nBits, char nEvent, int nStop);
	int OpenDev(TzhSerial* sport,char *Dev ,int nbaudrate);
	int WriteDev(TzhSerial* sport,unsigned char *szBuf,int nBufSize);
	int ReadDev(TzhSerial* sport,unsigned char *szBuf,int nBufSize);
	void CloseDev(TzhSerial* sport);

	#define SERIAL		TzhSerial
	/* 返回0连接成功 */
	#define zhSerialOpen	OpenDev
	#define zhSerialSend	WriteDev
	#define zhSerialRead	ReadDev
	#define zhSerialClose	CloseDev
	/* 返回1串口正常 0串口不正常 */
	#define zhSerialCheck	CheckStatus
#endif
/*////////////////////////////////////////////////////////////*/

#ifdef  __cplusplus
}
#endif

#endif // 

