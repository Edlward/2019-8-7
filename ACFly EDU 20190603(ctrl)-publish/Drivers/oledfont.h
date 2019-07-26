#ifndef __OLEDFONT_H
#define __OLEDFONT_H 

#include <stdbool.h>

//����
void FDraw_Clear( unsigned char Image[8][128] );
void FDraw_ClearLines( unsigned char Image[8][128] , unsigned char start , unsigned char count );

//Logo
bool FDraw_Logo( unsigned char Image[8][128] , unsigned char StRow , unsigned char StColumn );

//д��
bool FDraw_Font8x6( unsigned char Image[8][128] , const char* str , unsigned char StRow , unsigned char StColumn );
bool FDraw_Font16x8( unsigned char Image[8][128] , const char* str , unsigned char StRow , unsigned char StColumn );

//������
bool FDraw_TickCross8x6( unsigned char Image[8][128] , bool Tick, unsigned char StRow , unsigned char StColumn );
//����
bool FDraw_Point8x6( unsigned char Image[8][128] , unsigned char StRow , unsigned char StColumn );

/*����ֱ������*/
bool FDraw_VerticalProgressBar24x14( unsigned char Image[8][128] , float progress , unsigned char StRow , unsigned char StColumn );

#endif