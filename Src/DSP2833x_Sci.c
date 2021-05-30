// TI File $Revision: /main/2 $
// Checkin $Date: March 1, 2007   16:06:07 $
//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
Uint16 speed_dis=0;
 
Uint16 Speed_dis_2=0;
void InitSci_B(void)//232
{    
    
 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback 
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK, 
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003; 
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
    ScibRegs.SCIHBAUD    =0x0001;//9600
    ScibRegs.SCILBAUD    =0x00e7;
	ScibRegs.SCICCR.bit.LOOPBKENA =0; // disable loop back  
	ScibRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset 
	ScibRegs.SCIFFRX.all=0x204f;
    ScibRegs.SCIFFCT.all=0x0;
	ScibRegs.SCIFFRX.bit.RXFFIENA = 1;
    
}

void InitSci_C(void)//485
{    
    
 	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback 
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK, 
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003; 
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
    ScicRegs.SCIHBAUD    =0x0001;//9600
    ScicRegs.SCILBAUD    =0x00e7;
	ScicRegs.SCICCR.bit.LOOPBKENA =0; // disable loop back  
	ScicRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset 
	ScicRegs.SCIFFRX.all=0x204f;
    ScicRegs.SCIFFCT.all=0x0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1;
    
}

        

void ShowDisp(void)       //更新显示
{
	static unsigned int i=0;
     if(RS232_CNT>400)
    {
	switch(i)
	{
	case 0://显示故障
		ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        if(IPM_Fault==1)
        {
		ScibRegs.SCITXBUF =12;//故障
        }
        else if(O_Current==1)
        {
            ScibRegs.SCITXBUF =13;//过流

        }
        else
        {
            ScibRegs.SCITXBUF =22;//正常
        }
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        i++;
		break;
	case 1://显示A相电流
	    ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		ScibRegs.SCITXBUF=I_A;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		i++;
		break;
	case 2://显示运行速度
	    ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		ScibRegs.SCITXBUF=speed_dis;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        i++;
		break;
	case 3://显示b相电流
	    ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
	    ScibRegs.SCITXBUF=I_B;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		i++;
		break;
    case 4://显示母线电压
        ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
	    ScibRegs.SCITXBUF=U_dc_dis>>8;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        ScibRegs.SCITXBUF=U_dc_dis;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		i++;
		break;
    case 5://显示给定速度百分比
        ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        ScibRegs.SCITXBUF =speed_give;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        i++;
        break;
           case 6://显示
        ScibRegs.SCITXBUF =i;  
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
	    ScibRegs.SCITXBUF=BaseSpeed>>8;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
        ScibRegs.SCITXBUF=BaseSpeed;
        while(ScibRegs.SCICTL2.bit.TXEMPTY!=1);
		i++;
		break;
	default:
		i=0;
		break;
	}
    RS232_CNT=0;
     }
	
	
}

 

 
 

	
//===========================================================================
// End of file.
//===========================================================================
