// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#pragma CODE_SECTION(Ad_CaiJi, "ramfuncs");
#pragma CODE_SECTION(JiSuan_Dl, "ramfuncs");

extern _iq ia;
extern _iq ib;
extern _iq ic;


#define ADC_usDELAY  5000L

int32 AD_BUF[8] = {0,0,0,0,0,0,0,0};
int16 *AdcResult = (int16 *)0x4001;	//read AD convst result 

Uint16 I_A=0;
Uint16 I_B=0;
Uint16 I_C=0;

float32 IA_MAX=0;
float32 IB_MAX=0;
float32 IC_MAX=0;

Uint16 I_A_2=0;
Uint16 I_B_2=0;
Uint16 I_C_2=0;

float32 IA_2_MAX=0;
float32 IB_2_MAX=0;
float32 IC_2_MAX=0;

int16 HALL_U=0; //U相霍尔的零点
int16 HALL_W=0; //W相霍尔的零点

int16 HALL_U_2=0; //U相霍尔的零点
int16 HALL_W_2=0; //W相霍尔的零点

float32 U_dc=0;
float32 U_dc_2=0;
Uint16 U_dc_dis=0;
Uint16 U_dc_dis_2=0;
Uint16 GuoliuZhi=0;
Uint16 GuoliuZhi_2=0;
Uint16 O_Current=0;
Uint16 O_Current_2=0;
Uint16 DC_ON_flag=0;
Uint16 DC_ON_OPEN=0;
Uint32 DC_ON_CNT=0;

Uint16 DC_ON_flag_2=0;
Uint16 DC_ON_OPEN_2=0;
Uint32 DC_ON_CNT_2=0;

void DelayUS(Uint16 N_US) //1US延时 
{
    Uint16 i=0;  

	for(i=0;i<N_US;i++)
	{
	  asm("	NOP");
	
	}

} 
 
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;




    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_RATE define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}


void Ad_CaiJi(void)
{	
    Uint16 i=0,j=0; 


        EALLOW; 
     GpioDataRegs.GPADAT.bit.GPIO23=0;//convst ad 
    asm("	NOP");
	asm("	NOP");
	asm("	NOP");
    asm("	NOP");
    asm("	NOP");
     GpioDataRegs.GPADAT.bit.GPIO23=1;//convst ad
     EDIS;
     DelayUS(1);

     while(GpioDataRegs.GPBDAT.bit.GPIO33==1)
     {
	 
	 j++;
	 if(j==200)
	 {break;}

     
     
     };

     EALLOW; 
     GpioDataRegs.GPBDAT.bit.GPIO48=0;//ad cs
     EDIS;

	

     for(i=0;i<8;i++)
     {
        EALLOW;
        GpioDataRegs.GPADAT.bit.GPIO27=0;//ad rd
        EDIS;
        

        AD_BUF[i] = (*AdcResult);
       AD_BUF[i]=(AD_BUF[i]*305)/2000;//这里是真实值 ad7606 单位 mv

        EALLOW;
        GpioDataRegs.GPADAT.bit.GPIO27=1;
        EDIS;


     }
     if(Run_PMSM==2)
     {    //轴1
          HALL_U=AD_BUF[3];
          HALL_W=AD_BUF[2];

     }

     if(Run_PMSM_2==2)
     {   
          //轴2
          HALL_U_2=AD_BUF[1];
          HALL_W_2=AD_BUF[0];

     }

    //轴1
     ic=_IQdiv(_IQ(AD_BUF[3])-_IQ(HALL_U),_IQ(E_Ding_DianLiu));
     ic=_IQmpy(ic, _IQ(0.04));
     ia=_IQdiv(_IQ(AD_BUF[2])-_IQ(HALL_W),_IQ(E_Ding_DianLiu));
     ia=_IQmpy(ia, _IQ(0.04));
     ib=-ia-ic;    // Compute phase-c current

    //轴2
     ic_2=_IQdiv(_IQ(AD_BUF[1])-_IQ(HALL_U_2),_IQ(E_Ding_DianLiu_2));
     ic_2=_IQmpy(ic_2, _IQ(0.04));
     ia_2=_IQdiv(_IQ(AD_BUF[0])-_IQ(HALL_W_2),_IQ(E_Ding_DianLiu_2));
     ia_2=_IQmpy(ia_2, _IQ(0.04));
     ib_2=-ia_2-ic_2;    // Compute phase-c current

     

      EALLOW; 
     GpioDataRegs.GPBDAT.bit.GPIO48=1;//ad cs
     EDIS;

}


void DC_Link(void)//计算母线电压
{
   
    U_dc=0.2951*AD_BUF[5];//轴1
   
    U_dc_dis=U_dc; 

    if((Run_PMSM==1)&&(DC_ON_flag==0))//运行中掉电 则停机
    {
        if(U_dc_dis<5)//执行停机
        {
                DC_ON_1; 
        DC_ON_flag=1;
        DC_ON_OPEN=2;


        }

    }

  if(U_dc_dis>370)//过压，执行停机
   {
       DC_ON_1; 
       DC_ON_flag=1;
       DC_ON_OPEN=3;

   }



U_dc_2=0.2951*AD_BUF[4];//轴2
    U_dc_dis_2=U_dc_2;

        if((Run_PMSM_2==1)&&(DC_ON_flag_2==0))//运行中掉电 则停机
    {
        if(U_dc_dis_2<5)//执行停机
        {
                DC_ON2_1;
        DC_ON_flag_2=1;
        DC_ON_OPEN_2=2;


        }

    }

  if(U_dc_dis_2>370)//过压，执行停机
   {
       DC_ON2_1; 
       DC_ON_flag_2=1;
       DC_ON_OPEN_2=3;

   } 

    

}

void JiSuan_Dl(void)//计算电流
{
       float32 IA,IB,IC,IA_2,IB_2,IC_2;
    static Uint16 i=0;
    static Uint16 j=0;
    
    IA=(AD_BUF[3]-HALL_U)*0.4;//放大10倍 
    IB=(AD_BUF[2]-HALL_W)*0.4;//放大10倍
    IC=0-IA-IB;
    if(IB<0.0)
    {IB=-IB;}
    if(IA<0.0)
    {IA=-IA;}
     if(IC<0.0)
    {IC=-IC;}

    if(IA>IA_MAX)
    {
        IA_MAX=IA;
    }
    if(IB>IB_MAX)
    {
        IB_MAX=IB;
    }
      if(IC>IC_MAX)
    {
        IC_MAX=IC;
    }
    i++;
    if(i==300)
    {
    I_A=IA_MAX/1.414;//有效值
    I_B=IB_MAX/1.414;//有效值
    I_C=IC_MAX/1.414;//有效值
    i=0;
    IB_MAX=0;
    IA_MAX=0;
    IC_MAX=0;

    if((I_A>GuoliuZhi)||(I_B>GuoliuZhi)||(I_C>GuoliuZhi))
    {   DC_ON_1;
        Pwm_EN_1;
        eva_close();//过流保护， 停机
        O_Current=1;
       
           Run_PMSM=2;
            

    }

    }
    if((I_A>100)||(I_B>100)||(I_C>100))
    {   DC_ON_1;
        Pwm_EN_1;
        eva_close();//过流保护， 停机
        O_Current=2;
       
           Run_PMSM=2;
            

    }

//轴2
    IA_2=(AD_BUF[1]-HALL_U_2)*0.4;//放大10倍 
    IB_2=(AD_BUF[0]-HALL_W_2)*0.4;//放大10倍
    IC_2=0-IA_2-IB_2;
    if(IB_2<0.0)
    {IB_2=-IB_2;}
    if(IA_2<0.0)
    {IA_2=-IA_2;}
     if(IC_2<0.0)
    {IC_2=-IC_2;}

    if(IA_2>IA_2_MAX)
    {
        IA_2_MAX=IA_2;
    }
    if(IB_2>IB_2_MAX)
    {
        IB_2_MAX=IB_2;
    }
      if(IC_2>IC_2_MAX)
    {
        IC_2_MAX=IC_2;
    }
    j++;
    if(j==300)
    {
    I_A_2=IA_2_MAX/1.414;//有效值
    I_B_2=IB_2_MAX/1.414;//有效值
    I_C_2=IC_2_MAX/1.414;//有效值
    j=0;
    IB_2_MAX=0;
    IA_2_MAX=0;
    IC_2_MAX=0;

    if((I_A_2>GuoliuZhi_2)||(I_B_2>GuoliuZhi_2)||(I_C_2>GuoliuZhi_2))
    {   DC_ON2_1;
        Pwm_EN2_1;
        eva_close_2();//过流保护， 停机
        O_Current_2=1;
       
           Run_PMSM_2=2;
            

    }
   
    }
    if((I_A_2>100)||(I_B_2>100)||(I_C_2>100))
    {  DC_ON2_1;
        Pwm_EN2_1;
        eva_close_2();//过流保护， 停机
        O_Current_2=2;
       
           Run_PMSM_2=2;
            

    }

} 


