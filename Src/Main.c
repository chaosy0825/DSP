//*****************************************************************************************************
//Flash和RAM软件版本切换说明(程序默认为ram版本)
//
//一.切换为Flash烧写版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数取消注释
//2.将工程中的28335_RAM_lnk.cmd从工程中删除，添加CMD文件夹下的F28335.cmd文件，全编译一次即可烧写。
//
//二.切换为RAM在线仿真版本方法
//1.将主程序中的:MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
//               InitFlash();
//  两个函数注释掉
//2.将工程中的F28335.cmd从工程中删除，添加CMD文件夹下的28335_RAM_lnk.cmd文件，全编译一次即可。
//
//*****************************************************************************************************

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#pragma CODE_SECTION(EPWM_1_INT, "ramfuncs");

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void); 
interrupt void EPWM_1_INT(void);
interrupt void SCIBRX_ISR(void);
interrupt void INT3_ISR(void);
void Init_SiShu(void);



//*****************************************************************************************************
//全局变量定义与初始化
//***************************************************************************************************** 
float32 i=0;
float32 j=0;
float32 k=0;
Uint16 IsrTicker = 0;
Uint16 BackTicker = 0; //用于次数计数
Uint16 T1Period=0;     // T1定时器周期(Q0)
Uint16 T1Period_2=0;     // T1定时器周期(Q0)
Uint16 T3Period = 0;   
float32 Modulation=0.25;    // 调制比
float32 Modulation_2=0.25;    // 调制比
int16 MPeriod=0;
int32 Tmp=0;
int16 MPeriod_2=0;
int32 Tmp_2=0;

//:::::::::::::::::::::::::::位置环变量定义:::::::::::::::::::::::::::
long PlaceError=0,Place_now=0, Now_P=0,//圈数
              OutPreSat_Place=0;//位置变量值定义

long PlaceError_2=0,Place_now_2=0, Now_P_2=0,//圈数
              OutPreSat_Place_2=0;//位置变量值定义
Uint16 PlaceSetBit=0;  //位置设定标志位
Uint16 PlaceSetBit_2=0;  //位置设定标志位
int32 	PosCount = 0;
int32 	PosCount_2 = 0;

float32 MfuncF1=0;
float32 MfuncF2=0;
float32 MfuncF3=0;  
//===============转子初始位置定位=============================  
Uint16 LocationFlag=1;
Uint16 LocationEnd=0; 
Uint16 Position=1;
Uint16 LocationFlag_2=1;
Uint16 LocationEnd_2=0; 
Uint16 Position_2=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3; 
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;  

//===============DAC模拟===================================== 
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0; 

_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0; 
 
//===============转子速度计算===================================== 
Uint16 SpeedLoopPrescaler = 10;     // 速度环定标
Uint16 SpeedLoopCount = 1;          // 速度环计数  
_iq NewRawTheta=0;
_iq OldRawTheta=0; 
_iq SpeedRpm=0;                     //速度，单位：转/每分钟
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //速度，标幺值



Uint16 SpeedLoopPrescaler_2 = 10;     // 速度环定标
Uint16 SpeedLoopCount_2 = 1;          // 速度环计数  
_iq NewRawTheta_2=0;
_iq OldRawTheta_2=0; 
_iq SpeedRpm_2=0;                     //速度，单位：转/每分钟
Uint16 Hall_Fault_2=0;
_iq RawThetaTmp_2=0;
float32 SpeedRef_2=0;
_iq Speed_2=0;                        //速度，标幺值
 

//===============转子角度计算===================================
Uint16 DirectionQep=0;               //转子旋转方向
_iq RawTheta=0;
_iq OldRawThetaPos = 0;

Uint16 DirectionQep_2=0;               //转子旋转方向
_iq RawTheta_2=0;
_iq OldRawThetaPos_2 = 0;


_iq TotalPulse=0; 
_iq TotalPulse_2=0; 

_iq MechTheta = 0;                   //机械角度，单位：度
_iq ElecTheta = 0;                   //电气角度，单位：度
_iq	AnglePU=0;                       //角度标幺化
_iq	Cosine=0;
_iq	Sine=0;


_iq MechTheta_2 = 0;                   //机械角度，单位：度
_iq ElecTheta_2 = 0;                   //电气角度，单位：度
_iq	AnglePU_2=0;                       //角度标幺化
_iq	Cosine_2=0;
_iq	Sine_2=0;


Uint32 RS232_CNT=0;

//===============控制绕组电流计算============================ 
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ia_2=0;
_iq ib_2=0;
_iq ic_2=0;
_iq ialfa=0;
_iq ibeta=0; 
_iq id=0;
_iq iq=0; 

_iq ialfa_2=0;
_iq ibeta_2=0; 
_iq id_2=0;
_iq iq_2=0; 

//===============PI控制器参数计算============================ 
_iq ID_Given=0;
_iq ID_Ref=0;
_iq ID_Fdb=0;
_iq ID_Error=0;  

_iq ID_Up=0;
_iq ID_Up1=0;
_iq ID_Ui=0;
_iq ID_OutPreSat=0;
_iq ID_SatError=0;
_iq ID_OutMax=_IQ(1);
_iq ID_OutMin=_IQ(-1); 
_iq ID_Out=0;

_iq IQ_Given=0;
_iq IQ_Ref=0;
_iq IQ_Fdb=0;
_iq IQ_Error=0; 
 


_iq IQ_Up=0;
_iq IQ_Up1=0;
_iq IQ_Ui=0;
_iq IQ_OutPreSat=0;
_iq IQ_SatError=0;
_iq IQ_OutMax=_IQ(1);
_iq IQ_OutMin=_IQ(-1); 
_iq IQ_Out=0; 

_iq Speed_Given=_IQ(0.2); //速度给定    标幺值 0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_Ref=0;
_iq Speed_Fdb=0;
_iq Speed_Error=0; 
 

_iq Speed_Up=0;
_iq Speed_Ui=0;
_iq Speed_OutPreSat=0;
_iq Speed_SatError=0;
_iq Speed_OutMax=_IQ(0.99999);
_iq Speed_OutMin=-_IQ(0.99999);
_iq Speed_Out=0;  
Uint16 Speed_run=0;

_iq ID_2Given=0;
_iq ID_2Ref=0;
_iq ID_2Fdb=0;
_iq ID_2Error=0;  

_iq ID_2Up=0;
_iq ID_2Ui=0;
_iq ID_2OutPreSat=0;
_iq ID_2SatError=0;
_iq ID_2OutMax=_IQ(1);
_iq ID_2OutMin=_IQ(-1); 
_iq ID_2Out=0;

_iq IQ_2Given=0;
_iq IQ_2Ref=0;
_iq IQ_2Fdb=0;
_iq IQ_2Error=0; 
 


_iq IQ_2Up=0;
_iq IQ_2Ui=0;
_iq IQ_2OutPreSat=0;
_iq IQ_2SatError=0;
_iq IQ_2OutMax=_IQ(1);
_iq IQ_2OutMin=_IQ(-1); 
_iq IQ_2Out=0; 

_iq Speed_2Given=_IQ(0.2); //速度给定    标幺值 0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_2Ref=0;
_iq Speed_2Fdb=0;
_iq Speed_2Error=0; 
 

_iq Speed_2Up=0;
_iq Speed_2Ui=0;
_iq Speed_2OutPreSat=0;
_iq Speed_2SatError=0;
_iq Speed_2OutMax=_IQ(0.99999);
_iq Speed_2OutMin=-_IQ(0.99999);
_iq Speed_2Out=0;  
Uint16 Speed_2run=0;

//===============SVPWM计算==================================== 
Uint16 Sector = 0; 
_iq	Ualfa=0;  		
_iq	Ubeta=0;		
_iq	Ud=0;		
_iq	Uq=0;			
_iq	B0=0;			
_iq	B1=0;
_iq	B2=0;
_iq	X=0;
_iq	Y=0;
_iq	Z=0;
_iq	t1=0;
_iq	t2=0;
_iq	Ta=0;
_iq	Tb=0;
_iq	Tc=0;
_iq	MfuncD1=0;
_iq	MfuncD2=0;
_iq	MfuncD3=0; 


Uint16 Sector_2 = 0; 
_iq	Ualfa_2=0;  		
_iq	Ubeta_2=0;		
_iq	Ud_2=0;		
_iq	Uq_2=0;			
_iq	B0_2=0;			
_iq	B1_2=0;
_iq	B2_2=0;
_iq	X_2=0;
_iq	Y_2=0;
_iq	Z_2=0;
_iq	t1_2=0;
_iq	t2_2=0;
_iq	Ta_2=0;
_iq	Tb_2=0;
_iq	Tc_2=0;
_iq	MfuncD1_2=0;
_iq	MfuncD2_2=0;
_iq	MfuncD3_2=0; 
//===================================================================
Uint16 Run_PMSM=2;
Uint16 Run_PMSM_2=2;
_iq MechScaler=_IQ(0.0);          
_iq SpeedScaler=_IQ(0.00);        
Uint16 speed_give=0;
Uint16 HallAngle=0;
Uint16 BuChang=0;
int16 TotalCnt=0;
_iq RawCnt1=0;
_iq RawCnt2=0;
Uint16 ShangDian_Err=0;

_iq MechScaler_2=_IQ(0.0);          
_iq SpeedScaler_2=_IQ(0.00);        
Uint16 speed_give_2=0;
Uint16 HallAngle_2=0;
Uint16 BuChang_2=0;
int16 TotalCnt_2=0;
_iq RawCnt1_2=0;
_iq RawCnt2_2=0;
Uint16 ShangDian_Err_2=0;

Uint16 KEY_Type=1;                

//========================速度环PI参数=================================
_iq Speed_Kp=_IQ(8);
_iq Speed_Ki=_IQ(0.005);
//=====================================================================

//========================Q轴电流环PI参数==============================
_iq IQ_Kp=_IQ(0.3);
_iq IQ_Ki=_IQ(0.002);
//=====================================================================

//========================D轴电流环PI参数==============================
_iq ID_Kp=_IQ(0.3);
_iq ID_Ki=_IQ(0.002);
//=====================================================================

//轴2

//========================速度环PI参数=================================
_iq Speed_2Kp=_IQ(8);
_iq Speed_2Ki=_IQ(0.005);
//=====================================================================

//========================Q轴电流环PI参数==============================
_iq IQ_2Kp=_IQ(0.3);
_iq IQ_2Ki=_IQ(0.002);
//=====================================================================

//========================D轴电流环PI参数==============================
_iq ID_2Kp=_IQ(0.3);
_iq ID_2Ki=_IQ(0.002);
//=====================================================================




long PlaceSet=1000000;//位置环脉冲数
Uint16 PlaceEnable=0;//位置环使能  1 使能 ;  0 禁止

long PlaceSet_2=1000000;//位置环脉冲数
Uint16 PlaceEnable_2=0;//位置环使能  1 使能 ;  0 禁止

//=====================轴1电机 参数设置========================================
float32 E_Ding_DianLiu=8;        //设置电机的额定电流 单位A
_iq BaseRpm=_IQ(1500);              //设置电机额定转速
Uint16 BaseSpeed=1500;              //设置电机额定转速与BaseRpm相等
_iq PolePairs=_IQ(10);               //设置电机极对数
_iq LineEncoder=_IQ(2500);          //设置电机的增量式光电编码器线数
_iq Offset_Angle=_IQ(0.1);            //设置电机编码器Z相与电机A相的偏移角度，标么值
Uint16 ZhengFan=1;                  //设置PMSM电机正反转  1 正转 ;  0 反转

//=====================轴2电机 参数设置========================================
float32 E_Ding_DianLiu_2=4;        //设置电机的额定电流 单位A
_iq BaseRpm_2=_IQ(3000);              //设置电机额定转速
Uint16 BaseSpeed_2=3000;              //设置电机额定转速与BaseRpm相等
_iq PolePairs_2=_IQ(10);               //设置电机极对数
_iq LineEncoder_2=_IQ(2500);          //设置电机的增量式光电编码器线数
_iq Offset_Angle_2=_IQ(0);            //设置电机编码器Z相与电机A相的偏移角度，标么值
Uint16 ZhengFan_2=1;                  //设置PMSM电机正反转  1 正转 ;  0 反转

void main(void)
{


   InitSysCtrl();
 
   InitGpio(); 
   Pwm_EN_1;
   Pwm_EN2_1;

  
   DINT;
 
   InitPieCtrl(); 
   IER = 0x0000;
   IFR = 0x0000;
 
   InitPieVectTable();
 
   EALLOW;  // This is needed to write to EALLOW protected registers 
  // PieVectTable.TINT0 = &cpu_timer0_isr; 
   PieVectTable.EPWM1_INT=&EPWM_1_INT;
   PieVectTable.SCIRXINTB= &SCIBRX_ISR;   //设置串口B接受中断的中断向量
   PieVectTable.XINT3=&INT3_ISR;

   EDIS;    // This is needed to disable write to EALLOW protected registers
 
 // InitCpuTimers(); 
   InitSci_C();
   InitSci_B();
   InitSpi();

   
   
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
   InitFlash();
  
   InitEPwm_1_2_3();// 轴1 轴2 pwm初始化
   QEP_Init(); //轴1 轴2  qep 初始化
   Init_ch454();
   Init_SiShu();
   
 
	eva_close(); //轴1
    eva_close_2();//轴2
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   



    if(AD_BUF[5]<100)//轴1
   {
	Pwm_EN_0;//允许PWM使能
    
    }
   else
   {
    Pwm_EN_1;//禁止PWM使能
	ShangDian_Err=1;

   }


      if(AD_BUF[4]<100) //轴2 
   {
	Pwm_EN2_0;//允许PWM使能
    
    }
   else
   {
    Pwm_EN2_1;//禁止PWM使能
    ShangDian_Err_2=1;

	

   }
    
    
    DELAY_US(1000000);
    
   
   IER |= M_INT3;
   IER |= M_INT9;
   IER |= M_INT12;
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3
    
     Init_lcd();
 
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
    

   for(;;)
     {
        CPU_RUN();
        DC_Link();
        deal_key();
        LCD_DIS();   

   }

}

interrupt void EPWM_1_INT(void)
{
       _iq t_01,t_02,t_01_2,t_02_2;
       IPM_BaoHu();
    RS232_CNT=RS232_CNT+5;
    Show_time++;
    Show_time2++;
       if(Show_time2==1000)//1秒
    {
        Show_time2=0;
        lcd_dis_flag=1;


    }
    
   
  Read_key();
  Ad_CaiJi();  
   JiSuan_Dl();

    

if(Run_PMSM==1&&IPM_Fault==0)
{

	if(LocationFlag!=LocationEnd)
	{ 
            Modulation=0.95;
            HallAngle=0;
        if(GpioDataRegs.GPBDAT.bit.GPIO43) //W
        {
		HallAngle+=1;
        }
		if(GpioDataRegs.GPBDAT.bit.GPIO42)//V
		{
            HallAngle+=2;
            }
		
		if(GpioDataRegs.GPBDAT.bit.GPIO41)//U
		{
            HallAngle+=4;

        }
HallAngle=1;
        switch(HallAngle)
			{
				case 5:
					Position=PositionPhase60;
                    LocationFlag=LocationEnd;//定位结束
				EQep1Regs.QPOSCNT =BuChang*0+BuChang/2;  
                 OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				
				break;
				case 1:
					Position=PositionPhase360;
                     LocationFlag=LocationEnd;//定位结束
				EQep1Regs.QPOSCNT =BuChang*5+BuChang/2; 
               OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				break;
				case 3:
					Position=PositionPhase300;
                     LocationFlag=LocationEnd;//定位结束
				EQep1Regs.QPOSCNT =BuChang*4+BuChang/2; 
                 OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				break;
				case 2:
					Position=PositionPhase240;
                     LocationFlag=LocationEnd;//定位结束
				EQep1Regs.QPOSCNT =BuChang*3+BuChang/2; 
                  OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				break;
				case 6:
					Position=PositionPhase180;
                     LocationFlag=LocationEnd;//定位结束
				EQep1Regs.QPOSCNT =BuChang*2+BuChang/2; 
                  OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				break;
				case 4:
					Position=PositionPhase120;
                     LocationFlag=LocationEnd;//定位结束
				    EQep1Regs.QPOSCNT =BuChang*1+BuChang/2;  
                     OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
				break;
                default:
                     DC_ON_1;
                    Run_PMSM=2;//霍尔信号错误启动停止
                    eva_close();
                    Hall_Fault=1;
                    break;
			}
       

        
	} 
//=====================================================================================================
//初始位置定位结束，开始闭环控制
//=====================================================================================================
	else if(LocationFlag==LocationEnd)
	{  
		  
       
//======================================================================================================
//QEP角度计算
//====================================================================================================== 

// 旋转方向判定 
		DirectionQep = EQep1Regs.QEPSTS.bit.QDF;
		
        RawTheta = _IQ(EQep1Regs.QPOSCNT);
        
		if(DirectionQep ==1) //递增计数，代表顺时针；
		{
	
		 
              if((OldRawThetaPos> RawCnt2) && (RawTheta<_IQ(900)))
			{
				PosCount += TotalCnt;
			}

             
			Place_now= _IQtoF(RawTheta)+PosCount;
			OldRawThetaPos = RawTheta;
			
            
            
            
		}
		else if(DirectionQep ==0)//递减计数，代表逆时针
		{		 
             
			
              if((RawTheta> RawCnt1) && (OldRawThetaPos<_IQ(1000)))
			{
				PosCount -= TotalCnt;
			}
			Place_now = _IQtoF(RawTheta)+PosCount;
			OldRawThetaPos = RawTheta;
               
               
               
            
		} 
		MechTheta = _IQmpy(MechScaler,RawTheta);
         if(MechTheta>_IQ(360))
        {MechTheta=MechTheta-_IQ(360);}
         if(MechTheta<_IQ(-360))
        {MechTheta=MechTheta+_IQ(360);}
		ElecTheta = _IQmpy(PolePairs,MechTheta);   
	
		AnglePU=_IQdiv(ElecTheta,_IQ(360))+Offset_Angle;
	   	Sine = _IQsinPU(AnglePU);
	   	Cosine = _IQcosPU(AnglePU);    

      


//======================================================================================================
//QEP速度计算
//====================================================================================================== 

	    if (SpeedLoopCount>=SpeedLoopPrescaler)
	    {   
// 旋转方向判定 
			DirectionQep = EQep1Regs.QEPSTS.bit.QDF;			
 			NewRawTheta =_IQ(EQep1Regs.QPOSCNT);
// 计算机械角度
			if(DirectionQep ==1) //递增计数，
			{
               
                
				RawThetaTmp =  OldRawTheta-NewRawTheta ; 
				if(RawThetaTmp > _IQ(0))
				{
				 RawThetaTmp = RawThetaTmp - TotalPulse;  
				}
                
                
                
			}
			else if(DirectionQep ==0) //递减计数
			{

              
                
                RawThetaTmp =OldRawTheta-NewRawTheta; 
				if(RawThetaTmp < _IQ(0))
				{
				 RawThetaTmp = RawThetaTmp + TotalPulse;
				}
                
                
                
                
			}
			Speed = _IQmpy(RawThetaTmp,SpeedScaler);
			SpeedRpm = _IQmpy(BaseRpm,Speed);   				
			OldRawTheta = NewRawTheta;
               if(Speed<0)
			{speed_dis=_IQtoF(_IQmpy(Speed, _IQ(-100)));}
			else{
            speed_dis=_IQtoF(_IQmpy(Speed, _IQ(100)));}

		    SpeedLoopCount=1; 
			RawThetaTmp=0; 

//=================位置环控制===================================
  if(PlaceEnable ==1)
    {
        PlaceError = PlaceSet + Place_now;
  
		OutPreSat_Place = PlaceError;
		if((PlaceError<=10000)&&(PlaceError>=-10000))
        { 
           OutPreSat_Place = PlaceError/3; 
		}  

		
        if (OutPreSat_Place> 2000)
        {
          SpeedRef =  0.5;
        }
        else if (OutPreSat_Place< -2000)
        {
          SpeedRef =  -0.5;
        }
        else
        {
          SpeedRef = OutPreSat_Place/(float32)BaseSpeed;
        }

	   
   	}

//=================速度环PI===================================
			Speed_Ref=_IQ(SpeedRef);
			Speed_Fdb=Speed;

			Speed_Error=Speed_Ref - Speed_Fdb;

			Speed_Up=_IQmpy(Speed_Kp,Speed_Error);
			Speed_Ui=Speed_Ui + _IQmpy(Speed_Ki,Speed_Up) + _IQmpy(Speed_Ki,Speed_SatError);

			Speed_OutPreSat=Speed_Up+Speed_Ui;

			if(Speed_OutPreSat>Speed_OutMax)
				Speed_Out=Speed_OutMax;
			else if(Speed_OutPreSat<Speed_OutMin)
	 			Speed_Out=Speed_OutMin;
			else 
				Speed_Out=Speed_OutPreSat;  
	
			Speed_SatError=Speed_Out-Speed_OutPreSat;  

			IQ_Given=Speed_Out; 
            Speed_run=1;
		} 
	    else 
            {
                SpeedLoopCount++; 
	    }
        if(Speed_run==1)
        {

            
	    ialfa=ia;
		ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926));  

		id = _IQmpy(ialfa,Cosine) +_IQmpy(ibeta,Sine);
		iq = _IQmpy(ibeta,Cosine)- _IQmpy(ialfa,Sine) ; 

//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================  
		IQ_Ref=IQ_Given;
		IQ_Fdb=iq;

		IQ_Error=IQ_Ref-IQ_Fdb;

		IQ_Up=_IQmpy(IQ_Kp,IQ_Error);
		IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up) + _IQmpy(IQ_Ki,IQ_SatError);

		IQ_OutPreSat=IQ_Up+IQ_Ui;

		if(IQ_OutPreSat>IQ_OutMax)
			IQ_Out=IQ_OutMax;
		else if(IQ_OutPreSat<IQ_OutMin)
		 	IQ_Out=IQ_OutMin;
		else 
			IQ_Out=IQ_OutPreSat;  

		IQ_SatError=IQ_Out-IQ_OutPreSat;  

		Uq=IQ_Out;

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================  
		ID_Ref=ID_Given;
		ID_Fdb=id;

		ID_Error=ID_Ref-ID_Fdb;

		ID_Up=_IQmpy(ID_Kp,ID_Error);    
		ID_Ui=ID_Ui+_IQmpy(ID_Ki,ID_Up)+_IQmpy(ID_Ki,ID_SatError);   

		ID_OutPreSat=ID_Up+ID_Ui;    

		if(ID_OutPreSat>ID_OutMax)   
			ID_Out=ID_OutMax;
		else if(ID_OutPreSat<ID_OutMin)
		 	ID_Out=ID_OutMin;
		else 
			ID_Out=ID_OutPreSat;  

		ID_SatError=ID_Out-ID_OutPreSat;     

		Ud=ID_Out;

//======================================================================================================
//IPark变换
//====================================================================================================== 
		Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
		Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine); 
    
//======================================================================================================
//SVPWM实现
//====================================================================================================== 
        B0=Ubeta;
		B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 
		B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2

		Sector=0;
		if(B0>_IQ(0)) Sector =1;
		if(B1>_IQ(0)) Sector =Sector +2;
		if(B2>_IQ(0)) Sector =Sector +4; 

		X=Ubeta;//va
		Y=_IQmpy(_IQ(0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 vb
		Z=_IQmpy(_IQ(-0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2 vc

		
	 if(Sector==1)
		{
			t_01=Z;
			t_02=Y;

       if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Ta=Tb+t1;
			Tc=Ta+t2;
		}
		else if(Sector==2)
		{
			t_01=Y;
			t_02=-X;

             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tc=Ta+t1;
			Tb=Tc+t2;
 		} 
		else if(Sector==3)
 	    {
			t_01=-Z;
			t_02=X;

             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tb=Ta+t1;
			Tc=Tb+t2;	
	    } 
	    else if(Sector==4)
	    {
			t_01=-X;
			t_02=Z;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tb=Tc+t1;
			Ta=Tb+t2;
	    } 
	    else if(Sector==5)
	    {
			t_01=X;
			t_02=-Y;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Tc=Tb+t1;
	 		Ta=Tc+t2;
		}
		else if(Sector==6)
		{
			t_01=-Y;
			t_02=-Z;
             if((t_01+t_02)>_IQ(1))
       {
        t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
       t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

       }
       else
       { t1=t_01;
       t2=t_02;
       }

			Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
			Ta=Tc+t1;
			Tb=Ta+t2;
		} 

		MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));
		MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
		MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc)); 
//======================================================================================================
//EVA全比较器参数赋值，用于驱动电机
//====================================================================================================== 
	MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

	Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
	 EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

	Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
	 EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

	Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
	 EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2) 
	 
         
	}
	}




}
//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

if(Run_PMSM_2==1&&IPM_Fault_2==0)//轴2
{
    if(LocationFlag_2!=LocationEnd_2)
	{ 
            Modulation_2=0.95;
            HallAngle_2=0;
        if(GpioDataRegs.GPCDAT.bit.GPIO81) //W
        {
		HallAngle_2+=1;
        }
		if(GpioDataRegs.GPBDAT.bit.GPIO46)//V
		{
            HallAngle_2+=2;
            }
		
		if(GpioDataRegs.GPBDAT.bit.GPIO45)//U
		{
            HallAngle_2+=4;

        }
HallAngle_2=1;
        switch(HallAngle_2)
			{
				case 5:
					Position_2=PositionPhase60;
                    LocationFlag_2=LocationEnd_2;//定位结束
				EQep2Regs.QPOSCNT =BuChang_2*0+BuChang_2/2;  
                 OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				
				break;
				case 1:
					Position_2=PositionPhase360;
                     LocationFlag_2=LocationEnd_2;//定位结束
				EQep2Regs.QPOSCNT =BuChang_2*5+BuChang_2/2; 
               OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				break;
				case 3:
					Position_2=PositionPhase300;
                     LocationFlag_2=LocationEnd_2;//定位结束
				EQep2Regs.QPOSCNT =BuChang_2*4+BuChang_2/2; 
                 OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				break;
				case 2:
					Position_2=PositionPhase240;
                     LocationFlag_2=LocationEnd_2;//定位结束
				EQep2Regs.QPOSCNT =BuChang_2*3+BuChang_2/2; 
                  OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				break;
				case 6:
					Position_2=PositionPhase180;
                     LocationFlag_2=LocationEnd_2;//定位结束
				EQep2Regs.QPOSCNT =BuChang_2*2+BuChang_2/2; 
                  OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				break;
				case 4:
					Position_2=PositionPhase120;
                     LocationFlag_2=LocationEnd_2;//定位结束
				    EQep2Regs.QPOSCNT =BuChang_2*1+BuChang_2/2;  
                     OldRawTheta_2=_IQ(EQep2Regs.QPOSCNT);
				break;
                default:
                     DC_ON2_1;
                    Run_PMSM_2=2;//霍尔信号错误启动停止
                    eva_close_2();
                    Hall_Fault_2=1;
                    break;
			}
       

        
	} 

    	else if(LocationFlag_2==LocationEnd_2)
	{  
		  
       
//======================================================================================================
//QEP角度计算
//====================================================================================================== 

// 旋转方向判定 
		DirectionQep_2 = EQep2Regs.QEPSTS.bit.QDF;
		
        RawTheta_2 = _IQ(EQep2Regs.QPOSCNT);
        
		if(DirectionQep_2 ==1) //递增计数，代表顺时针；
		{
	
		 
              if((OldRawThetaPos_2> RawCnt2_2) && (RawTheta_2<_IQ(900)))
			{
				PosCount_2 += TotalCnt_2;
			}

             
			Place_now_2= _IQtoF(RawTheta_2)+PosCount_2;
			OldRawThetaPos_2 = RawTheta_2;
			
            
            
            
		}
		else if(DirectionQep_2 ==0)//递减计数，代表逆时针
		{		 
             
			
              if((RawTheta_2> RawCnt1_2) && (OldRawThetaPos_2<_IQ(1000)))
			{
				PosCount_2 -= TotalCnt_2;
			}
			Place_now_2 = _IQtoF(RawTheta_2)+PosCount_2;
			OldRawThetaPos_2 = RawTheta_2;
               
               
               
            
		} 
		MechTheta_2 = _IQmpy(MechScaler_2,RawTheta_2);
         if(MechTheta_2>_IQ(360))
        {MechTheta_2=MechTheta_2-_IQ(360);}
         if(MechTheta_2<_IQ(-360))
        {MechTheta_2=MechTheta_2+_IQ(360);}
		ElecTheta_2 = _IQmpy(PolePairs_2,MechTheta_2);   
	
		AnglePU_2=_IQdiv(ElecTheta_2,_IQ(360))+Offset_Angle_2;
	   	Sine_2 = _IQsinPU(AnglePU_2);
	   	Cosine_2 = _IQcosPU(AnglePU_2); 
//======================================================================================================
//QEP速度计算
//====================================================================================================== 

	    if (SpeedLoopCount_2>=SpeedLoopPrescaler_2)
	    {   
// 旋转方向判定 
			DirectionQep_2 = EQep2Regs.QEPSTS.bit.QDF;			
 			NewRawTheta_2 =_IQ(EQep2Regs.QPOSCNT);
// 计算机械角度
			if(DirectionQep_2 ==1) //递增计数，
			{
               
                
				RawThetaTmp_2 =  OldRawTheta_2-NewRawTheta_2 ; 
				if(RawThetaTmp_2 > _IQ(0))
				{
				 RawThetaTmp_2 = RawThetaTmp_2 - TotalPulse_2;  
				}
                
                
                
			}
			else if(DirectionQep_2 ==0) //递减计数
			{

              
                
                RawThetaTmp_2 =OldRawTheta_2-NewRawTheta_2; 
				if(RawThetaTmp_2 < _IQ(0))
				{
				 RawThetaTmp_2 = RawThetaTmp_2 + TotalPulse_2;
				}
                
                
                
                
			}
			Speed_2 = _IQmpy(RawThetaTmp_2,SpeedScaler_2);
			SpeedRpm_2 = _IQmpy(BaseRpm_2,Speed_2);   				
			OldRawTheta_2 = NewRawTheta_2;
               if(Speed_2<0)
			{Speed_dis_2=_IQtoF(_IQmpy(Speed_2, _IQ(-100)));}
			else{
            Speed_dis_2=_IQtoF(_IQmpy(Speed_2, _IQ(100)));}

		    SpeedLoopCount_2=1; 
			RawThetaTmp_2=0; 


//=================位置环控制===================================
  if(PlaceEnable_2 ==1)
    {
        PlaceError_2 = PlaceSet_2 + Place_now_2;
  
		OutPreSat_Place_2 = PlaceError_2;
		if((PlaceError_2<=10000)&&(PlaceError_2>=-10000))
        { 
           OutPreSat_Place_2 = PlaceError_2/3; 
		}  

		
        if (OutPreSat_Place_2> 2000)
        {
          SpeedRef_2 =  0.5;
        }
        else if (OutPreSat_Place_2< -2000)
        {
          SpeedRef_2 =  -0.5;
        }
        else
        {
          SpeedRef_2 = OutPreSat_Place_2/(float32)BaseSpeed_2;
        }

	   
   	}
  //=================速度环PI===================================
			Speed_2Ref=_IQ(SpeedRef);
			Speed_2Fdb=Speed_2;

			Speed_2Error=Speed_2Ref - Speed_2Fdb;

			Speed_2Up=_IQmpy(Speed_2Kp,Speed_2Error);
			Speed_2Ui=Speed_2Ui + _IQmpy(Speed_2Ki,Speed_2Up) + _IQmpy(Speed_2Ki,Speed_2SatError);

			Speed_2OutPreSat=Speed_2Up+Speed_2Ui;

			if(Speed_2OutPreSat>Speed_2OutMax)
				Speed_2Out=Speed_2OutMax;
			else if(Speed_2OutPreSat<Speed_2OutMin)
	 			Speed_2Out=Speed_2OutMin;
			else 
				Speed_2Out=Speed_2OutPreSat;  
	
			Speed_2SatError=Speed_2Out-Speed_2OutPreSat;  

			IQ_2Given=Speed_2Out; 
            Speed_2run=1;
		} 
	    else 
            {
                SpeedLoopCount_2++; 
	    }
         if(Speed_2run==1)
        {

            
	        ialfa_2=ia_2;
		ibeta_2=_IQmpy(ia_2,_IQ(0.57735026918963))+_IQmpy(ib_2,_IQ(1.15470053837926));  

		id_2 = _IQmpy(ialfa_2,Cosine_2) +_IQmpy(ibeta_2,Sine_2);
		iq_2 = _IQmpy(ibeta_2,Cosine_2)- _IQmpy(ialfa_2,Sine_2) ; 

//======================================================================================================
//IQ电流PID调节控制
//======================================================================================================  
		IQ_2Ref=IQ_2Given;
		IQ_2Fdb=iq_2;

		IQ_2Error=IQ_2Ref-IQ_2Fdb;

		IQ_2Up=_IQmpy(IQ_2Kp,IQ_2Error);
		IQ_2Ui=IQ_2Ui + _IQmpy(IQ_2Ki,IQ_2Up) + _IQmpy(IQ_2Ki,IQ_2SatError);

		IQ_2OutPreSat=IQ_2Up+IQ_2Ui;

		if(IQ_2OutPreSat>IQ_2OutMax)
			IQ_2Out=IQ_2OutMax;
		else if(IQ_2OutPreSat<IQ_2OutMin)
		 	IQ_2Out=IQ_2OutMin;
		else 
			IQ_2Out=IQ_2OutPreSat;  

		IQ_2SatError=IQ_2Out-IQ_2OutPreSat;  

		Uq_2=IQ_2Out;

//======================================================================================================
//ID电流PID调节控制
//======================================================================================================  
		ID_2Ref=ID_2Given;
		ID_2Fdb=id_2;

		ID_2Error=ID_2Ref-ID_2Fdb;

		ID_2Up=_IQmpy(ID_2Kp,ID_2Error);    
		ID_2Ui=ID_2Ui+_IQmpy(ID_2Ki,ID_2Up)+_IQmpy(ID_2Ki,ID_2SatError);   

		ID_2OutPreSat=ID_2Up+ID_2Ui;    

		if(ID_2OutPreSat>ID_2OutMax)   
			ID_2Out=ID_2OutMax;
		else if(ID_2OutPreSat<ID_2OutMin)
		 	ID_2Out=ID_2OutMin;
		else 
			ID_2Out=ID_2OutPreSat;  

		ID_2SatError=ID_2Out-ID_2OutPreSat;     

		Ud_2=ID_2Out;

//======================================================================================================
//IPark变换
//====================================================================================================== 
		Ualfa_2 = _IQmpy(Ud_2,Cosine_2) - _IQmpy(Uq_2,Sine_2);
		Ubeta_2 = _IQmpy(Uq_2,Cosine_2) + _IQmpy(Ud_2,Sine_2); 
//======================================================================================================
//SVPWM实现
//====================================================================================================== 
        B0_2=Ubeta_2;
		B1_2=_IQmpy(_IQ(0.8660254),Ualfa_2)- _IQmpy(_IQ(0.5),Ubeta_2);// 0.8660254 = sqrt(3)/2 
		B2_2=_IQmpy(_IQ(-0.8660254),Ualfa_2)- _IQmpy(_IQ(0.5),Ubeta_2); // 0.8660254 = sqrt(3)/2

        Sector_2=0;
		if(B0_2>_IQ(0)) Sector_2 =1;
		if(B1_2>_IQ(0)) Sector_2 =Sector_2 +2;
		if(B2_2>_IQ(0)) Sector_2 =Sector_2 +4; 

		X_2=Ubeta_2;//va
		Y_2=_IQmpy(_IQ(0.8660254),Ualfa_2)+ _IQmpy(_IQ(0.5),Ubeta_2);// 0.8660254 = sqrt(3)/2 vb
		Z_2=_IQmpy(_IQ(-0.8660254),Ualfa_2)+ _IQmpy(_IQ(0.5),Ubeta_2); // 0.8660254 = sqrt(3)/2 vc

		
	 if(Sector_2==1)
		{
			t_01_2=Z_2;
			t_02_2=Y_2;

       if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Tb_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Ta_2=Tb_2+t1_2;
			Tc_2=Ta_2+t2_2;
		}
		else if(Sector_2==2)
		{
			t_01_2=Y_2;
			t_02_2=-X_2;

             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Ta_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Tc_2=Ta_2+t1_2;
			Tb_2=Tc_2+t2_2;
 		} 
		else if(Sector_2==3)
 	    {
			t_01_2=-Z_2;
			t_02_2=X_2;

             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Ta_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Tb_2=Ta_2+t1_2;
			Tc_2=Tb_2+t2_2;	
	    } 
	    else if(Sector_2==4)
	    {
			t_01_2=-X_2;
			t_02_2=Z_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Tc_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Tb_2=Tc_2+t1_2;
			Ta_2=Tb_2+t2_2;
	    } 
	    else if(Sector_2==5)
	    {
			t_01_2=X_2;
			t_02_2=-Y_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Tb_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Tc_2=Tb_2+t1_2;
	 		Ta_2=Tc_2+t2_2;
		}
		else if(Sector_2==6)
		{
			t_01_2=-Y_2;
			t_02_2=-Z_2;
             if((t_01_2+t_02_2)>_IQ(1))
       {
        t1_2=_IQmpy(_IQdiv(t_01_2, (t_01_2+t_02_2)),_IQ(1));
       t2_2=_IQmpy(_IQdiv(t_02_2, (t_01_2+t_02_2)),_IQ(1));

       }
       else
       { t1_2=t_01_2;
       t2_2=t_02_2;
       }

			Tc_2=_IQmpy(_IQ(0.5),(_IQ(1)-t1_2-t2_2));
			Ta_2=Tc_2+t1_2;
			Tb_2=Ta_2+t2_2;
		} 

		MfuncD1_2=_IQmpy(_IQ(2),(_IQ(0.5)-Ta_2));
		MfuncD2_2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb_2));
		MfuncD3_2=_IQmpy(_IQ(2),(_IQ(0.5)-Tc_2)); 
//======================================================================================================
//EVA全比较器参数赋值，用于驱动电机
//====================================================================================================== 
	MPeriod_2 = (int16)(T1Period_2 * Modulation_2);              // Q0 = (Q0 * Q0)

	Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD1_2;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
	 EPwm4Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

	Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD2_2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
	 EPwm5Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

	Tmp_2 = (int32)MPeriod_2 * (int32)MfuncD3_2;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
	 EPwm6Regs.CMPA.half.CMPA = (int16)(Tmp_2>>16) + (int16)(T1Period_2>>1); // Q0 = (Q15->Q0)/2 + (Q0/2) 
	 
         
	}
	}



    
}



if(DC_ON_flag==1)
{
    if(Run_PMSM==0)
    {
    DC_ON_CNT++;
    if(DC_ON_CNT==15000)
    {
          if(U_dc_dis>15)//检测到有给动力电
        {
          Run_PMSM=1;
		eva_open(); 
        DC_ON_OPEN=0;
       
         }
          else
          {
            DC_ON_1;
            DC_ON_OPEN=1;//反馈主电没有
            Run_PMSM=2;


          }
        
        DC_ON_flag=0;
		DC_ON_CNT=0;
       

    }
    }
    else
    {
        if(U_dc_dis<10)//执行停机命令
        {
        eva_close();
        Run_PMSM=2;
        DC_ON_flag=0;
        DC_ON_CNT=0;




        }




    }
} 


if(DC_ON_flag_2==1)
{
    if(Run_PMSM_2==0)
    {
    DC_ON_CNT_2++;
    if(DC_ON_CNT_2==15000)
    {
          if(U_dc_dis_2>15)//检测到有给动力电
        {
          Run_PMSM_2=1;
		eva_open_2(); 
        DC_ON_OPEN_2=0;
       
         }
          else
          {
            DC_ON2_1;
            DC_ON_OPEN_2=1;//反馈主电没有
            Run_PMSM_2=2;


          }
        
        DC_ON_flag_2=0;
		DC_ON_CNT_2=0;
       

    }
    }
    else
    {
        if(U_dc_dis_2<10)//执行停机命令
        {
        eva_close_2();
        Run_PMSM_2=2;
        DC_ON_flag_2=0;
        DC_ON_CNT_2=0;




        }




    }
}






EPwm1Regs.ETCLR.bit.INT=1;//清除中断标志位
PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}



interrupt void SCIBRX_ISR(void)     // SCI-B
{
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}

void Init_SiShu(void)
{
    float32 temp;
    int16 tempcnt;

    float32 temp_2;
    int16 tempcnt_2;

 temp=2949120.0/LineEncoder;
 MechScaler=_IQ(temp);

 temp=491520000.0/LineEncoder;
 temp=temp/BaseSpeed;
 SpeedScaler=_IQ(temp);

  temp=LineEncoder/49152.0;
 temp=(temp*32768)/PolePairs;
 BuChang=temp;


 TotalCnt=_IQtoF(_IQmpy(LineEncoder, PolePairs));
 
 tempcnt=TotalCnt-1000;
 RawCnt1=_IQ(tempcnt);

 tempcnt=TotalCnt-100;
 RawCnt2=_IQ(tempcnt);




  temp_2=2949120.0/LineEncoder_2;
 MechScaler_2=_IQ(temp_2);

 temp_2=491520000.0/LineEncoder_2;
 temp_2=temp_2/BaseSpeed_2;
 SpeedScaler_2=_IQ(temp_2);

  temp_2=LineEncoder_2/49152.0;
 temp_2=(temp_2*32768)/PolePairs_2;
 BuChang_2=temp_2;


 TotalCnt_2=_IQtoF(_IQmpy(LineEncoder_2, _IQ(4)));
 
 tempcnt_2=TotalCnt_2-1000;
 RawCnt1_2=_IQ(tempcnt_2);

 tempcnt_2=TotalCnt_2-100;
 RawCnt2_2=_IQ(tempcnt_2);








 

 GuoliuZhi=15*E_Ding_DianLiu;
 GuoliuZhi_2=15*E_Ding_DianLiu_2;
 E_Ding_DianLiu=1.414*E_Ding_DianLiu;
 E_Ding_DianLiu_2=1.414*E_Ding_DianLiu_2;
 
 

 }

interrupt void INT3_ISR(void)
{ 
   
PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}



void eva_close_2(void)//轴2 关闭pwm输出
{
     EALLOW;
       //  1.3.5强制高，2.4.6有效 

       GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;   // Configure GPIO0 as EPWM4A
     
     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // Configure GPIO2 as EPWM5A
     
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;   // Configure GPIO4 as EPWM6A
    

    GpioCtrlRegs.GPADIR.bit.GPIO6=1;
    GpioCtrlRegs.GPADIR.bit.GPIO8=1;
    GpioCtrlRegs.GPADIR.bit.GPIO10=1;

    GpioDataRegs.GPASET.bit.GPIO6=1;
    GpioDataRegs.GPASET.bit.GPIO8=1;
    GpioDataRegs.GPASET.bit.GPIO10=1; 

      
       
       
   
    EPwm4Regs.CMPA.half.CMPA =3375; // 
EPwm5Regs.CMPA.half.CMPA = 3375; // 
EPwm6Regs.CMPA.half.CMPA = 3375; // 
   EDIS;
    Run_PMSM_2=2;
   LocationFlag_2=1;
   Speed_2Ui=0;
   ID_2Ui=0;
   IQ_2Ui=0;
   Position_2=1;
   j=0;
   Speed_dis_2=0;
   IQ_2Given=0;
   OldRawTheta_2=0;
   SpeedRef_2=0;
   speed_give_2=0;
   Modulation_2=0.25;    // 调制比
   O_Current_2=0;
   PosCount_2=0;
   OldRawThetaPos_2=0;
   Hall_Fault_2=0;
   Speed_2run=0;


}

//===========================================================================
// No more.
//===========================================================================
