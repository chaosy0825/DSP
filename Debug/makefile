################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./Src/128x64.obj" \
"./Src/DSP2833x_ADC_cal.obj" \
"./Src/DSP2833x_Adc.obj" \
"./Src/DSP2833x_CodeStartBranch.obj" \
"./Src/DSP2833x_CpuTimers.obj" \
"./Src/DSP2833x_DefaultIsr.obj" \
"./Src/DSP2833x_EPwm.obj" \
"./Src/DSP2833x_GlobalVariableDefs.obj" \
"./Src/DSP2833x_Gpio.obj" \
"./Src/DSP2833x_MemCopy.obj" \
"./Src/DSP2833x_PieCtrl.obj" \
"./Src/DSP2833x_PieVect.obj" \
"./Src/DSP2833x_Sci.obj" \
"./Src/DSP2833x_Spi.obj" \
"./Src/DSP2833x_SysCtrl.obj" \
"./Src/DSP2833x_usDelay.obj" \
"./Src/Dis.obj" \
"./Src/Main.obj" \
"../Cmd/DSP2833x_Headers_nonBIOS.cmd" \
"../Cmd/F28335.cmd" \
"../Lib/IQmath_fpu32.lib" \
"../Lib/rts2800_fpu32.lib" \
$(GEN_CMDS__FLAG) \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include Cmd/subdir_vars.mk
-include Lib/subdir_vars.mk
-include Src/subdir_vars.mk
-include Cmd/subdir_rules.mk
-include Lib/subdir_rules.mk
-include Src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
2Axis-PMSM-idsp.taobao.com.out \

EXE_OUTPUTS__QUOTED += \
"2Axis-PMSM-idsp.taobao.com.out" \


# All Target
all: 2Axis-PMSM-idsp.taobao.com.out

# Tool invocations
2Axis-PMSM-idsp.taobao.com.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"D:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -g --diag_warning=225 --display_error_number --diag_wrap=off -z -m"2Axis-PMSM-idsp.taobao.com.map" --stack_size=0x400 --warn_sections -i"D:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/lib" -i"D:/study/MODEL/2Axis-PMSM-idsp.taobao.com/Lib" -i"D:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/include" --reread_libs --display_error_number --diag_wrap=off --xml_link_info="2Axis-PMSM-idsp.taobao.com_linkInfo.xml" --rom_model -o "2Axis-PMSM-idsp.taobao.com.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "Src\128x64.obj" "Src\DSP2833x_ADC_cal.obj" "Src\DSP2833x_Adc.obj" "Src\DSP2833x_CodeStartBranch.obj" "Src\DSP2833x_CpuTimers.obj" "Src\DSP2833x_DefaultIsr.obj" "Src\DSP2833x_EPwm.obj" "Src\DSP2833x_GlobalVariableDefs.obj" "Src\DSP2833x_Gpio.obj" "Src\DSP2833x_MemCopy.obj" "Src\DSP2833x_PieCtrl.obj" "Src\DSP2833x_PieVect.obj" "Src\DSP2833x_Sci.obj" "Src\DSP2833x_Spi.obj" "Src\DSP2833x_SysCtrl.obj" "Src\DSP2833x_usDelay.obj" "Src\Dis.obj" "Src\Main.obj" 
	-$(RM) "Src\128x64.d" "Src\DSP2833x_Adc.d" "Src\DSP2833x_CpuTimers.d" "Src\DSP2833x_DefaultIsr.d" "Src\DSP2833x_EPwm.d" "Src\DSP2833x_GlobalVariableDefs.d" "Src\DSP2833x_Gpio.d" "Src\DSP2833x_MemCopy.d" "Src\DSP2833x_PieCtrl.d" "Src\DSP2833x_PieVect.d" "Src\DSP2833x_Sci.d" "Src\DSP2833x_Spi.d" "Src\DSP2833x_SysCtrl.d" "Src\Dis.d" "Src\Main.d" 
	-$(RM) "Src\DSP2833x_ADC_cal.d" "Src\DSP2833x_CodeStartBranch.d" "Src\DSP2833x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

