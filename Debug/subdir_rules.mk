################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2806x_Adc.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_Adc.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CodeStartBranch.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CpuTimers.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_CpuTimers.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_DefaultIsr.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_DefaultIsr.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_EPwm.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_EPwm.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_GlobalVariableDefs.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/source/F2806x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieCtrl.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieVect.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_PieVect.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_Sci.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_Sci.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_SysCtrl.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_SysCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_TempSensorConv.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_TempSensorConv.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_usDelay.obj: C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/source/F2806x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/ti/ccs1110/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/math/FPUfastRTS/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/libraries/control/DCL/c28/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/libraries/utilities/math_blocks/include/v4.3" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/common/include" --include_path="C:/ti/c2000/C2000Ware_MotorControl_SDK_3_03_00_00/c2000ware/device_support/f2806x/headers/include" --define=_DEBUG --define=_FLASH --define=LARGE_MODEL -g --diag_suppress=552 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


