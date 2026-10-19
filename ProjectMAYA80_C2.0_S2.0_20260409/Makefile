# Linux build of the CCS F1C_FLASH configuration (TMS320F28377S / F28379D).
# Compiler version matches .cproject: C2000 CGT 22.6.x LTS, COFF ABI.
# Windows-only include paths in the F1C_RAM configuration are not used here.

CGT_ROOT ?= /opt/ti/ti-cgt-c2000_22.6.1.LTS
CC       := $(CGT_ROOT)/bin/cl2000
OFD      := $(CGT_ROOT)/bin/ofd2000
NM       := $(CGT_ROOT)/bin/nm2000
DIS      := $(CGT_ROOT)/bin/dis2000

PROJ   := ProjectMAYA80_C2.0_S2.0_20260427
CONFIG := F1C_FLASH
OUTDIR := $(CONFIG)
TARGET := $(OUTDIR)/$(PROJ).out
MAP    := $(OUTDIR)/$(PROJ).map

INCLUDES := \
	--include_path=$(CGT_ROOT)/include \
	--include_path=libs/Compact_027 \
	--include_path=libs/driverlib \
	--include_path=libs \
	--include_path=APP/Include \
	--include_path=APP/Src \
	--include_path=BSW/BoardDrv/Include \
	--include_path=BSW/Serve/Include \
	--include_path=Algs/Include \
	--include_path=BSW/BoardDrv/FWLib/F2837xD_headers/include \
	--include_path=BSW/BoardDrv/FWLib/F2837xD_common/include \
	--include_path=libs/app_libs/motor_control/libs/resolver/v101/include \
	--include_path=libs/app_libs/motor_control/math_blocks/v4.3 \
	--include_path=libs/math/IQmath/v160/include

CFLAGS := \
	-v28 \
	-mt \
	--float_support=fpu32 \
	--cla_support=cla1 \
	--vcu_support=vcu2 \
	--tmu_support=tmu0 \
	--opt_level=2 \
	--opt_for_speed=4 \
	--fp_mode=relaxed \
	--advice:performance=all \
	--define=_DEBUG \
	--define=SMO \
	--define=_FLASH \
	--define=CPU1 \
	--define=LARGE_MODEL \
	--diag_warning=225 \
	--diag_wrap=off \
	--display_error_number \
	--abi=coffabi \
	-g \
	--quiet \
	$(INCLUDES)

LIB_PATHS := \
	--search_path=$(CGT_ROOT)/lib \
	--search_path=libs/app_libs/position_manager/v01_00_00_00/bissc/Float/lib \
	--search_path=libs/app_libs/position_manager/v01_00_00_00/endat22/Float/lib \
	--search_path=libs/dsp/VCU/v2_10_00_00/lib \
	--search_path=libs/math/IQmath/v160/lib \
	--search_path=$(CGT_ROOT)/include \
	--search_path=. \
	--search_path=libs/app_libs/motor_control/libs/resolver/v101/lib

LIBS := \
	--library=libc.a \
	--library=c28x_vcu0_library_fpu32.lib \
	--library=rts2800_fpu32.lib \
	--library=IQmath_fpu32.lib

CMD_FILES := \
	28377S_FLASH_lnk.cmd \
	Cmd/F2837xD_Headers_BIOS_cpu1.cmd \
	Cmd/F2837xD_Headers_clb_cpu1.cmd

C_SRCS := \
	APP/Src/Background_task.c \
	APP/Src/canHandle.c \
	APP/Src/fault_detect.c \
	APP/Src/HPMC_main_80.c \
	APP/Src/MotorCtrl.c \
	APP/Src/pid_gain_schedule.c \
	APP/Src/trip_zone.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Adc.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_CpuTimers.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_DefaultISR.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Dma.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_ECap.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Emif.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_EPwm.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_EQep.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Gpio.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_I2C.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Ipc.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Ipc_Driver.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Ipc_Driver_Lite.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Ipc_Driver_Util.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Mcbsp.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_PieCtrl.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_PieVect.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Sci.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_sci_io.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_sdfm_drivers.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Spi.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_struct.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_SysCtrl.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_TempSensorConv.c \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_Upp.c \
	BSW/BoardDrv/FWLib/F2837xD_headers/source/F2837xD_GlobalVariableDefs.c \
	BSW/BoardDrv/Src/ConfigPWM.c \
	BSW/BoardDrv/Src/spi_resolver.c \
	BSW/BoardDrv/Src/Startup.c \
	BSW/BoardDrv/Src/TLF35584.c \
	BSW/Serve/Src/ServAD2S.c \
	BSW/Serve/Src/ServECap.c \
	BSW/Serve/Src/Servi2c.c \
	BSW/Serve/Src/ServMotorCan.c \
	BSW/Serve/Src/ServMotorTemp.c \
	BSW/Serve/Src/ServNTCTemp.c \
	BSW/Serve/Src/ServSysTick.c \
	libs/driverlib/can.c \
	libs/driverlib/interrupt.c \
	libs/driverlib/sysctl.c \
	libs/driverlib/systick.c \
	libs/driverlib/uart.c \
	libs/driverlib/usb.c

ASM_SRCS := \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_CodeStartBranch.asm \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_DBGIER.asm \
	BSW/BoardDrv/FWLib/F2837xD_common/source/F2837xD_usDelay.asm

CLA_SRCS := \
	APP/Src/resolver.cla

OBJS := \
	$(patsubst %.c,$(OUTDIR)/%.obj,$(C_SRCS)) \
	$(patsubst %.asm,$(OUTDIR)/%.obj,$(ASM_SRCS)) \
	$(patsubst %.cla,$(OUTDIR)/%.obj,$(CLA_SRCS))

.PHONY: all clean verify

all: $(TARGET)

$(TARGET): $(OBJS) $(CMD_FILES)
	$(CC) -z \
		--stack_size=0x800 \
		--map_file=$(MAP) \
		--output_file=$@ \
		--xml_link_info=$(OUTDIR)/$(PROJ)_linkInfo.xml \
		--entry_point=_c_int00 \
		--define=CLA_SCRATCHPAD_SIZE=0x200 \
		--define=CLA_C=1 \
		--diag_wrap=off \
		--display_error_number \
		--quiet \
		$(LIB_PATHS) \
		$(OBJS) \
		$(CMD_FILES) \
		$(LIBS)
	@echo "Built $(TARGET)"

$(OUTDIR)/%.obj: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) --asm_directory=$(@D) --output_file=$@ $<

$(OUTDIR)/%.obj: %.asm
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) --asm_directory=$(@D) --output_file=$@ $<

$(OUTDIR)/%.obj: %.cla
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) --asm_directory=$(@D) --output_file=$@ $<

verify: $(TARGET)
	@test -x $(CC)
	@$(CC) -version
	@ls -lh $(TARGET) $(MAP)
	@echo "---- object file ----"
	@$(OFD) $(TARGET) | grep -E 'File Type:|Machine:|Entry Point:|CPU Generation:|Linker Version:'
	@echo "---- linker entry ----"
	@grep -F 'ENTRY POINT SYMBOL' $(MAP)
	@echo "---- motor-control symbols ----"
	@$(NM) $(TARGET) | grep -E 'MotorControlISR| _main|code_start| _c_int00|Cla_AD2SB_Read'
	@echo "---- MotorControlISR disassembly ----"
	@$(DIS) -q $(TARGET) | awk 'BEGIN{found=0; n=0} /_MotorControlISR:/{found=1} found{print; n++} n>=24{exit} END{if(!found) exit 1}' 

clean:
	rm -rf $(OUTDIR)
