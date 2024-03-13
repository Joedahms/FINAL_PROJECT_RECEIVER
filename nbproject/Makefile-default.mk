#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=receiver_main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/receiver_main.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/receiver_main.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/receiver_main.p1

# Source Files
SOURCEFILES=receiver_main.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F4331
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/receiver_main.p1: receiver_main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/receiver_main.p1.d 
	@${RM} ${OBJECTDIR}/receiver_main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=none   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/receiver_main.p1 receiver_main.c 
	@-${MV} ${OBJECTDIR}/receiver_main.d ${OBJECTDIR}/receiver_main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/receiver_main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/receiver_main.p1: receiver_main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/receiver_main.p1.d 
	@${RM} ${OBJECTDIR}/receiver_main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/receiver_main.p1 receiver_main.c 
	@-${MV} ${OBJECTDIR}/receiver_main.d ${OBJECTDIR}/receiver_main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/receiver_main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../PIC18_pc_pwm.X/dist/default/debug/PIC18_pc_pwm.X.a ../PIC18_xl5.X/dist/default/debug/PIC18_xl5.X.a ../PIC18_spi.X/dist/default/debug/PIC18_spi.X.a ../PIC18_adc.X/dist/default/debug/PIC18_adc.X.a  
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=none  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -mrom=default,-1dc0-1fff -mram=default,-2f4-2ff,-f9c-f9c,-fd4-fd4,-fdb-fdf,-fe3-fe7,-feb-fef,-ffd-fff  $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ../PIC18_pc_pwm.X/dist/default/debug/PIC18_pc_pwm.X.a ../PIC18_xl5.X/dist/default/debug/PIC18_xl5.X.a ../PIC18_spi.X/dist/default/debug/PIC18_spi.X.a ../PIC18_adc.X/dist/default/debug/PIC18_adc.X.a 
	@${RM} ${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.hex 
	
else
${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../PIC18_pc_pwm.X/dist/default/production/PIC18_pc_pwm.X.a ../PIC18_xl5.X/dist/default/production/PIC18_xl5.X.a ../PIC18_spi.X/dist/default/production/PIC18_spi.X.a ../PIC18_adc.X/dist/default/production/PIC18_adc.X.a 
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=ignore -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mno-default-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/FINAL_PROJECT.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ../PIC18_pc_pwm.X/dist/default/production/PIC18_pc_pwm.X.a ../PIC18_xl5.X/dist/default/production/PIC18_xl5.X.a ../PIC18_spi.X/dist/default/production/PIC18_spi.X.a ../PIC18_adc.X/dist/default/production/PIC18_adc.X.a 
	
endif


# Subprojects
.build-subprojects:
	cd ../PIC18_pc_pwm.X && ${MAKE}  -f Makefile CONF=default
	cd ../PIC18_xl5.X && ${MAKE}  -f Makefile CONF=default
	cd ../PIC18_spi.X && ${MAKE}  -f Makefile CONF=default
	cd ../PIC18_adc.X && ${MAKE}  -f Makefile CONF=default


# Subprojects
.clean-subprojects:
	cd ../PIC18_pc_pwm.X && rm -rf "build/default" "dist/default"
	cd ../PIC18_xl5.X && rm -rf "build/default" "dist/default"
	cd ../PIC18_spi.X && rm -rf "build/default" "dist/default"
	cd ../PIC18_adc.X && rm -rf "build/default" "dist/default"

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
