#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/src/Battery_Magagement_System/BMS.o \
	${OBJECTDIR}/src/Genasun_Watt_Sensor/MPPT.o \
	${OBJECTDIR}/src/Steering_Wheel/Serial.o \
	${OBJECTDIR}/src/Wrappers/canbus.o \
	${OBJECTDIR}/test/main_read_bms.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-pthread -std=c++11 -g -w
CXXFLAGS=-pthread -std=c++11 -g -w

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/solarboattwente

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/solarboattwente: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/solarboattwente ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/src/Battery_Magagement_System/BMS.o: src/Battery_Magagement_System/BMS.cpp
	${MKDIR} -p ${OBJECTDIR}/src/Battery_Magagement_System
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Battery_Magagement_System/BMS.o src/Battery_Magagement_System/BMS.cpp

${OBJECTDIR}/src/Genasun_Watt_Sensor/MPPT.o: src/Genasun_Watt_Sensor/MPPT.cpp
	${MKDIR} -p ${OBJECTDIR}/src/Genasun_Watt_Sensor
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Genasun_Watt_Sensor/MPPT.o src/Genasun_Watt_Sensor/MPPT.cpp

${OBJECTDIR}/src/Steering_Wheel/Serial.o: src/Steering_Wheel/Serial.cpp
	${MKDIR} -p ${OBJECTDIR}/src/Steering_Wheel
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Steering_Wheel/Serial.o src/Steering_Wheel/Serial.cpp

${OBJECTDIR}/src/Wrappers/canbus.o: src/Wrappers/canbus.cpp
	${MKDIR} -p ${OBJECTDIR}/src/Wrappers
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/src/Wrappers/canbus.o src/Wrappers/canbus.cpp

${OBJECTDIR}/test/main_read_bms.o: test/main_read_bms.cpp
	${MKDIR} -p ${OBJECTDIR}/test
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/test/main_read_bms.o test/main_read_bms.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
