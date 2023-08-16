################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../arch/CGRA.cpp \
../arch/DataPath.cpp \
../arch/FU.cpp \
../arch/Module.cpp \
../arch/PE.cpp \
../arch/Port.cpp \
../arch/RegFile.cpp 

CPP_DEPS += \
./arch/CGRA.d \
./arch/DataPath.d \
./arch/FU.d \
./arch/Module.d \
./arch/PE.d \
./arch/Port.d \
./arch/RegFile.d 

OBJS += \
./arch/CGRA.o \
./arch/DataPath.o \
./arch/FU.o \
./arch/Module.o \
./arch/PE.o \
./arch/Port.o \
./arch/RegFile.o 


# Each subdirectory must supply rules for building sources it contributes
arch/%.o: ../arch/%.cpp arch/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-arch

clean-arch:
	-$(RM) ./arch/CGRA.d ./arch/CGRA.o ./arch/DataPath.d ./arch/DataPath.o ./arch/FU.d ./arch/FU.o ./arch/Module.d ./arch/Module.o ./arch/PE.d ./arch/PE.o ./arch/Port.d ./arch/Port.o ./arch/RegFile.d ./arch/RegFile.o

.PHONY: clean-arch

