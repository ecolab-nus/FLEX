################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../mapper/HeuristicMapper.cpp \
../mapper/PathFinderMapper.cpp \
../mapper/SimulatedAnnealingMapper.cpp 

CPP_DEPS += \
./mapper/HeuristicMapper.d \
./mapper/PathFinderMapper.d \
./mapper/SimulatedAnnealingMapper.d 

OBJS += \
./mapper/HeuristicMapper.o \
./mapper/PathFinderMapper.o \
./mapper/SimulatedAnnealingMapper.o 


# Each subdirectory must supply rules for building sources it contributes
mapper/%.o: ../mapper/%.cpp mapper/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-mapper

clean-mapper:
	-$(RM) ./mapper/HeuristicMapper.d ./mapper/HeuristicMapper.o ./mapper/PathFinderMapper.d ./mapper/PathFinderMapper.o ./mapper/SimulatedAnnealingMapper.d ./mapper/SimulatedAnnealingMapper.o

.PHONY: clean-mapper

