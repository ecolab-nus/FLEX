################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../lisa/LISAController.cpp \
../lisa/LISADFG.cpp \
../lisa/LISAMapper.cpp \
../lisa/LISASchedule.cpp \
../lisa/gnn.cpp 

CPP_DEPS += \
./lisa/LISAController.d \
./lisa/LISADFG.d \
./lisa/LISAMapper.d \
./lisa/LISASchedule.d \
./lisa/gnn.d 

OBJS += \
./lisa/LISAController.o \
./lisa/LISADFG.o \
./lisa/LISAMapper.o \
./lisa/LISASchedule.o \
./lisa/gnn.o 


# Each subdirectory must supply rules for building sources it contributes
lisa/%.o: ../lisa/%.cpp lisa/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-lisa

clean-lisa:
	-$(RM) ./lisa/LISAController.d ./lisa/LISAController.o ./lisa/LISADFG.d ./lisa/LISADFG.o ./lisa/LISAMapper.d ./lisa/LISAMapper.o ./lisa/LISASchedule.d ./lisa/LISASchedule.o ./lisa/gnn.d ./lisa/gnn.o

.PHONY: clean-lisa

