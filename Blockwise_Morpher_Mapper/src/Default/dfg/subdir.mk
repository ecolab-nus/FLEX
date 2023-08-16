################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../dfg/DFG.cpp \
../dfg/DFGEdge.cpp \
../dfg/DFGNode.cpp 

CPP_DEPS += \
./dfg/DFG.d \
./dfg/DFGEdge.d \
./dfg/DFGNode.d 

OBJS += \
./dfg/DFG.o \
./dfg/DFGEdge.o \
./dfg/DFGNode.o 


# Each subdirectory must supply rules for building sources it contributes
dfg/%.o: ../dfg/%.cpp dfg/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-dfg

clean-dfg:
	-$(RM) ./dfg/DFG.d ./dfg/DFG.o ./dfg/DFGEdge.d ./dfg/DFGEdge.o ./dfg/DFGNode.d ./dfg/DFGNode.o

.PHONY: clean-dfg

