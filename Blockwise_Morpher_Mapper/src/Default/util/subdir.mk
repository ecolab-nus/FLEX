################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../util/tinyxml2.cpp 

CPP_DEPS += \
./util/tinyxml2.d 

OBJS += \
./util/tinyxml2.o 


# Each subdirectory must supply rules for building sources it contributes
util/%.o: ../util/%.cpp util/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-util

clean-util:
	-$(RM) ./util/tinyxml2.d ./util/tinyxml2.o

.PHONY: clean-util

