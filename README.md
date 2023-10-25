# FLEX

This repository contains architecture, compiler and design space exploration framework for FLEX, a novel CGRA architecture with flexible execution spectrum. FLEX introduces spatio-temporal vector dataflow execution for variable vector length(v). For each application kernel, FLEX DSE framework will perform exploration and will choose the best execution mode/ vector length(v). Please find the [paper](https://www.comp.nus.edu.sg/~tulika/FLEX_ICCAD.pdf) for more details.


### FLEX DSE Framework
![Screenshot from 2023-08-11 15-53-30](https://github.com/ecolab-nus/FLEX/assets/5539032/80609e42-0802-4d19-8b3e-ca8f7ff74567)


### Directory Structure
- [`Blockwise_Morpher_Mapper/`](Blockwise_Morpher_Mapper): FLEX Blockwise mapper implementation.
- [`Kernels/`](Kernels): Application kernels with architecture description in JSON.
- [`Microarchitecture/`](Microarchitecture): FLEX architecture RTL with scripts for synthesis, simulation and power estimation with Cadence tools.
- [`scripts/`](scripts): Scripts used for DSE.
- [`README.md`](README.md): This instructions file.
- [`install.sh`](install.sh): Script for building the tool.
- [`run_flex_dse.py`](run_flex_dse.py): Python script for performing automated design space exploration.


### Getting started

#### Prerequisites
Tested on C++ 8.5.0

Install JSON:
https://blog.csdn.net/jiaken2660/article/details/105155257


    git clone https://github.com/nlohmann/json.git
    mkdir build
    cd build
    cmake ../
    make -j2
    sudo make install

**_NOTE:-**
Please update all the tool scripts in the [`Microarchitecture/`](Microarchitecture) folder with your local tool setup. Find the following list for setup files.
- [`setproj.sh`](Microarchitecture/bin/setproj.sh): Contains cadence tool locations.
- [`makepaths.mk`](Microarchitecture/bin/makepaths.mk): Standard cell library specifics.
- [`Makefile`](Microarchitecture/Makefile): Change gen command according to your memory compilers.
- [`flex.sv`](Microarchitecture/rtl/flex.sv), [`configurator.sv`](Microarchitecture/rtl/configurator.sv): Change memories according to your generated memories.

#### Build on your machine
    source venv/bin/activate
    chmod +x install.sh
    ./install.sh

#### Run example DSE
python3 run_flex_dse.py <`Kernel name`> <`Max vector size supported by kernel`> <`Max vector size supported by hardware`>
    python3 run_flex_dse.py fir 32 8
    

# Citation

