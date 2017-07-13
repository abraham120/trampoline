# Bunch of running examples for S32K144

This directory provides a set of examples ready for use on a 
dk_autosar board from DRIMAES.

## Prerequisite

### Compile

You need a compiler for your plateform.
Trampoline has been previously compiled with industrial compilers and GNU GCC.
The examples have all been compiled with :

`
(GNU Tools for ARM Embedded Processors) 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 227977]
from 
https://launchpad.net/gcc-arm-embedded
`

We present hereafter the successive steps to follow to get the first example [ apm_demo ] run.

1 - Step into the example directory

    > cd examples/cortex/armv7em/s32k144/dk_autosar/apm_demo

2 - Generate all source files from a configuration with the command :

    > goil --target=cortex/armv7em/s32k144/dk_autosar -v --templates=../../../../../../goil/templates/ apm_demo.oil

3 - Compile

    > ./make.py

You are now ready to use apm_demo_exe

## Tips

## The examples

### apm_demo

This example for apm demo.
