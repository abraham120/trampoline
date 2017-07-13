|=-----=[ apm_demo example ]=-----=|

This application is a simple periodic example wich toggles the blue led of the board.
Have a look into "apm_demo.oil" file.

The system is based scheduled with a 1ms SysTick "SystemCounter".

Configure the application with

`
goil --target=cortex/armv7em/s32k144/dk_autosar --templates=../../../../../../goil/templates/ apm_demo.oil
`
