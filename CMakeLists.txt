# ONEWIRE
set( srcs "onewire.c" "onewire_platform.c" "ds18x20.c" "ds1990x.c" "ds248x.c" )
set( include_dirs "." )
set( requires common endpoints hal_esp32 )
set( priv_requires commands irmacos printfx rtos-support stringsX syslog systiming )
idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})
