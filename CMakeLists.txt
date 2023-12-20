# ONEWIRE

set( srcs "onewire.c" "onewire_platform.c" "ds18x20.c" "ds1990x.c" "ds248x.c" )
set( include_dirs "." )
set( priv_include_dirs )
idf_build_get_property( cmakeREQUIRES cmakeREQUIRES )
set( requires hal_esp32 common endpoints ${cmakeREQUIRES} )
set( priv_requires commands main printfx rtos-support stringsX syslog systiming )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
