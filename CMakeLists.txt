idf_component_register(
	SRCS "onewire.c" "onewire_platform.c"
		"ds18x20.c" "ds18x20_cmds.c" 
		"ds1990x.c" "ds248x.c"
	INCLUDE_DIRS "."
	PRIV_REQUIRES commands irmacos main printfx rules stringsX systiming values
)
