idf_component_register(
	SRCS "onewire.c" "onewire_platform.c" "ds18x20.c" "ds1990x.c" "ds248x.c"
	INCLUDE_DIRS "."
	PRIV_INCLUDE_DIRS
	REQUIRES
	PRIV_REQUIRES commands irmacos printf rules stringsX systiming values
)
