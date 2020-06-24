#idf_component_register(	SRCS onewire.c onewire_platform.c ds18x20.c ds1990x.c ds2482.c ds2484.c ds248x.c
idf_component_register(	SRCS onewire.c onewire_platform.c ds18x20.c ds1990x.c ds248x.c
						INCLUDE_DIRS . 
						REQUIRES common hal_esp32 statistics
						PRIV_REQUIRES actuators commands common endpoints hal_esp32 irmacos pca9555 printf rules syslog systiming values
						)
