idf_component_register(	SRCS ds18x20.c ds1990x.c ds2482.c 
						INCLUDE_DIRS . 
						REQUIRES common statistics onewire hal_esp32
						PRIV_REQUIRES endpoints syslog printf common systiming values hal_esp32 irmacos rules actuators pca9555
						)
