px4_add_library(drivers_board
	${PX4_SOURCE_DIR}/src/drivers/boards/common/board_identity.c # TODO: this is horrible and should be fixed
	sitl_led.c
)