compile:
	arduino-cli compile --fqbn arduino:avr:mega base_driver

upload:
	arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega base_driver

all: compile upload

clean:
	rm base_driver/*.hex base_driver/*.elf