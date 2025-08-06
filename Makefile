
.PHONY: all build run
all:
	cd ./build && make
build:
	cd ./build && cmake .. -G "MinGW Makefiles"
run:
	./build/tinyrenderer.exe
