SRC = ../src/main.c ../src/world.c ../src/vm.c ../src/physics.c ../src/render.c ../src/inflate.c ../src/pack_data.S ../src/gfx.c ../src/ui.c ../src/save.c ../src/font.c platform_host.c
CFLAGS = -O2 -g -Wall -Wno-unused-function -DHOST -DARENA_SIZE=150000 -I../src -Wa,-I../src

nd: $(SRC) ../src/*.h ../src/pack.bin
	gcc $(CFLAGS) -o nd $(SRC) -lm

clean:
	rm -f nd
