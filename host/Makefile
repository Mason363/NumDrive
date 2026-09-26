SRC = ../src/main.c ../src/world.c ../src/vm.c ../src/physics.c ../src/render.c ../src/inflate.c ../src/pack_data.S ../src/gfx.c ../src/ui.c ../src/save.c ../src/font.c platform_host.c
CFLAGS = -O2 -g -Wall -Wno-unused-function -DHOST -DARENA_SIZE=600000 -I../src -Wa,-I../src

nd: $(SRC) ../src/*.h ../src/pack.bin
	gcc $(CFLAGS) -o nd $(SRC) -lm

clean:
	rm -f nd

# 32-bit ARM build of the harness, run under qemu-arm (exact device memory layout)
nd_arm: $(SRC) ../src/*.h ../src/pack.bin
	arm-none-eabi-gcc -mcpu=cortex-a9 -mthumb -mfloat-abi=hard -mfpu=vfpv3-d16 --specs=rdimon.specs -O2 -DARMTEST \
	  -DHOST -DARENA_PROFILE -I../src -Wa,-I../src -o nd_arm $(SRC) -lm
