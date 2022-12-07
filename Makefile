CC = iverilog
FLAGS = -Wall -Winfloop
EXE = lab6a.out
SRCS = control.v library.v cpu.v testbench.v
GTKW = waveform.gtkw

all:
	$(CC) $(FLAGS) -o $(EXE) $(SRCS)
	zip 03466_03470.zip $(SRCS) $(GTKW) constants.h Makefile
	vvp $(EXE)
	gtkwave tb_dumpfile.vcd $(GTKW)

clean:
	rm -rf $(EXE)
