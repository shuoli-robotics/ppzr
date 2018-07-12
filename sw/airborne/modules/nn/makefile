CC = gcc
FLAGS = -Wall -std=c89 -pedantic -lm
FILES = main.c nn.c nn_params.c
OUT_EXE = out

build: $(FILES)
	$(CC) $(FLAGS) $(FILES) -o $(OUT_EXE)

clean:
	rm -f $(OUT_EXE)
