fnc: multi.c
	gcc -pthread -DFORK -g -lncurses -Wall $< -o $@-fork
	gcc -pthread -DFORK -S -fverbose-asm -o ${<:.c=}-fork.s -lncurses -Wall $<
	gcc -pthread -g -lncurses -Wall $< -o $@-thread
	gcc -pthread -g -S -fverbose-asm -o ${<:.c=}-thread.s -lncurses -Wall $<
