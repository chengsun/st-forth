sim : sim.cpp
	g++ -ggdb -Wall -o $@ $^ -lSDL

clean :
	rm sim
