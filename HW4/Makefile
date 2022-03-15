OS := $(shell uname)
ifeq ($(OS),Darwin)
	FLAGS := -lm -ldl -framework OpenGL -framework GLUT
else
	FLAGS := -lm -ldl -lglut -lGL -lGLU
endif

all: main.o vec3.o BSPTree.o 
	g++ -o render vec3.o BSPtree.o main.cpp $(FLAGS)

main.o: main.cpp vec3.hpp BSPTree.hpp
	g++ -c main.cpp

vec3.o: vec3.cpp vec3.hpp
	g++ -c vec3.cpp

BSPTree.o: BSPTree.cpp BSPTree.hpp
	g++ -c BSPTree.cpp

run:
	./render

clean:
	rm render main.o vec3.o BSPTree.o


