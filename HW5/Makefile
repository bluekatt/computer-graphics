OS := $(shell uname)
ifeq ($(OS),Darwin)
	FLAGS := -lm -ldl -framework OpenGL -framework GLUT
else
	FLAGS := -lm -ldl -lglut -lGL -lGLU
endif

all: main.o shape.o vec3.o BSPTree.o
	c++ -o render vec3.o BSPtree.o shape.o main.cpp

main.o: main.cpp vec3.hpp BSPTree.hpp shape.hpp
	c++ -c main.cpp

shape.o: shape.cpp shape.hpp
	c++ -c shape.cpp

vec3.o: vec3.cpp vec3.hpp
	c++ -c vec3.cpp

BSPTree.o: BSPTree.cpp BSPTree.hpp
	c++ -c BSPTree.cpp

run:
	./render

clean:
	rm render main.o vec3.o BSPTree.o shape.o


