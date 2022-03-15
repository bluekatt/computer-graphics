OS := $(shell uname)
ifeq ($(OS),Darwin)
	FLAGS := -lm -ldl -framework OpenGL -framework GLUT
else
	FLAGS := -lm -ldl -lglut -lGL -lGLU
endif

all:
	g++ -o render pick.cpp $(FLAGS)

run:
	./render

clean:
	rm render


