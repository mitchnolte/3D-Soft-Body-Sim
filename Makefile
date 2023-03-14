TARGET  := build/Softbody
LIBS    := -lglfw3dll -lglew32 -lglu32 -lopengl32 -lm
OBjECTS := src/main.o src/simulation.o src/soft_body.o src/vector.o

$(TARGET): $(OBjECTS)
	g++ -o $(TARGET) $(OBjECTS) $(LIBS)


main.o: soft_body.h vector.h
simulation.o: soft_body.h
soft_body.o: soft_body.h
vector.o: vector.h
soft_body.h: vector.h

clean:
	rm $(TARGET).exe $(OBjECTS)
