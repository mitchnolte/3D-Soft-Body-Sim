TARGET  := build/Softbody
# OBjECTS := main.o shaders.o
# LIBS    := -lglfw3dll -lglew32 -lglu32 -lopengl -lm
OBjECTS := src/main.o src/vector.o src/soft_body.o
LIBS    :=

$(TARGET): $(OBjECTS)
	g++ -o $(TARGET) $(OBjECTS) $(LIBS)


main.o: soft_body.h vector.h
vector.o: vector.h
soft_body.o: soft_body.h
soft_body.h: vector.h

clean:
	rm $(TARGET).exe $(OBjECTS)
