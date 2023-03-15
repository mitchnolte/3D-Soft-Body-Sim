TARGET  := build/Softbody
LIBS    := -lglfw3dll -lglew32 -lglu32 -lopengl32 -lm
OBjECTS := src/main.o src/shaders.o src/renderer src/camera.o src/simulation.o src/soft_body.o src/vector.o src/rk4_solver.o

$(TARGET): $(OBjECTS)
	g++ -o $(TARGET) $(OBjECTS) $(LIBS)


main.o: shaders.h simulation.h soft_body.h vector.h
shaders.o: shaders.h
renderer.o: renderer.h soft_body.h
camera.o: camera.h
simulation.o: simulation.h soft_body.h
soft_body.o: soft_body.h
vector.o: vector.h
rk4_solver.o: rk4_solver.h

renderer.h: camera.h mesh.h simulation.h soft_body.h
mesh.h: soft_body.h
soft_body.h: vector.h
rk4_solver.h: vector.h

clean:
	rm $(TARGET).exe $(OBjECTS)
