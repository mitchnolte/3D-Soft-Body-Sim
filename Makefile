TARGET  := build/Softbody
LIBS    := -lglfw3dll -lglew32 -lglu32 -lopengl32 -lm
OBjECTS := src/main.o src/shaders.o src/renderer.o src/camera.o src/mesh.o src/simulation.o src/vector.o src/rk4_solver.o src/soft_body.o

$(TARGET): $(OBjECTS)
	g++ -o $(TARGET) $(OBjECTS) $(LIBS)


main.o: shaders.h renderer.h simulation.h soft_body.h vector.h
shaders.o: shaders.h
renderer.o: renderer.h soft_body.h
camera.o: camera.h
mesh.o: mesh.h
simulation.o: simulation.h soft_body.h
soft_body.o: soft_body.h
vector.o: vector.h
rk4_solver.o: rk4_solver.h

renderer.h: camera.h mesh.h simulation.h soft_body.h
mesh.h: soft_body.h
soft_body.h: vector.h rk4_solver.h
rk4_solver.h: vector.h

clean:
	rm $(TARGET).exe $(OBjECTS)
