TARGET   := build/Softbody
OBJECTS  := src/main.o src/renderer.o src/camera.o src/mesh.o\
            src/simulation.o src/vector.o src/rk4_solver.o src/soft_body.o\
					  src/soft_cube.o src/rigid_body.o src/collision_data.o
LIBS     := -lglfw3dll -lglew32 -lglu32 -lopengl32 -lm -lwinmm
FLAGS    := -static-libgcc -static-libstdc++


$(TARGET): $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LIBS) $(FLAGS)

src/main.o: src/shaders.h src/renderer.h src/mesh.h src/simulation.h\
            src/soft_body.h src/soft_cube.h src/rigid_body.h
src/renderer.o:       src/renderer.h src/mesh.h src/camera.h src/soft_body.h
src/camera.o:         src/camera.h
src/mesh.o:           src/mesh.h src/vector.h src/soft_body.h
src/simulation.o:     src/simulation.h src/vector.h src/soft_body.h\
                      src/soft_cube.h src/rigid_body.h src/collision_data.h
src/vector.o:         src/vector.h
src/rk4_solver.o:     src/rk4_solver.h src/vector.h
src/soft_body.o:      src/soft_body.h src/rigid_body.h src/rk4_solver.h\
                      src/vector.h src/collision_data.h
src/soft_cube.o:      src/soft_cube.h src/vector.h src/soft_body.h src/mesh.h
src/rigid_body.o:     src/rigid_body.h src/soft_body.h src/vector.h\
                      src/mesh.h src/collision_data.h
src/collision_data.o: src/collision_data.h src/soft_body.h src/vector.h

clean:
	rm $(TARGET).exe $(OBJECTS)
