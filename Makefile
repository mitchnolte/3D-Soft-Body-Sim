TARGET   := build/Softbody
LIBS     := -lglfw3dll -lglew32 -lglu32 -lopengl32 -lm
OBjECTS  := src/main.o src/shaders.o src/renderer.o src/camera.o src/mesh.o\
            src/simulation.o src/vector.o src/rk4_solver.o src/soft_body.o\
					  src/soft_body_factory.o src/rigid_body.o
CXXFLAGS := -std=c++17


$(TARGET): $(OBjECTS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $(TARGET) $(OBjECTS) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c -o $@ $<

# Dependencies
src/main.o: src/shaders.h src/renderer.h src/simulation.h src/soft_body_factory.h\
            src/rigid_body.h src/vector.h
src/shaders.o: src/shaders.h
src/renderer.o: src/renderer.h src/soft_body.h
src/camera.o: src/camera.h
src/mesh.o: src/mesh.h
src/simulation.o: src/simulation.h src/soft_body.h src/collision_data.h
src/vector.o: src/vector.h
src/rk4_solver.o: src/rk4_solver.h
src/soft_body.o: src/soft_body.h
src/soft_body_factory.o: src/soft_body_factory.h
src/rigid_body.o: src/rigid_body.h

src/renderer.h: src/camera.h src/mesh.h src/simulation.h src/soft_body.h
src/mesh.h: src/vector.h src/soft_body.h
src/simulation.h: src/soft_body.h src/rigid_body.h
src/rk4_solver.h: src/vector.h
src/soft_body.h: src/vector.h src/rk4_solver.h
src/soft_body_factory.h: src/soft_body.h src/mesh.h
src/rigid_body.h: src/soft_body.h src/collision_data.h

clean:
	rm $(TARGET).exe $(OBjECTS)
