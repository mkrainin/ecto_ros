all: build
	cd build && $(MAKE)

build:
	mkdir build && cd build && cmake ..

.PHONY : clean

clean:
	cd build && $(MAKE)

install:
	cd build && sudo checkinstall -y --nodoc --pkgname ecto-ros-0.1.0 make install << "ecto_ros 0.1.0\n\n"
