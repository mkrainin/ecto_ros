all: build
	cd build && $(MAKE)

build:
	mkdir build && cd build && cmake ..

.PHONY : clean wipe

clean:
	cd build && $(MAKE) clean

wipe:
	rm -rf build

install: all
	cd build && checkinstall -y --nodoc --pkgname ecto-ros-0.1.0 $(MAKE) install << "ecto_ros 0.1.0\n\n"
