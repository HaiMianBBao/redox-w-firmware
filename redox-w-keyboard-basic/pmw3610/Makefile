build:
	cmake -S . -B build -G "Unix Makefiles"
	cd build && make -j8

clean:
	rm -rf build

menuconfig:
	menuconfig

.PHONY:build clean menuconfig