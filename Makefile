default:
	[ -f extlibs/sandbox/lib/libsandbox.a ] \
		|| (git submodule update --init \
			&& cd extlibs/sandbox \
			&& make -j$(shell nproc))
	[ -f build/Makefile ] || (mkdir -p build ; cd build ; cmake ..)
	$(MAKE) -C build

visualizer:
	mkdir -p build && cd build && cmake -DWITH_VISUALIZER=ON ..
	$(MAKE) -C build

install: default
	$(MAKE) -C build install

clean:
	rm -rf build/

