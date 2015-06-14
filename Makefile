default:
	[ -f build/Makefile ] || (mkdir -p build ; cd build ; cmake ..)
	$(MAKE) -C build

visualizer: sandbox
	mkdir -p build && cd build && cmake -DWITH_VISUALIZER=ON ..
	$(MAKE) -C build

install: default
	$(MAKE) -C build install

sandbox:
	[ -f extlibs/sandbox/lib/libsandbox.a ] \
		|| (git submodule update --init \
			&& cd extlibs/sandbox \
			&& make -j$(shell nproc))

clean:
	rm -rf build/

