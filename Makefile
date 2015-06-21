.PHONY: default
default:
	[ -f build/Makefile ] || (mkdir -p build ; cd build ; cmake ..)
	$(MAKE) -C build

.PHONY: visualizer
visualizer: sandbox
	mkdir -p build && cd build && cmake -DWITH_VISUALIZER=ON ..
	$(MAKE) -C build

.PHONY: benchmark
benchmark:
	mkdir -p build && cd build && cmake -DWITH_BENCHMARK=ON ..
	$(MAKE) -C build

.PHONY: install
install: default
	$(MAKE) -C build install

.PHONY: sandbox
sandbox:
	[ -f extlibs/sandbox/lib/libsandbox.a ] \
		|| (git submodule update --init \
			&& cd extlibs/sandbox \
			&& make -j$(shell nproc))

.PHONY: clean
clean:
	rm -rf build/

