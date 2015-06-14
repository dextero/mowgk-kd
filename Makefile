default:
	[ -f extlibs/sandbox/lib/libsandbox.a ] \
		|| (git submodule update --init \
			&& cd extlibs/sandbox \
			&& make -j$(shell nproc))
	[ -f build/Makefile ] || (mkdir -p build ; cd build ; cmake ..)
	$(MAKE) -C build

clean:
	rm -rf build/

