default:
	[ -f extlibs/sandbox/lib/libsandbox.a ] \
		|| (git submodule update --init \
			&& cd extlibs/sandbox \
			&& make -j$(shell nproc))
	[ -d data ] || $(MAKE) update-data
	[ -f build/Makefile ] || (mkdir -p build ; cd build ; cmake ..)
	$(MAKE) -C build

clean:
	rm -rf build/

update-data:
	wget -O /tmp/kd-data.tgz http://student.agh.edu.pl/~mradomsk/private/kd-data.tgz && tar xf /tmp/kd-data.tgz

push-data:
	tar zcf /tmp/kd-data.tgz data && scp /tmp/kd-data.tgz mradomsk@student.agh.edu.pl:public_html/private/

