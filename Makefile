

# Print all suppressions:  --gen-suppressions=all
valgrind:
	 valgrind --leak-check=yes \
            --suppressions=./.valgrind/golang.supp \
            ../fips-deploy/lsd-slam-pangolin-gui/linux-make-unittest/LSD --config d2_ex1605l3_one.conf 2>&1 | tee valgrind.txt


run:
	./fips run LSD -- --config ${CURDIR}/d2_ex1605l3_one.conf	

gdb:
	./fips gdb LSD -- --config ${CURDIR}/d2_ex1605l3_one.conf
