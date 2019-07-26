# NOTICE: this Makefile has been generated, so your
#         modifications may get lost. To rebuild this
#         file, run the './configure' script.

# The list of the first-level subdirectories.
DIRS=$$(find /Users/Judithli/autofocus3-master3/runtime-af3_phoenix.product/AF3-Project-Directory/RaspberryPI -mindepth 1 -maxdepth 1 -not -name ".*" -type d)

# The default build target.
all:
	@for D in ${DIRS}; do make -C $$D || exit $$?; done

# The default clean target.
clean:
	@for D in ${DIRS}; do make -C $$D clean; done
# The Mr. Proper build target.
mrproper:
	@for D in ${DIRS}; do make -C $$D mrproper; done
	@rm Makefile
