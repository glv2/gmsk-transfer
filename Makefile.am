AUTOMAKE_OPTIONS = foreign dist-lzip no-dist-gzip subdir-objects
ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libgmsk-transfer.la
libgmsk_transfer_la_SOURCES = src/gmskframesync.h src/gmskframesync.c src/gmsk-transfer.c
libgmsk_transfer_la_LDFLAGS = -version-info 1:0:0

bin_PROGRAMS = gmsk-transfer
gmsk_transfer_SOURCES = src/main.c
gmsk_transfer_LDADD = libgmsk-transfer.la

include_HEADERS = src/gmsk-transfer.h

examplesdir =
examples_PROGRAMS = examples/duplex examples/echo-server
examples_duplex_SOURCES = examples/duplex.c
examples_duplex_CFLAGS = -I src
examples_duplex_LDADD = libgmsk-transfer.la -lpthread
examples_echo_server_SOURCES = examples/echo-server.c
examples_echo_server_CFLAGS = -I src
examples_echo_server_LDADD = libgmsk-transfer.la

check_PROGRAMS = tests/test-library-callback tests/test-library-file
tests_test_library_callback_SOURCES = tests/test-library-callback.c
tests_test_library_callback_CFLAGS = -I src
tests_test_library_callback_LDADD = libgmsk-transfer.la
tests_test_library_file_SOURCES = tests/test-library-file.c
tests_test_library_file_CFLAGS = -I src
tests_test_library_file_LDADD = libgmsk-transfer.la
TESTS = tests/test-library-callback tests/test-library-file tests/test-program.sh

EXTRA_DIST = examples/duplex-ppp.sh tests/test-program.sh
