AUTOMAKE_OPTIONS = foreign dist-lzip no-dist-gzip subdir-objects
ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libgmsk-transfer.la
libgmsk_transfer_la_SOURCES = src/gmsk-transfer.c
libgmsk_transfer_la_LDFLAGS = -version-info 1:0:0

bin_PROGRAMS = gmsk-transfer
gmsk_transfer_SOURCES = src/main.c
gmsk_transfer_LDADD = libgmsk-transfer.la

include_HEADERS = src/gmsk-transfer.h

check_PROGRAMS = tests/test-library
tests_test_library_SOURCES = tests/test-library.c
tests_test_library_CFLAGS = -I src
tests_test_library_LDADD = libgmsk-transfer.la
TESTS = tests/test-library tests/test-program.sh

EXTRA_DIST = tests/test-program.sh
