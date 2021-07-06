AUTOMAKE_OPTIONS = foreign dist-lzip no-dist-gzip subdir-objects
ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libgmsk-transfer.la
libgmsk_transfer_la_SOURCES = src/gmsk-transfer.c
libgmsk_transfer_la_LDFLAGS = -version-info 1:0:0

bin_PROGRAMS = gmsk-transfer
gmsk_transfer_SOURCES = src/main.c
gmsk_transfer_LDADD = libgmsk-transfer.la

include_HEADERS = src/gmsk-transfer.h

TESTS = test.sh
EXTRA_DIST = test.sh
