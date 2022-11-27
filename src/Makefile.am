lib_LTLIBRARIES = libgmsk-transfer.la
libgmsk_transfer_la_SOURCES = \
  gettext.h \
  gmskframesync.c \
  gmskframesync.h \
  gmsk-transfer.c \
  gmsk-transfer.h
libgmsk_transfer_la_LDFLAGS = -version-info 1:0:0

bin_PROGRAMS = gmsk-transfer
gmsk_transfer_SOURCES = gettext.h gmsk-transfer.h main.c
gmsk_transfer_CFLAGS = -DLOCALEDIR=\"$(localedir)\"
gmsk_transfer_LDADD = libgmsk-transfer.la
