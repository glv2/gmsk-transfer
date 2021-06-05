AC_INIT(gmsk-transfer, 1.0.0)
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR(gmsk-transfer.c)

dnl Check for toolchain and install components
AC_PROG_CC
AC_PROG_INSTALL

dnl Check for standard headers
AC_CHECK_HEADERS([complex.h math.h signal.h stdio.h stdlib.h unistd.h])

dnl Check for functions
AC_CHECK_FUNCS([malloc free])
AC_CHECK_FUNCS([printf fprintf])
AC_CHECK_FUNCS([fopen fclose])
AC_CHECK_FUNCS([strtof strtol strtoul])
AC_CHECK_FUNCS([bzero memcpy strcasecmp strchr])
AC_CHECK_FUNCS([getopt])
AC_CHECK_FUNCS([signal])

dnl Check for libraries
AC_CHECK_LIB(m, crealf, [], AC_MSG_ERROR([math library required]))

AC_CHECK_HEADERS(pthread.h, [], AC_MSG_ERROR([POSIX threads header required]))
AC_CHECK_LIB(pthread, pthread_mutex_init, [], AC_MSG_ERROR([POSIX threads library required]))

AC_CHECK_HEADERS(libhackrf/hackrf.h, [], AC_MSG_ERROR([hackrf header required]))
AC_CHECK_LIB(hackrf, hackrf_init, [], AC_MSG_ERROR([hackrf library required]))

AC_CHECK_HEADERS(liquid/liquid.h, [], AC_MSG_ERROR([liquid-dsp header required]))
AC_CHECK_LIB(liquid, gmskframegen_create, [], AC_MSG_ERROR([liquid-dsp library required]))

AC_CONFIG_FILES(Makefile)
AC_OUTPUT
