# Copyright (C) 2011 Science and Technology Facilities Council.

# Author:  Jonathan Hogg               STFC   2011-09-20

####################################################################

# initialise automake
AC_INIT(coinhsl, 2021.05.05, hsl@stfc.ac.uk)
AC_CONFIG_SRCDIR([ma27/ma27d.f])
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([subdir-objects])

# check for required programs
AC_PROG_FC
# Set f77 compiler to be same as f90 compiler unless explcitly told otherwise
if test "x$F77" = "x"; then
   F77="$FC"
fi
if test "x$FFLAGS" = "x"; then
   FFLAGS="$FCFLAGS"
fi
AC_PROG_F77
AC_PROG_CC
AC_PROG_LIBTOOL

# Define a macros to state availability, as per coin expectations
AC_DEFUN([HSL_CHECK_PKG],[
   pkg=$1
   name=$2
   prec=$3
   prec_cap=$4
   AC_MSG_CHECKING([for $name (${prec})])
   if test -r "$srcdir/$pkg/${pkg}${prec}.f"; then
      AC_DEFINE(COINHSL_HAS_$2_$4, [1], [Define to 1 if $2 ($3) is available.])
      coin_has_$1_$3=yes
      AC_MSG_RESULT([yes])
   else
      coin_has_$1_$3=no
      AC_MSG_RESULT([no])
   fi
   AM_CONDITIONAL(HAVE_$2_$4, test x$coin_has_$1_$3 = xyes)
   ])
AC_DEFUN([HSL_CHECK_PKG_F90],[
   pkg=$1
   name=$2
   prec=$3
   prec_cap=$4
   AC_MSG_CHECKING([for $name ($3)])
   if test -r "$srcdir/$pkg/${pkg}${prec}.f90"; then
      AC_DEFINE(COINHSL_HAS_$2_$4, [1], [Define to 1 if $2 ($3) is available.])
      coin_has_$1_$3=yes
      AC_MSG_RESULT([yes])
   else
      coin_has_$1_$3=no
      AC_MSG_RESULT([no])
   fi
   AM_CONDITIONAL(HAVE_$2_$4, test x$coin_has_$1_$3 = xyes)
   ])

AC_MSG_CHECKING([for f90 deps (deps90.f90)])
if test -r "$srcdir/common/deps90.f90"; then
   AC_DEFINE(COINHSL_HAS_DEPS90, [1], [Define to 1 if common/deps90.f90 is available.])
   coin_has_deps90=yes
   AC_MSG_RESULT([yes])
else
   coin_has_deps90=no
   AC_MSG_RESULT([no])
fi
AM_CONDITIONAL(HAVE_DEPS90, test x$coin_has_deps90 = xyes)

HSL_CHECK_PKG(mc19,MC19,s,S)
HSL_CHECK_PKG(mc19,MC19,d,D)
HSL_CHECK_PKG(ma27,MA27,s,S)
HSL_CHECK_PKG(ma27,MA27,d,D)
HSL_CHECK_PKG(ma28,MA28,s,S)
HSL_CHECK_PKG(ma28,MA28,d,D)
HSL_CHECK_PKG(ma57,MA57,s,S)
HSL_CHECK_PKG(ma57,MA57,d,D)
HSL_CHECK_PKG_F90(hsl_ma77,MA77,s,S)
HSL_CHECK_PKG_F90(hsl_ma77,MA77,d,D)
HSL_CHECK_PKG_F90(hsl_ma86,MA86,s,S)
HSL_CHECK_PKG_F90(hsl_ma86,MA86,d,D)
HSL_CHECK_PKG_F90(hsl_ma97,MA97,s,S)
HSL_CHECK_PKG_F90(hsl_ma97,MA97,d,D)

# HSL_MC68 is an i.f90 not a d.f90 so can't use HSL_CHECK_PKG_F90
AC_MSG_CHECKING([for MC68 (i)])
if test -r "$srcdir/hsl_mc68/C/hsl_mc68i_ciface.f90"; then
   AC_DEFINE(COINHSL_HAS_MC68_I, [1], [Define to 1 if MC68 (i) is available.])
   coin_has_hsl_mc68_i=yes
   AC_MSG_RESULT([yes])
else
   coin_has_hsl_mc68_i=no
   AC_MSG_RESULT([no])
fi
AM_CONDITIONAL(HAVE_MC68_I, test x$coin_has_hsl_mc68_i = xyes)

# Look for a metis we can compile
AC_MSG_CHECKING([for metis to compile])
if test -r "$srcdir/metis-4.0.3/Lib/balance.c"; then
   AC_DEFINE(COINHSL_HAS_METIS, [1], [Define to 1 if METIS is available to compile.])
   coin_has_metis=yes
   AC_MSG_RESULT([yes])
else
   coin_has_metis=no
   AC_MSG_RESULT([no])
fi
AM_CONDITIONAL(HAVE_METIS, test x$coin_has_metis = xyes)

# Check if we're on Windows and have to do extra legwork for DLLs
AC_CHECK_HEADERS(windows.h)
AM_CONDITIONAL(TARGET_DLL, test x$ac_cv_header_windows_h = xyes)

# Check if we have BLAS (only used for windows as dep of dll)
AX_BLAS

# output data
AC_CONFIG_FILES([Makefile coinhsl.pc])
AC_CONFIG_HEADER(config.h CoinHslConfig.h)
AC_OUTPUT
