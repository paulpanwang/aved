dnl Process this file with autoconf to produce a configure script.
dnl This macro is a must

AC_INIT([mbarivision.C], [dcline@mbari.org])
AC_PREREQ(2.50)
AC_PREFIX_DEFAULT(/usr/local/aved)
AC_LANG(C++)

##########################################################################
### Some macro definitions

dnl AC_LATE_WARN(msg) msg will be given as a warning, and will be
dnl repeated at the end of running the configure script, making it more
dnl likely that the user will notice it.

dnl AC_CHECK_LIB_CXX(libname, includes, function-call,
dnl                  action-on-success, action-on-fail,
dnl                  extra-libs-for-link)
AC_DEFUN([AC_CHECK_LIB_CXX],
	 [AC_MSG_CHECKING([for lib$1])
	 libs_save=$LIBS
	 LIBS="$LIBS -l$1 $6"
	 AC_TRY_LINK([$2], [$3], [havelib_$1=yes], [havelib_$1=no])
	 LIBS=$libs_save
	 if test "x$havelib_$1" = xyes; then
	    AC_MSG_RESULT(yes)
	    $4
	 else
	    AC_MSG_RESULT(no)
	    $5
	 fi])

dnl AC_CHECK_HEADERS_CXX(aname, includes, function-call,
dnl                  action-on-success, action-on-fail)
AC_DEFUN([AC_CHECK_HEADERS_CXX],
	 [AC_MSG_CHECKING([for headers in $1])
	 AC_TRY_LINK([$2], [$3], [haveheaders_$1=yes], [haveheaders_$1=no])
	 LIBS=$libs_save
	 if test "x$haveheaders_$1" = xyes; then
	    AC_MSG_RESULT(yes)
	    $4
	 else
	    AC_MSG_RESULT(no)
	    $5
	 fi])

dnl This handles libraries with the dash in them. It's an alternative
dnl to the AC_CHECK_LIB_CXX above
dnl AC_CHECK_LIB_CXX_DASH(libname, includes, function-call,
dnl                  action-on-success, action-on-fail,
dnl                  extra-libs-for-link)
AC_DEFUN([AC_CHECK_LIB_CXX_DASH],
	 [AC_MSG_CHECKING([for lib$2])
	 libs_save=$LIBS
	 LIBS="$LIBS -l$2 $7"
	 AC_TRY_LINK([$3], [$4], [havelib_$1=yes], [havelib_$1=no])
	 LIBS=$libs_save
	 if test "x$havelib_$1" = xyes; then
	    AC_MSG_RESULT(yes)
	    $5
	 else
	    AC_MSG_RESULT(no)
	    $6
	 fi])

dnl Checks for c compiler.
AC_PROG_CC

dnl Checks for c compiler.
AC_PROG_CXX 

# Enable users to specify where Saliency is installed.
# TODO: Add saliency version check
# The default is in /usr/local
AC_ARG_WITH([saliency],
            [AC_HELP_STRING([--with-saliency=DIR],
                            [where the saliency is installed (e.g., /usr/local/saliency )
            [default=/usr/local/saliency]])],
            , 
	    [with_saliency=/usr/local/saliency])

# Use the supplied (or default) saliency root directory to set
# Makefile variables 
if test -d "$with_saliency" ; then
      
  saliency_incdir="-I$with_saliency/src"
  saliency_libdir="-L$with_saliency/build/obj"

  cppflags_save="$CPPFLAGS"
  ldflags_save="$LDFLAGS"
  CPPFLAGS="$CPPFLAGS $saliency_incdir"
  LDFLAGS="$LDFLAGS $saliency_libdir" 

  AC_CHECK_HEADERS_CXX(saliency, [
#include <Component/ModelManager.H>
#include <Neuro/StdBrain.H>
], [],
	               [with_saliency=$with_saliency],
                       [AC_MSG_ERROR([iLab Neuromorphic Vision C++ Toolkit
                        Error finding saliency root directory here: "$with_saliency".
                        Set the correct root path with --with-saliency-root=DIR
                        For more information about getting the toolkit see http://ilab.usc.edu/toolkit/.])
                       ])
   else
      AC_MSG_ERROR([iLab Neuromorphic Vision C++ Toolkit
                        Error finding saliency root directory here: "$with_saliency".
                        Set the correct root path with --with-saliency-root=DIR
                        For more information about getting the toolkit see http://ilab.usc.edu/toolkit/.])
   fi

CPPFLAGS="$cppflags_save"
LDFLAGS="$ldflags_save"

AC_SUBST(with_saliency)

# Enable users to specify where Xercesc is installed.
# Note the 2.7.0 version is required
# The default is to have the libraries in the users home directory
AC_ARG_WITH([xercesc],
            [AC_HELP_STRING([--with-xercesc=DIR],
                            [where the Xercesc is installed (e.g., $HOME/Xerces-C_2_7_0 )
            [default=$HOME/Xerces-C_2_7_0]])],
            ,
            [with_xercesc=$HOME/Xerces-C_2_7_0])
 
# Use the supplied (or default) Xercesc directory to set
# makefile variables
if test -d "$with_xercesc" ; then

   xercesc_incdir="$with_xercesc/src"
   xercesc_libdir="$with_xercesc/lib"

   cppflags_save="$CPPFLAGS"
   ldflags_save="$LDFLAGS"
   CPPFLAGS="$CPPFLAGS -I$xercesc_incdir"
   LDFLAGS="$LDFLAGS -L$xercesc_libdir"

   AC_CHECK_LIB_CXX_DASH(xercesc, xerces-c, [
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/util/XMLDateTime.hpp>
], [ ],
                       [with_xercesc=$with_xercesc],
                       [AC_MSG_ERROR([Xercesc directory "$with_xercesc" missing or incorrect.
                        Set the correct path with --with-xercesc=DIR])
                       ],
                       [-lxerces-c -lpthread])
   else
      AC_MSG_ERROR([Xercesc directory $with_xercesc missing or incorrect.
                    Set the correct path with --with-xercesc=DIR   ])
   fi

AC_SUBST(with_xercesc)

# Enable users to specify where OpenCV is installed.
# Note the 2.4.9 version is required
# The default is to have the libraries in the /usr/local/ directory
AC_ARG_WITH([opencv],
            [AC_HELP_STRING([--with-opencv=DIR],
                            [where the OpenCV is installed (e.g., /usr/local/ )
            [default=/usr/local/]])],
            ,
            [with_opencv=/usr/local/])

# Use the supplied (or default) OpenCV directory to set
# makefile variables
if test -d "$with_opencv" ; then

   opencv_cppflags_save="$CPPFLAGS"
   opencv_ldflags_save="$LDFLAGS"
   CPPFLAGS="$CPPFLAGS -I/usr/include -I$with_opencv/include"
   LDFLAGS="$LDFLAGS -L$with_opencv/lib"

   AC_CHECK_LIB_CXX_DASH(opencv, opencv_imgproc, [
#include <opencv2/imgproc/imgproc.hpp>
], [ ],
                       [with_opencv=$with_opencv],
                       [AC_MSG_ERROR([OpenCV directory "$with_opencv" missing or incorrect.
                        Set the correct path with --with-opencv=DIR])
                       ],
                       [-lopencv_imgproc ])
   else
      AC_MSG_ERROR([OpenCV directory $with_opencv missing or incorrect.
                    Set the correct path with --with_opencv=DIR   ])
   fi

AC_SUBST(with_opencv)

CPPFLAGS="$cppflags_save"
LDFLAGS="$ldflags_save"

AC_MSG_CHECKING([for library directory /usr/local/lib64])
 if test -d /usr/local/lib64; then
    LDFLAGS="$LDFLAGS -L/usr/local/lib64"
    AC_MSG_RESULT([Added -L/usr/local/lib64 to LDFLAGS])
 else
    AC_MSG_RESULT([Not Found; not added to LDFLAGS])
 fi

AC_MSG_CHECKING([for library directory /usr/local/lib])
if test -d /usr/local/lib; then
   LDFLAGS="$LDFLAGS -L/usr/local/lib"
   AC_MSG_RESULT([Added -L/usr/local/lib to LDFLAGS])
else
   AC_MSG_RESULT([Not Found; not added to LDFLAGS])
fi


 
dnl Create these files, making substitutions if necessary
AC_OUTPUT([Makefile])


