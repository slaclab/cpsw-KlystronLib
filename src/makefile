CPSW_DIR=/afs/slac/g/lcls/package/cpsw/framework/R3.6.4/src

CCACHE_DISABLE=1

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

HEADERS += KlystronFw.h

KlystronLib_SRCS  = TrigPulse.cc KlystronFw.cc
KlystronLib_LIBS  = yaml-cpp $(CPSW_LIBS)

SHARED_LIBRARIES_YES+=KlystronLib
STATIC_LIBRARIES_YES+=KlystronLib

Klystron_tst_SRCS =  KlystronFw_tst.cc
Klystron_tst_LIBS = KlystronLib.a cpsw.a yaml-cpp.a boost_system.a pthread dl rt

PROGRAMS=Klystron_tst

include $(CPSW_DIR)/rules.mak
