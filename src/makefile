CPSW_DIR=/afs/slac/g/lcls/package/cpsw/framework/R3.5.3/src

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

HEADERS += KlysMrFw.h

klysLib_SRCS  = TrigPulse.cc KlysMrFw.cc
klysLib_LIBS  = yaml-cpp $(CPSW_LIBS)

SHARED_LIBRARIES_YES+=klysLib
STATIC_LIBRARIES_YES+=klysLib

klys_tst_SRCS =  KlysMrFw_tst.cc
klys_tst_LIBS = klysLib.a cpsw.a yaml-cpp.a boost_system.a pthread dl rt

PROGRAMS=klys_tst

include $(CPSW_DIR)/rules.mak
