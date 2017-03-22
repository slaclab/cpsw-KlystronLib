CPSW_DIR=/afs/slac/g/lcls/package/cpsw/framework/R3.5.3/src

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

llrfLib_SRCS  = TrigPulse.cc LlrfMrFw.cc
llrfLib_LIBS  = yaml-cpp $(CPSW_LIBS)

SHARED_LIBRARIES_YES+=llrfLib
STATIC_LIBRARIES_YES+=llrfLib

llrf_tst_SRCS =  LlrfMrFw_tst.cc
llrf_tst_LIBS = llrfLib.a cpsw.a yaml-cpp.a boost_system.a pthread dl rt

PROGRAMS=llrf_tst

include $(CPSW_DIR)/rules.mak
