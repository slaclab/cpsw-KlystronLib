CPSW_DIR=/afs/slac/g/lcls/package/cpsw/framework/R4.4.1/src

CCACHE_DISABLE=1

SRCDIR=.
-include $(SRCDIR)/release.mak

include $(CPSW_DIR)/defs.mak

INSTALL_DIR=../

HEADERS += TrigPulse.cc KlystronFw.h

KlystronLib_SRCS  = KlystronFw.cc
KlystronLib_LIBS  = $(CPSW_LIBS)

SHARED_LIBRARIES_YES+=KlystronLib
STATIC_LIBRARIES_YES+=KlystronLib

Klystron_tst_SRCS =  KlystronFw_tst.cc
Klystron_tst_LIBS = KlystronLib
Klystron_tst_LIBS += $(CPSW_LIBS)

PROGRAMS=Klystron_tst

include $(CPSW_DIR)/rules.mak
