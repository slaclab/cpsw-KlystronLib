
ifeq ($(PACKAGE_TOP),)
ifneq ($(EPICS_PACKAGE_TOP),)
PACKAGE_TOP=$(EPICS_PACKAGE_TOP)
else
$(error PACKAGE_TOP or EPICS_PACKAGE_TOP must be provided in the environment or on the command line)
endif
endif

CPSW_FRAMEWORK_VERSION=R4.5.2
CPSW_DIR=$(PACKAGE_TOP)/cpsw/framework/$(CPSW_FRAMEWORK_VERSION)/src

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
