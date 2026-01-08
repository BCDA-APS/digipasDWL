# NOTE: paths relative to where they get installed (e.g. ./O.rhel9-x86_64)

TOP=../..

include $(TOP)/configure/CONFIG

# Define IOC library name
LIBRARY_IOC += digipasDWL

# install digipasDWLSupport.dbd into <top>/dbd
DBD += digipasDWLSupport.dbd
digipasDWLSupport_DBD += digipasDWL.dbd

# source files to be compiled and added to the library
digipasDWL_SRCS += digipasDWL.cpp

# Libraries needed for digipasDWL
digipasDWL_LIBS += asyn
digipasDWL_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
