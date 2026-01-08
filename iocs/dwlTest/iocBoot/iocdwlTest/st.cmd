# ../../bin/${EPICS_HOST_ARCH}/dwlTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocdwlTestLinux.dbd")
iocdwlTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "dwlTest:")

< digipasDWL.iocsh

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
