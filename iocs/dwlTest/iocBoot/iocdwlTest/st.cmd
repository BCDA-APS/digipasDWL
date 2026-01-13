# ../../bin/${EPICS_HOST_ARCH}/dwlTest st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocdwlTestLinux.dbd")
iocdwlTestLinux_registerRecordDeviceDriver(pdbbase)

epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "dwlTest:")

epicsEnvSet("DWL_PORT", "DWL_CONN")
# # Local USB (via serial to USB adapter)
# drvAsynSerialPortConfigure("$(DWL_PORT)", "/dev/ttyUSB0", 0, 0, 0)
# asynSetOption("$(DWL_PORT)", 0, "baud", 115200)
# asynSetOption("$(DWL_PORT)", 0, "bits", 8)
# asynSetOption("$(DWL_PORT)", 0, "parity", "none")
# asynSetOption("$(DWL_PORT)", 0, "stop", 1)

# MOXA serial server, Port 1
# Serial settings are set in the MOXA
drvAsynIPPortConfigure("$(DWL_PORT)", "192.168.127.254:4001", 0, 0, 0)

# Initialize
# USA = 0x42
# Chicago = 0x05
# DigipasDWLConfig(asyn_port, driver_port, mode, country_code, city_code)
DigipasDWLConfig("$(DWL_PORT)", "DWL1", "Single", 0x42, 0x05)

# Load PVs
dbLoadRecords("$(DIGIPAS_DWL)/db/digipasDWL.db", "P=$(PREFIX),R=DWL,PORT=DWL1")

# asynRecord for debugging
dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=asyn_$(DWL_PORT), PORT=$(DWL_PORT), ADDR=0, OMAX=256, IMAX=256")

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
