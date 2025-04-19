< envPaths
errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/pcoApp.dbd")
pcoApp_registerRecordDeviceDriver(pdbbase) 

# Use this line for a specific board
epicsEnvSet("CAM_ID", "0")

# Prefix for all records
epicsEnvSet("PREFIX", "13ES1:")
# The port name for the detector
epicsEnvSet("PORT",   "PCO1")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "14192")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "10640")
# Really large queue so we can stream to disk at full camera speed
epicsEnvSet("QSIZE",  "2000")   
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# Define NELEMENTS to be enough for a 1920x1080 (mono) image
epicsEnvSet("NELEMENTS", "2073600")

# ADPcoConfig(const char *portName, const char *cameraId,
#                 size_t maxMemory, int priority, int stackSize)
ADPcoConfig("$(PORT)", "$(CAM_ID=0)")

# Main database.  This just loads and modifies ADBase.template
dbLoadRecords("$(ADPCO)/db/pco.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT)")

# Create a standard arrays plugin
NDStdArraysConfigure("Image1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=$(NELEMENTS)")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADPCO)/db")

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")

