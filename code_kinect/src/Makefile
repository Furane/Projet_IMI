include CommonDefs.mak

BIN_DIR = ../Bin

INC_DIRS = \
	../Common \
	../../Include \
	../../../../kinect/NiTE-Linux-x64-2.2/Include \
	../../../../kinect/OpenNI-Linux-x64-2.2/Include \
	$(OPENNI2_INCLUDE) \
	$(NITE2_INCLUDE) 
	

SRC_FILES = *.cpp

LIB_DIRS += $(OPENNI2_REDIST) \
	    $(NITE2_REDIST64)
		

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = SimpleUserTracker

CFLAGS += -Wall

include CommonCppMakefile
