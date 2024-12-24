# Project Name
TARGET = _Envelope

USE_DAISYSP_LGPL = 1

# Sources
CPP_SOURCES = _Envelope.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP

# Includes FatFS source files within project.
USE_FATFS = 1

# set optimization so my code fits on the flash :)
OPT = -Os

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

