#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := Spacr
EXTRA_COMPONENT_DIRS := $(realpath C:/Users/sagar/Desktop/esp-idf/components/esp-aws-iot)
include $(IDF_PATH)/make/project.mk

