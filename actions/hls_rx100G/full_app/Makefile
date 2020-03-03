#
# Copyright 2017 International Business Machines
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#
# Generate HDL version of the HLS sources
#
# The generated HDL depends on the chip which is used and
# therefore must match what is being used to build the
# toplevel SNAP bitstream.
#
# FIXME Pass part_number and other parameters from toplevel
#      build-system as required.
#


# Finding $SNAP_ROOT
ifndef SNAP_ROOT
# check if we are in sw folder of an action (three directories below snap root)
ifneq ("$(wildcard ../../../ActionTypes.md)","")
SNAP_ROOT=$(abspath ../../../)
else
$(info You are not building your software from the default directory (/path/to/snap/actions/<action_name>/sw) or specified a wrong $$SNAP_ROOT.)
$(error Please make sure that $$SNAP_ROOT is set up correctly.)
endif
endif

SNAP_LIBS=${SNAP_ROOT}/software/lib/libsnap.a -L/home/jungfrau/git/pslse/libcxl -lcxl 
SNAP_INCLUDE=-I${SNAP_ROOT}/software/include -I/home/jungfrau/git/pslse/libcxl -I/home/jungfrau/git/pslse/common

HDF5_PATH=/opt/hdf5-1.10.5/include/
CXX=g++
CPPFLAGS= -I. -I../include -I${HDF5_PATH}/include ${SNAP_INCLUDE}
CXXFLAGS= -O3 -g -Wall
LDFLAGS= -O3 -L${HDF5_PATH}/lib -llz4 -lm -lpthread -lhdf5 -lz -libverbs

SRCS=JFReceiver.o parse_input.o sharedVariables.o CompressionThread.o IB_Transport.cpp TCP_Transport.cpp #SnapThread.o 

all: JFReceiver JFWriter

JFWriter: JFWriter.o IB_Transport.cpp TCP_Transport.cpp
	$(CXX) JFWriter.o IB_Transport.cpp TCP_Transport.cpp -o JFWriter $(JF_LDLIBS) ${LDFLAGS}

JFReceiver: $(SRCS)
	$(CXX) $(SRCS) -o JFReceiver $(JF_LDLIBS) ${LDFLAGS} #${SNAP_LIBS}

clean:
	rm -f *.o JFReceiver HDF5Writer
 
# If you have the host code outside of the default snap directory structure, 
# change to /path/to/snap/actions/software.mk
# include $(SNAP_ROOT)/actions/software.mk
