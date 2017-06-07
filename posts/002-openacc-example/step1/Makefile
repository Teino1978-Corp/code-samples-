# Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

PGCC     = pgcc
HMPP     = hmpp

CC       ?= gcc
ACC      ?= $(PGCC)

CCFLAGS  ?= -I../common

PGCC_ACCFLAGS ?= -fast -Minfo=accel -acc
PGCC_OMPFLAGS ?= -fast -Minfo -mp

HMPP_FLAGS = --codelet-required

HMPP_ACCFLAGS ?= -std=c99 -O3
HMPP_OMPFLAGS ?= -std=c99 -O3 -fopenmp -lm

ifeq ($(ACC), $(PGCC))
    ACCFLAGS = $(PGCC_ACCFLAGS)
    OMPFLAGS = $(PGCC_OMPFLAGS)
    ACC_COMMAND = $(ACC)
    OMP_COMMAND = $(ACC)
else
    ACCFLAGS = $(HMPP_ACCFLAGS)
    OMPFLAGS = $(HMPP_OMPFLAGS)
    ACC_COMMAND = $(HMPP) $(HMPP_FLAGS) $(CC)
    OMP_COMMAND = $(CC)
endif

BIN =  laplace2d_omp laplace2d_acc

all: $(BIN)

laplace2d_acc: laplace2d.c
	$(ACC_COMMAND) $(CCFLAGS) -o $@ $< $(ACCFLAGS)

laplace2d_omp: laplace2d.c
	$(OMP_COMMAND) $(CCFLAGS) -o $@ $< $(OMPFLAGS) 

clean:
	$(RM) $(BIN) *.hmf*
