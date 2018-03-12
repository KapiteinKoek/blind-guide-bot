# Copyright 2018 Anne Kolmans, Dylan ter Veen, Jarno Brils, Renée van Hijfte, and Thomas Wiepking (TU/e Project Robots Everywhere 2017/2018 Q3 Group 12)
#
#  Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

BINARIES = blindguide

CC = gcc
CFLAGS = -Wall -g -c
LDLIBS = -lm

all:	$(BINARIES)

clean:
	rm -f *.o $(BINARIES)

blindguide: blindguide.o

blindguide.o: blindguide.c blindguide.h

