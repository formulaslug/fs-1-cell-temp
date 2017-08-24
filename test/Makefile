# This makefile recursively finds, compiles, and links .c and .cpp files.
#
# The following targets are supported by this makefile.
#
# all
# clean
#
# Defining VERBOSE before running make (i.e., VERBOSE=1 make) will print all
# commands ran.

NAME := tests

CC := gcc
CFLAGS := -O0 -g3 -Wall -std=c11

CXX := g++
CXXFLAGS := -O0 -g3 -Wall -std=c++1z

# Specify defines with -D directives here
DEFINES :=

# Specify include paths with -I directives here
IFLAGS := -Isrc/gtest -Isrc/gtest/include

LD := g++

# Specify libs with -l directives here
LDFLAGS := -pthread

# Platform specific variables
ifeq ($(OS), Windows_NT)
	# Assign executable name
	EXEC := $(NAME).exe
else
	# Assign executable name
	EXEC := $(NAME)
endif

SRCDIR := src

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Recursively find all C source files
SRC_C := $(call rwildcard,$(SRCDIR)/,*.c)

# Recursively find all C++ source files
SRC_CC := $(call rwildcard,$(SRCDIR)/,*.cc)
SRC_CXX := $(call rwildcard,$(SRCDIR)/,*.cpp)

# Create raw list of object files
C_OBJ := $(SRC_C:.c=.o)
CC_OBJ := $(SRC_CC:.cc=.o)
CXX_OBJ := $(SRC_CXX:.cpp=.o)

# Create list of object files
OBJDIR := build
C_OBJ := $(addprefix $(OBJDIR)/,$(C_OBJ))
CC_OBJ := $(addprefix $(OBJDIR)/,$(CC_OBJ))
CXX_OBJ := $(addprefix $(OBJDIR)/,$(CXX_OBJ))

.PHONY: all
all: $(OBJDIR)/$(EXEC)

# Define a string comparison function: "String EQual"
seq = $(and $(findstring $1,$2),$(findstring $2,$1))

# Define function for determining if given target is within a list of targets
# (If $1 is an element of $2)
# Returns a list whose size equals the number of occurences of $1 in $2
targetelem = $(foreach goal,$2, \
$(if $(call seq, $1 , $(goal) ),found,))

# If 'clean' won't be built, generate the dependency files
ifeq (,$(strip $(call targetelem,clean,$(MAKECMDGOALS))))
-include $(C_OBJ:.o=.d) $(CC_OBJ:.o=.d) $(CXX_OBJ:.o=.d)
endif

$(OBJDIR)/$(EXEC): $(C_OBJ) $(CC_OBJ) $(CXX_OBJ)
	@mkdir -p $(@D)
	@echo Linking $@
ifdef VERBOSE
	$(LD) -o $@ $(C_OBJ) $(CC_OBJ) $(CXX_OBJ) $(LDFLAGS)
else
	@$(LD) -o $@ $(C_OBJ) $(CC_OBJ) $(CXX_OBJ) $(LDFLAGS)
endif

# Pattern rule for building object file from C source
# The -MMD flag generates .d files to track changes in header files included in
# the source.
$(C_OBJ): $(OBJDIR)/%.o: %.c
	@mkdir -p $(@D)
	@echo Building C object $@
ifdef VERBOSE
	$(CC) $(CFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
else
	@$(CC) $(CFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
endif

# Pattern rule for building object file from C++ source
# The -MMD flag generates .d files to track changes in header files included in
# the source.
$(CC_OBJ): $(OBJDIR)/%.o: %.cc
	@mkdir -p $(@D)
	@echo Building CXX object $@
ifdef VERBOSE
	$(CXX) $(CXXFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
else
	@$(CXX) $(CXXFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
endif

# Pattern rule for building object file from C++ source
# The -MMD flag generates .d files to track changes in header files included in
# the source.
$(CXX_OBJ): $(OBJDIR)/%.o: %.cpp
	@mkdir -p $(@D)
	@echo Building CXX object $@
ifdef VERBOSE
	$(CXX) $(CXXFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
else
	@$(CXX) $(CXXFLAGS) $(DEFINES) $(IFLAGS) -MMD -c -o $@ $<
endif

# Cleans the build directory
.PHONY: clean
clean:
	@echo Removing object files
ifdef VERBOSE
	-$(RM) -r $(OBJDIR)
else
	-@$(RM) -r $(OBJDIR)
endif
