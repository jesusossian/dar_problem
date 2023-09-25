SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

########### FROM CPLEX MAKEFILE ###############
# Compiler selection
CCC = g++ -O0 
# Compiler options
CCOPT = -std=c++17 -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD # -g for debugging

# change these lines to the location of cplex 
#CPLEXDIR      = /opt/ibm/ILOG/CPLEX_Studio1210/cplex
#CONCERTDIR    = /opt/ibm/ILOG/CPLEX_Studio1210/concert
CPLEXDIR = /opt/ibm/ILOG/CPLEX_Studio201/cplex
CONCERTDIR = /opt/ibm/ILOG/CPLEX_Studio201/concert


# Link options and libraries
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) 
CCLNFLAGS = -lconcert -lilocplex -lcplex -m64 -lm -lpthread -ldl

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) 



# Compiler settings - Can be customized.
CC = g++
CFLAGS = -std=c++17 -g -O3 -Wall 


# Set directory for static library and binaries
# Defaults to ./lib and ./bin
DARPH_LIB_DIR = ./lib
DARPH_BIN_DIR = ./bin

# Set names of executables
MILP3_EXE = $(DARPH_BIN_DIR)/darp_milp_3
MILP6_EXE = $(DARPH_BIN_DIR)/darp_milp_6

# Set name of libraries needed by applicaitons
LIBS= -ldarph -lm

# Various directories needed by the library and applications
INC_DIR= -I./inc/
LIB_DIR = -L$(DARPH_LIB_DIR)
DARPH_LIB = $(DARPH_LIB_DIR)/libdarph.a

SRCS= ./src/DARP.cpp ./src/DARPIO.cpp ./src/DARPGraph.cpp ./src/DARPSolver.cpp ./src/EBMILP.cpp 

OBJS=$(SRCS:.cpp=.o) ./src/DARPMILPs.o 


MILP3_SRC = ./src/apps/darp_milp_3.cpp
MILP6_SRC = ./src/apps/darp_milp_6.cpp




all: $(DARPH_LIB) darp_milp_3 darp_milp_6

$(DARPH_LIB): $(OBJS) 
	mkdir -p $(DARPH_LIB_DIR)
	$(AR) $(ARFLAGS) $@ $(OBJS) 
	ranlib $@
	rm -rf $(OBJS)
	
	

.cpp.o:
	$(CCC) $(CCFLAGS) -c $(INC_DIR) $< -o $@

./src/DARPMILPs.o:
	$(CCC) $(CCFLAGS) -c $(INC_DIR) ./src/DARPMILPs.cpp -o ./src/DARPMILPs.o 



darp_milp_3: $(OBJS) $(MILP3_SRC)
	mkdir -p $(DARPH_BIN_DIR)
	$(CCC) $(CCFLAGS) $(CCLNDIRS) $(MILP3_SRC) $(INC_DIR) $(LIB_DIR) $(LIBS) -o $(MILP3_EXE) $(CCLNFLAGS)

darp_milp_6: $(OBJS) $(MILP6_SRC)
	mkdir -p $(DARPH_BIN_DIR)
	$(CCC) $(CCFLAGS) $(CCLNDIRS) $(MILP6_SRC) $(INC_DIR) $(LIB_DIR) $(LIBS) -o $(MILP6_EXE) $(CCLNFLAGS)


clean: 
	-rm -rf $(OBJS)
	-rm -rf $(DARPH_LIB)
	-rm -rf $(MILP3_EXE)
	-rm -rf $(MILP6_EXE)
	/bin/rm -rf *.o *~ 
	/bin/rm -rf *.mps *.ord *.sos *.sav *.net *.msg *.log *.clp



