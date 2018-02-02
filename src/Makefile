lambda-objs =  LambdaSyncCtrlObj.o \
			LambdaDetInfoCtrlObj.o \
			LambdaInterface.o \
			LambdaCamera.o

SRCS = $(lambda-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
	-I../sdk/include \
	-pthread -fPIC -g 

all:	Lambda.o

Lambda.o:	$(lambda-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)

.PHONY: check-syntax
check-syntax:
	$(CXX) -Wall -Wextra -fsyntax-only $(CXXFLAGS) $(CHK_SOURCES)
