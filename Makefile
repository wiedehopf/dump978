CC=gcc
CFLAGS+=-Wall -Werror -O2 -g -Ilibs

CXX=g++
CXXFLAGS+=-std=c++11 -Wall -Werror -O2 -g -Ilibs

LIBS=-lboost_system -lboost_program_options -lboost_regex -lSoapySDR -lpthread

all: dump978 faup978

dump978: dump978_main.o socket_output.o message_dispatch.o fec.o libs/fec/init_rs_char.o libs/fec/decode_rs_char.o sample_source.o soapy_source.o convert.o demodulator.o uat_message.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS)

faup978: faup978_main.o socket_input.o uat_message.o track.o faup978_reporter.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@ $(LIBS)

format:
	clang-format -style=file -i *.cc *.h

clean:
	rm -f *.o fec/*.o dump978 faup978
