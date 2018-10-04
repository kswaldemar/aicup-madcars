CP_FLAGS=-std=gnu99 -ffast-math -DCHIPMUNK_FFI -DCP_USE_CGPOINTS=0 -DNDEBUG -fPIC -O3 -m64
S_FLAGS=-std=c++1z -O3 -DNDEBUG

CP_CC=gcc-5
S_CC=g++-7

CP_SOURCES=$(shell find ./chipmunk_src -type f -name '*.c')
CP_OBJECTS=$(CP_SOURCES:.c=.o)
CP_INCLUDE=./chipmunk_src/include

S_SOURCES=$(shell find ./solution -type f -name '*.cpp')
S_OBJECTS=$(S_SOURCES:.cpp=.o)


.PHONY: clean

all: ${COMPILED_FILE_PATH}

${COMPILED_FILE_PATH}: libchipmunk.a $(S_OBJECTS)
	$(S_CC) $(S_FLAGS) -o ${COMPILED_FILE_PATH} -I$(CP_INCLUDE) $(S_OBJECTS) libchipmunk.a -lpthread -lm

libchipmunk.a: $(CP_OBJECTS)
	ar rcs libchipmunk.a $(CP_OBJECTS)

$(CP_OBJECTS):  %.o: %.c
	$(CP_CC) $(CP_FLAGS) -I$(CP_INCLUDE) -c $< -o $@

$(S_OBJECTS): %.o: %.cpp
	$(S_CC) $(S_FLAGS) -I$(CP_INCLUDE) -c $< -o $@

clean:
	rm -f $(CP_OBJECTS) $(S_OBJECTS) libchipmunk.a