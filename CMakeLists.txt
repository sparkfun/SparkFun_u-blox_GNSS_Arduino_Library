file(GLOB SOURCES src/*.cpp)
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS src
    REQUIRES arduino
)
