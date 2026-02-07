cmake_minimum_required(VERSION 3.15)
project(EnvelopeShiftRegisterTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_subdirectory(
    ${CMAKE_CURRENT_LIST_DIR}/../../../libDaisy/tests/googletest
    ${CMAKE_BINARY_DIR}/googletest
    EXCLUDE_FROM_ALL
)

set(MIDI_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../midi)

add_library(shift_register STATIC
    ${MIDI_LIB_DIR}/ShiftRegisterMidi.cpp
)
target_include_directories(shift_register PUBLIC
    ${MIDI_LIB_DIR}
)
target_compile_options(shift_register PRIVATE -Wall -Wextra -Wpedantic)

add_executable(shift_register_tests
    ShiftRegisterMidiTest.cpp
    SequentialNotesTest.cpp
)
target_link_libraries(shift_register_tests
    PRIVATE
        shift_register
        gtest
        gtest_main
)
target_include_directories(shift_register_tests PRIVATE ${MIDI_LIB_DIR})

add_test(NAME shift_register_tests COMMAND shift_register_tests)


