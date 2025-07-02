# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.


cmake_minimum_required (VERSION 3.31.6)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(ControlSim LANGUAGES CXX)

# Set C++23 globally
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Enable Hot Reload for MSVC
if(POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# Create empty executable first
add_executable(${PROJECT_NAME})

add_compile_definitions(ENABLE_DEBUG_DRAW=1)

# Path to MATLAB installation
set(MATLAB_ROOT "E:/Program Files/MATLAB/R2024b")

# Include directories (adjust if needed)
include_directories("${MATLAB_ROOT}/extern/include")

# Library directories
set(MATLAB_LIB_DIRS "${MATLAB_ROOT}/extern/lib/win64/microsoft")

link_directories("${MATLAB_LIB_DIRS}")


# Include subdirectories (they'll add sources)
add_subdirectory(external)
add_subdirectory(src)


# Link the required MATLAB libraries
target_link_libraries(${PROJECT_NAME}  PRIVATE
  "${MATLAB_LIB_DIRS}/libmat.lib"  
  "${MATLAB_LIB_DIRS}/libmx.lib"  
)

# Enable clang-tidy
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks" OFF)

if(ENABLE_CLANG_TIDY)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_CLANG_TIDY "clang-tidy;--extra-arg=/EHsc;-checks=clang-analyzer-*,
            cppcoreguidelines-*,
            modernize-*,
            performance-*,
            readability-*,
            bugprone-*,
            -readability-math-missing-parentheses,
            -modernize-use-trailing-return-type"
    )
endif()


# Copy media files to binary directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/car.png"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/car.png"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/TestTrack.mat"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/TestTrack.mat"
)
