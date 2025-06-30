# Handle 'model' directory recursively
file(GLOB_RECURSE MODEL_SRC_FILES 
    "*.cpp"               # All .cpp files under model/ and its subdirs
    "*.h"                 
)
## Exclude files from a specific subdirectory (e.g., model/ignore_this/)
#list(FILTER MODEL_SRC_FILES EXCLUDE REGEX "mapReaders/MATLAB/.*")

# Add model sources to the project
target_sources(${PROJECT_NAME} PRIVATE ${MODEL_SRC_FILES})

# Include model directories recursively
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)