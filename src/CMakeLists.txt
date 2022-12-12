# add source files
file(GLOB_RECURSE SOURCE
    "./*"
)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE})

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    "./"
)