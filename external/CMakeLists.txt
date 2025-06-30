include(FetchContent)

FetchContent_Declare(
  sfml
  GIT_REPOSITORY "https://github.com/SFML/SFML"
  GIT_TAG 5383d2b3948f805af55c9f8a4587ac72ec5981d1 # 2.6.2
)

message(STATUS "Fetching SFML...")

# No need to build audio and network modules
set(SFML_BUILD_AUDIO FALSE)
set(SFML_BUILD_NETWORK FALSE)

FetchContent_MakeAvailable(sfml)


FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui
  GIT_TAG c6e0284ac58b3f205c95365478888f7b53b077e2 # 1.89.9
)

FetchContent_MakeAvailable(imgui)

FetchContent_Declare(
  imgui-sfml
  GIT_REPOSITORY https://github.com/SFML/imgui-sfml
  GIT_TAG 64e09feda29266572799e5ded66eac5cb0a749aa # 2.6.x
)

message(STATUS "Fetching ImGui-SFML...")

set(IMGUI_DIR ${imgui_SOURCE_DIR})
set(IMGUI_SFML_FIND_SFML OFF)
set(IMGUI_SFML_IMGUI_DEMO ON)

FetchContent_MakeAvailable(imgui-sfml)
