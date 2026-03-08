cmake_minimum_required(VERSION 3.14)
project(WillowSDK VERSION 5.2.0 LANGUAGES CXX)

# Declare Willow as a header-only INTERFACE library
add_library(willow INTERFACE)
add_library(Willow::willow ALIAS willow)

# Expose the include/ directory to consumers
target_include_directories(willow INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optional CURL dependency feature flag
option(WILLOW_ENABLE_CURL "Enable built-in HTTP client using libcurl" OFF)

if(WILLOW_ENABLE_CURL)
    find_package(CURL REQUIRED)
    target_link_libraries(willow INTERFACE CURL::libcurl)
    target_compile_definitions(willow INTERFACE WILLOW_ENABLE_CURL)
endif()