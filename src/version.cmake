# Generate build version header from git info
# Called at build time by CMake

# Get git commit hash
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${GIT_DIR}
    OUTPUT_VARIABLE GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_COMMIT)
    set(GIT_COMMIT "unknown")
endif()

# Get git tag (if on a tag)
execute_process(
    COMMAND git describe --tags --exact-match HEAD
    WORKING_DIRECTORY ${GIT_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_TAG)
    set(GIT_TAG "")
endif()

# Get branch name
execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${GIT_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT GIT_BRANCH)
    set(GIT_BRANCH "unknown")
endif()

# Check if working tree is dirty
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${GIT_DIR}
    OUTPUT_VARIABLE GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(GIT_STATUS)
    set(GIT_DIRTY "-dirty")
else()
    set(GIT_DIRTY "")
endif()

# Get build timestamp
string(TIMESTAMP BUILD_TIME "%Y-%m-%d %H:%M:%S" UTC)

# Generate the header - compute version string at cmake time, not compile time
if(GIT_TAG)
    set(VERSION_STRING "${GIT_TAG} (${GIT_COMMIT}${GIT_DIRTY})")
else()
    set(VERSION_STRING "${GIT_BRANCH}/${GIT_COMMIT}${GIT_DIRTY}")
endif()

file(WRITE ${OUTPUT_FILE}
"/* Auto-generated build version - DO NOT EDIT */
#ifndef ZMK_BUILD_VERSION_H
#define ZMK_BUILD_VERSION_H

#define ZMK_BUILD_COMMIT \"${GIT_COMMIT}${GIT_DIRTY}\"
#define ZMK_BUILD_TAG \"${GIT_TAG}\"
#define ZMK_BUILD_BRANCH \"${GIT_BRANCH}\"
#define ZMK_BUILD_TIME \"${BUILD_TIME}\"
#define ZMK_BUILD_VERSION \"${VERSION_STRING}\"

#endif /* ZMK_BUILD_VERSION_H */
")

message(STATUS "Generated build version: ${GIT_BRANCH}/${GIT_COMMIT}${GIT_DIRTY}")
