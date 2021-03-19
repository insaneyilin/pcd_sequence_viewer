#!/bin/bash

set -e

BASEDIR=$(dirname $0)
JOBS=$(grep -c ^processor /proc/cpuinfo)

MAKEFLAGS=" -j${JOBS} -l${JOBS}"
CMAKE_OPTIONS="-DCMAKE_BUILD_TYPE=Release"

# for printing colored text
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NONE='\033[0m'

function clean() {
  if [ -d build ]; then
    rm -r build
  fi
  if [ -d output ]; then
    rm -r output
  fi
  echo -e "\n${BLUE}build & output removed.${NONE}"
}

function run_cmake() {
  echo -e "\n${BLUE}Run cmake ...${NONE}"
  if [ ! -d build ]; then
    mkdir -p build
  fi
  cd build && cmake ${CMAKE_OPTIONS} ..
  cd -
}

function run_build() {
  if [ ! -d build ]; then
    run_cmake
  fi
  echo -e "\n${BLUE}Start compiling ...${NONE}"
  cd build && make ${MAKEFLAGS}
  cd -
}

function print_usage() {
  echo -e "\n${GREEN}Usage${NONE}:
  .${BOLD}/build.sh${NONE} [OPTION]"
  echo -e "\n${GREEN}Options${NONE}:
  ${BLUE}all${NONE}:     clean all and build
  ${BLUE}partial${NONE}: incremental build
  ${BLUE}cmake${NONE}:    run cmake
  ${BLUE}clean${NONE}:   clean all
  ${BLUE}help${NONE}:    print usage
  "
}

function main() {
  local option=$1

  case $option in
    all)
      clean
      run_cmake
      run_build
      ;;
    partial)
      run_build
      ;;
    cmake)
      run_cmake
      ;;
    clean)
      clean
      ;;
    help)
      print_usage
      ;;
    *)
      print_usage
      ;;
  esac
}

main $@
