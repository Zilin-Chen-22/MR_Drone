# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-src")
  file(MAKE_DIRECTORY "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-src")
endif()
file(MAKE_DIRECTORY
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-build"
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix"
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/tmp"
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/src/mdss-populate-stamp"
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/src"
  "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/src/mdss-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/src/mdss-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/zilin/Documents/UofT/beta_fly_sim/build_debug/_deps/mdss-subbuild/mdss-populate-prefix/src/mdss-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
