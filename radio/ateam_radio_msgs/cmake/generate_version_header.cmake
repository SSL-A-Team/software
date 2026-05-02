# Copyright 2026 A Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


function(generate_version_header)
  set(SOFT_COMMS_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/software-communication")

  find_package(Git REQUIRED)

  execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
      WORKING_DIRECTORY ${SOFT_COMMS_SUBMODULE_DIR}
      OUTPUT_VARIABLE SOFT_COMMS_SUBMODULE_SHA
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
  )

  string(SUBSTRING "${SOFT_COMMS_SUBMODULE_SHA}" 0 8 SOFT_COMMS_SUBMODULE_SHA_SHORT)
  set(_hex_string "0x${SOFT_COMMS_SUBMODULE_SHA_SHORT}")
  math(EXPR SOFT_COMMS_SUBMODULE_SHA_SHORT_NUM "${_hex_string} + 0")

  execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain
    WORKING_DIRECTORY ${SOFT_COMMS_SUBMODULE_DIR}
    OUTPUT_VARIABLE SOFT_COMMS_SUBMODULE_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(SOFT_COMMS_SUBMODULE_STATUS)
    set(SOFT_COMMS_SUBMODULE_DIRTY true)
  else()
    set(SOFT_COMMS_SUBMODULE_DIRTY false)
  endif()

  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/templates/version.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/ateam_generated/include/version.hpp"
    @ONLY
  )

endfunction()
