# Copyright 2025 A Team
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

function(generate_conversion_code)
  set(oneValueArgs SOURCE DESTINATION)
  set(multiValueArgs STRUCTS)
  cmake_parse_arguments(PARSE_ARGV 0 arg "" "${oneValueArgs}" "${multiValueArgs}")
  if(NOT arg_SOURCE)
    message(FATAL_ERROR "Source header must be specified.")
  endif()
  if(NOT arg_DESTINATION)
    message(FATAL_ERROR "Destination directory must be specified.")
  endif()
  if(NOT arg_STRUCTS)
    message(FATAL_ERROR "At least one struct must be specified.")
  endif()

  file(MAKE_DIRECTORY "${arg_DESTINATION}")

  set(_generation_script "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_conversion_code.py")
  if(NOT EXISTS "${_generation_script}")
    message(FATAL_ERROR "Script ${_generation_script} does not exist.")
  endif()

  set(${generated_msgs_files} "")
  foreach(struct ${arg_STRUCTS})
    list(APPEND generated_msgs_files "${arg_DESTINATION}/${struct}.msg")
  endforeach()

  execute_process(
    COMMAND python3 ${_generation_script} ${arg_DESTINATION} ${arg_SOURCE} ${arg_STRUCTS}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  if(result)
    message(FATAL_ERROR "Failed to generate conversion code: ${error}")
  endif()
endfunction()
