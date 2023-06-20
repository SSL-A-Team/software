// Copyright 2021 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef ATEAM_COMMON__STATUS_HPP_
#define ATEAM_COMMON__STATUS_HPP_

// Use backtrace so we get line numbers and files
#define BOOST_STACKTRACE_USE_ADDR2LINE

#include <utility>
#include <string>
#include <iostream>

#include <boost/outcome.hpp>
#include <boost/stacktrace.hpp>

// Utilities to improve the error checking workflow
//
// We now have access to a ateam::Status or ateam::StatusOr<T> type.
// Both are used as a return from a function to signal success or faliure
//
// Eg:
// ateam::Status log_to_console(std::string);
// or
// ateam::StatusOr<double> sqrt(double);
//
// The sqrt function will then be able to return a failure if the input is invalid.
// Eg:
// ateam::StatusOr<double> sqrt(double input) {
//   ATEAM_SCHECK(input >= 0, "input must be non-negative");
//   return math.sqrt(input);
// }
//
// This will allow significant improvements to safety surrounding assumptions a function
// can ensure.


// From the user POV, we can simplify catching and throwing these exceptions.
// There are two main classes user functions, ATEAM_ASSIGN_OR_* and ATEAM_*CHECK
//
// ATEAM_ASSIGN_OR_RETURN(auto var, func_that_returns_statusor(), "failure string");
// will set var to the result of the function if and only if, the function doesn't fail.
// If it fails, it will propegate the failure up the stack
//
// ATEAM_ASSIGN_OR_THROW(auto var, func_that_returns_statusor(), "failure string");
// will set var to the result of the function if and only if, the function doesn't fail.
// If it fails, it will throw an exception
//
// ATEAM_SCHECK(condition, "failure string") will check that the condition is true.
// If the condition is false, it will return a status::Failure with the given failure string.
//
// ATEAM_CHECK(condition, "failure string") will check that the condition is true.
// If the condition is false, it will throw an exception with the given failure string.
//
// In most code, you will always use ATEAM_ASSIGN_OR_RETURN or ATEAM_SCHECK to propogate the
// errors correctly.


namespace ateam
{

class ErrorType
{
public:
  std::string cause;
  std::stringstream stack_trace;

  ErrorType() {}

  explicit ErrorType(std::string cause)
  : cause(cause)
  {
    stack_trace << boost::stacktrace::stacktrace() << std::endl;
  }
};

std::ostream & operator<<(std::ostream & os, const ErrorType & error);

using Status = BOOST_OUTCOME_V2_NAMESPACE::result<void, ErrorType,
    BOOST_OUTCOME_V2_NAMESPACE::policy::terminate>;

template<typename T>
using StatusOr = BOOST_OUTCOME_V2_NAMESPACE::result<T, ErrorType,
    BOOST_OUTCOME_V2_NAMESPACE::policy::terminate>;

// Wrappers around success / failure so we can simplify the interface
Status Ok();
Status Failure(std::string && s);

template<typename T>
StatusOr<T> Ok(T && s)
{
  return BOOST_OUTCOME_V2_NAMESPACE::success(std::move(s));
}
template<typename T>
StatusOr<T> Failure(std::string && s)
{
  return ErrorType(std::move(s));
}

#define ATEAM_ASSIGN_OR_RETURN(lhs, rhs, text) \
  lhs = ({auto __status_or_lhs = rhs; \
      if (!__status_or_lhs) { \
        return ateam::Failure(text); \
      } \
      __status_or_lhs.value()});

#define ATEAM_ASSIGN_OR_THROW(lhs, rhs, text) \
  lhs = ({auto __status_or_lhs = rhs; \
      if (!__status_or_lhs) { \
        std::stringstream s; \
        s << text << "\n" << __status_or_lhs.error() << std::endl; \
        std::cerr << s.str(); \
        throw s.str(); \
      } __status_or_lhs.value();});

#define ATEAM_SCHECK(x, text) \
  do { \
    auto __value = x; \
    if (!__value) { \
      return ateam::Failure(text); \
    } \
  } while (false);

#define ATEAM_CHECK(x, text) \
  do { \
    auto __value = x; \
    if (!__value) { \
      std::stringstream s; \
      s << ateam::Failure(text).error() << std::endl; \
      std::cerr << s.str(); \
      throw s.str(); \
    } \
  } while (false);

}  // namespace ateam
#endif  // ATEAM_COMMON__STATUS_HPP_
