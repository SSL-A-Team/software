#include "ateam_common/status.hpp"

std::ostream & ateam::operator<<(std::ostream & os, const ErrorType & error)
{
  os << error.cause << "\n\n" << error.stack_trace.str();
  return os;
}

ateam::Status ateam::Ok()
{
  return BOOST_OUTCOME_V2_NAMESPACE::success();
}
ateam::Status ateam::Failure(std::string && s)
{
  return ateam::ErrorType(std::move(s));
}