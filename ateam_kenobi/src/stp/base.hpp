#ifndef STP__BASE_HPP_
#define STP__BASE_HPP_

#include <string>
#include <nlohmann/json.hpp>
#include "visualization/overlays.hpp"

namespace ateam_kenobi::stp
{

struct Options
{
  std::string name;
  visualization::Overlays overlays;
  nlohmann::json play_info;
};

class Base
{
public:
  explicit Base(std::string name)
  : name_(name)
  {
  }

  explicit Base(Options options)
  : name_(options.name),
    overlays_(options.overlays),
    play_info_(options.play_info)
  {
  }

  virtual ~Base() = default;

  template<typename ChildType, typename ... Args>
  ChildType createChild(std::string child_name, Args &&... args)
  {
    Options options{
      child_name,
      overlays_.getChild(child_name),
      play_info_[child_name]
    };
    return ChildType(options, std::forward<Args>(args)...);
  }

  const std::string & getName() const
  {
    return name_;
  }

  visualization::Overlays & getOverlays()
  {
    return overlays_;
  }

  nlohmann::json & getPlayInfo()
  {
    return play_info_;
  }

private:
  std::string name_;
  visualization::Overlays overlays_;
  nlohmann::json play_info_;

};

}  // namespace ateam_kenobi::stp

#endif  // STP__BASE_HPP_
