#ifndef AUTO_AIM__VOTER_HPP
#define AUTO_AIM__VOTER_HPP

#include <vector>

#include "armor.hpp"

namespace auto_aim
{
/*统计不同装甲板属性组合出现的次数。结合你在 RM 自瞄里做的事，这个东西一看就是：*/
class Voter
{
public:
  Voter();
  void vote(const Color color, const ArmorName name, const ArmorType type);
  std::size_t count(const Color color, const ArmorName name, const ArmorType type);

private:
  std::vector<std::size_t> count_;
  std::size_t index(const Color color, const ArmorName name, const ArmorType type) const;
};
}  // namespace auto_aim

#endif