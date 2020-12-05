#ifndef IPOPT_PINO_FLOATING_BASE_HXX_
#define IPOPT_PINO_FLOATING_BASE_HXX_

#include "ipopt-pino//robot/floating_base.hpp"

namespace ipoptpino {

inline int FloatingBase::dim_passive() const {
  return dim_passive_;
}


inline std::vector<int> FloatingBase::passive_joint_indices() const {
  return passive_joint_indices_;
}


inline bool FloatingBase::has_floating_base() const {
  return has_floating_base_;
}

} // namespace ipoptpino 

#endif // IPOPT_PINO_FLOATING_BASE_HXX_ 