/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "loam_tools/camera.h"

namespace loam_tools {
  std::ostream &operator<<(std::ostream &os, const Camera &c) {
    os << "K: " << std::endl << c.K << std::endl << " dist: " << std::endl << c.dist << std::endl << " res: " << std::endl << c.res[0] << "x" << c.res[1];
    return (os);
  }
}
