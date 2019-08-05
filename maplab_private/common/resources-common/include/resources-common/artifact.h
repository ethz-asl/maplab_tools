#ifndef RESOURCES_COMMON_ARTIFACT_H_
#define RESOURCES_COMMON_ARTIFACT_H_

#include <maplab-common/file-system-tools.h>

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

namespace resources {

struct Artifact {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  float confidence;
  std::string class_id;
  int64_t timestamp;

  std::vector<double> T_S_Artifact;

  bool operator==(const Artifact& other) const {
    bool is_same = (confidence - other.confidence) < 1e-3;
    is_same &= timestamp == other.timestamp;
    is_same &= class_id == other.class_id;
    return is_same;
  }

  void print(std::ostream& out = std::cout) const {
    out << class_id << "; " << confidence << "; "; 
    std::copy(T_S_Artifact.begin(), T_S_Artifact.end(), 
        std::ostream_iterator<double>(out, " "));
    out << "\n";
  }
};

} // namespace resources

#endif // RESOURCES_COMMON_ARTIFACT_H_
