#ifndef MAPLAB_SERVICE_TYPE_H_
#define MAPLAB_SERVICE_TYPE_H_

namespace maplab {

enum class ServiceType {
  kSanityService = 0u,
  kRunMaplabConsole = 1u, 
  kFetchAllMaps = 2u,
  kOptimizeMapsAlone = 3u,
  kOptimizeMapsTogether = 4u,
};

} // namespace maplab

#endif // MAPLAB_SERVICE_TYPE_H_
