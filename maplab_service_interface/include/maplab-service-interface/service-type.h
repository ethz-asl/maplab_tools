#ifndef MAPLAB_SERVICE_TYPE_H_
#define MAPLAB_SERVICE_TYPE_H_

namespace maplab {

enum class ServiceType {
  kSanityService = 0u,
  kRunMaplabConsole = 1u, 
  KFetchAllMaps = 2u,
};

} // namespace maplab

#endif // MAPLAB_SERVICE_TYPE_H_
