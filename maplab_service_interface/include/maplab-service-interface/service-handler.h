#ifndef MAPLAB_SERVICE_HANDLER_H_
#define MAPLAB_SERVICE_HANDLER_H_

#include <maplab-service-interface/service-type.h>

#include <string>

namespace maplab {

class ServiceHandler final {
  public:
    static bool handleServiceCall(const ServiceType type);

  private:
    ServiceHandler() = default;

    static bool sanityServiceCall();
    static bool runMaplabConsole();
    static bool fetchAllMaps(); 

};

} // namespace maplab

#endif // MAPLAB_SERVICE_HANDLER_H_
