#ifndef MAPLAB_SERVICE_HANDLER_H_
#define MAPLAB_SERVICE_HANDLER_H_

#include <maplab-service-interface/service-type.h>

#include <string>

namespace maplab {

class ServiceHandler final {
  public:
    ServiceHandler() = default;
    bool handleServiceCall(const ServiceType type);

  private:

    bool sanityServiceCall();
    bool runMaplabConsole();
    bool fetchAllMaps(); 
    bool optimizeMapsAlone(); 
    bool optimizeMapsTogether(); 

};

} // namespace maplab

#endif // MAPLAB_SERVICE_HANDLER_H_
