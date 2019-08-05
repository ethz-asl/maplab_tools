#include <maplab-service-interface/service-handler.h>
#include <glog/logging.h>
#include <cstdlib>

namespace maplab {

static const std::string kScriptFolder = "./home/berlukas/Documents/scripts/";
static const std::string kSanityScript = kScriptFolder + "test.sh";

bool ServiceHandler::handleServiceCall(const ServiceType type) {
  switch (type) {
    case ServiceType::kSanityService:
      return ServiceHandler::sanityServiceCall();

    case ServiceType::kRunMaplabConsole:
      return ServiceHandler::runMaplabConsole();

    case ServiceType::KFetchAllMaps:
      return ServiceHandler::fetchAllMaps();

    default:
      LOG(ERROR) << "Unkown service call type.";
      return false;
  }
}

bool ServiceHandler::sanityServiceCall() {
  LOG(INFO) << "--- CALLED THE SANITY SERVICE CALL -----------------------------";
  return system(kSanityScript.c_str()) == 0;
}

bool ServiceHandler::runMaplabConsole() {
  LOG(INFO) << "[MaplabServiceInterface] Starting maplab console";
  return true; 
}

bool ServiceHandler::fetchAllMaps() {
  LOG(INFO) << "[MaplabServiceInterface] Synchronizing with all maps from robots";
  return true; 
}

} // namespace maplab
