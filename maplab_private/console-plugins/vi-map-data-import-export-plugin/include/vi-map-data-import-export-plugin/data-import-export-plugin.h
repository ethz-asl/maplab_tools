#ifndef VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_
#define VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base.h>
#include <console-common/console.h>
#include <vi-map/vi-map.h>
#include <map-manager/map-manager.h>


namespace data_import_export {

class DataImportExportPlugin : public common::ConsolePluginBase {
 public:
  explicit DataImportExportPlugin(common::Console* console);

  std::string getPluginId() const override {
    return "data_import_export";
  }

 private:
  int exportMissionInfo() const;
  int exportPosesVelocitiesAndBiasesToCsv() const;
  int exportNCameraCalibration() const;
  int importGpsDataFromRosbag() const;
  int exportGpsUtmToCsv() const;
  int exportGpsWgsToCsv() const;
  int exportArtifactDetections() const;
  int getAllMissionIds(
      vi_map::VIMapManager::MapWriteAccess& map,
      vi_map::MissionIdList* mission_ids) const;
};

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_PLUGIN_DATA_IMPORT_EXPORT_PLUGIN_H_