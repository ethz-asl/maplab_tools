#ifndef VI_MAP_DATA_IMPORT_EXPORT_EXPORT_ARTIFACT_DETECTIONS_H_
#define VI_MAP_DATA_IMPORT_EXPORT_EXPORT_ARTIFACT_DETECTIONS_H_

#include <string>

#include <vi-map/vi-map.h>

namespace data_import_export {

int exportArtifactDetections(const vi_map::MissionIdList& mission_ids,
    const vi_map::VIMap& map, const std::string& output_folder);

}  // namespace data_import_export

#endif  // VI_MAP_DATA_IMPORT_EXPORT_EXPORT_ARTIFACT_DETECTIONS_H_
