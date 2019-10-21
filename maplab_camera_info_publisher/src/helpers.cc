#include <maplab-camera-info-publisher/helpers.h>
#include <sensor_msgs/image_encodings.h>

namespace maplab {

const std::string getEncoding(const bool conversion_flag) {
	if (conversion_flag)
		return sensor_msgs::image_encodings::MONO8;
	else 
		return sensor_msgs::image_encodings::BGR8;
}

} // namespace maplab
