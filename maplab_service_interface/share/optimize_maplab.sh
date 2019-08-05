# Sparsify the landmarks a bit by filtering the distance
rtl --vi_map_landmark_quality_max_distance_from_closest_observer=15 --vi_map_landmark_quality_min_distance_from_closest_observer=0.25 --vi_map_landmark_quality_min_observation_angle_deg=5 --vi_map_landmark_quality_min_observers=5

# Alignment of maps if there are few loop closures
aam --lc_min_inlier_ratio 0.15 --lc_edge_min_inlier_ratio 0.15 --lc_ransac_pixel_sigma 3

# Relax if it keeps pulling the map back and forth
relax  --lc_edge_min_distance_meters=0.1

# loop closure on keyframed map
lc --lc_min_inlier_ratio=0.15 --lc_edge_min_inlier_ratio=0.15 --lc_ransac_pixel_sigma=3 --lc_only_against_other_missions=true --lc_min_inlier_count=7
