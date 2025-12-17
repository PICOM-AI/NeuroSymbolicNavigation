cd ../..
touch ./hdmap/map_files/pointcloud_map.pcd
docker run --rm -it -e QT_QPA_PLATFORM=offscreen \
-v $(pwd)/hdmap/map_files/map.osm:/app/map.osm \
-v $(pwd)/hdmap/map_files/3D_Model:/app/3D_Model \
-v $(pwd)/hdmap/map_files/pointcloud_map.pcd:/app/pointcloud_map.pcd \
hiveintel/osm-3d-pcd-pipeline:latest /bin/bash