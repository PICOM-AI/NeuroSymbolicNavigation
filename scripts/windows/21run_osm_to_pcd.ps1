Set-Location ..\..
$currentDir = (Get-Location).Path
New-Item -ItemType File -Path "${currentDir}/hdmap/map_files/pointcloud_map.pcd" -Force | Out-Null
docker run --rm -it -e QT_QPA_PLATFORM=offscreen `
    -v "${currentDir}/hdmap/map_files/map.osm:/app/map.osm" `
    -v "${currentDir}/hdmap/map_files/3D_Model:/app/3D_Model" `
    -v "${currentDir}/hdmap/map_files/pointcloud_map.pcd:/app/pointcloud_map.pcd" `
    hiveintel/osm-3d-pcd-pipeline:latest /bin/bash

