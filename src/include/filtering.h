/*
 * filtering.h
 * Interface to filter a point cloud
 * Part of Floor Segmentation project
 * Copyright (C) 2013 Jose Pardeiro <jose.pardeiro@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef FILTERING_H_
#define FILTERING_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class Filtering {
public:
	Filtering();
	virtual ~Filtering();
	
/**
	 * Function to performs the point cloud filtering
	 * @param cloud input cloud on PointXYZRGBA format
	 * @return filtered point cloud on PointXYZRGBA format
	 * */
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filter (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
};

#endif /* FILTERING_H_ */
