#ifndef LANELET2_EXTENSION_H
#define LANELET2_EXTENSION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include "hdmap/MapBin.h"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <sstream>
#include <string>

namespace lanelet
{
    namespace extension
    {
        // Map保存到二进制文件
        void toBinMsg(const lanelet::LaneletMapPtr &map, hdmap::MapBin *msg);
        void fromBinMsg(const hdmap::MapBin &msg, lanelet::LaneletMapPtr map);
    } //namespace extension
} //namespace lanelet

#endif