#include "extension.h"

namespace lanelet
{
    namespace extension
    {
        void toBinMsg(const lanelet::LaneletMapPtr &map, hdmap::MapBin *msg)
        {
            if (msg == nullptr)
            {
                ROS_ERROR_STREAM(__FUNCTION__ << "msg is null pointer!");
                return;
            }

            std::stringstream ss;
            boost::archive::binary_oarchive oa(ss);
            oa << *map;
            auto id_counter = lanelet::utils::getId();
            oa << id_counter;

            std::string data_str(ss.str());

            msg->data.clear();
            msg->data.assign(data_str.begin(), data_str.end());
        }

        void fromBinMsg(const hdmap::MapBin &msg, lanelet::LaneletMapPtr map)
        {
            if (!map)
            {
                ROS_ERROR_STREAM(__FUNCTION__ << ": map is null pointer!");
                return;
            }

            std::string data_str;
            data_str.assign(msg.data.begin(), msg.data.end());

            std::stringstream ss;
            ss << data_str;
            boost::archive::binary_iarchive oa(ss);
            oa >> *map;
            lanelet::Id id_counter;
            oa >> id_counter;
            lanelet::utils::registerId(id_counter);
            // *map = std::move(laneletMap);
        }
    } //namespace extension
} //namespace lanelet
