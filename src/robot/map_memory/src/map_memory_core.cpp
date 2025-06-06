#include "map_memory_core.hpp"

using namespace std;
namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

// Costmap callback function
void MapMemoryCore::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
// Store the latest costmap (From psuedocode)
  latest_costmap = *msg;
  costmap_updated = true;
}

// Callback for odometry updates from psuedocode
void MapMemoryCore::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Compute distance traveled
    double distance = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map = true;
    }
}

// Timer-based map update
void MapMemoryCore::updateMap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& map_pub) {
    if (should_update_map && costmap_updated) {
        integrateCostmap();
        global_map.header.stamp = rclcpp::Clock().now();
        global_map.header.frame_id = "sim_world";
        map_pub->publish(global_map);
        should_update_map = false;
        costmap_updated = false;
    }
}

void MapMemoryCore::integrateCostmap() {
  // Initial Integration of costmap into global map
  if (global_map.data.empty()) {
     // Set map metadata
        global_map.info.resolution = 0.1;
        global_map.info.width = 300;  
        global_map.info.height = 300; 
        // Bottom-left corner in global frame
        global_map.info.origin.position.x = -15.0; 
        global_map.info.origin.position.y = -15.0;
        // Currently no rotation
        global_map.info.origin.orientation.w = 1.0; 
        // Initialize all cells to unknown (-1)
        global_map.data.resize(global_map.info.width * global_map.info.height, -1);
        

        return;  // Exit early since no costmap merging is needed yet
  }

  // 2. Merge the latest costmap cells into global_map
  for (size_t i = 0; i < latest_costmap.info.height; ++i) {
    for (size_t j = 0; j < latest_costmap.info.width; ++j) {

      int costmap_index = static_cast<int>(i * latest_costmap.info.width + j);

      // If unknown, skip
      if (latest_costmap.data[costmap_index] == -1) {
        continue;
      }

      // Real-world position in costmap’s frame
      double local_x = latest_costmap.info.origin.position.x + (j + 0.5) * latest_costmap.info.resolution;
      double local_y = latest_costmap.info.origin.position.y + (i + 0.5) * latest_costmap.info.resolution;

      // Convert that to global map coordinates
      double gx = (local_x - global_map.info.origin.position.x) / global_map.info.resolution;
      double gy = (local_y - global_map.info.origin.position.y) / global_map.info.resolution;

      int global_x = static_cast<int>(std::floor(gx));
      int global_y = static_cast<int>(std::floor(gy));

      // Bounds check
      if (global_x < 0 || global_y < 0 || global_x >= static_cast<int>(global_map.info.width) || global_y >= static_cast<int>(global_map.info.height))
        continue;
      
      int global_index = global_y * global_map.info.width + global_x;

      // Merge the cell
      if (latest_costmap.data[costmap_index] > 80 && global_map.data[global_index] < 20) {
        global_map.data[global_index] = latest_costmap.data[costmap_index] * 0.8 + global_map.data[global_index] * 0.2;
      }
      else {
        global_map.data[global_index] = latest_costmap.data[costmap_index] * 0.5 + global_map.data[global_index] * 0.5;
      }
    }
  }
}
}