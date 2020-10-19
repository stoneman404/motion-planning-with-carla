#include "traffic_lights/traffic_light_list.hpp"
namespace planning {

TrafficLightList &TrafficLightList::Instance() {
  static TrafficLightList instance;
  return instance;
}
}