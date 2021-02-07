#include "beacons.h"

#include <cmath>
#include <random>
#include "display.h"

BeaconMap::BeaconMap()
{
    std::mt19937 rand_gen;
    std::uniform_real_distribution<double> pos_dis(-500.0,500.0);
    for (int i =0; i < 200; i++){addBeacon(pos_dis(rand_gen),pos_dis(rand_gen));}
}

void BeaconMap::addBeacon(double x, double y)
{
    m_beacon_map.push_back(BeaconData(x,y,m_beacon_map.size()));
}

BeaconData BeaconMap::getBeaconWithId(int id) const
{
    for (const BeaconData& beacon : m_beacon_map){if (beacon.id == id){return beacon;}}
    return BeaconData();
}

std::vector<BeaconData> BeaconMap::getBeaconsWithinRange(double x, double y, double range) const
{
    std::vector<BeaconData> beacons;
    for (const BeaconData& beacon : m_beacon_map)
    {
        double delta_x = beacon.x - x;
        double delta_y = beacon.y - y;
        double beacon_range = std::sqrt(delta_x*delta_x + delta_y*delta_y);
        if (beacon_range < range)
        {
            beacons.push_back(beacon);
        }
    }
    return beacons;
}

std::vector<BeaconData> BeaconMap::getBeacons() const
{
    return m_beacon_map;
}

void BeaconMap::render(Display& disp) const
{
    const std::vector<Vector2> beacon_lines = {{1,0},{0,1},{0,-1},{1,0}};
    disp.setDrawColour(255,255,0);
    for (const auto& beacon : m_beacon_map){disp.drawLines(offsetPoints(beacon_lines, Vector2(beacon.x,beacon.y)));}
}