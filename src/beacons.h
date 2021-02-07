#ifndef INCLUDE_AKFSFSIM_BEACONS_H
#define INCLUDE_AKFSFSIM_BEACONS_H

#include <vector>

class Display;

struct BeaconData
{
    double x,y;
    int id;
    BeaconData():x(0.0),y(0.0),id(-1){}
    BeaconData(double xPos, double yPos):x(xPos),y(yPos),id(-1){}
    BeaconData(double xPos, double yPos, int beaconId):x(xPos),y(yPos),id(beaconId){}
};

class BeaconMap
{
    public:

        BeaconMap();

        void addBeacon(double x, double y);

        BeaconData getBeaconWithId(int id) const;
        std::vector<BeaconData> getBeaconsWithinRange(double x, double y, double range) const;
        std::vector<BeaconData> getBeacons() const;

        void render(Display& disp) const;

    private:

        std::vector<BeaconData> m_beacon_map;    
};


#endif  // INCLUDE_AKFSFSIM_BEACONS_H