cplusplus {{
#include "veins/base/utils/Coord.h"
#include <inet/linklayer/ieee80211/mac/Ieee80211Mac.h>
}}

namespace veins;

class noncobject Coord;
class inet::MACAddress extends void;

packet BSM
{
    Coord senderPos;
    Coord senderSpeed;
    Coord senderAccel;
    Coord senderHeading;
    double senderWidth;
    double senderLength;
    inet::MACAddress senderAddress;

    Coord senderPosConfidence;
    Coord senderSpeedConfidence;
    Coord senderHeadingConfidence;
    Coord senderAccelConfidence;
    int senderMbType;
    int senderAttackType;
    Coord senderGpsCoordinates;
    unsigned long senderPseudonym;
};
