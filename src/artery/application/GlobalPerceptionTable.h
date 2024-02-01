#ifndef ARTERY_GLOBALPERCEPTIONTABLE_H_AL7SS9KT
#define ARTERY_GLOBALPERCEPTIONTABLE_H_AL7SS9KT

#include "artery/application/CaObject.h"
#include "artery/application/CpmObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <cstdint>
#include <functional>
#include <map>

namespace artery
{

class Timer;

class GlobalPerceptionTable
{
public:
    using StationID = uint32_t;
    using Cam = vanetza::asn1::Cam;
    using Cpm = vanetza::asn1::Cpm;

    using CpmPredicate = std::function<bool(const Cpm&)>;
    
    const LocalEnvironmentModel* mlocalEnvmod;      


    GlobalPerceptionTable(const Timer&);
    void updatePerception(const CpmObject&);
    void dropExpired();
    unsigned count(const CpmPredicate&) const;

private:
    struct PerceptionEntry
    {
        PerceptionEntry(const CpmObject&, omnetpp::SimTime);
        PerceptionEntry(PerceptionEntry&&) = default;
        PerceptionEntry& operator=(PerceptionEntry&&) = default;

        omnetpp::SimTime expiry;
        CpmObject object;

    };

    const Timer& mTimer;
    std::map<StationID, PerceptionEntry> mCpMessages;
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

