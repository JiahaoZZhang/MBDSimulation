#include "artery/application/GlobalPerceptionTable.h"
#include "artery/application/Timer.h"
#include <omnetpp/csimulation.h>
#include <cassert>
#include <algorithm>

namespace artery
{

GlobalPerceptionTable::GlobalPerceptionTable(const Timer& timer) :
    mTimer(timer)
{
}

void GlobalPerceptionTable::updatePerception(const CpmObject& obj)
{
    const vanetza::asn1::Cpm& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->cpm.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received CAM is out of bounds";
        return;
    }

    PerceptionEntry entry(obj, expiry);
    auto found = mCpMessages.find(msg->header.stationID);
    if (found != mCpMessages.end()) {
        found->second = std::move(entry);
    } else {
        mCpMessages.emplace(msg->header.stationID, std::move(entry));
    }
}

void GlobalPerceptionTable::dropExpired()
{
    const auto now = omnetpp::simTime();
    for (auto it = mCpMessages.begin(); it != mCpMessages.end();) {
        if (it->second.expiry < now) {
            it = mCpMessages.erase(it);
        } else {
            ++it;
        }
    }
}

unsigned GlobalPerceptionTable::count(const CpmPredicate& predicate) const
{
    return std::count_if(mCpMessages.begin(), mCpMessages.end(),
            [&predicate](const std::pair<const StationID, PerceptionEntry>& map_entry) {
                const Cpm& cpm = map_entry.second.object.asn1();
                return predicate(cpm);
            });
}

GlobalPerceptionTable::PerceptionEntry::PerceptionEntry(const CpmObject& obj, omnetpp::SimTime t) :
    expiry(t), object(obj)
{
}

} // namespace artery
