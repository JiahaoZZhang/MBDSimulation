#include <artery/application/CpmObject.h>
#include <omnetpp.h>
#include <cassert>

namespace artery
{

using namespace vanetza::asn1;

Register_Abstract_Class(CpmObject)

CpmObject::CpmObject(Cpm&& cpm) :
    m_cpm_wrapper(std::make_shared<Cpm>(std::move(cpm)))
{
}

CpmObject& CpmObject::operator=(Cpm&& cpm)
{
    m_cpm_wrapper = std::make_shared<Cpm>(std::move(cpm));
    return *this;
}

CpmObject::CpmObject(const Cpm& cpm) :
    m_cpm_wrapper(std::make_shared<Cpm>(cpm))
{
}

CpmObject& CpmObject::operator=(const Cpm& cpm)
{
    m_cpm_wrapper = std::make_shared<Cpm>(cpm);
    return *this;
}

CpmObject::CpmObject(const std::shared_ptr<const Cpm>& ptr) :
    m_cpm_wrapper(ptr)
{
    assert(m_cpm_wrapper);
}

CpmObject& CpmObject::operator=(const std::shared_ptr<const Cpm>& ptr)
{
    m_cpm_wrapper = ptr;
    assert(m_cpm_wrapper);
    return *this;
}

std::shared_ptr<const Cpm> CpmObject::shared_ptr() const
{
    assert(m_cpm_wrapper);
    return m_cpm_wrapper;
}

const vanetza::asn1::Cpm& CpmObject::asn1() const
{
    return *m_cpm_wrapper;
}


using namespace omnetpp;

class CpmStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CpmObject*>(object)) {
            const auto id = cpm->asn1()->header.messageId;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("cpmStationId", CpmStationIdResultFilter)


class CpmGenerationTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto cpm = dynamic_cast<CpmObject*>(object)) {
            unsigned long generationTime;
            asn_INTEGER2ulong(&cpm->asn1()->payload.managementContainer.referenceTime,&generationTime);
            fire(this, t, generationTime, details);
        }
    }
};



Register_ResultFilter("cpmGenerationTime", CpmGenerationTimeResultFilter)

} // namespace artery