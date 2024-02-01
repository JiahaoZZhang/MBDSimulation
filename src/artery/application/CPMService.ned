package artery.application;
import artery.application.ItsG5Service;

simple CPMService like ItsG5Service
{
    parameters:
        @signal[CpmReceived](type=CaObject);
        @signal[CpmSent](type=CaObject);

        @statistic[reception](source=CpmReceived;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);
        @statistic[transmission](source=CpmSent;record=vector(cpmStationId)?,vector(cpmGenerationDeltaTime)?);

    double maxInterval @unit(s) = default(1.0s);
    bool fixedRate = default(true);
}