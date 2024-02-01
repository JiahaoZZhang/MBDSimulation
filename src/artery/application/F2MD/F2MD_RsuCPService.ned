package artery.application.F2MD;
import artery.application.ItsG5Service;

simple F2MD_RsuCPService like ItsG5Service
{
    parameters:
        
        double GenerationInterval @unit(s) = default(1.0s);
        double MAX_CPM_HISTORY_TIME = default(15.0);

        double RSUorientationAngle = default(0.0);
        int RSUroadReferenceID = default(0);
}