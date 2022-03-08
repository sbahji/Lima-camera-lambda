#ifndef LAMBDASYNCCTRLOBJ_H
#define LAMBDASYNCCTRLOBJ_H

#include "lima/Debug.h"
#include "LambdaInclude.h"
#include "lima/HwSyncCtrlObj.h"
#include "lima/HwInterface.h"

namespace lima
{
  namespace Lambda
  {
    class Camera;

    /*******************************************************************
     * \class SyncCtrlObj
     * \brief Control object providing Lambda synchronization interface
     *******************************************************************/

    class LIBLAMBDA_API SyncCtrlObj: public HwSyncCtrlObj
    {
    DEB_CLASS_NAMESPC(DebModCamera, "SyncCtrlObj", "Lambda");

    public:
    	SyncCtrlObj(Camera& cam);
    	virtual ~SyncCtrlObj();

    	virtual bool checkTrigMode(TrigMode trig_mode);
    	virtual void setTrigMode(TrigMode trig_mode);
    	virtual void getTrigMode(TrigMode& trig_mode);

    	virtual void setExpTime(double exp_time);
    	virtual void getExpTime(double& exp_time);

    	virtual void setLatTime(double lat_time);
    	virtual void getLatTime(double& lat_time);

    	virtual void setNbHwFrames(int nb_frames);
    	virtual void getNbHwFrames(int& nb_frames);

    	virtual void getValidRanges(ValidRangesType& valid_ranges);

    private:
    	Camera& m_cam;
    };

  } // namespace Lambda
} // namespace lima

#endif // LAMBDASYNCCTRLOBJ_H
