#ifndef LAMBDAINTERFACE_H
#define LAMBDAINTERFACE_H

#include "lima/Debug.h"
#include "LambdaCompatibility.h"
#include "lima/HwInterface.h"
#include "lima/HwBufferMgr.h"


using namespace std;

namespace lima
{
namespace Lambda
{
class Camera;
class DetInfoCtrlObj;
class SyncCtrlObj;
/*******************************************************************
 * \class Interface
 * \brief Lambda hardware interface
 *******************************************************************/

class LIBLAMBDA_API Interface: public HwInterface
{
DEB_CLASS_NAMESPC(DebModCamera, "LambdaInterface", "Lambda");

public:
	Interface(Camera& cam);
	virtual ~Interface();

	//- From HwInterface
	virtual void getCapList(CapList&) const;
	virtual void reset(ResetLevel reset_level);
	virtual void prepareAcq();
	virtual void startAcq();
	virtual void stopAcq();
	virtual void getStatus(StatusType& status);
	virtual int getNbHwAcquiredFrames();
	
    //! get the camera object to access it directly from client
    Camera& getCamera() { return m_cam;}

private:
	Camera& m_cam;
	CapList m_cap_list;
	DetInfoCtrlObj* m_det_info;
	SyncCtrlObj* m_sync;
    HwEventCtrlObj * m_event;
};

} // namespace Lambda
} // namespace lima

#endif // LAMBDAINTERFACE_H
