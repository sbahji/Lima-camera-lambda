
#include "LambdaCamera.h"
#include "LambdaDetInfoCtrlObj.h"
#include "LambdaSyncCtrlObj.h"
#include "LambdaInterface.h"


using namespace lima;
using namespace lima::Lambda;
using namespace std;



/*******************************************************************
 * \brief Hw Interface constructor
 *******************************************************************/

Interface::Interface(Camera& cam) :
		m_cam(cam)
{
	DEB_CONSTRUCTOR();
	m_det_info = new DetInfoCtrlObj(cam);
	m_sync = new SyncCtrlObj(cam);
	HwBufferCtrlObj *m_bufferCtrlObj = cam.getBufferCtrlObj();

	m_cap_list.push_back(HwCap(m_det_info));
	m_cap_list.push_back(HwCap(m_bufferCtrlObj));
	m_cap_list.push_back(HwCap(m_sync));

    //event capability
    m_event = cam.getEventCtrlObj();    
    m_cap_list.push_back(HwCap(m_event));

	Size image_size;
	m_det_info->getMaxImageSize(image_size);
	ImageType image_type;
	m_det_info->getDefImageType(image_type);
	FrameDim frame_dim(image_size, image_type);
	m_bufferCtrlObj->setFrameDim(frame_dim);
	m_bufferCtrlObj->setNbConcatFrames(1);
	m_bufferCtrlObj->setNbBuffers(2);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
Interface::~Interface()
{
	DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
	DEB_MEMBER_FUNCT();
	cap_list = m_cap_list;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(reset_level);

	stopAcq();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::prepareAcq()
{
	DEB_MEMBER_FUNCT();

	m_cam.prepareAcq();

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::startAcq()
{
	DEB_MEMBER_FUNCT();
	m_cam.startAcq();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::stopAcq()
{
	DEB_MEMBER_FUNCT();
	m_cam.stopAcq();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{

	Camera::Status camera_status = m_cam.getStatus();
	switch (camera_status)
    {
    case Camera::Ready:
      status.set(HwInterface::StatusType::Ready);
      break;
    case Camera::Exposure:
      status.set(HwInterface::StatusType::Exposure);
      break;
    case Camera::Readout:
      status.set(HwInterface::StatusType::Readout);
      break;
    case Camera::Latency:
      status.set(HwInterface::StatusType::Latency);
      break;
    case Camera::Fault:
      status.set(HwInterface::StatusType::Fault);
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
	DEB_MEMBER_FUNCT();
	int aNbAcquiredFrames;

	aNbAcquiredFrames = m_cam.getNbAcquiredFrames();

	DEB_RETURN() << DEB_VAR1(aNbAcquiredFrames);
	return aNbAcquiredFrames;
}
//-----------------------------------------------------
