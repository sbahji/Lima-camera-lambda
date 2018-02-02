#include <cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> 
#include <netdb.h>

#include "lima/Exceptions.h"


#include "LambdaCamera.h"

#include <fsdetector/lambda/LambdaSysImpl.h>

using namespace lima;
using namespace lima::Lambda;


//---------------------------------------------------------------------------------------
//! Camera::CameraThread::CameraThread()
//---------------------------------------------------------------------------------------
Camera::CameraThread::CameraThread(Camera& cam)
  : m_cam(&cam)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::CameraThread - BEGIN";
	m_cam->m_acq_frame_nb = 0;
	m_force_stop = false;
	m_shFrameErrorCode = 0;
	DEB_TRACE() << "CameraThread::CameraThread - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::start()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::start()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::start - BEGIN";
	CmdThread::start();
	waitStatus(Ready);
	DEB_TRACE() << "CameraThread::start - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::init()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::init()
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::init - BEGIN";
	setStatus(Ready);
	DEB_TRACE() << "CameraThread::init - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execCmd()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execCmd(int cmd)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "CameraThread::execCmd - BEGIN";
	int status = getStatus();
	switch (cmd)
	{
		case StartAcq:
			if (status != Ready)
				throw LIMA_HW_EXC(InvalidValue, "Not Ready to StartAcq");
			execStartAcq();
			break;
	}
	DEB_TRACE() << "CameraThread::execCmd - END";
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execStartAcq()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execStartAcq()
{
	
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "CameraThread::execStartAcq - BEGIN";
  setStatus(Exposure);
  
  StdBufferCbMgr& buffer_mgr = m_cam->m_bufferCtrlObj.getBuffer();
  buffer_mgr.setStartTimestamp(Timestamp::now());
  
  
  
  int acq_frame_nb;
  int nb_frames = m_cam->m_nb_frames;
    
  // start acquisition
  m_cam->m_objDetSys->StartImaging();
  
  m_cam->m_acq_frame_nb = 0;
  acq_frame_nb = 0;

  bool continueAcq = true;
  while(continueAcq && (!m_cam->m_nb_frames || m_cam->m_acq_frame_nb < m_cam->m_nb_frames)){

    if(m_cam->m_objDetSys->GetQueueDepth()>0){
      short* ptrshData;
      int* ptrnData;
      char* ptrchData;
      bool bValid;
      int nDataLength;
      int m_nSizeX;
      int m_nSizeY;
      int m_nDepth;
      int m_nDataType;
      int lambda_frame_nb;
      ImageType image_type;
      
      if(m_cam->m_bBuildInCompressor){
	ptrchData =  m_cam->m_objDetSys->GetCompressedData(acq_frame_nb,
							   m_shFrameErrorCode,
							   nDataLength);
      } else {  //decoded image,without pre-compression

	m_cam->m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	
	if(m_nDepth == 12)
	  m_nDataType = 1; //short
	else if(m_nDepth == 24)
	  m_nDataType = 2; //int

	m_cam->getImageType(image_type);
	FrameDim frame_dim( m_nSizeX, m_nSizeY, image_type);
	
	if(m_nDataType == 1){ // short

	  int not_correct_frame = 1;
	  while(not_correct_frame){
	    ptrshData = m_cam->m_objDetSys->GetDecodedImageShort(lambda_frame_nb,
								 m_shFrameErrorCode);
	    
	    
	    DEB_TRACE() << "Prepare the Frame ptr - " << DEB_VAR1(acq_frame_nb);
	    setStatus(Readout);
	    
	    if(lambda_frame_nb == acq_frame_nb + 1){
	      DEB_TRACE() << "copy data into the Frame ptr - " << DEB_VAR1(m_nSizeX*m_nSizeY);
	      m_cam->m_sframe = (short*) ptrshData;
	      void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
	      memcpy((short *)ptr, (short *)m_cam->m_sframe, frame_dim.getMemSize()); //we need a nb of BYTES .
	      not_correct_frame = 0;
	    }
	      
	  }
	  
	} else if(m_nDataType == 2){ // int
	  // get the address of the image in memory
	  ptrnData = m_cam->m_objDetSys->GetDecodedImageInt(lambda_frame_nb,
							    m_shFrameErrorCode);
	  
	  DEB_TRACE() << "Prepare the Frame ptr - " << DEB_VAR1(acq_frame_nb);
	  setStatus(Readout);
	  DEB_TRACE() << "copy data into the Frame ptr - " << DEB_VAR1(m_nSizeX*m_nSizeY);
	  m_cam->m_frame = (int*) ptrnData;
	  void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
	  memcpy((int *)ptr, (int *)m_cam->m_frame, frame_dim.getMemSize()); //we need a nb of BYTES .
	}
      }
	 
	
      buffer_mgr.setStartTimestamp(Timestamp::now());
	  
      DEB_TRACE() << "Declare a new Frame Ready.";
      HwFrameInfoType frame_info;
      frame_info.acq_frame_nb = m_cam->m_acq_frame_nb;
      buffer_mgr.newFrameReady(frame_info);
	  
      acq_frame_nb++;
      m_cam->m_acq_frame_nb = acq_frame_nb;

    }

    if(m_force_stop){
      continueAcq = false;
      m_force_stop = false;
      break;
    }

    
  } /* End while */
  
  
  // stop acquisition
  m_cam->m_objDetSys->StopImaging();
  /*
  if (m_cam->m_frame){
    delete[] m_cam->m_frame;
    m_cam->m_frame = 0;
  }
  if (m_cam->m_sframe){
    delete[] m_cam->m_sframe;
    m_cam->m_sframe = 0;
  }
  */
  
  setStatus(Ready);
  
  DEB_TRACE() << "CameraThread::execStartAcq - END";
}


//---------------------------------------------------------------------------------------
//! Camera::Camera()
//---------------------------------------------------------------------------------------
Camera::Camera(std::string& config_path):
m_thread(*this),
m_roi_s1(0),
m_roi_s2(2047),
m_roi_sbin(1),
m_roi_p1(0),
m_roi_p2(2047),
m_roi_pbin(1),
m_nb_frames(1),
m_exposure(1.0),
m_int_acq_mode(0),
m_bufferCtrlObj()
{
	DEB_CONSTRUCTOR();
	DEB_TRACE() << "Camera::Camera";
	m_bBuildInCompressor = false;
	m_objDetSys = new LambdaSysImpl(config_path);
	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	
	m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	m_size = Size(m_nSizeX,m_nSizeY);
	
	m_thread.start();
}

//---------------------------------------------------------------------------------------
//! Camera::~Camera()
//---------------------------------------------------------------------------------------
Camera::~Camera()
{
	DEB_DESTRUCTOR();
	DEB_TRACE() << "Camera::~Camera";
	stopAcq();
}

//---------------------------------------------------------------------------------------
//! Camera::getStatus()
//---------------------------------------------------------------------------------------
Camera::Status Camera::getStatus()
{
	DEB_MEMBER_FUNCT();

	int thread_status = m_thread.getStatus();

	DEB_RETURN() << DEB_VAR1(thread_status);
	
	switch (thread_status)
	{
		case CameraThread::Ready:
			return Camera::Ready;
		case CameraThread::Exposure:
			return Camera::Exposure;
		case CameraThread::Readout:
			return Camera::Readout;
		case CameraThread::Latency:
			return Camera::Latency;
		default:
			throw LIMA_HW_EXC(Error, "Invalid thread status");
	}
}

//---------------------------------------------------------------------------------------
//! Camera::setNbFrames()
//---------------------------------------------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setNbFrames - " << DEB_VAR1(nb_frames);
	if (nb_frames < 0)
		throw LIMA_HW_EXC(InvalidValue, "Invalid nb of frames");
	  
	if(nb_frames == 0){
	  m_objDetSys->SetNImages(1000000);
	} else {
	  m_objDetSys->SetNImages(nb_frames);
	}
	m_nb_frames = nb_frames;
}

//---------------------------------------------------------------------------------------
//! Camera::getNbFrames()
//---------------------------------------------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_RETURN() << DEB_VAR1(m_nb_frames);

	nb_frames = m_nb_frames;
}

//---------------------------------------------------------------------------------------
//! Camera::getDetectorModel()
//---------------------------------------------------------------------------------------
void Camera::getDetectorModel(std::string& model)
{
	DEB_MEMBER_FUNCT();
	stringstream ss;
	ss <<"ModId "<< m_objDetSys->GetModuleID()
	    <<" - Firmware "<<m_objDetSys->GetFirmwareVersion()
	    <<" - Detector Core "<<m_objDetSys->GetDetCoreVersion()
	    <<" - Liblambda "<<m_objDetSys->GetLibLambdaVersion();
	model = ss.str();
}

//---------------------------------------------------------------------------------------
//! Camera::getNbAcquiredFrames()
//---------------------------------------------------------------------------------------
int Camera::getNbAcquiredFrames()
{
	return m_acq_frame_nb;
}

//---------------------------------------------------------------------------------------
//! Camera::prepareAcq()
//---------------------------------------------------------------------------------------
void Camera::prepareAcq()
{
	DEB_MEMBER_FUNCT();
	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	
	m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	m_size = Size(m_nSizeX,m_nSizeY);
}

//---------------------------------------------------------------------------------------
//! Camera::startAcq()
//---------------------------------------------------------------------------------------
void Camera::startAcq()
{
	DEB_MEMBER_FUNCT();

	m_thread.m_force_stop = false;
	m_acq_frame_nb = 0;

	m_thread.sendCmd(CameraThread::StartAcq);
	m_thread.waitNotStatus(CameraThread::Ready);
}

//---------------------------------------------------------------------------------------
//! Camera::stopAcq()
//---------------------------------------------------------------------------------------
void Camera::stopAcq()
{
	DEB_MEMBER_FUNCT();

	m_thread.m_force_stop = true;

	m_thread.sendCmd(CameraThread::StopAcq);
	m_thread.waitStatus(CameraThread::Ready);
}

//---------------------------------------------------------------------------------------
//! Camera::reset()
//---------------------------------------------------------------------------------------
void Camera::reset()
{
	DEB_MEMBER_FUNCT();
	//@todo maybe something to do!
}

//---------------------------------------------------------------------------------------
//! Camera::getExpTime()
//---------------------------------------------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
	DEB_MEMBER_FUNCT();
	//	AutoMutex aLock(m_cond.mutex());
	exp_time = m_exposure / 1E3;//the lima standrad unit is second AND default detector unit is ms
	DEB_RETURN() << DEB_VAR1(exp_time);
}

//---------------------------------------------------------------------------------------
//! Camera::setExpTime()
//---------------------------------------------------------------------------------------
void Camera::setExpTime(double  exp_time)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setExpTime - " << DEB_VAR1(exp_time);

	m_exposure = exp_time * 1E3;//default detector unit is ms
	m_objDetSys->SetShutterTime(m_exposure);
}


//---------------------------------------------------------------------------------------
//! Camera::setTrigMode()
//---------------------------------------------------------------------------------------
void Camera::setTrigMode(TrigMode  mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTrigMode - " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);

	switch (mode) {
	case IntTrig:
	  m_objDetSys->SetTriggerMode(0); // Internal trigger
	  break;
	case IntTrigMult:	  
	case ExtTrigMult:
	  m_objDetSys->SetTriggerMode(1); // External trigger. Once dectector receives trigger, it takes predefined image numbers.
	  break;
	case ExtTrigSingle:
	  m_objDetSys->SetTriggerMode(2); // External trigger. Each trigger pulse takes one image.
	  break;
	case ExtGate:
	case ExtStartStop:
	case ExtTrigReadout:
	default:
		THROW_HW_ERROR(Error) << "Cannot change the Trigger Mode of the camera, this mode is not managed !";
		break;
	}
	
	
}

//---------------------------------------------------------------------------------------
//! Camera::getTrigMode()
//---------------------------------------------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_trigger_mode;	
	DEB_RETURN() << DEB_VAR1(mode);
}


//---------------------------------------------------------------------------------------
//! Camera::setShutterMode()
//---------------------------------------------------------------------------------------
void Camera::setShutterMode(int mode)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setShutterMode - " << DEB_VAR1(mode);
	DEB_PARAM() << DEB_VAR1(mode);
}

//---------------------------------------------------------------------------------------
//! Camera::getShutterMode()
//---------------------------------------------------------------------------------------
void Camera::getShutterMode(int& mode)
{
	DEB_MEMBER_FUNCT();
	mode = m_shutter_mode;
	DEB_RETURN() << DEB_VAR1(mode);
}

//---------------------------------------------------------------------------------------
//! Camera::getTemperature()
//---------------------------------------------------------------------------------------
double Camera::getTemperature()
{
	DEB_MEMBER_FUNCT();
}

//---------------------------------------------------------------------------------------
//! Camera::getTemperatureSetPoint()
//---------------------------------------------------------------------------------------
double Camera::getTemperatureSetPoint()
{
	DEB_MEMBER_FUNCT();
}

//---------------------------------------------------------------------------------------
//! Camera::setTemperatureSetPoint()
//---------------------------------------------------------------------------------------
void Camera::setTemperatureSetPoint(double temperature)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setTemperatureSetPoint - " << DEB_VAR1(temperature);

}

//---------------------------------------------------------------------------------------
//! Camera::getInternalAcqMode()
//---------------------------------------------------------------------------------------
std::string Camera::getInternalAcqMode()
{
	DEB_MEMBER_FUNCT();
	std::string mode = "UNKNOWN";
	if (m_int_acq_mode == 0)
	{
		DEB_RETURN() << DEB_VAR1("STANDARD");
		mode = "STANDARD";
	}
	else if (m_int_acq_mode == 1)
	{
		DEB_RETURN() << DEB_VAR1("CONTINUOUS");
		mode = "CONTINUOUS";
	}
	else if (m_int_acq_mode == 2)
	{
		DEB_RETURN() << DEB_VAR1("FOCUS");
		mode = "FOCUS";
	}
	return mode;
}

//---------------------------------------------------------------------------------------
//! Camera::setInternalAcqMode()
//---------------------------------------------------------------------------------------
void Camera::setInternalAcqMode(std::string mode)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(mode);

	if (mode == "STANDARD")
		m_int_acq_mode = 0;
	else if (mode == "CONTINUOUS")
		m_int_acq_mode = 1;
	else if (mode == "FOCUS")
		m_int_acq_mode = 2;
	else
		THROW_HW_ERROR(Error) << "Incorrect Internal Acquisition mode !";
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(type);
}

//---------------------------------------------------------------------------------------
//! Camera::getImageType()
//---------------------------------------------------------------------------------------
void Camera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	
	m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);
	
	if(m_nDepth == 12)
	  type = Bpp12; //short
	else if(m_nDepth == 24)
	  type = Bpp24; //int
	return;
}


//---------------------------------------------------------------------------------------
//! Camera::setBin()
//---------------------------------------------------------------------------------------
void Camera::setBin(const Bin& bin)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setBin";
	DEB_PARAM() << DEB_VAR1(bin);

	m_roi_sbin = bin.getX();
	m_roi_pbin = bin.getY();
}

//---------------------------------------------------------------------------------------
//! Camera::getBin()
//---------------------------------------------------------------------------------------
void Camera::getBin(Bin& bin)
{
	DEB_MEMBER_FUNCT();
	Bin tmp_bin(m_roi_sbin, m_roi_pbin);
	bin = tmp_bin;

	DEB_RETURN() << DEB_VAR1(bin);
}

//---------------------------------------------------------------------------------------
//! Camera::checkBin()
//---------------------------------------------------------------------------------------
void Camera::checkBin(Bin& bin)
{

	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::checkBin";
	DEB_PARAM() << DEB_VAR1(bin);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::checkRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	hw_roi = set_roi;

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::getRoi()
//---------------------------------------------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
	DEB_MEMBER_FUNCT();
	Point point1(m_roi_s1, m_roi_p1);
	Point point2(m_roi_s2, m_roi_p2);
	Roi tmp_roi(point1, point2);

	hw_roi = tmp_roi;

	DEB_RETURN() << DEB_VAR1(hw_roi);
}

//---------------------------------------------------------------------------------------
//! Camera::setRoi()
//---------------------------------------------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setRoi";
	DEB_PARAM() << DEB_VAR1(set_roi);
	if (!set_roi.isActive())
	{
		DEB_TRACE() << "Roi is not Enabled";
	}
	else
	{
		//To avoid a double binning, API pvcam apply Roi & Binning together AND Lima Too !		
		Bin aBin;
		getBin(aBin);
		Roi UnbinnedRoi = set_roi.getUnbinned(aBin);
		Point tmp_top = UnbinnedRoi.getTopLeft();
		////

		m_roi_s1 = tmp_top.x;
		m_roi_p1 = tmp_top.y;

		Point tmp_bottom = UnbinnedRoi.getBottomRight();

		m_roi_s2 = tmp_bottom.x;
		m_roi_p2 = tmp_bottom.y;
	}
}

void Camera::getImageSize(Size& size) {
	DEB_MEMBER_FUNCT();

	size = m_size;
}

HwBufferCtrlObj* Camera::getBufferCtrlObj() {
    return &m_bufferCtrlObj;
}

unsigned short Camera::getDistortionCorrection(){
  return m_objDetSys->GetDistortionCorrecttionMethod();
}
