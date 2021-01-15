#include <cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> 
#include <netdb.h>

#include "lima/Exceptions.h"


#include "LambdaCamera.h"

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
	m_shFrameErrorCode = FrameStatusCode::FRAME_OK;
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
	case StopAcq:
	  setStatus(Ready);
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
  m_cam->detector->startAcquisition();
  
  m_cam->m_acq_frame_nb = 0;
  acq_frame_nb = 0;

  bool continueAcq = true;
  while(continueAcq && (!m_cam->m_nb_frames || m_cam->m_acq_frame_nb < m_cam->m_nb_frames)){
		if(m_cam->receiver->framesQueued()>0){
			const Frame* frame;
			
			bool bValid;
			int nDataLength;
			int m_nSizeX;
			int m_nSizeY;
			int m_nDepth;
			int m_nDataType;
			int lambda_frame_nb;
			ImageType image_type;
			
			if(m_cam->m_bBuildInCompressor){
				frame =  m_cam->receiver->frame(1500);
				if (frame != nullptr) {
					acq_frame_nb = frame->nr();
					m_shFrameErrorCode = frame->status();
					auto ptrch_data = reinterpret_cast<const uint8_t*>(frame->data());
					nDataLength = frame->size();
					// process frame data, i.e. copy using memcpy(<dest>, frame_data, frame_size);
					m_cam->receiver->release(acq_frame_nb);
				}
			} else {  //decoded image,without pre-compression
				m_nSizeX = m_cam->receiver->frameWidth();
				m_nSizeY = m_cam->receiver->frameHeight();
				m_nDepth = m_cam->receiver->frameDepth();
			
				if(m_nDepth == 12)
				m_nDataType = 1; //short
				else if(m_nDepth == 24)
				m_nDataType = 2; //int

				m_cam->getImageType(image_type);
				FrameDim frame_dim( m_nSizeX, m_nSizeY, image_type);
				
				if(m_nDataType == 1){ // short
					int not_correct_frame = 1;
					while(not_correct_frame){
						frame =  m_cam->receiver->frame(1500);
						if (frame != nullptr) {
							lambda_frame_nb = frame->nr();
							m_shFrameErrorCode= frame->status();
							auto ptrsh_data = reinterpret_cast<const uint16_t*>(frame->data());
							// process frame data
							
							DEB_TRACE() << "Prepare the Frame ptr - " << DEB_VAR1(acq_frame_nb);
							setStatus(Readout);
							
							if(lambda_frame_nb == acq_frame_nb + 1){
								DEB_TRACE() << "copy data into the Frame ptr - " << DEB_VAR1(m_nSizeX*m_nSizeY);
								m_cam->m_sframe = (short*) ptrsh_data;
								void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
								memcpy((short *)ptr, (short *)m_cam->m_sframe, frame_dim.getMemSize()); //we need a nb of BYTES .
								not_correct_frame = 0;
							}
							m_cam->receiver->release(lambda_frame_nb);
						}		
					}
				} else if(m_nDataType == 2){ // int
					frame =  m_cam->receiver->frame(1500);
					if (frame != nullptr) {
						lambda_frame_nb = frame->nr();
						m_shFrameErrorCode= frame->status();
						auto ptrn_data = reinterpret_cast<const uint16_t*>(frame->data());
						// process frame data
						
						DEB_TRACE() << "Prepare the Frame ptr - " << DEB_VAR1(acq_frame_nb);
						setStatus(Readout);
						DEB_TRACE() << "copy data into the Frame ptr - " << DEB_VAR1(m_nSizeX*m_nSizeY);
						m_cam->m_frame = (int*) ptrn_data;
						void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);
						memcpy((int *)ptr, (int *)m_cam->m_frame, frame_dim.getMemSize()); //we need a nb of BYTES .
						m_cam->receiver->release(lambda_frame_nb);
					}
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
  m_cam->detector->stopAcquisition();
  
  setStatus(Ready);
  
  DEB_TRACE() << "CameraThread::execStartAcq - END";
}


//---------------------------------------------------------------------------------------
//! Camera::Camera()
//---------------------------------------------------------------------------------------
Camera::Camera(std::string& config_path):
m_thread(*this),
m_nb_frames(1),
m_exposure(1.0),
m_bufferCtrlObj()
{
	DEB_CONSTRUCTOR();
	DEB_TRACE() << "Camera::Camera";
	m_bBuildInCompressor = false;

	libxsp_system = createSystem(config_path);
	detector = std::dynamic_pointer_cast<lambda::Detector>(
							       libxsp_system->detector("lambda")
							       );
	receiver = libxsp_system->receiver("lambda/1");
	libxsp_system->connect();
	libxsp_system->initialize();
	
	int m_nSizeX;
	int m_nSizeY;
	int m_nDepth;
	m_nSizeX = receiver->frameWidth();
	m_nSizeY = receiver->frameHeight();
	m_nDepth = receiver->frameDepth();
	

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
	
	Camera::Status status;
	switch (thread_status)
	{
		case CameraThread::Ready:
		  status = Camera::Ready; break;
		case CameraThread::Exposure:
		  status = Camera::Exposure; break;
		case CameraThread::Readout:
		  status = Camera::Readout; break;
		case CameraThread::Latency:
		  status = Camera::Latency; break;
		default:
		  throw LIMA_HW_EXC(Error, "Invalid thread status");
	}
	return status;
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
	  detector->setFrameCount(1000000);
	} else {
	  detector->setFrameCount(nb_frames);
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
	auto chip_ids = detector->chipIds(1);
	ss << "Nb. modules " << detector->numberOfModules()
	   <<" - Mod #1 Id "<< chip_ids[0];
	//<<" - Firmware "<< detector->firmwareVersion(1)
	//<<" - Liblambda "<< libraryVersion();
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
	
	//m_objDetSys->GetImageFormat(m_nSizeX,m_nSizeY,m_nDepth);

	m_nSizeX = receiver->frameWidth();
	m_nSizeY = receiver->frameHeight();
	m_nDepth = receiver->frameDepth();
	
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
	//m_objDetSys->SetShutterTime(m_exposure);
	detector->setShutterTime(m_exposure);
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
	  detector->setTriggerMode(lambda::TrigMode::SOFTWARE); // Internal trigger
	  break;
	case ExtTrigSingle:
	  detector->setTriggerMode(lambda::TrigMode::EXT_SEQUENCE); // External trigger. Once dectector receives trigger, it takes predefined image numbers.
	  break;
	case ExtTrigMult:
	  detector->setTriggerMode(lambda::TrigMode::EXT_FRAMES); // External trigger. Each trigger pulse takes one image.
	  break;
	case ExtGate:
	  //detector->setTriggerMode(lambda::TrigMode::EXIT_FRAME_GATED);
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
//! Camera::getEnergyThreshold()
//! energy threshold in KeV
//---------------------------------------------------------------------------------------
void Camera::getEnergyThreshold(double& energy)
{
	DEB_MEMBER_FUNCT();
	// up to 8 thresholds can be set, we only use the first one
	auto thresholds = detector->thresholds();
	energy = thresholds[0];
	DEB_RETURN() << DEB_VAR1(energy);
}

//---------------------------------------------------------------------------------------
//! Camera::setEnergyThreshold()
//! energy threshold in KeV
//---------------------------------------------------------------------------------------
void Camera::setEnergyThreshold(double energy)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setEnergyThreshold - " << DEB_VAR1(energy);
	DEB_PARAM() << DEB_VAR1(energy);
	detector->setThresholds(std::vector<double>{energy});
}

//---------------------------------------------------------------------------------------
//! Camera::getTemperature()
//---------------------------------------------------------------------------------------
void Camera::getTemperature(double &temperature)
{
	DEB_MEMBER_FUNCT();
	auto module_nr = 1;
	auto temps = detector->temperature(module_nr);
	temperature = temps[0];
}

//---------------------------------------------------------------------------------------
//! Camera::getHumidity()
//---------------------------------------------------------------------------------------
void Camera::getHumidity(double &percent)
{
	DEB_MEMBER_FUNCT();
	auto module_nr = 1;
	auto hum = detector->humidity(module_nr);
	percent = hum;
}


//---------------------------------------------------------------------------------------
//! Camera::getHighVoltage()
//! high voltage in Volt
//---------------------------------------------------------------------------------------
void Camera::getHighVoltage(double& voltage)
{
	DEB_MEMBER_FUNCT();
	auto module_nr = 1;
	auto hv = detector->voltage(module_nr);
	voltage = hv;
	DEB_RETURN() << DEB_VAR1(voltage);
}

//---------------------------------------------------------------------------------------
//! Camera::setHighVoltage()
//! high voltage in Volt
//---------------------------------------------------------------------------------------
void Camera::setHighVoltage(double voltage)
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setHighVoltage - " << DEB_VAR1(voltage);
	DEB_PARAM() << DEB_VAR1(voltage);
	auto module_nr = 1;
	detector->setVoltage(module_nr, voltage);
}
//---------------------------------------------------------------------------------------
//! Camera::getInternalAcqMode()
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
	
	m_nSizeX = receiver->frameWidth();
	m_nSizeY = receiver->frameHeight();
	m_nDepth = receiver->frameDepth();
	if(m_nDepth == 12)
	  type = Bpp12; //short
	else if(m_nDepth == 24)
	  type = Bpp24; //int
	return;
}

void Camera::getImageSize(Size& size) {
	DEB_MEMBER_FUNCT();

	size = m_size;
}

HwBufferCtrlObj* Camera::getBufferCtrlObj() {
    return &m_bufferCtrlObj;
}

void Camera::getDistortionCorrection(bool &is_on){
  is_on = (receiver->interpolation() == Interpolation::ON);
}
