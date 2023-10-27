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
    DEB_TRACE() << "CameraThread::CameraThread - END";
}

/************************************************************************
//! Camera::CameraThread::~CameraThread()
 ************************************************************************/
Camera::CameraThread::~CameraThread()
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "The CameraThread thread was terminated.";
    if (getStatus() != CmdThread::InInit)
    {
        CmdThread::abort();  
    }
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
        m_cam->detector->stopAcquisition();
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
    DEB_TRACE() << "Starting Acquisition...";
    m_cam->detector->startAcquisition();
    DEB_TRACE() << "Acquisition Started";
    
    m_cam->m_acq_frame_nb = 0;
    acq_frame_nb = 0;

    bool continueAcq = true;
    while(continueAcq && (!m_cam->m_nb_frames || m_cam->m_acq_frame_nb < m_cam->m_nb_frames))
    {
          if(m_cam->receiver->framesQueued() > 0)
        {
            const xsp::Frame* frame;

            int nDepth;
            int m_nDataType;

            nDepth = m_cam->receiver->frameDepth();
        
            if(nDepth == 12)
                m_nDataType = 1; //short
            else if(nDepth == 24)
                m_nDataType = 2; //int

            //- get the Frame
            frame =  m_cam->receiver->frame(1500);
            if (frame != nullptr)
            {    
                DEB_TRACE() << "Prepare the Frame ptr - " << DEB_VAR1(acq_frame_nb);
                setStatus(Readout);
        
                void *ptr = buffer_mgr.getFrameBufferPtr(acq_frame_nb);

                if(m_nDataType == 1) // short (12 bits)
                    memcpy((short*)ptr, frame->data(), frame->size()); //we need a nb of BYTES .
                else if(m_nDataType == 2) // int (24 bits)
                    memcpy((int*)ptr, frame->data(), frame->size()); //we need a nb of BYTES .
                
                m_cam->receiver->release(frame);
            }

            buffer_mgr.setStartTimestamp(Timestamp::now());
            
            DEB_TRACE() << "Declare a new Frame Ready.";
            HwFrameInfoType frame_info;
            frame_info.acq_frame_nb = m_cam->m_acq_frame_nb;
            buffer_mgr.newFrameReady(frame_info);
            
            acq_frame_nb++;
            m_cam->m_acq_frame_nb = acq_frame_nb;
        }

        if(m_force_stop)
        {
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
Camera::Camera(std::string& config_file):
                                        m_thread(*this),
                                        m_config_file(config_file),
                                        m_nb_frames(1),
                                        m_exposure(1.0),
                                        m_bufferCtrlObj()
{
    DEB_CONSTRUCTOR();
    DEB_TRACE() << "Camera::Camera";
    
    try
    {
        libxsp_system = xsp::createSystem(config_file);
    }
    catch(const xsp::Error& e)
    {
        THROW_HW_ERROR(Error) << "Error in creating the xsp system, reason: " << e.what() << " (with config file: " << config_file << ")";
    }
    if (libxsp_system == nullptr)
    {
        THROW_HW_ERROR(Error) << "No system created. Aborting ...";
    }
    
    try
    {
        libxsp_system->connect();
        libxsp_system->initialize();
    }
    catch (const xsp::RuntimeError& e)
    {
        THROW_HW_ERROR(Error) << "Cannot initialize detector connection, reason: \n" << e.what();
    }

    detector = std::dynamic_pointer_cast<xsp::lambda::Detector>(
                                   libxsp_system->detector("lambda")
                                   );

    //- get detector model
    m_detector_model = "";
    std::stringstream ss;
    auto chip_ids = detector->chipIds(1);
    ss << "Nb. modules " << detector->numberOfModules()
       <<" - Mod #1 Id "<< chip_ids[0];
    m_detector_model = ss.str();

    try
    {
        //receiver = libxsp_system->receiver("lambda/1");
        receiver = libxsp_system->postDecoder("lambda"); //- work with several receivers
    }
    catch(const xsp::Error& e)
    {
        THROW_HW_ERROR(Error) << "xsp error: " << e.what() << " (with config file: " << config_file << ")";
    }
    
    m_size = Size(receiver->frameWidth(),receiver->frameHeight());
    
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
      
    if(nb_frames == 0)
    {
      detector->setFrameCount(16777215); //- Max possible value (3 bytes integer : 24 bits)
    } 
    else 
    {
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
    model = m_detector_model;
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
    //    AutoMutex aLock(m_cond.mutex());
    exp_time = m_exposure / 1E3;//the lima standard unit is second AND default detector unit is ms
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

    switch (mode) 
    {
    case IntTrig:
      detector->setTriggerMode(xsp::lambda::TrigMode::SOFTWARE); // Internal trigger
      break;
    case ExtTrigSingle:
      detector->setTriggerMode(xsp::lambda::TrigMode::EXT_SEQUENCE); // External trigger. Once dectector receives trigger, it takes predefined image numbers.
      break;
    case ExtTrigMult:
      detector->setTriggerMode(xsp::lambda::TrigMode::EXT_FRAMES); // External trigger. Each trigger pulse takes one image.
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
//! Camera::getLowerEnergyThreshold()
//! lower energy threshold in KeV
//---------------------------------------------------------------------------------------
void Camera::getLowerEnergyThreshold(double& energy_threshold)
{
    DEB_MEMBER_FUNCT();
    // up to 8 thresholds can be set, we only use the first one
    auto thresholds = detector->thresholds();
    // at cold start we get an empty double vector
    if (thresholds.size() != 0)
        energy_threshold = thresholds[0];    
    else
        energy_threshold = -1; 
    DEB_RETURN() << DEB_VAR1(energy_threshold);
}

//---------------------------------------------------------------------------------------
//! Camera::getUpperEnergyThreshold()
//! upper energy threshold in KeV
//---------------------------------------------------------------------------------------
void Camera::getUpperEnergyThreshold(double& energy_threshold)
{
    DEB_MEMBER_FUNCT();
    // we use the second threshold
    auto thresholds = detector->thresholds();
    // at cold start we get an empty double vector
    if (thresholds.size() >= 2)
        energy_threshold = thresholds[1];    
    else
        energy_threshold = -1; 

    DEB_RETURN() << DEB_VAR1(energy_threshold);
}

//---------------------------------------------------------------------------------------
//! Camera::setEnergyThresholds()
//! lower and upper energy threshold in KeV
//---------------------------------------------------------------------------------------
void Camera::setEnergyThresholds(double lower_energy_threshold, double upper_energy_threshold)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setEnergyThresholds - set lowerThreshold " << DEB_VAR1(lower_energy_threshold);
    DEB_PARAM() << DEB_VAR1(lower_energy_threshold);

    DEB_TRACE() << "Camera::setEnergyThresholds - set upperThreshold " << DEB_VAR1(upper_energy_threshold);
    DEB_PARAM() << DEB_VAR1(upper_energy_threshold);

    detector->setThresholds(std::vector<double>{lower_energy_threshold, upper_energy_threshold});
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
void Camera::getHumidity(double &humidity_percent)
{
    DEB_MEMBER_FUNCT();
    auto module_nr = 1;
    humidity_percent = detector->humidity(module_nr);
    DEB_RETURN() << DEB_VAR1(humidity_percent);
}

//---------------------------------------------------------------------------------------
//! Camera::getHighVoltage()
//! high voltage in Volt
//---------------------------------------------------------------------------------------
void Camera::getHighVoltage(double& high_voltage)
{
    DEB_MEMBER_FUNCT();
    auto module_nr = 1;
    high_voltage = detector->voltage(module_nr);
    DEB_RETURN() << DEB_VAR1(high_voltage);
}

//---------------------------------------------------------------------------------------
//! Camera::setHighVoltage()
//! high voltage in Volt
//---------------------------------------------------------------------------------------
void Camera::setHighVoltage(double high_voltage)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(high_voltage);

    auto module_nr = 1;
    detector->setVoltage(module_nr, high_voltage);
}
//----------------------------------------------------
//! Camera::setImageType()
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
    DEB_MEMBER_FUNCT();
    DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(type);

    xsp::lambda::BitDepth depth;
    if(type == Bpp12)
        depth = xsp::lambda::BitDepth::DEPTH_12;
    else if(type == Bpp24)
        depth = xsp::lambda::BitDepth::DEPTH_24;

    detector->setBitDepth(depth);
}

//---------------------------------------------------------------------------------------
//! Camera::getImageType()
//---------------------------------------------------------------------------------------
void Camera::getImageType(ImageType& type)
{
    DEB_MEMBER_FUNCT();

    xsp::lambda::BitDepth depth;
    depth = detector->bitDepth();

    if(depth == xsp::lambda::BitDepth::DEPTH_12)
        type = Bpp12;
    else if(depth == xsp::lambda::BitDepth::DEPTH_24)
        type = Bpp24;
    return;
}

//---------------------------------------------------------------------------------------
//! Camera::getImageSize()
//! get the image size
//---------------------------------------------------------------------------------------
void Camera::getImageSize(Size& size) 
{
    DEB_MEMBER_FUNCT();

    size = m_size;
}

//---------------------------------------------------------------------------------------
//! Camera::getBufferCtrlObj()
//! get the BufferCtrlObj
//---------------------------------------------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj() 
{
    return &m_bufferCtrlObj;
}

//---------------------------------------------------------------------------------------
//! Camera::getDistortionCorrection()
//! get the value of the Distortion Correction
//---------------------------------------------------------------------------------------
void Camera::getDistortionCorrection(bool &is_on)
{
  is_on = detector->interpolationEnabled();
}

//---------------------------------------------------------------------------------------
//! Camera::setDistortionCorrection()
//! Enable/Disable the Distortion Correction
//---------------------------------------------------------------------------------------
void Camera::setDistortionCorrection(bool flag)
{
    if (flag)
        detector->enableInterpolation();
    else
        detector->disableInterpolation();
}

//---------------------------------------------------------------------------------------
//! Camera::getLibVersion()
//! Library version
//---------------------------------------------------------------------------------------
std::string Camera::getLibVersion()
{
    DEB_MEMBER_FUNCT();
    return xsp::libraryVersion();
}

//---------------------------------------------------------------------------------------
//! Camera::getConfigFile()
//! configuration file
//---------------------------------------------------------------------------------------
std::string Camera::getConfigFile()
{
    DEB_MEMBER_FUNCT();
    return m_config_file;
}

//---------------------------------------------------------------------------------------
//! Camera::hasFeature()
//! returns true or false if the feature is present or not in the camera model
//---------------------------------------------------------------------------------------
bool Camera::hasFeature(xsp::lambda::Feature feature)
{
    DEB_MEMBER_FUNCT();
    auto module_nr = 1;
    auto w_has_feature = detector->hasFeature(module_nr, feature);
    DEB_RETURN() << DEB_VAR1(w_has_feature);
    return w_has_feature;
}

//---------------------------------------------------------------------------------------
//! Camera::getLinearityCorrection()
//! get the value of if countrate correction is enabled
//---------------------------------------------------------------------------------------
void Camera::getLinearityCorrection(bool &is_on)
{
    is_on = detector->countrateCorrectionEnabled();
}

//---------------------------------------------------------------------------------------
//! Camera::setLinearityCorrection()
//! Enable/Disable the Countrate Correction
//---------------------------------------------------------------------------------------
void Camera::setLinearityCorrection(bool flag)
{
    if (flag)
    {
        detector->enableCountrateCorrection();
    }
    else
    {
        detector->disableCountrateCorrection();
    }
}

//---------------------------------------------------------------------------------------
//! Camera::getSaturationFlag()
//! returns whether flagging of saturated pixels is enabled
//---------------------------------------------------------------------------------------
void Camera::getSaturationFlag(bool &is_on)
{
    is_on = detector->saturationFlagEnabled();
}

//---------------------------------------------------------------------------------------
//! Camera::setSaturationFlag()
//! Enable/Disable the Saturation Flag
//---------------------------------------------------------------------------------------
void Camera::setSaturationFlag(bool flag)
{
    if (flag)
    {
        //If the count is above a saturation threshold 
        //then the MSB of the unused bits within the frame is set
        detector->enableSaturationFlag();
    }
    else
    {
        detector->disableSaturationFlag();
    }
}

//----------------------------------------------------------------------------------------------------
//! Camera::getSaturationThreshold()
//! get the value of actual saturation threshold in counts/s/pixel of the specified module
//----------------------------------------------------------------------------------------------------
void Camera::getSaturationThreshold(int &saturation_threshold)
{
    //saturationThreshold(int module_nr)
    //module_nr: module number
    saturation_threshold = detector->saturationThreshold(1);
}

//---------------------------------------------------------------------------------------
//! Camera::setSaturationThreshold()
//! sets global saturation threshold in counts/s/pixel
//---------------------------------------------------------------------------------------
void Camera::setSaturationThreshold(int saturation_threshold)
{
    for(int module_nr = 1 ; module_nr <= detector->numberOfModules() ; ++module_nr)
    {    
        detector->setSaturationThreshold(module_nr, saturation_threshold);
    }
}

//---------------------------------------------------------------------------------------
//! Camera::getChargeSumming()
//! sets charge summing mode
//---------------------------------------------------------------------------------------
void Camera::getChargeSumming(bool &is_charge_summing)
{
    DEB_MEMBER_FUNCT();

    if(xsp::lambda::ChargeSumming::ON == detector->chargeSumming())
    {
        is_charge_summing = true;
    }
    else if(xsp::lambda::ChargeSumming::OFF == detector->chargeSumming())
    {
        is_charge_summing = false;
    }

    DEB_RETURN() << DEB_VAR1(is_charge_summing);
}

//---------------------------------------------------------------------------------------
//! Camera::setChargeSumming()
//! sets charge summing mode
//---------------------------------------------------------------------------------------
void Camera::setChargeSumming(int is_charge_summing)
{
    if(is_charge_summing)
    {
        detector->setChargeSumming(xsp::lambda::ChargeSumming::ON);
    }
    else
    {
        detector->setChargeSumming(xsp::lambda::ChargeSumming::OFF);
    }
}