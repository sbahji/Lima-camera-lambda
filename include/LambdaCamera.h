#ifndef LAMBDACAMERA_H
#define LAMBDACAMERA_H

#include "lima/Debug.h"
#include "Lambda.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"

#include <libxsp.h>

class BufferCtrlObj;

namespace lima
{
namespace Lambda
{
  using namespace xsp;
  using namespace xsp::lambda;

  typedef unique_ptr<xsp::System> uptr_sys;
  typedef shared_ptr<xsp::lambda::Detector> sptr_det;
  typedef shared_ptr<xsp::Receiver> sptr_recv;
  
//    class Camera
class LIBLAMBDA_API Camera
{
	DEB_CLASS_NAMESPC(DebModCamera,"Camera","Lambda");
	friend class Interface;
public:
	
	enum Status
	{
		Ready, Exposure, Readout, Latency, Fault
	};

	Camera(std::string& config_path);
	~Camera();
	
	//LambdaInterface *m_objDetSys;
	uptr_sys libxsp_system;
	sptr_det detector;
	sptr_recv receiver;
	
	void getExpTime(double& exp_time);
	void setExpTime(double  exp_time);

	Status getStatus();
	int  getNbAcquiredFrames();
	
	void getDetectorModel(std::string& model);
	void startAcq();
	void stopAcq();
	void prepareAcq();
	void reset();

	void setNbFrames(int nb_frames);
	void getNbFrames(int& nb_frames);
	

	HwBufferCtrlObj* getBufferCtrlObj();

	//-- Camera specific configuration parameters
	void getEnergyThreshold(double &energy);
	void setEnergyThreshold(double energy);	
	void getTemperature(double &temperature);
	void getDistortionCorrection(bool &is_on);
	void getHumidity(double &percent);
	void getHighVoltage(double &voltage);
	void setHighVoltage(double voltage);

	//-- Synch control object

	TrigMode m_trigger_mode;
	void setTrigMode(TrigMode  mode);
	void getTrigMode(TrigMode& mode);

	//-- Shutter managment : TODO Shutter control object
	//---------------------------------------------------------------------------------------
	void setShutterMode(int mode);
	void getShutterMode(int& mode);
	
	void getImageSize(Size& size);	
	void setImageType(ImageType type);
	void getImageType(ImageType& type);
	
private:
	class CameraThread: public CmdThread
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "Lambda");
	public:
		enum
		{ // Status
			Ready = MaxThreadStatus, Exposure, Readout, Latency,
		};

		enum
		{ // Cmd
			StartAcq = MaxThreadCmd, StopAcq,
		};

		CameraThread(Camera& cam);

		virtual void start();
		bool m_force_stop;
		FrameStatusCode m_shFrameErrorCode;

	protected:
		virtual void init();
		virtual void execCmd(int cmd);
	private:
		void execStartAcq();
		void execStopAcq();
		Camera* m_cam;

	};
	friend class CameraThread;
	
	double m_exposure;
	int m_nb_frames;

        Size m_size;
	
	int *m_frame;
	short *m_sframe;

	// Buffer control object
	SoftBufferCtrlObj m_bufferCtrlObj;

	bool m_bBuildInCompressor;
	
	/* main acquisition thread*/
	CameraThread 	m_thread;
	int 		m_acq_frame_nb;
	
};
}
}
#endif
