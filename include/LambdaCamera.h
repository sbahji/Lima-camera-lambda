#ifndef LAMBDACAMERA_H
#define LAMBDACAMERA_H

#include "lima/Debug.h"
#include "LambdaInclude.h"
#include "lima/Constants.h"
#include "lima/HwBufferMgr.h"
#include "lima/ThreadUtils.h"

#include <libxsp.h>

class BufferCtrlObj;

namespace lima
{
namespace Lambda
{
  typedef unique_ptr<xsp::System> uptr_sys;
  typedef shared_ptr<xsp::lambda::Detector> sptr_det;
  //typedef shared_ptr<xsp::Receiver> sptr_recv;
  typedef shared_ptr<xsp::PostDecoder> sptr_recv;
  
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

	Camera(std::string& config_file);
	~Camera();
	
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
	void getLowerEnergyThreshold(double &energy);
	void getUpperEnergyThreshold(double &energy);
	void setEnergyThresholds(double lower_energy_threshold, double upper_energy_threshold);

	void getTemperature(double &temperature);
	void getDistortionCorrection(bool &is_on);
	void setDistortionCorrection(bool flag);
	void getHumidity(double &percent);
	bool hasFeature(xsp::lambda::Feature feature);
	void getHighVoltage(double &voltage);
	void setHighVoltage(double voltage);
	std::string getLibVersion();
	std::string getConfigFile();

	//- Sync control object
	TrigMode m_trigger_mode;
	void setTrigMode(TrigMode  mode);
	void getTrigMode(TrigMode& mode);

	void getImageSize(Size& size);	
	void setImageType(ImageType type);
	void getImageType(ImageType& type);

	//- decoding settings
	void getLinearityCorrection(bool &is_on);
	void setLinearityCorrection(bool flag);
	void getSaturationFlag(bool &is_on);
	void setSaturationFlag(bool flag);
	void getSaturationThreshold(int &saturation_threshold);
	void setSaturationThreshold(int saturation_threshold);
	void getChargeSumming(bool &is_charge_summing);
	void setChargeSumming(int is_charge_summing);

private:
	class CameraThread: public CmdThread
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "Lambda");
	public:
		enum
		{ // Status
			Ready = MaxThreadStatus ,
            Exposure                ,
            Readout                 ,
            Latency                 ,
		};

		enum
		{ // Cmd
			StartAcq = MaxThreadCmd ,
            StopAcq                 ,
		};

        // constructor
		CameraThread(Camera& cam);

        // destructor
        virtual ~CameraThread();

		virtual void start();
		bool m_force_stop;

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
	std::string m_config_file;

	std::string m_detector_model;

    Size m_size;

	// Buffer control object
	SoftBufferCtrlObj m_bufferCtrlObj;

	/* main acquisition thread*/
	CameraThread 	m_thread;
	int 		m_acq_frame_nb;
	
};
}
}
#endif
