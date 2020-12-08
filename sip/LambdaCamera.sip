namespace Lambda
{
  class Camera
  {
%TypeHeaderCode
#include <LambdaCamera.h>
%End
  public:
    Camera(std::string& config_path);
    ~Camera();
      
    int getNbAcquiredFrames();

    void getImageSize(Size& size /Out/);
    void getImageType(ImageType& type /Out/);
	
   // -- Buffer control object
   HwBufferCtrlObj* getBufferCtrlObj();
	
    void setNbFrames(int );
    void getNbFrames(int&);
	      
    void 	startAcq();	
    void 	stopAcq();
    void        prepareAcq();
    void	reset();

    void getTrigMode(TrigMode &mode /Out/);
    void setTrigMode(TrigMode mode);

    void getEnergyThreshold(double &energy /Out/);
    void setEnergyThreshold(double energy);
    void getTemperature(double &temperature /Out/); 
    void getDistortionCorrection(bool &is_on /Out/);
    void getHumidity(double &percent /Out/);
    void getHighVoltage(double &voltage /Out/);
    void setHighVoltage(double voltage);

    private:
      Camera(const Lambda::Camera&);
	
  };
};
