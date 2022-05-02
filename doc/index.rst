.. _camera-lambda:

Lambda / Xspectrum
---------------------------

.. image:: lambda.png

Intoduction
```````````

LAMBDA is a next-generation pixel detector for X-rays, based on Medipix3 technology. It is a photon-counting detector, making it effectively noise free, and it offers a high frame rate of up to 23,000 frames per second (with no readout deadtime) and a small pixel size of 55 µm. It is available in a wide variety of sizes and configurations for different applications, and can be equipped with different sensor materials to allow high detection efficiency even at high X-ray energies. The system also has “colour imaging” capabilities, where X-rays hitting the detector can be divided into two energy ranges (**\***). Developed by DESY for use at the PETRA-III synchrotron, the system is designed for high reliability, and has external triggering and gating capability for synchronisation with the rest of the experiment. It can be easily integrated into common beamline control systems.


Installation & Module configuration
````````````````````

Follow the generic instructions in :ref:`build_installation`. If using CMake directly, add the following flag:

.. code-block:: sh

 -DLIMACAMERA_LAMBDA=true

For the Tango server installation, refers to :ref:`tango_installation`.


Initialisation and Capabilities
````````````````````````````````


Camera initialisation
......................

The camera will be initialized  by created the :cpp:class:`Lambda::Camera` object. The contructor
will take care of your detector configuration according to the SDK installation setup done before.
The Camera::Camera() constructor required to pass the full path to the configuration directory installed
on the control computer. The standard path should be /opt/xsp/config .

Std capabilites
................

This plugin has been implement in respect of the mandatory capabilites but with some limitations which
are due to the camera and SDK features.  We provide here further information for a better understanding
of the detector specific capabilities.

* HwDetInfo

  getCurrImageType/getDefImageType():  Bpp16 only.

  setCurrImageType(): this method do not change the image type which is fixed to Bpp16.

* HwSync

  get/setTrigMode(): the supported mode are IntTrig, ExtTrigSingle, ExtTrigMult and ExtGate

Optional capabilites
........................

None of the hardware capability like HwRoi, HwBin have been implemented.


Configuration
`````````````

No Specific hardware configuration are needed. The detector is sold with a control computer equiped with hardware and software.


How to use
````````````
This is a python code example for a simple test:

.. code-block:: python

  from Lima import Lambda
  from Lima import Core

  cam = Lambda.Camera('/opt/xsp/config')
  hwint = Lambda.Interface(cam)
  ct = Core.CtControl(hwint)

  acq = ct.acquisition()

  # set the detector energy threshold
  cam.setEnergyThreshold(6.0)

  # setting new file parameters and autosaving mode
  saving=ct.saving()

  # set saving in HDF5 bitshuffle compression
  pars=saving.getParameters()
  pars.directory='/data1/test_lima'
  pars.prefix='test1_'
  pars.suffix='.h5'
  pars.fileFormat=Core.CtSaving.HDF5BS
  pars.savingMode=Core.CtSaving.AutoFrame
  saving.setParameters(pars)

  # now ask for 2 sec. exposure and 10 frames
  acq.setAcqExpoTime(2)
  acq.setAcqNbFrames(10)

  ct.prepareAcq()
  ct.startAcq()

  # wait for last image (#9) ready
  lastimg = ct.getStatus().ImageCounters.LastImageReady
  while lastimg !=9:
    time.sleep(1)
    lastimg = ct.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = ct.ReadImage(0)
