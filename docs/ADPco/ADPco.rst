=====
ADPco
=====

:author: Xiaoqiang Wang, Paul Scherrer Institute

.. contents:: Contents

.. _Excelitas:    https://www.excelitas.com
.. _pco.edge:     https://www.excelitas.com/product-category/pcoedge-cooled-scmos-cameras
.. _pco.sdk:      https://www.excelitas.com/product/pco-software-development-kits

Overview
--------

ADPco uses Excelitas pco.sdk_. It should work on both Windows and Linux, although
it has only been tested on Linux.  It has been  tested only with the pco.edge_ 5.5 CHLS cameras and
Silicon Software microEnable 5 marathon VF2 cameralink HS frame grabber.
It may work with other pco cameras, but feathers may be missing. E.g. it does not support
acquire to and read from camera internal memory.

The header files and Windows import libraries are include, so it is ready to build for Windows.

For Linux, the pco.sdk is provided by vendor as a debian package.
The header files and libraries are located in /opt/pco/pco.sdk  For non-debian derived systems,
it is simplest to install it on debian system and copy files over. The other option is
to use *dpkg-deb* tool to extract the package contents, if it is provided by your Linux distribution.

Define this in the areaDetector top level CONFIG_SITE.local file,::

  WITH_PCO = YES
  PCO_INCLUDE = /opt/pco/pco.sdk/include
  PCO_LIB = /opt/pco/pco.sdk/lib

ADPco driver
------------
ADPco inherits from ADDriver.  It adds the following parameters and EPICS records that are
specific to ADPco.

.. cssclass:: table-bordered table-striped table-hover
.. list-table::
   :header-rows: 1
   :widths: auto

   * - EPICS record names
     - Record types
     - drvInfo string
     - Description
   * - TriggerSoftware
     - bo
     - PCO_TRIGGER_SOFTWARE
     - Generate a software trigger when processed.
   * - DelayTime, DelayTime_RBV
     - ao,ai
     - PCO_DELAY_TIME
     - Delay time in seconds before expsure starts.
   * - FramePeriod_RBV
     - ai
     - PCO_FRAME_PERIOD
     - How much time is required to take a single image.
   * - PixelRate, PixelRate_RBV
     - mbbo,mbbi
     - PCO_PIXEL_RATE
     - Pixel rate choices of the camera.
   * - AdcMode, AdcMode_RBV
     - mbbo,mbbi
     - PCO_ADC_MODE
     - If supported by the camera, sensor data can be read out using either single ADC or two ADCs.
       In single ADC operation linearity of image data is enhanced, using dual ADC operation readout
       is faster and allows higher frame rates.
   * - CameraSetup, CameraSetup_RBV
     - mbbo,mbbi
     - PCO_CAMERA_SETUP
     - The camera shutter mode. Changing this paramter will reboot the camera. The driver will appear disconnected.
       If asyn autoconnect is not disabled, the driver will attempt periodic reconnect.
   * - BitAlignement, BitAlignement_RBV
     - mbbo,mbbi
     - PCO_BIT_ALIGNMENT
     - If the dynamic resolution of the camera is less than 16 bit/pixel and because the transferred
       image data is always sent as one WORD(16 bit) per pixel, the data can be either MSB or LSB aligned.
   * - RebootCamera
     - bo
     - PCO_REBOOT_CAMERA
     - Reboot the camera when processed. The driver will appear disconnected. If asyn autoconnect is not disabled,
       the driver will attempt periodic reconnect.
   * - AcquireMode, AcquireMode_RBV
     - mbbo,mbbi
     - PCO_ACQUIRE_MODE
     - The function of <acq enbl> input.
   * - FramesDropped_RBV
     - ai
     - PCO_FRAMES_DROPPED
     - The number of frames dropped since the beginning of current acquisition.

IOC startup script
------------------
The command to configure an ADPco camera in the startup script is::

  ADPcoConfig(const char *portName, const char *cameraId,
                  size_t maxMemory, int priority, int stackSize)

``portName`` is the name for the ADPco port driver

``cameraId`` is used to identify which camera to control. On Windows it opens the next avialable camera no matter the input.
 On Linux, the it can be the camera serial number.

``maxMemory`` is the maximum amount of memory the NDArrayPool is allowed to allocate.  0 means unlimited.

``priority`` is the priority of the port thread.  0 means medium priority.

``stackSize`` is the stack size.  0 means medium size.


MEDM screens
------------
