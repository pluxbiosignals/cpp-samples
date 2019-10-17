/**
 * \file
 * \copyright  Copyright 2015-2019 PLUX - Wireless Biosignals, S.A.
 * \version    1.7
 * \date       April 2019
 
 \mainpage
 The PLUX C++ API brings to C++ applications all the functionality of PLUX devices.
 The Plux::BaseDev class (or any of its derived classes) encapsulates a connection to a PLUX device.
 Each Plux::BaseDev-derived class provides methods to access the specific functionality of certain PLUX devices.
 
 The API is not thread-safe since its methods cannot be called simultaneously by different threads.
 The only exception to this rule is the Plux::BaseDev::interrupt() method.
 
 The API is composed of the plux.h header file and the pre-compiled library files for Windows, Mac OS and Linux platforms.
 
 The current API version string is available in the macro #PLUX_API_VERSION.
 
 The API has several bindings to other programming languages. The Python binding supports all API features. The Java binding and the .Net binding (Windows only) support a limited set of features from the API.
 
 \namespace Plux
 The PLUX C++ API namespace
 */

#ifndef _PLUXHEADER_
#define _PLUXHEADER_

#include <map>
#include <string>
#include <vector>
#include <time.h>

#define PLUX_API_VERSION   "1.7"   ///< Current API version string


namespace Plux
{
   typedef std::string        String;  ///< String type.
   typedef std::vector<int>   Ints;    ///< Vector of int type.
   typedef std::vector<bool>  Bools;   ///< Vector of bool type.
   
   /// Information about a device found by BaseDev::findDevices().
   struct DevInfo
   {
      /// Device path (the path to be passed to the device class constructor).
      /// \see BaseDev::BaseDev()
      String   path;
      
      /// Device description as returned in its \c description property.
      /// \see BaseDev::getProperties()
      String   description;
   };
   typedef std::vector<DevInfo>  DevInfos;   ///< Vector of DevInfo type.
   
   /// This class encapsulates a value of one of following data types: bool, int, float and #String
   /// \see Properties
   class Variant
   {
   public:
      /// Data type enumeration
      enum Type
      {
         TypeNone,   ///< No data
         TypeBool,   ///< bool data type
         TypeInt,    ///< int data type
         TypeFloat,  ///< float data type
         TypeString, ///< #String data type
      };
      
      const Type type;  ///< Data type
      
      union
      {
         bool   b;
         int    i;
         float  f;
         String *s;
      }; ///< Data value
      
      Variant(void)     : type(TypeNone) , i(0) {}    ///< Constructs an empty %Variant.
      Variant(bool  _b) : type(TypeBool) , b(_b) {}   ///< Constructs a %Variant with a bool value.
      Variant(int   _i) : type(TypeInt)  , i(_i) {}   ///< Constructs a %Variant with an int value.
      Variant(float _f) : type(TypeFloat), f(_f) {}   ///< Constructs a %Variant with a float value.
      Variant(const char *_s) : type(TypeString), s(new String(_s)) {}     ///< Constructs a %Variant with a #String value from a const char*.
      Variant(const String &_s) : type(TypeString), s(new String(_s)) {}   ///< Constructs a %Variant with a #String value.
      
      /// Constructs a %Variant as a copy of another %Variant.
      Variant(const Variant &v) : type(v.type)
      {
         if (v.type == TypeString)
            s = new String(*v.s);
         else
            i = v.i;
      }
      
      ~Variant(void) {if (type==TypeString) delete s;}
   };
   
   /// Map from #String keywords to Variant types.
   /// \see BaseDev::getProperties(), SessionSource::properties
   typedef std::map<String, Variant>  Properties;

   /** Generic PLUX C++ API exception.
    This exception is the base class for all specific exceptions.
    Catch this exception class to ensure that all PLUX C++ API exceptions are caught.
    */
   class Exception
   {
   public:
      String dbgInfo;

      Exception(const String &_info) : dbgInfo(_info) {}

      /// Returns the exception description string.
      virtual String getDescription(void) const
      {
         return "Generic PLUX exception.";
      }
   };
   
   /** Base class for all notification exceptions.
    A notification exception is thrown when an external condition is preventing the API from succeeding its current operation.
    */
   class NotifException : public Exception
   {
   public:
      NotifException(const String &_info) : Exception(_info) {}
   };

   /** Namespace for all notification exceptions.
    These exceptions derive from NotifException class.
    A notification exception is thrown when an external condition is preventing the API from succeeding its current operation.
    */
   namespace Notification
   {
      /// %Exception thrown if there was a problem while opening the communication port.
      class OpeningPort : public NotifException
      {
      public:
         OpeningPort(const String &_info) : NotifException(_info) {}

         String getDescription(void) const
         {
            return "The communication port does not exist or it is already being used.";
         }
      };
      
      /// %Exception thrown if there was a problem while initializing the communication port.
      class InitializingPort : public NotifException
      {
      public:
         InitializingPort(const String &_info) : NotifException(_info) {}

         String getDescription(void) const
         {
            return "The communication port could not be initialized.";
         }
      };
      
      /// %Exception thrown if no Bluetooth adapter was found while trying to connect to a Bluetooth device.
      class AdapterNotFound : public NotifException
      {
      public:
         AdapterNotFound(const String &_info) : NotifException(_info) {}

         String getDescription(void) const
         {
            return "No Bluetooth adapter was found.";
         }
      };
      
      /// %Exception thrown if the requested device path was not found.
      class DeviceNotFound : public NotifException
      {
      public:
         DeviceNotFound(const String &_info) : NotifException(_info) {}

         String getDescription(void) const
         {
            return "The device could not be found.";
         }
      };
      
      /// %Exception thrown if the connection with the device was lost.
      class ContactingDevice : public NotifException
      {
      public:
         ContactingDevice(const String &_info) : NotifException(_info) {}

         String getDescription(void) const
         {
            return "The communication with the device was lost.";
         }
      };
   };
   
   /** Base class for all error exceptions.
    An error exception is thrown when an application programming error is preventing the API from succeeding its current operation.
    */
   class ErrorException : public Exception
   {
   public:
      ErrorException(const String &_info) : Exception(_info) {}
   };
   
   /** Namespace for all error exceptions.
    These exceptions derive from ErrorException class.
    An error exception is thrown when an application programming error is preventing the API from succeeding its current operation.
    */
   namespace Error
   {
      /// %Exception thrown if a method call has an invalid parameter value.
      class InvalidParameter : public ErrorException
      {
      public:
         InvalidParameter(const String &_info) : ErrorException(_info) {}

         String getDescription(void) const
         {
            return "Invalid parameter.";
         }
      };
      
      /// %Exception thrown if the requested operation cannot be completed due to current device state.
      class InvalidOperation : public ErrorException
      {
      public:
         /// \cond
         const int code;
         InvalidOperation(const String &_info, int _code = 0) : ErrorException(_info), code(_code) {}
         /// \endcond
         
         String getDescription(void) const
         {
            return "Invalid operation on current device state (code " + std::to_string(code) + ").";
         }
      };
      
      /// %Exception thrown if the requested operation is not supported by the device.
      class NotSupported : public ErrorException
      {
      public:
         NotSupported(const String &_info) : ErrorException(_info) {}

         String getDescription(void) const
         {
            return "Operation not supported by the device.";
         }
      };
      
      /** %Exception thrown if an invalid object instance method was called.
       A BaseDev object instance becomes invalid when it is successfully promoted to a derived class.
       \see SignalsDev::SignalsDev(BaseDev &baseDev), MemoryDev::MemoryDev(BaseDev &baseDev)
       */
      class InvalidInstance : public ErrorException
      {
      public:
         InvalidInstance(const String &_info) : ErrorException(_info) {}

         String getDescription(void) const
         {
            return "The object instance is invalid.";
         }
      };
      
      /// %Exception thrown if an API module is missing or invalid (DLL or .so file).
      class MissingModule : public ErrorException
      {
      public:
         const String module; ///< Filename of the missing module.
         MissingModule(const String &_info, const String &_module = "") :
            ErrorException(_info), module(_module) {} ///< %Exception constructor.

         String getDescription(void) const
         {
            return "The API module " + module + " is missing or invalid.";
         }
      };
   };

   /// %Event timestamp class.
   struct Clock
   {
      /// %Clock source type enumeration.
      enum Source
      {
         None,       ///< No timestamp value.
         RTC,        ///< Device real-time clock timestamp.
         FrameCount, ///< Acquisition frame counter timestamp.
         /// \cond
         Bluetooth,
         //DigAccel,
         /// \endcond
      };
      
      Source source; ///< %Clock source for this timestamp.
      int value;     ///< Timestamp value.
   };
   
   /// %Event base class.
   struct Event
   {
      /// %Event type enumeration.
      enum Type
      {
         // BaseDev events
         
         // SignalsDev events
         TypeDigInUpdate = 3,    ///< %Event is a EvtDigInUpdate object.
         TypeSchedChange = 4,    ///< %Event is a EvtSchedChange object.
         TypeSync = 5,           ///< %Event is a EvtSync object.
         TypeGestFeatures = 7,   ///< %Event is a EvtGestFeatures object.
         TypeDisconnect = 8,     ///< %Event is a EvtDisconnect object.
         TypeSignalGood = 9,     ///< %Event is a EvtSignalGood object.
         TypeBattery = 10,       ///< %Event is a EvtBattery object.
         
         // internal use only
         /// \cond
         TypeTest = 0x70,
         /// \endcond
      };
      
      Type type;  ///< %Event type.
   };
   
   // BaseDev events
   
   // SignalsDev events
   /// Digital port input change event class.
   struct EvtDigInUpdate : public Event
   {
      Clock timestamp;  ///< %Event timestamp.
      int   channel;    ///< The digital input which changed state, starting at zero.
      bool  state;      ///< New state of digital port input. If true, new state is High, otherwise it is Low.
   };
   
   /// %Session schedule change event class.
   struct EvtSchedChange : public Event
   {
      /// %Schedule action enumeration.
      enum Action
      {
         SchedStarted = 0,       ///< A scheduled session has started.
         SchedEnded = 1,         ///< A scheduled session has ended.
         SchedCannotStart = 2,   ///< A scheduled session could not start.
      };
      
      Action   action;           ///< Change that occurred in a session schedule.
      time_t   schedStartTime;   ///< Schedule::startTime attribute value of the schedule (unique schedule identifier).
   };
   
   /// Synchronization event class.
   struct EvtSync : public Event
   {
      std::vector<Clock> timestamps;   ///< Synchronized timestamps.
   };
   
   /// Gesture features event class.
   /// This event is sent by GestureWatch devices only.
   struct EvtGestFeatures : public Event
   {
      //! \{
      int seq, length, zcrossht, macc, mean90hb, acczorientation;
      //! \}
   };
   
   /// Device disconnect event class.
   struct EvtDisconnect : public Event
   {
      /// Disconnect reason enumeration.
      enum Reason
      {
         Timeout = 1,         ///< Idle connection timeout (15 min) has elapsed.
         ButtonPressed = 2,   ///< Device button was pressed.
         BatDischarged = 4,   ///< Device battery is discharged.
      };
      
      Reason   reason;  ///< Reason for the device disconnection.
   };
   
   /// Sensor signal good event class.
   struct EvtSignalGood : public Event
   {
      int   port;    ///< Sensor port.
      bool  isGood;  ///< True if the sensor signal is valid; false otherwise.
   };
   
   /// Battery state event class.
   struct EvtBattery : public Event
   {
      float voltage,    ///< Battery voltage in Volts.
      percentage; ///< Remaining battery charge as a percentage of full capacity.
   };
   
   // internal use only
   /// \cond
   struct EvtTest : public Event
   {
      int subtype, len;
      const void *data;
   };
   /// \endcond
   
   /** Base class for all PLUX devices.
    This class provides the basic functionality common to all PLUX devices.
    A BaseDev device can be instantiated if a specific device functionality is not required.
    A BaseDev instance can be promoted afterwards to a derived class instance, based on device properties.
    This promotion is done by passing the BaseDev instance to the constructor of the derived class.
    
    Alternatively, a derived class can be directly instantiated from the path string if a specific device functionality is required.
    
    It is not possible to demote a derived class instance to a BaseDev instance.
    */
   class BaseDev
   {
   public:
      /// Product ID enumeration 
      enum ProductId
      {
         PID_BioPlux        = 0x0101,   // 1.1
         PID_BiosignalsPlux = 0x0201,   // 2.1
         PID_MotionPlux     = 0x020A,   // 2.10
         PID_OpenBan        = 0x0214,   // 2.20
         PID_fNIRS          = 0x021E,   // 2.30
         PID_StimPlux       = 0x0301,   // 3.1
         PID_MuscleBan      = 0x0502,   // 5.2
         PID_BITalino       = 0x0601,   // 6.1
         PID_BITalinoRev    = 0x0602,   // 6.2
      };

      /** Finds PLUX devices within the given domain.
       \param domain Domain of search (optional). It can be "BTH" for a Classic Bluetooth search,
       "BLE" for a Bluetooth Low Energy search, or "USB" for an USB search.
       If it is not given (or if it is an empty string), a search is attempted in all domains.
       */
      static DevInfos findDevices(const String &domain = "");
      
      /** Connects to a PLUX device.
       \param path Path to the PLUX device. It can have one of the following formats:
       Format               | Meaning                          | Supported platforms
       -------------------- | -------------------------------- | -------------------
       xx:xx:xx:xx:xx:xx    | Classic Bluetooth MAC address    | All platforms
       BTHxx:xx:xx:xx:xx:xx | Classic Bluetooth MAC address    | All platforms
       BLExx:xx:xx:xx:xx:xx | Bluetooth Low Energy MAC address | Windows
       COMn                 | COM serial/virtual port          | Windows
       /dev/...             | Serial device                    | Mac OS and Linux
       USB or USBn          | USB adapter connection           | Windows
       */
      BaseDev(const String &path);
      
      /** Disconnects from the device.
       If the device is in real-time acquisition (through SignalsDev.start()), it is automatically stopped.
       */
      virtual ~BaseDev(void);
      
      /** Returns the device properties.
       The device properties map #String keywords to Variant values. The currently defined keywords are:
       Keyword        | Meaning
       -------------- | -------
       \c description | Device description string
       \c fwVersion   | Device firmware version
       \c hwVersion   | Device hardware version (not present in old devices)
       \c memorySize  | Device internal memory size in kBytes (only on MemoryDev instances) \n (zero if internal memory is not available)
       \c path        | Device path string as passed to constructor
       \c productID   | Device product identifier
       \c uid         | Device unique identifier string
       
       The \c fwVersion, \c hwVersion and \c productID properties are composed of two numbers (the major and minor numbers).
       These properties and are coded as 2-byte integers where the higher byte is the major number
       and the lower byte is the minor number.
       */
      Properties  getProperties(void);
      
      /// \cond
      void setParameter(int port, int index, const void *data, int dataLen);
      int  getParameter(int port, int index, void *data, int maxLen);
      
      void reset(void);
      /// \endcond
      
      /** Returns the remaining battery charge as a percentage of full capacity.
       The returned value is -1 if the device is charging.
       */
      float getBattery(void);
      
      /** Sets the receiving timeout value for loop().
       \param timeout Timeout value in milliseconds (optional).
       If \c timeout is 0, an immediate timeout condition occurs in loop() if there are no pending messages from device to dispatch.
       If \c timeout is not given or if it is -1, a timeout condition never occurs.
       \see onTimeout()
       */
      void setTimeout(int timeout = -1);
      
      /** Runs the device message loop.
       The message loop receives and dispatches the messages from the device to the callbacks.
       This method returns when a callback returns true.
       \remarks This method cannot be called from a callback.
       */
      void loop(void);
      
      /** Sends an interrupt signal to loop().
       This method can be called from a thread while loop() is running on another thread.
       The onInterrupt() callback will be called from loop() (in the context of its thread).
       \param param An optional parameter to be passed to onInterrupt() callback.
       \remarks This method cannot be called from a callback, and it cannot be called if loop() is not running in another thread.
       */
      void interrupt(void *param = NULL);
      
      /** Event callback.
       This callback is called by message loop when an event is received from the device.
       In order to receive device events, an application must derive BaseDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       \param evt Received event, which can be a EvtDigInUpdate, EvtDisconnect, EvtSchedChange or EvtSync object.
       \return Return true to exit message loop or false otherwise. Default callback returns false.
       */
      virtual bool onEvent(const Event &evt) {return false;}
      
      /** Timeout callback.
       This callback is called by message loop when a timeout condition occurred.
       In order to receive timeout conditions, an application must derive BaseDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       \return Return true to exit message loop or false otherwise. Default callback returns false.
       \see setTimeout()
       */
      virtual bool onTimeout(void) {return false;}
      
      /** Interrupt signal callback.
       This callback is called by message loop after interrupt() is called from another thread.
       In order to receive interrupt signals, an application must derive BaseDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       \param param Optional parameter passed to interrupt() (or NULL if no parameter was given).
       \return Return true to exit message loop or false otherwise. Default callback returns false.
       */
      virtual bool onInterrupt(void *param) {return false;}
      
      
      // internal use only
      /// \cond
      struct X;
      X *x;
      
   protected:
      BaseDev(BaseDev &original);
      
   private:
      BaseDev& operator=(const BaseDev&); // assignments are not allowed
      /// \endcond
   };

   struct Sensor
   {
      /// %Sensor class enumeration
      enum Class
      {
         UNKNOWN_CLASS,
         EMG,
         ECG,
         LIGHT,
         EDA,
         BVP,
         RESP,
         XYZ,
         SYNC,
         EEG,
         SYNC_ADAP,
         SYNC_LED,
         SYNC_SW,
         USB,
         FORCE,
         TEMP,
         VPROBE,
         BREAKOUT,
         OXIMETER,
         GONI,
         UPPER_BOUND
      };

      /// %Sensor sleeve color enumeration
      enum Color
      {
         UNKNOWN_COLOR,
         BLACK,
         GRAY,
         WHITE,
         DARKBLUE,
         LIGHTBLUE,
         RED,
         GREEN,
         YELLOW,
         ORANGE
      };

      uint64_t    uid;
      Class       clas;
      int         serialNum, hwVersion, type;
      time_t      productionTime;
      Color       color;
      Properties  characteristics, measurCalib;
   };

   /** Map of ports to Sensor data.
   \see SignalsDev::getSensors, Schedule::sources
   */
   typedef std::map<int, Sensor> Sensors;

   /** Acquisition source class.
    An acquisition source is a sensor connected to a device port.
    Sampling frequency divisor and sampling resolution can be set for each source.
    Each source can have up to 8 channels.
    All source channels share the same frequency divisor and sampling resolution settings defined for the source.
    \see SignalsDev::start(float baseFreq, const Sources &sources), Schedule::sources
    */
   struct Source
   {
      int   port;          ///< %Source port (1...8 for analog ports). Default value is zero.
      int   freqDivisor;   ///< %Source frequency divisor from acquisition base frequency (>= 1). Default value is 1.
      int   nBits;         ///< %Source sampling resolution in bits (8, 16 or 24). The value 24 is accepted for SpO2 / fNIRS sensors only. Default value is 16.  
      int   chMask;        ///< Bitmask of source channels to sample (bit 0 is channel 0, etc). Default value is 1 (channel 0 only).
      
      /** Initializes all structure fields.
       All arguments are optional. A structure field is initialized with the corresponding argument value, if provided.
       Otherwise, the field is initialized with its default value.
       */
      Source(int _port=0, int _freqDivisor=1, int _nBits=16, int _chMask=0x01) :
         port(_port), freqDivisor(_freqDivisor), nBits(_nBits), chMask(_chMask) {}
   };
   
   /** Vector of Source type.
    \see SignalsDev::start(float baseFreq, const Sources &sources), Schedule::sources
    */
   typedef std::vector<Source> Sources;
   
   /// Base class for PLUX signal-acquiring devices.
   class SignalsDev : public BaseDev
   {
   public:
      /** Connects to a PLUX device.
       \param path Path to the PLUX device. It has the same meaning as in BaseDev::BaseDev().
       */
      SignalsDev(const String &path);
      
      /** Promotes a BaseDev instance to SignalsDev.
       \param baseDev Instance to be promoted. If promotion is successful, \c baseDev is no longer valid.
       */
      SignalsDev(BaseDev &baseDev);
      
      void getSensors(Sensors &sensors);

      /** Starts a real-time acquisition session.
       This method is a shortcut to start(float baseFreq, const Sources &sources).
       It can be called if all requested sources are to be sampled at the same frequency and resolution,
       and at their channel 0 only.
       \param freq Acquisition sampling frequency in Hertz.
       \param portMask Bitmask of the device ports to acquire.
       The least significant bit corresponds to port 1, the next bit to port 2, etc. See Source::port.
       \param nBits Sampling resolution in bits. It can be 8 or 16. This parameter is ignored for BITalino devices.
       */
      void start(float freq, int portMask, int nBits);
      
      /** Starts a real-time acquisition session.
       This method is a shortcut to start(float baseFreq, const Sources &sources).
       It can be called if all requested sources are to be sampled at the same frequency and resolution,
       and at their channel 0 only.
       \param freq Acquisition sampling frequency in Hertz.
       \param ports Ports to acquire. See Source::port.
       \param nBits Sampling resolution in bits. It can be 8 or 16. This parameter is ignored for BITalino devices.
       */
      void start(float freq, const Ints &ports, int nBits);
      
      /** Starts a real-time acquisition session.
       \param baseFreq Acquisition base sampling frequency in Hertz.
       \param sources Signal sources to acquire. See Source.
       \remarks This method is not supported on BITalino devices.
       */
      void start(float baseFreq, const Sources &sources);
      
      /** Stops a real-time acquisition session.
       \remarks Call MemoryDev::stopSessionAcq() to stop an internal acquisition session.
       */
      void stop(void);
      
      /** Returns the total number of acquisition channels.
       The returned value is the total number of channels across all the acquisition sources, or zero if the device is stopped.
       This value is the number of elements of the data array passed to the onRawFrame() callback.
       */
      int getNumChannels(void);
      
      /** Sets the digital output state.
       \param state Output state to assign. If true, output is set to High, otherwise it is set to Low.
       \remarks On BITalino devices, only the first output (O1) is assigned to the given state. The other outputs are set to Low.
       Call BITalinoDev::setDOut() to assign all digital outputs.
       */
      void setDOut(bool state);
      
      /** Raw frames callback.
       This callback is called by message loop when a real-time data acquisition frame is received from the device.
       In order to receive data frames, an application must derive SignalsDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       \param nSeq Sequence number of the frame. This number is zero for the first frame of the acquisition session,
       and then incremented by one for every subsequent frame. If this number differs more than one between consecutive
       frames, it means that data frames for the missing sequence numbers were lost, possibly due to connection problems.
       \param data Frame sampling data. Each value is acquired from a channel of each requested source,
       in the same source order as given in start(float baseFreq, const Sources &sources), or in the same port order as given in
       start(float freq, const Ints &ports, int nBits), or from lower ports to higher ports as given in
       start(float freq, int portMask, int nBits). For each multi-channel source, the values are presented from lower channels
       to higher channels. The values range for 16-bit samples is 0...65535 and for 8-bit samples is 0...255 .
       The values for a source with a frequency divisor greater than one are updated only in the frames with a sequence number
       multiple of the frequency divisor value. The number of elements of this array can be obtained by calling getNumChannels().
       \return Return true to exit message loop or false otherwise. Default callback returns false.
       */
      virtual bool onRawFrame(int nSeq, const int data[]) {return false;}
   };

   /** %Session schedule class.
    A session schedule contains all the information needed to start an internal acquisition session.
    \see MemoryDev::addSchedule(), MemoryDev::getSchedules()
    */
   struct Schedule
   {
      /** Scheduled session start time or condition (1 or a time_t value).
       If this attribute has the value 1, the scheduled session will start when the digital port input changes.
       Otherwise, the scheduled session will start when the device real-time clock
       reaches this value.
       
       This attribute value is unique to each schedule on the device,
       so this attribute is used as a schedule identifier in MemoryDev::deleteSchedule() and Session::schedStartTime.
       Default value is 1.
       */
      time_t   startTime;
      
      /** Scheduled session duration limit in seconds.
       If this attribute is zero, no session duration limit is set.
       An internal acquisition session can always be stopped by calling MemoryDev::stopSessionAcq().
       Default value is zero.
       */
      int      duration;
      int      nSessions;     ///< Number of sessions to be recorded. Default value is one.
      int      repeatPeriod;  ///< Repetition period in seconds. Default value is zero.
      float    baseFreq;      ///< Acquisition base sampling frequency in Hertz. Default value is 1000.
      Sources  sources;       ///< Scheduled session sources.
      String   text;          ///< Optional user text to store in Session::text attribute of saved sessions (maximum of 70 characters).
      
      /** Initializes all structure fields.
       All arguments are optional. A structure field is initialized with the corresponding argument value, if provided.
       Otherwise, the field is initialized with its default value.
       */
      Schedule(void) : startTime(1), duration(0), baseFreq(1000), nSessions(1), repeatPeriod(0) {}
   };
   typedef std::vector<Schedule> Schedules;  ///< Vector of Schedule type.
   
   /** Saved acquisition source class with sensor information.
    \see Session::sources
    */
   struct SessionSource : public Source
   {
      Sensor     sensor;      ///< Information about the Sensor associated with the source.
      Properties properties;  ///< Additional source properties.
   };
   typedef std::vector<SessionSource> SessionSources;  ///< Vector of SessionSource type.
   
   /** Stored session header class.
    MemoryDev::getSessions() returns objects of this class.
    */
   struct Session
   {
      /** %Session start time.
       This attribute value is unique to each session stored on the device,
       so this attribute is used as a session identifier in MemoryDev::replaySession().
       */
      time_t      startTime;
      
      /** The Schedule::startTime attribute value of the schedule associated with this session,
       which is an unique schedule identifier.
       */
      time_t         schedStartTime;
      int            nFrames;       ///< Total number of frames in the stored session.
      float          baseFreq;      ///< %Session acquisition base sampling frequency in Hertz.
      SessionSources sources;       ///< %Session sources.
      String         text;          ///< Optional user text from the Schedule::text attribute of the schedule associated with this session.
      Properties     properties;    ///< Additional session properties.
   };
   typedef std::vector<Session> Sessions;  ///< Vector of Session type.
   
   /// Base class for PLUX signal-acquiring devices with internal memory.
   class MemoryDev : public SignalsDev
   {
   public:
      /** Connects to a PLUX device.
       \param path Path to the PLUX device. It has the same meaning as in BaseDev::BaseDev().
       */
      MemoryDev(const String &path);
      
      /** Promotes a BaseDev instance to MemoryDev.
       \param baseDev Instance to be promoted. If promotion is successful, \c baseDev is no longer valid.
       */
      MemoryDev(BaseDev &baseDev);
      
      /** Sets the device real-time clock.
       \param t Time to set on the device (optional). If this parameter is not given, current host time is used.
       */
      void   setTime(time_t t=0);
      
      /// Returns the device current real-time clock.
      time_t getTime(void);
      
      /** Returns all session schedules stored on the device.
       The returned schedules can refer to a running session (a running schedule, at most one such schedule) or
       to a session to run in the future. The schedules were previously added by calling addSchedule() or start().
       \param schs Reference to a #Schedules object to be filled by this method.
       \return Index to the running schedule in the returned vector, starting at 1; if zero, no schedule is running.
       \remarks On current devices firmware, only one schedule can be stored.
       */
      int  getSchedules(Schedules &schs);
      
      /** Adds a session schedule to the device.
       An internal acquisition session (i.e., an acquisition to internal memory) will start
       when the schedule start time or condition is met.
       \param sch %Schedule to add.
       \remarks On current devices firmware, only one schedule can be stored.
       */
      void addSchedule(const Schedule &sch);
      
      /** Deletes a session schedule from the device.
       \param startTime Schedule::startTime attribute value of the schedule to delete.
       \remarks A running schedule cannot be deleted.
       */
      void deleteSchedule(time_t startTime);
      
      /** Deletes all session schedules from the device.
       \remarks A running schedule cannot be deleted, so this method cannot be called while the device is acquiring.
       */
      void deleteAllSchedules(void);
      
      /** Stops an internal acquisition session.
       \remarks Call stop() to stop a real-time acquisition.
       */
      void stopSessionAcq(void);
      
      /** Returns the headers of all sessions stored on the device.
       \param sessions Reference to a #Sessions object to be filled by this method.
       \remarks This method cannot be called while the device is acquiring.
       */
      void getSessions(Sessions &sessions);
      
      /** Replays a session stored on the device.
       While a session is being replayed, this method runs a message loop which replaces loop().
       The message loop receives and dispatches real-time messages and stored session data to the callbacks.
       When a callback returns true, the session replay ends and this method returns.
       The method returns if the end of stored session was reached.
       \param startTime Session::startTime attribute value (as returned by getSessions()) of the session to replay.
       \param iniFrame Frame sequence number from which the session starts to be replayed (if ommited, start from frame 0).
       \remarks This method cannot be called from a callback. This method cannot be called while the device is acquiring.
       */
      void replaySession(time_t startTime, int iniFrame=0);
      
      /** Deletes all sessions stored on the device.
       \remarks This method cannot be called while the device is acquiring.
       */
      void deleteAllSessions(void);
      
      /** Returns the amount of memory used by all sessions stored on the device in kBytes.
       \remarks This method cannot be called while the device is acquiring.
       \see \c memorySize property in getProperties().
       */
      int  getMemoryUsed(void);
      
      /** Raw frames callback for stored sessions replay.
       This callback is called by replaySession() for every frame to be replayed from the stored session.
       In order to receive data frames, an application must derive MemoryDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       
       This callback parameters are equivalent to onRawFrame() callback parameters.
       \return Return true to exit replaySession() or false otherwise. Default callback returns false.
       */
      virtual bool onSessionRawFrame(int nSeq, const int data[]) {return false;}
      
      /** %Event callback for stored sessions replay.
       This callback is called by replaySession() for every event to be replayed from the stored session.
       In order to receive device events, an application must derive MemoryDev class (or any of its derived classes) to a new class
       and override this method in the new class.
       \param evt Replayed event, which can be a EvtDigInUpdate or EvtSync object.
       \return Return true to exit replaySession() or false otherwise. Default callback returns false.
       */
      virtual bool onSessionEvent(const Event &evt) {return false;}
   };

   /// Base class for Plux BITalino devices.
   class BITalinoDev : public SignalsDev
   {
   public:
      /// Current device state returned by BITalinoDev::getState()
      struct State
      {
         int   analog[6],     ///< Array of analog inputs values (0...1023).
         battery,       ///< Battery voltage value (0...1023).
         batThreshold;  ///< Low-battery LED threshold (last value set with BITalinoDev::setBatThreshold()).
         /// Array of digital ports states (false for low level or true for high level).
         /// The array contents are: I1 I2 O1 O2.
         bool  digital[4];
      };
      
      /** Connects to a BITalino device.
       \param path Path to the BITalino device. It has the same meaning as in BaseDev::BaseDev().
       */
      BITalinoDev(const String &path);
      
      /** Promotes a BaseDev instance to BITalinoDev.
       \param baseDev Instance to be promoted. If promotion is successful, \c baseDev is no longer valid.
       */
      BITalinoDev(BaseDev &baseDev);
      
      /** Assigns the digital outputs states.
       * \param[in] output Vector of booleans to assign to digital outputs, starting at first output (O1).
       * On each vector element, false sets the output to low level and true sets the output to high level.
       * The vector must contain exactly 4 elements for original BITalino (4 digital outputs)
       * or exactly 2 elements for BITalino 2 (2 digital outputs).
       * \remarks This method must be called only during an acquisition on original BITalino. On BITalino 2 there is no restriction.
       */
      void setDOut(const Bools &output);
      
      /** Sets the battery voltage threshold for the low-battery LED.
       * \param[in] threshold Battery voltage threshold.
       * Parameter value | Voltage Threshold
       * --------------- | -----------------
       *               0 |   3.4 V
       *            ...  |   ...
       *              63 |   3.8 V
       * \remarks This method cannot be called during an acquisition.
       */
      void setBatThreshold(int threshold);
      
      /** Assigns the analog (PWM) output value (BITalino 2 only).
       * \param[in] value Analog output value to set (0...255).
       * The analog output voltage is given by: V (in Volts) = 3.3 * (value+1)/256
       */
      void setPWM(int value);
      
      /** Returns current device state (BITalino 2 only).
       * \remarks This method cannot be called during an acquisition.
       */
      State getState(void);
   };

   /// This class is not covered in this documentation.
   class StimDev : public BaseDev
   {
      /// \cond
   public:
      StimDev(const String &path);
      StimDev(BaseDev &baseDev);
      
      void  startSession(int stateChanges = 0);
      void  stop(void);
      void  startMode(int mode, float time = 0);
      void  setWaveOnMode(const float *wave, int mode, bool isVoltage = false);
      void  setFrequencyOnMode(int freq, int mode);
      void  setTimeOnState(float time, int state);
      void  setModeOnState(int state, int mode);
      void  setMaxStateChanges(int maxChanges);
      void  setNextStateOnState(int state, int nextState);
      void  assignTriggerToMode(int mode);
      void  outputUnitPulse(int mode);
      void  setHVoltageState(bool HV);
      void  setCalib(int comID, int dacID, int calibVal);
      /// \endcond
   };
}

#endif // _PLUXHEADER_
