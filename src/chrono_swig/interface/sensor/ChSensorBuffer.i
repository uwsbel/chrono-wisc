%{

/* Includes the header in the wrapper code */
#include "chrono_sensor/sensors/radar/ChRadarTypes.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"

using namespace chrono;
using namespace chrono::sensor;

%}

%shared_ptr(chrono::sensor::AccelData)
%shared_ptr(chrono::sensor::GyroData)
%shared_ptr(chrono::sensor::MagnetData)
%shared_ptr(chrono::sensor::GPSData)
%shared_ptr(chrono::sensor::Sensor)

%shared_ptr(chrono::sensor::SensorBuffer)
%shared_ptr(chrono::sensor::SensorBufferT)



%shared_ptr(chrono::sensor::ChOptixSensor)
%shared_ptr(chrono::sensor::ChCameraSensor)
%shared_ptr(chrono::sensor::ChSegmentationCamera)
%shared_ptr(chrono::sensor::ChDepthCamera)
%shared_ptr(chrono::sensor::ChLidarSensor)
%shared_ptr(chrono::sensor::ChRadarSensor)
%shared_ptr(chrono::sensor::ChPhysCameraSensor)

#ifdef CHRONO_HAS_OPTIX
  %shared_ptr(chrono::sensor::ChPhysRadarSensor)
  %shared_ptr(chrono::sensor::PhysRadarFrame)
  %shared_ptr(chrono::sensor::SensorHostPhysRadarFrame)
  %shared_ptr(chrono::sensor::SensorDevicePhysRadarFrame)

  // Direction of arrival is a CUDA vector type with no meaning on the Python side; the bearing
  // of a return is reported by the detections instead.
  %ignore chrono::sensor::RadarPath::dir_rx;
  %ignore chrono::sensor::RadarAntennaElement;

  %include "../../../chrono_sensor/sensors/radar/ChRadarTypes.h"
  %template(RadarObjectVector) std::vector<chrono::sensor::RadarObject>;
#endif

/* Parse the header file to generate wrappers */
%include "../../../chrono_sensor/sensors/ChSensorBuffer.h"

// BufferT Templates

#ifdef CHRONO_HAS_OPTIX

  //camera
  %template(UserR8Buffer) chrono::sensor::SensorBufferT<std::shared_ptr<char[]>>;
  %template(UserRGBA8Buffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>>;
  
  //lidar
  %template(UserDISensorBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>;
  %template(UserXYZISensorBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>;
  %template(UserDIBuffer) chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>>;
  %template(UserXYZIBuffer) chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>>;
  
  //radar
  %template(UserRadarBuffer) chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>;
  %template(UserRadarXYZBuffer) chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>;
  
  //depth camera
  %template(UserDepthBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>>;

#endif

//dynamic
%template(UserAccelBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>>;
%template(UserGyroBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>>;
%template(UserMagnetBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>>;
%template(UserGPSBuffer) chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>>;

//
// ADDITIONAL C++ FUNCTIONS / CLASSES THAT ARE USED ONLY FOR PYTHON WRAPPER
//

#ifdef CHRONO_HAS_OPTIX

  %extend chrono::sensor::SensorConfig<std::shared_ptr<chrono::sensor::PixelDI[]>> {
          public:
          bool HasData() {
              return !($self->Buffer==NULL);
          }
  };
  
  %extend chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>> {
          public:
          bool HasData() {
              return !($self->Buffer==NULL);
          }
  };

  %extend chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>{
      public:
      bool HasData(){
          return !($self->Buffer==NULL);
      }
  }

  %extend chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>{
      public:
      bool HasData(){
          return !($self->Buffer==NULL);
      }
  }

  %extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>> {
    public:
    bool HasData() {
      return !($self->Buffer==NULL);
    }
  };

  %extend chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>> {
  public:
  bool HasData() {
      return !($self->Buffer==NULL);
  }
  };
  
  
  %extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>> {
  public:
  bool HasData() {
      return !($self->Buffer==NULL);
  }
  };


#ifdef CHRONO_PYTHON_NUMPY
  %extend chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelDI[]>> {
  public:
  void GetDIData(float** vec, int* h, int* w, int* c) {
      *h = $self->Height;
      *w = $self->Width;
      *c = sizeof(PixelDI)/sizeof(float);
      *vec = reinterpret_cast<float*>($self->Buffer.get());
  }
  };

  %extend chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarReturn[]>>{
      void GetRadarData(float** vec, int* h, int* w, int* c){
          *h = $self->Height;
          *w = $self->Width;
          *c = sizeof(RadarReturn)/sizeof(float);
          *vec = reinterpret_cast<float*>($self->Buffer.get());
      }
  }
  
  %extend chrono::sensor::RadarBufferT<std::shared_ptr<chrono::sensor::RadarXYZReturn[]>>{
      void GetRadarXYZData(float** vec, int* h, int* w, int* c){
          *h = $self->Height;
          *w = $self->Width;
          *c = sizeof(RadarXYZReturn)/sizeof(float);
          *vec = reinterpret_cast<float*>($self->Buffer.get());
      }
  }

  // Wave-domain radar frame. Each accessor returns a view of the frame's own memory, so it stays
  // valid only while the frame does, and it is empty unless the access filter was asked for that
  // part of the frame.
  %extend chrono::sensor::PhysRadarFrame {
      /// Complex cube as [channel][doppler][2 * range], real and imaginary interleaved.
      void GetCubeData(float** vec, int* h, int* w, int* c) {
          const bool present = static_cast<bool>($self->Cube);
          *h = present ? (int)$self->NumChannels : 0;
          *w = present ? (int)$self->NumDopplerBins : 0;
          *c = present ? 2 * (int)$self->NumRangeBins : 0;
          *vec = present ? reinterpret_cast<float*>($self->Cube.get()) : nullptr;
      }

      /// Beamformed power as [azimuth][doppler][range], linear watts.
      void GetAnglePowerData(float** vec, int* h, int* w, int* c) {
          const bool present = static_cast<bool>($self->AnglePower);
          *h = present ? (int)$self->NumAzimuthBins : 0;
          *w = present ? (int)$self->NumDopplerBins : 0;
          *c = present ? (int)$self->NumRangeBins : 0;
          *vec = present ? $self->AnglePower.get() : nullptr;
      }

      /// Detection power map as [doppler][range][1], linear watts.
      void GetPowerMapData(float** vec, int* h, int* w, int* c) {
          const bool present = static_cast<bool>($self->PowerMap);
          *h = present ? (int)$self->NumDopplerBins : 0;
          *w = present ? (int)$self->NumRangeBins : 0;
          *c = present ? 1 : 0;
          *vec = present ? $self->PowerMap.get() : nullptr;
      }

      /// Per-cell detection threshold as [doppler][range][1], linear watts.
      void GetThresholdMapData(float** vec, int* h, int* w, int* c) {
          const bool present = static_cast<bool>($self->ThresholdMap);
          *h = present ? (int)$self->NumDopplerBins : 0;
          *w = present ? (int)$self->NumRangeBins : 0;
          *c = present ? 1 : 0;
          *vec = present ? $self->ThresholdMap.get() : nullptr;
      }

      /// One detection. Out of range or absent, the returned detection reads all zero.
      chrono::sensor::RadarDetection GetDetection(unsigned int index) {
          if (!$self->Detections || index >= $self->NumDetections)
              return chrono::sensor::RadarDetection();
          return $self->Detections[index];
      }
  }

  %extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelRGBA8[]>> {
  public:
  void GetRGBA8Data(uint8_t** vec, int* h, int* w, int* c) {
      *h = $self->Height;
      *w = $self->Width;
      *c = sizeof(PixelRGBA8)/sizeof(unsigned char);
      *vec = reinterpret_cast<uint8_t*>($self->Buffer.get());
  }
  };

  %extend chrono::sensor::LidarBufferT<std::shared_ptr<chrono::sensor::PixelXYZI[]>> {
  public:
  void GetXYZIData(float** vec, int* h, int* w, int* c) {
      *h = $self->Height;
      *w = $self->Width;
      *c = sizeof(PixelXYZI)/sizeof(float);
      *vec = reinterpret_cast<float*>($self->Buffer.get());
  }
  };

  %extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::PixelDepth[]>> {
  public:
  void GetDepthData(float** vec, int* h, int* w, int* c) {
      *h = $self->Height;
      *w = $self->Width;
      *c = sizeof(PixelDepth)/sizeof(float);
      *vec = reinterpret_cast<float*>($self->Buffer.get());
  }
  };

#endif  

#endif

////    AccelData Extension
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};

////    GyroData Extension
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};

////    MagnetData Extension
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};

////    GPSData Extension
%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>> {
public:
bool HasData() {
    return !($self->Buffer==NULL);
}
};

////    char8 Extension
%extend chrono::sensor::SensorBufferT<std::shared_ptr<char[]>> {
        public:
        bool HasData() {
            return !($self->Buffer==NULL);
        }
};

#ifdef CHRONO_PYTHON_NUMPY

%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::AccelData[]>> {
public:
void GetAccelData(double** vec, int* n) {
    *n = sizeof(AccelData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};

%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GyroData[]>> {
public:
void GetGyroData(double** vec, int* n) {
    *n = sizeof(GyroData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};

%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::MagnetData[]>> {
public:
void GetMagnetData(double** vec, int* n) {
    *n = sizeof(MagnetData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};


%extend chrono::sensor::SensorBufferT<std::shared_ptr<chrono::sensor::GPSData[]>> {
public:
void GetGPSData(double** vec, int* n) {
    *n = sizeof(GPSData)/sizeof(double);
    *vec = reinterpret_cast<double*>($self->Buffer.get());
}
};

%extend chrono::sensor::SensorBufferT<std::shared_ptr<char[]>> {
        public:
        void GetChar8Data(uint8_t** vec, int* h, int* w, int* c) {
            *h = $self->Height;
            *w = $self->Width;
            *c = 1;
            *vec = reinterpret_cast<uint8_t*>($self->Buffer.get());
        }
};

#endif
