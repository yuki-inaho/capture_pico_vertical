#pragma once
#include "header.h"
#include "SensorWrapper.h"
#include "VerticalPicoSensorClass.h"

class VerticalPicoSensorSetter{
    public:
        void initialize(int _image_width, int _image_height, int _image_fps); 
        void setSensorObject(std::vector<SensorWrapper> &_sensor_vec);   
        int getNumSensor();
        bimap_t bm_idx2serial;
        
    private:    


        int n_sensor;
        CameraManager manager;        
        std::vector<VerticalPicoSensor> pico_sensor_list;        

        int left_ind;
        int right_ind;

        PicoListener listener;
        royale::Vector<std::unique_ptr<royale::ICameraDevice>> camCameraDeviceVec;
};