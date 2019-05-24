#include "VerticalPicoSensorClass.h"

using namespace std;
using namespace royale;

void
VerticalPicoSensor::initialize(std::string _serial_number, int _image_width, int _image_height, int _image_fps, std::unique_ptr<ICameraDevice>& cameraDevice, PicoListener &listener){
    // Pico Flexxの始動処理
    cout << "Trying to open : " << _serial_number << endl;

    // the camera device is now available and CameraManager can be deallocated here
    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        cerr << "Could not open pico flexx" << endl;
        return;
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return;
    }
    // retrieve the lens parameters from Royale

    LensParameters lensParameters;
    status = cameraDevice->getLensParameters (lensParameters);
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Can't read out the lens parameters" << endl;
        return;
    }

    listener.setLensParameters (lensParameters);
    // register a data listener
    //if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return;
    }
    
    // exposure time is available between 100-2000
/*
    if (cameraDevice->setExposureTime(500) != CameraStatus::SUCCESS) //
    {
        cerr << "Error setting exposure time" << endl;
        return;
    }
*/

    if (cameraDevice->setExposureMode(royale::ExposureMode::AUTOMATIC) != CameraStatus::SUCCESS) //50 500
    {
        cerr << "Error setting exposure time" << endl;
        return ;
    }

/*
    cout << "test" << endl;
    // exposure time is available between 100-2000
    if (cameraDevice->setFrameRate(5) != CameraStatus::SUCCESS) //
    {
        cerr << "Error setting fps" << endl;
        return;
    }
    cout << "test" << endl;
*/
    // register a data listener
    //if (cameraDevice->setFilterLevel(FilterLevel::Legacy) != CameraStatus::SUCCESS)
    if (cameraDevice->setFilterLevel(FilterLevel::Full) != CameraStatus::SUCCESS)
    {
        cerr << "Error filter level" << endl;
        return;
    }

    // register a data listener
    //if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return;
    }

    name = "Pico Flexx";
    image_width = _image_width;
    image_height = _image_height;

    serial_number = _serial_number;
    type = SensorType::TOF;

    //vertical
    camera_param.fx = lensParameters.focalLength.second;
    camera_param.fy = lensParameters.focalLength.first;
    camera_param.cx = lensParameters.principalPoint.second;
    camera_param.cy = lensParameters.principalPoint.first;

    cout << "fx:" << camera_param.fx << endl;
    cout << "fy:" << camera_param.fy << endl;
    cout << "cx:" << camera_param.cx << endl;
    cout << "cy:" << camera_param.cy << endl;
}

void
VerticalPicoSensor::update(){

}

void
VerticalPicoSensor::start(std::unique_ptr<ICameraDevice>& cameraDevice){
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return ;
    }
}

void
VerticalPicoSensor::stop(std::unique_ptr<ICameraDevice>& cameraDevice){
    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return;
    }
}

cv::Mat
VerticalPicoSensor::getRGBImage(PicoListener &listener){
    cv::Mat _RGB;
    _RGB = listener.getGrayMat().clone();
    cv::rotate(_RGB, _RGB, cv::ROTATE_90_CLOCKWISE);
    cv::cvtColor(_RGB, _RGB, COLOR_GRAY2RGB);
    return _RGB;
}

cv::Mat
VerticalPicoSensor::getDepthImage(PicoListener &listener){
    cv::Mat depth;
    depth = listener.getZMat().clone();
    cv::rotate(depth, depth, cv::ROTATE_90_CLOCKWISE);
    return depth;
}

cv::Mat
VerticalPicoSensor::getColorizedDepthImage(PicoListener &listener){
    cv::Mat _depth, _depth_color ;
    _depth = listener.getZMat().clone();

    for(int y=0;y< _depth.rows;y++){
      for(int x=0;x< _depth.cols;x++){
        if(_depth.at<float>(y,x)>2.00)
            _depth.at<float>(y,x) = 0;
      }
    }

    _depth.convertTo(_depth_color, CV_8UC1, 255.0 / 2.0 );

    cv::Mat depth_color = cv::Mat::zeros( image_height,
                                image_width,
                                CV_8UC3);


    for(int y=0;y< depth_color.rows;y++){
        for(int x=0;x< depth_color.cols;x++){
            if(_depth_color.at<uchar>(y,x) == 0){
                depth_color.at<Vec3b>(y,x)[0] = _depth_color.at<uchar>(y,x);
                depth_color.at<Vec3b>(y,x)[1] = 0;
                depth_color.at<Vec3b>(y,x)[2] = 0;
            }else{
                depth_color.at<Vec3b>(y,x)[0] = _depth_color.at<uchar>(y,x);
                depth_color.at<Vec3b>(y,x)[1] = 255;
                depth_color.at<Vec3b>(y,x)[2] = 255;
            }
        }
    }

    cvtColor(depth_color, depth_color, COLOR_HSV2BGR);
    cv::Mat depth_color_out;
    depth_color_out = depth_color.clone();
    cv::rotate(depth_color_out, depth_color_out, cv::ROTATE_90_CLOCKWISE);

    return depth_color_out;
}

cv::Mat
VerticalPicoSensor::getIRImage(PicoListener &listener){
    cv::Mat gray;
    gray = listener.getGrayMat().clone();
    cv::rotate(gray, gray, cv::ROTATE_90_CLOCKWISE);
    return gray;
}

CameraParameter
VerticalPicoSensor::getCameraParameter(){
    return camera_param;
}

