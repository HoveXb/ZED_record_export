/********************************************************************************* 
  *Copyright(C),Your Company 
  *FileName:  recordZed 
  *Author:  HoveXb
  *Version:  
  *Date:  2022.01.05
  *Description:  Recored Zed2i's data in Euroc format for ORB-SLAM3 
  *usage:./recordZed ./out.svo
  *Others:  //其他内容说明 

**********************************************************************************/  
// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "utils.hpp"

#include<fstream>
#include<vector>
#include <iomanip>
#include<thread>
#include <time.h>

// Using namespace
using namespace sl;
using namespace std;


void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void ImuDataGrab(Camera &zed,string stCurrentTime);


int main(int argc, char **argv) {

    if (argc < 3) {
        cout << "Usage : ./recordZed ./zedSetting.yml ./compus202201.svo\n";
        return EXIT_FAILURE;
    }

    time_t currentTime=time(NULL);
	char chCurrentTime[256];
	strftime(chCurrentTime,sizeof(chCurrentTime),"%Y%m%d%H%M%S",localtime(&currentTime));
    string stCurrentTime=chCurrentTime;       
    // Create a ZED camera
    Camera zed;

    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.load(argv[1]);

    // Open the camera
    auto returned_state  = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // string svo_savepath = "svo_"+stCurrentTime+".svo";
    // char svo_csavepath[]  = "svo_";
    // strncpy(svo_csavepath, svo_savepath.c_str(), svo_savepath.length() + 1); 
    // String svo_Savepath=svo_csavepath;
    returned_state = zed.enableRecording(RecordingParameters(argv[2], SVO_COMPRESSION_MODE::LOSSLESS));
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Recording ZED : ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    // Start recording SVO, stop with Ctrl-C command
    print("SVO is Recording, use Ctrl-C to stop." );
    sl::RecordingStatus rec_status;
    uint64_t t_curr;

    string ImgTimeStampSavePath = "ImgTimeStamp"+stCurrentTime+".txt";
    ofstream CameraTimestamp(ImgTimeStampSavePath);
    SetCtrlHandler();
    thread thread_imu(ImuDataGrab,ref(zed),stCurrentTime);
    usleep(10*1000);//主线程睡眠10ms,保证IMU数据先记录
    while (!exit_app) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            t_curr = zed.getTimestamp(TIME_REFERENCE::IMAGE).data_ns;
            rec_status = zed.getRecordingStatus();
            if (rec_status.status)
            {
                CameraTimestamp<<t_curr<<endl;
            }
        }
    }
    thread_imu.join();
    // Stop recording
    zed.disableRecording();
    zed.close();
    CameraTimestamp.close();

    return EXIT_SUCCESS;
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout <<"[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout<<" ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

void ImuDataGrab(Camera &zed,string stCurrentTime)
{
    string filename="IMU_Data"+stCurrentTime+".txt";

    ofstream IMU_Data(filename);
    uint64_t imu_t_curr;
    sl::float3 imu_linear_acceleration;
    sl::float3 imu_angular_velocity;
    SensorsData sensors_data;
    SensorsData::IMUData imu_data;
    uint64_t last_imu_ts=0;

    while(!exit_app){
    zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    imu_data=sensors_data.imu;
    imu_t_curr=imu_data.timestamp.data_ns;
    // Check if a new IMU sample is available
    if (imu_t_curr > last_imu_ts) {
        last_imu_ts = imu_t_curr;
        imu_linear_acceleration = imu_data.linear_acceleration;
        imu_angular_velocity=imu_data.angular_velocity*M_PI/180;
        IMU_Data<<imu_t_curr<<","<<imu_angular_velocity[0]<<","
        <<imu_angular_velocity[1]<<","<<imu_angular_velocity[2]<<","
        <<imu_linear_acceleration[0]<<","<<imu_linear_acceleration[1]<<","
        <<imu_linear_acceleration[2]<<endl;
    }
    usleep(500);
    }
    // 再记录几组IMU数据，确保IMU数据时间段大于图像数据时间段
    for(int n =0;n<2000;n++)
    {
    zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    imu_data=sensors_data.imu;
    imu_t_curr=imu_data.timestamp.data_ns;
    // Check if a new IMU sample is available
    if (imu_t_curr > last_imu_ts) {
        last_imu_ts = imu_t_curr;
        imu_linear_acceleration = imu_data.linear_acceleration;
        imu_angular_velocity=imu_data.angular_velocity*M_PI/180;
        IMU_Data<<imu_t_curr<<","<<imu_angular_velocity[0]<<","
        <<imu_angular_velocity[1]<<","<<imu_angular_velocity[2]<<","
        <<imu_linear_acceleration[0]<<","<<imu_linear_acceleration[1]<<","
        <<imu_linear_acceleration[2]<<endl;
    }
    usleep(500);
    }
    IMU_Data.close();
}