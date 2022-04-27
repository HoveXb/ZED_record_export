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
    sl::float3 linear_acceleration;
    sl::float3 angular_velocity;
    float array_zedimu2zedcam[16]={0.999992,-0.000335738,0.00402267,0.023,
                                0.000342525,0.999999,-0.00168662,-0.00022,
                                -0.0040221,0.00168798,0.99999,-0.002,
                                0.0,0.0,0.0,1.0};
    sl::Matrix4f trans_zedimu2zedcam(array_zedimu2zedcam);
    float array_zedcam2OrbImu[16]={0.0,-1.0,0.0,0.0,
                                1.0, 0.0,0.0,0.0,
                                0.0, 0.0,1.0,0.0,
                                0.0, 0.0,0.0,1.0};
    sl::Matrix4f trans_zedcam2OrbImu(array_zedcam2OrbImu);
    sl::Matrix4f trans_zedimu2orbimu = trans_zedcam2OrbImu*trans_zedimu2zedcam;
    SensorsData sensors_data;
    SensorsData::IMUData imu_data;
    sl::Timestamp last_imu_ts=0;
    while(!exit_app){
    zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    imu_data=sensors_data.imu;
    // Check if a new IMU sample is available
    if (sensors_data.imu.timestamp > last_imu_ts) {
        last_imu_ts = sensors_data.imu.timestamp;
        imu_t_curr = imu_data.timestamp.data_ns;
        linear_acceleration = imu_data.linear_acceleration;
        angular_velocity=imu_data.angular_velocity*M_PI/180;
        sl::float4 linear_acceleration_homo(linear_acceleration,1.0);
        sl::float4 angular_velocity_homo(angular_velocity,1.0);
        sl::Matrix4f acceleration_result = trans_zedimu2orbimu*linear_acceleration_homo;
        sl::Matrix4f angular_velocity_result = trans_zedimu2orbimu*angular_velocity_homo;
        IMU_Data<<imu_t_curr<<","<<angular_velocity_result.r00<<","
        <<angular_velocity_result.r10<<","<<angular_velocity_result.r20<<","
        <<acceleration_result.r00<<","<<acceleration_result.r10<<","
        <<acceleration_result.r20<<endl;//归一化？
    }
    usleep(1000);
    }
    // 再记录几组IMU数据，确保IMU数据时间段大于图像数据时间段
    for(int n =0;n<2000;n++)
    {
    zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    imu_data=sensors_data.imu;
    // Check if a new IMU sample is available
    if (sensors_data.imu.timestamp > last_imu_ts) {
        last_imu_ts = sensors_data.imu.timestamp;
        imu_t_curr = imu_data.timestamp.data_ns;
        linear_acceleration = imu_data.linear_acceleration;
        angular_velocity=imu_data.angular_velocity*M_PI/180;
        sl::float4 linear_acceleration_homo(linear_acceleration,1.0);
        sl::float4 angular_velocity_homo(angular_velocity,1.0);
        sl::Matrix4f acceleration_result = trans_zedimu2orbimu*linear_acceleration_homo;
        sl::Matrix4f angular_velocity_result = trans_zedimu2orbimu*angular_velocity_homo;
        IMU_Data<<imu_t_curr<<","<<angular_velocity_result.r00<<","
        <<angular_velocity_result.r10<<","<<angular_velocity_result.r20<<","
        <<acceleration_result.r00<<","<<acceleration_result.r10<<","
        <<acceleration_result.r20<<endl;
    }
    usleep(1000);
    }
    IMU_Data.close();
}