/********************************************************************************* 
  @Copyright(C),
  @FileName:  getCameraConfig.cpp
  @Author:  HoveXb
  @Version:  
  @Date:  2022.01.05
  @Description:  get camera configration
  @Others:  //其他内容说明 
**********************************************************************************/  
#include <sl/Camera.hpp>
using namespace sl;
using namespace std;
// #include<fstream>
int main(int argc, char **argv) {

    // string cameraCongfigOut = "./VGA_100HZ_CameraConfig";
    InitParameters init_params;
    /**     enum class UNIT {
         MILLIMETER, International System, 1/1000 METER. 
         CENTIMETER, International System, 1/100 METER. 
         METER, International System, 1 METER 
         INCH,  Imperial Unit, 1/12 FOOT 
         FOOT,  Imperial Unit, 1 FOOT 
        ///@cond SHOWHIDDEN 
        LAST
         ///@endcond
     */
    init_params.coordinate_units = UNIT::METER;
    // HD2K, /**< 2208*1242 (x2), available framerates: 15 fps.*/
    // HD1080, /**< 1920*1080 (x2), available framerates: 15, 30 fps.*/
    // HD720, /**< 1280*720 (x2), available framerates: 15, 30, 60 fps.*/
    // VGA, /**< 672*376 (x2), available framerates: 15, 30, 60, 100 fps.*/
    init_params.camera_resolution=RESOLUTION::HD720;
    init_params.camera_fps=15;
    // enum class COORDINATE_SYSTEM {
    // IMAGE, /**< Standard coordinates system in computer vision. Used in OpenCV : see here : http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html */
    // LEFT_HANDED_Y_UP, /**< Left-Handed with Y up and Z forward. Used in Unity with DirectX. */
    // RIGHT_HANDED_Y_UP, /**< Right-Handed with Y pointing up and Z backward. Used in OpenGL. */
    // RIGHT_HANDED_Z_UP, /**< Right-Handed with Z pointing up and Y forward. Used in 3DSMax. */
    // LEFT_HANDED_Z_UP, /**< Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine. */
    // RIGHT_HANDED_Z_UP_X_FWD, /**< Right-Handed with Z pointing up and X forward. Used in ROS (REP 103). */
    // ///@cond SHOWHIDDEN 
    // LAST
    // ///@endcond
    // Create a ZED camera object  
    init_params.coordinate_system=COORDINATE_SYSTEM::IMAGE;
    init_params.depth_mode = DEPTH_MODE::NONE;
    init_params.save("./zed_HD720_15HZ_Setting");
    Camera zed;

    // Open the camera
    ERROR_CODE returned_state = zed.open(init_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // Get camera information (ZED serial number)
    auto camera_infos = zed.getCameraInformation();
    //A 输出IMU中的相关参数
    //1.陀螺仪相关参数
    //  struct SensorParameters {
    //     sl::SENSOR_TYPE type; /**< The type of the sensor as \ref DEVICE_SENSORS
    //     float resolution; /**< The resolution of the sensor. */
    //     float sampling_rate; /**< The sampling rate (or ODR) of the sensor. 
    //     sl::float2 range; /**< The range values of the sensor. MIN: `range.x`, MAX: `range.y` */
    //     float noise_density; /**< also known as white noise, given as continous (frequency independant). Units will be expressed in sensor_unit/√(Hz). `NAN` if the information is not available */
    //     float random_walk; /**< derived from the Allan Variance, given as continous (frequency independant). Units will be expressed in sensor_unit/s/√(Hz).`NAN` if the information is not available */
    //     sl::SENSORS_UNIT sensor_unit; /**< The string relative to the measurement unit of the sensor. */
    //     bool isAvailable;
    // ofstream imu_timestamp_txt(cameraCongfigOut);
    printf("gyroscope random_walk: %f deg/s^1.5 \n", camera_infos.sensors_configuration.gyroscope_parameters.random_walk);
    printf("gyroscope noise_density: %f m/s^0.5 \n", camera_infos.sensors_configuration.gyroscope_parameters.noise_density);
    printf("gyroscope sampling_rate: %f HZ\n", camera_infos.sensors_configuration.gyroscope_parameters.sampling_rate);
    //2.角速度记相关参数
    printf("accelerometer random_walk: %f m/s^2.5 \n", camera_infos.sensors_configuration.accelerometer_parameters.random_walk);
    printf("accelerometer noise_density: %f m/s^1.5 \n", camera_infos.sensors_configuration.accelerometer_parameters.noise_density);
    printf("accelerometer sampling_rate: %f HZ\n", camera_infos.sensors_configuration.accelerometer_parameters.sampling_rate);
    
    //B 坐标转换关系
    // IMU 至左相机的转换矩阵
    Matrix4f T_CI=camera_infos.sensors_configuration.camera_imu_transform;
    cout<<endl<<"IMU to Left camera transform matrix:"<<endl;
    cout<<T_CI(0,0)<<"\t"<<T_CI(0,1)<<"\t"<<T_CI(0,2)<<"\t"<<T_CI(0,3)<<endl;
    cout<<T_CI(1,0)<<"\t"<<T_CI(1,1)<<"\t"<<T_CI(1,2)<<"\t"<<T_CI(1,3)<<endl;
    cout<<T_CI(2,0)<<"\t"<<T_CI(2,1)<<"\t"<<T_CI(2,2)<<"\t"<<T_CI(2,3)<<endl;
    cout<<T_CI(3,0)<<"\t"<<T_CI(3,1)<<"\t"<<T_CI(3,2)<<"\t"<<T_CI(3,3)<<endl;
    T_CI.inverse();
    // 左相机至IMU的转换矩阵
    cout<<endl<<"Left camera to IMU transform matrix:"<<endl;
    cout<<T_CI(0,0)<<"\t"<<T_CI(0,1)<<"\t"<<T_CI(0,2)<<"\t"<<T_CI(0,3)<<endl;
    cout<<T_CI(1,0)<<"\t"<<T_CI(1,1)<<"\t"<<T_CI(1,2)<<"\t"<<T_CI(1,3)<<endl;
    cout<<T_CI(2,0)<<"\t"<<T_CI(2,1)<<"\t"<<T_CI(2,2)<<"\t"<<T_CI(2,3)<<endl;
    cout<<T_CI(3,0)<<"\t"<<T_CI(3,1)<<"\t"<<T_CI(3,2)<<"\t"<<T_CI(3,3)<<endl;

    //相机外参
    cout<<"CameraBaseline(m):"<<camera_infos.calibration_parameters.getCameraBaseline()<<endl;
    Matrix4f T_RL=camera_infos.calibration_parameters.stereo_transform;
    cout<<"Left to Right camera transform:\n";
    cout<<T_RL(0,0)<<"\t"<<T_RL(0,1)<<"\t"<<T_RL(0,2)<<"\t"<<T_RL(0,3)<<endl;
    cout<<T_RL(1,0)<<"\t"<<T_RL(1,1)<<"\t"<<T_RL(1,2)<<"\t"<<T_RL(1,3)<<endl;
    cout<<T_RL(2,0)<<"\t"<<T_RL(2,1)<<"\t"<<T_RL(2,2)<<"\t"<<T_RL(2,3)<<endl;
    cout<<T_RL(3,0)<<"\t"<<T_RL(3,1)<<"\t"<<T_RL(3,2)<<"\t"<<T_RL(3,3)<<endl;
    
    cout<<"left_cam.cx:"<<camera_infos.calibration_parameters.left_cam.cx<<endl;
    cout<<"left_cam.cy:"<<camera_infos.calibration_parameters.left_cam.cy<<endl;
    cout<<"left_cam.fx:"<<camera_infos.calibration_parameters.left_cam.fx<<endl;
    cout<<"left_cam.fy:"<<camera_infos.calibration_parameters.left_cam.fy<<endl;
    // cout<<"right_cam.cx:"<<camera_infos.calibration_parameters.right_cam.cx<<endl;
    // cout<<"right_cam.cy:"<<camera_infos.calibration_parameters.right_cam.cy<<endl;
    // cout<<"right_cam.fx:"<<camera_infos.calibration_parameters.right_cam.fx<<endl;
    // cout<<"right_cam.fy:"<<camera_infos.calibration_parameters.right_cam.fy<<endl;
    cout<<"right_cam.width:"<<camera_infos.calibration_parameters.right_cam.image_size.width<<endl;
    cout<<"right_cam.height:"<<camera_infos.calibration_parameters.right_cam.image_size.height<<endl;

    // SensorsData sensors_data;
    // SensorsData::IMUData imu_data;
    // if(zed.grab() == ERROR_CODE::SUCCESS){
    // zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE); // Retrieve only frame synchronized data
    
    // // Extract IMU data
    // imu_data = sensors_data.imu;
    // cout<<imu_data.timestamp.data_ns<<endl;
    // cout<<zed.getTimestamp(TIME_REFERENCE::IMAGE).data_ns;

    // Retrieve linear acceleration and angular velocity
    // sl::float3 linear_acceleration = imu_data.linear_acceleration;
    // cout<<"linear_acceleration:"<<linear_acceleration<<endl;
    // }


    // float array_zedimu2zedcam[16]={0.999992,0.000342525,-0.0040221,-0.0230078,
    //                             -0.000335738,0.999999,0.00168798,0.000231098,
    //                             0.00402267,-0.00168662,0.999991,0.00190709,
    //                             0.0,0.0,0.0,1.0};
    // sl::Matrix4f trans_zedimu2zedcam(array_zedimu2zedcam);
    // float array_zedcam2OrbImu[16]={0.0,-1.0,0.0,0.0,
    //                             1.0, 0.0,0.0,0.0,
    //                             0.0, 0.0,1.0,0.0,
    //                             0.0, 0.0,0.0,1.0};
    // sl::Matrix4f trans_zedcam2OrbImu(array_zedcam2OrbImu);
    // sl::Matrix4f trans_zedimu2orbimu = trans_zedcam2OrbImu*trans_zedimu2zedcam;
    // sl::Matrix4f transform = trans_zedcam2OrbImu*trans_zedimu2zedcam;
    // cout<<endl<<"Left camera to IMU transform matrix in ORB-SLAM3 should be:"<<endl;
    // cout<<transform(0,0)<<"\t"<<transform(0,1)<<"\t"<<transform(0,2)<<"\t"<<transform(0,3)<<endl;
    // cout<<transform(1,0)<<"\t"<<transform(1,1)<<"\t"<<transform(1,2)<<"\t"<<transform(1,3)<<endl;
    // cout<<transform(2,0)<<"\t"<<transform(2,1)<<"\t"<<transform(2,2)<<"\t"<<transform(2,3)<<endl;
    // cout<<transform(3,0)<<"\t"<<transform(3,1)<<"\t"<<transform(3,2)<<"\t"<<transform(3,3)<<endl;

    zed.close();
    return EXIT_SUCCESS;
}