#include <sl/Camera.hpp>
using namespace sl;
using namespace std;

int main(int argc, char const *argv)
{
    InitParameters init_params;
    init_params.coordinate_units = UNIT::METER;
    init_params.camera_resolution=RESOLUTION::HD720;
    init_params.camera_fps=30;
    init_params.coordinate_system=COORDINATE_SYSTEM::IMAGE;
    init_params.depth_mode = DEPTH_MODE::NONE;
    // init_params.save("./zed_HD720_params");
    Camera zed;

    ERROR_CODE returned_state = zed.open(init_params);
    if (returned_state != ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    SensorsData sensors_data;
    SensorsData::IMUData imu_data;
    for(int i=0;i<100;i++)
    {
    zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
    imu_data = sensors_data.imu;
    cout<<imu_data.linear_acceleration<<endl;
    sleep(1);
    }
    zed.close();
    /* code */
    return 0;
}
