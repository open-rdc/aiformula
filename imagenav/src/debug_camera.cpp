#include "debug_camera/debug_camera.hpp"
#include "utilities/data_utils.hpp"

namespace debug_Camera{
DebugCamera::DebugCamera(const rclcpp::NodeOptions &options) : DebugCamera("", options) {}

DebugCamera::DebugCamera(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("debug_camera_node", name_space, options)
{
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::SVGA;
    init_params.camera_fps = 60;

    // 検出がframeごとに実行
    detection_parameters.image_sync = true;
    // 同一IDを保持
    detection_parameters.enable_tracking = true;
    // オブジェクト上に2Dマスクを作成
    detection_parameters.enable_mask_output = false;

    OpenCamera();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&DebugCamera::DebugObjectDetection, this));
}

DebugCamera::~DebugCamera()
{
    zed.disableObjectDetection();
    zed.close();

    cout << "camera closed" << endl;
}

void DebugCamera::OpenCamera()
{
    err = zed.opend(init_params);
    if(err != ERROR_CODE::SUCCESS)
        exit(-1);
}

void DebugCamera::DebugObjectDetection()
{
    ObjectDetectionRuntimeParameters detection_parameters_runtime;
    detection_parameters_runtime.detection_confidence_threshold = 40;

    Object objects;
    while(zed.grad() == ERROR_CODE::SUCESS)
    {
        err = zed.retrieveObjects(objects, detection_parameters_runtime);

        if(objects.is_new)
        {
            cout << objects.object_list.size() << "Object deteted" << endl;

            first_object = object.objet_list[0];
            cout << "first object position : " << first_object.position;
        }
    }
}

} // namespace

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<debug_camera::DebugCamera>());
  rclcpp::shutdown();
  return 0;
}