#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <iostream>

Spinnaker::CameraPtr
find_camera_with_model_name(const Spinnaker::SystemPtr system, const char * model_name) {
    Spinnaker::CameraList cameras = system->GetCameras();
    for (unsigned int i=0; i < cameras.GetSize(); ++i) {
        Spinnaker::CameraPtr camera = cameras[i];
        Spinnaker::GenApi::INodeMap& node_map = camera->GetTLDeviceNodeMap();
        const Spinnaker::GenApi::CStringPtr node = node_map.GetNode("DeviceModelName");
        if (node->GetValue() == model_name) {
            return camera;
        }
    }
    return nullptr;
}

int
main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Expected model name and packet size in bytes as 2 positional arguments." << std::endl;
        return -1;
    }
    
    int result = 0;
    const char * model_name = argv[1];
    const int packet_size_bytes = std::stoi(argv[2]);
    std::cout <<  "Setting packet size for " << model_name << " to " << packet_size_bytes << " bytes." << std::endl;

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraPtr camera = find_camera_with_model_name(system, model_name);

    if (camera.IsValid()) {
        std::cout << "Found camera model " << model_name << std::endl;
        camera->Init();
        camera->GevSCPSPacketSize = packet_size_bytes;
        std::cout << "Set packet size to " << camera->GevSCPSPacketSize() << std::endl;
        camera->DeInit();
    } else {
        std::cerr << "Failed to find camera model " << model_name << ". Is it connected?" << std::endl;
        result = -1;
    }

    // Clean up camera before releasing the instance.
    camera = nullptr;
    system->ReleaseInstance();
    return result;
}
