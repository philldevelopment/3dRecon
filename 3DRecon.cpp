#include "3DRecon.h"
#include "colorizer.h"

using namespace std;

3DRecon::3DRecon(string yaml_file){
    pointCloudGenerator = new ark::PointCloudGenerator(yaml_file);
    slam = new ark::ORBSLAMSystem("/ORBSLAM src/Vocabulary/ORBvoc.txt", yaml_file, ark::ORBSLAMSystem::RGBD, true);

    slam->AddKeyFrameAvailableHandler([pointCloudGenerator](const ark::RGBDFrame &keyFrame) {
        return pointCloudGenerator->OnKeyFrameAvailable(keyFrame);
    }, "PointCloudFusion");
}

void 3DRecon::Start(){
    slam->Start();
    pointCloudGenerator->Start();
    tframe = 1;
}

bool 3DRecon::IsRunning(){
    return slam->isRunning;
}

void 3DRecon::Update(cv::Mat imRGB,cv::Mat imD){
    slam->PushFrame(imRGB, imD, tframe);
    pointCloudGenerator->GetPly(ss);
}

stringstream 3DRecon::GetCurrentTSDFMesh(){
    return ss;
}


void 3DRecon::Stop(bool saveToFile, string filePath = "model.ply"){
    slam->RequestStop();
    pointCloudGenerator->RequestStop();
    if (saveToFile)
        pointCloudGenerator->SavePly(filePath);
}

void 3DRecon::Colorize(int resolution = 1024, string textureImageName = "model_tex.png", string finalOutputName = "model_colorized.ply"){
    colorizer->Colorize(ss, resolution, textureImageName, finalOutputName);
}

void 3DRecon::Simplify(){
    colorizer->decimateAndProject(ss);
}

void 3DRecon::~3DRecon(){
    delete ss;
    delete pointCloudGenerator;
    delete slam;
    delete colorizer;
    delete tframe;
}