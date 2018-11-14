#include "3DRecon.h"
#include "colorizer.h"

using namespace std;


//have a mesh class (PCL) (points + triangles)
//be able to update mesh without slam, just (image, location) (update())
//delete()
//save()
//load() (can take previously created meshes and update them)
//simplify() (from colorizer)




3DRecon::3DRecon(string yaml_file){
    pointCloudGenerator = new ark::PointCloudGenerator(yaml_file);
}

void 3DRecon:AddSLAMSystem(ark::ORBSLAMSystem slam){
    slam->AddKeyFrameAvailableHandler([pointCloudGenerator](const ark::RGBDFrame &keyFrame) {
            return pointCloudGenerator->OnKeyFrameAvailable(keyFrame);
        }, "PointCloudFusion");
}

void 3DRecon::Start(){
    pointCloudGenerator->Start();
}

void 3DRecon::UpdateMesh(cv::Mat &imRGB,cv::Mat &imD, cv::Mat &Twc){
    pointCloudGenerator->
    pointCloudGenerator->GetPly(ss);
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
    delete stringstreamply;
    delete pointCloudGenerator;
    delete slam;
    delete colorizer;
    delete tframe;
}