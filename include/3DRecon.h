//API

#ifndef 3D_RECON_H
#define 3D_RECON_H

#include <iostream>
#include <algorithm>
#include <thread>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <ORBSLAMSystem.h>
#include <PointCloudGenerator.h>
#include <Colorizer.h>


namespace ark{

    class 3DRecon{
    public:
        3DRecon(string yaml_file);

        void Start();

        bool IsRunning();

        void Update(cv::Mat imRGB, cv::Mat imD);

        stringstream GetCurrentTSDFMesh();

        void Stop(bool saveToFile, string filePath);

        void Colorize(int resolution, string textureImageName, string finalOutputName);

        void Simplify();

    private:
        ark::PointCloudGenerator *pointCloudGenerator;
        ark::ORBSLAMSystem *slam;
        ark::Colorizer *colorizer;
        int tframe;
        stringstream ss;
    }
}



#endif