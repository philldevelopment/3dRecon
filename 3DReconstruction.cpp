#include "3DRecon.h"
#include "colorizer.h"


//:
//Current data structure to store Mesh: std::vector<Vertex> and std::vector<Faces>
//Initialize create new TSDF instance
//Update(imRBG, imgD, Twc) -> process frame inside TSDF
//GetModel gets the current mesh model
//Save2file() saves the model into .ply file
//Render() renders the model, this is from tsdf, we need colorizor eventually
//LoadFromFile() get data from .ply, update mesh model
//TODO:
//simplify() // decimate
//colorize() //takes in mesh model, create texture map, and colorize


namespace ark{

    3DRecon::3DRecon(){}

    3DRecon::Initialize(std::string strSettingsFile){
        cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
        fx_ = fSettings["Camera.fx"];
        fy_ = fSettings["Camera.fy"];
        cx_ = fSettings["Camera.cx"];
        cy_ = fSettings["Camera.cy"];
        width_ = fSettings["Camera.width"];
        height_ = fSettings["Camera.height"];
        depthfactor_ = fSettings["DepthMapFactor"];
        maxdepth_ = fSettings["MaxDepth"];

        float v_g_o_x = fSettings["Voxel.Origin.x"];
        float v_g_o_y = fSettings["Voxel.Origin.y"];
        float v_g_o_z = fSettings["Voxel.Origin.z"];

        float v_size = fSettings["Voxel.Size"];

        float v_trunc_margin = fSettings["Voxel.TruncMargin"];

        int v_g_d_x = fSettings["Voxel.Dim.x"];
        int v_g_d_y = fSettings["Voxel.Dim.y"];
        int v_g_d_z = fSettings["Voxel.Dim.z"];

        mpGpuTsdfGenerator = new GpuTsdfGenerator(width_,height_,fx_,fy_,cx_,cy_, maxdepth_,
                                                           v_g_o_x,v_g_o_y,v_g_o_z,v_size,
                                                           v_trunc_margin,v_g_d_x,v_g_d_y,v_g_d_z);
    }


    void 3DRecon::Update(const cv::Mat &imRGB, const cv::Mat &imD, const cv::Mat &Twc) {
        float cam2base[16];
        for(int r=0;r<3;++r)
            for(int c=0;c<4;++c)
                cam2base[r*4+c] = Twc.at<float>(r,c);
        cam2base[12] = 0.0f;
        cam2base[13] = 0.0f;
        cam2base[14] = 0.0f;
        cam2base[15] = 1.0f;

        //building global mesh model
        mpGpuTsdfGenerator->processFrame((float *)imD.datastart, (unsigned char *)imRGB.datastart, cam2base);
        cout << "TSDF processed" << endl;
    }

    void 3DRecon::GetModel() {
        mpGpuTsdfGenerator->SaveModel(faces, vertices);
    }

    void 3DRecon::Save2file(std::string filename) { //*.ply
        ofstream plyFile;
        plyFile.open(filename);

        //from tsdf2mesh in tsdf.cu
        plyFile << "ply\nformat ascii 1.0\ncomment stanford bunny\nelement vertex ";
        plyFile << vertices.size() << "\n";
        plyFile
                << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n";
        plyFile << "element face " << faces.size() << "\n";
        plyFile << "property list uchar int vertex_index\nend_header\n";
        for (auto v : vertices) {
            plyFile << v.x << " " << v.y << " " << v.z << " " << (int) v.r << " " << (int) v.g << " " << (int) v.b
                    << "\n";
        }
        for (auto f : faces) {
            plyFile << "3 " << f.vIdx[0] << " " << f.vIdx[1] << " " << f.vIdx[2] << "\n";
        }
        plyFile.close();
        cout << "File saved" << endl;
    }

    //parse ply file using VTK library
    //update mesh model
    void 3DRecon::LoadFromFile(std::string inputFileName) { 
        //not decimated, refer tp colorizer line 96 and line 79
        vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
        reader->SetFileName(inputFileName.c_str());
        reader->Update();
        vtkSmartPointer<vtkPolyData> mesh = reader->GetOutput();

        std::vector<Vertex> verts;
        std::vector<Face> polygons;
        //NOT SURE to use GetNumberOfPoints or GetNumberOfVerts; GetPoint or GetVerts
        verts.reserve(mesh->GetNumberOfPoints());
        polygons.reserve(mesh->GetNumberOfPolys());
        for (int i = 0; i < mesh->GetNumberOfPoints(); ++i) {
            //get XYZRGB points from vertices list in ply
            verts.push_back(Vertex(mesh->GetPoint(i)[0], mesh->GetPoint(i)[1], mesh->GetPoint(i)[2]), mesh->GetPoint(i)[3], mesh->GetPoint(i)[4], mesh->GetPoint(i)[5]);
        }

        //NOT SURE to use GetCell or GetPolys
        for (int j = 0; j < mesh->GetNumberOfPolys(); ++j) {
            //get indices from polygon/triangle list in ply
            Face f(mesh->GetCell(j)->GetPointId(0), mesh->GetCell(j)->GetPointId(1), mesh->GetCell(j)->GetPointId(2));
            polygons.push_back(f);
        }

        //update our model
        myVertices = verts;
        myFaces = polygons;
    }


    void 3DRecon::Render(){
        mpGpuTsdfGenerator->render();
    }

    void 3DRecon::Colorize(int resolution = 1024, std::string textureImageName = "model_tex.png", std::string finalOutputName = "model_colorized.ply"){
        colorizer->Colorize(ss, resolution, textureImageName, finalOutputName);
    }

    void 3DRecon::Simplify(){
        colorizer->decimateAndProject(ss);
    }

    void 3DRecon::~3DRecon(){}
}