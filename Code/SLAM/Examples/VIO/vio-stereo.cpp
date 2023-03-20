#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "system/System.h"

#include "imu/IMUdata.h"
#include "common/Configparam.h"

#include <boost/foreach.hpp>
#include <fstream>
#include <time.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
using namespace std;

typedef struct ImageList
{
    double timeStamp;
    string imgName;
} ICell;

void loadImageList(char * imagePath, std::vector<ICell> &iListData)
{
    ifstream inf;
    inf.open(imagePath, ifstream::in);
    const int cnt = 2;

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;
    ICell temp;
    getline(inf, line);
    while (!inf.eof())
    {
        getline(inf, line);

        comma = line.find(',', 0);

        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> temp.timeStamp ;
        temp.timeStamp = temp.timeStamp / 1e9;

        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            //i = atof(line.substr(comma + 1,comma2-comma-1).c_str());
            temp.imgName = line.substr(comma + 1, comma2 - comma - 1).c_str();
            ++j;
            comma = comma2;
        }
        iListData.push_back(temp);
        j = 0;
    }

    iListData.pop_back();
    inf.close();

}


void loadIMUFile(char * imuPath, std::vector<kms_slam::IMUData> &vimuData)
{
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    char imuTime[14] = {0};
    double acc[3] = {0.0};
    double grad[3] = {0.0};
    double imuTimeStamp = 0;

    getline(inf, line);
    while (!inf.eof())
    {
        getline(inf, line);

        comma = line.find(',', 0);
        //string temp = line.substr(0,comma);
        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> imuTimeStamp;

        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            switch (j)
            {
                case 0:
                    grad[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 1:
                    grad[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 2:
                    grad[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 3:
                    acc[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 4:
                    acc[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
                case 5:
                    acc[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                    break;
            }

            ++j;
            comma = comma2;
        }
        kms_slam::IMUData tempImu(grad[0], grad[1], grad[2], acc[0], acc[1], acc[2], imuTimeStamp / 1e9);
        vimuData.push_back(tempImu);
        j = 0;
    }

    vimuData.pop_back();

    inf.close();

}

int main(int argc, char **argv)
{

    kms_slam::System SLAM("/home/lacie/Github/Master-Thesis/Code/SLAM/Vocabulary/ORBvoc.txt",
                          "/home/lacie/Github/Master-Thesis/Code/SLAM/Examples/VIO/EuRoC.yaml",
                          kms_slam::System::STEREO,
                          true);

    kms_slam::StereoConfigParam config("/home/lacie/Github/Master-Thesis/Code/SLAM/Examples/VIO/EuRoC.yaml");

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    char *fullPath = new char[500];// = {0};
    char *fullPathR = new char[500];// = {0};
    memset(fullPath, 0, 500);
    memset(fullPath, 0, 500);

    std::vector<kms_slam::IMUData> allimuData;
    std::vector<ICell> iListData;

    loadIMUFile("/home/lacie/Data/MH_01_easy/mav0/imu0/data.csv", allimuData);

    loadImageList("/home/lacie/Data/MH_01_easy/mav0/cam0/data.csv", iListData);


    double ImgFirstTime = iListData[0].timeStamp;
    for (int j = 0; j < allimuData.size() - 1; j++)
    {
        if (ImgFirstTime - allimuData[j]._t < 1 / 1e4)
        {

            allimuData.erase(allimuData.begin(), allimuData.begin() + j);
            break;
        }
    }

    cout << std::setprecision(13) << "first Img time, first Imu timeStamp: " << iListData[0].timeStamp << ",     " << allimuData[0]._t << endl;
    if (iListData[0].timeStamp - allimuData[0]._t > 1 / 1e4)
        cerr << "the timestamp of first Imu is not equal to the first Img!" << endl;


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(iListData.size());

    int nImages = iListData.size();
    if (nImages < 1)
    {
        cerr << endl << "There is no enough images." << endl;
    }

    if (config._K_l.empty() || config._K_r.empty() || config._P_l.empty() || config._P_r.empty() || config._R_l.empty() || config._R_r.empty() || config._D_l.empty() || config._D_r.empty() ||
        config._rows_l == 0 || config._rows_r == 0 || config._cols_l == 0 || config._cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(config._K_l, config._D_l, config._R_l, config._P_l.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_l, config._rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(config._K_r, config._D_r, config._R_r, config._P_r.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_r, config._rows_r), CV_32F, M1r, M2r);

    cv::Mat im, imR, imLeftRect, imRightRect;



    for (int j = 0; j < iListData.size() - 1; j++)
    {
        std::vector<kms_slam::IMUData> vimuData;

        for (unsigned int i = 0; i < 10; i++)
        {
            if (bAccMultiply98)
            {
                allimuData[10 * j + i]._a(0) *= g3dm;
                allimuData[10 * j + i]._a(1) *= g3dm;
                allimuData[10 * j + i]._a(2) *= g3dm;
            }
            kms_slam::IMUData imudata(allimuData[10 * j + i]._g(0), allimuData[10 * j + i]._g(1), allimuData[10 * j + i]._g(2),
                                       allimuData[10 * j + i]._a(0), allimuData[10 * j + i]._a(1), allimuData[10 * j + i]._a(2), allimuData[10 * j + i]._t); //j*0.0005+i*0.00005
            vimuData.push_back(imudata);
        }


        string temp = iListData[j + 1].imgName.substr(0, iListData[j].imgName.size() - 1);

        sprintf(fullPath, "%s/%s", "/home/lacie/Data/MH_01_easy/mav0/cam0/data", temp.c_str());
        sprintf(fullPathR, "%s/%s", "/home/lacie/Data/MH_01_easy/mav0/cam1/data", temp.c_str());
        im = cv::imread(fullPath, 0);
        imR = cv::imread(fullPathR, 0);

        static double startT = -1;
        if (startT < 0)
            startT = iListData[j + 1].timeStamp;
        if (iListData[j + 1].timeStamp < startT + config._testDiscardTime)
        {
            im = cv::Mat::zeros(im.rows, im.cols, im.type());
            imR = cv::Mat::zeros(im.rows, im.cols, im.type());
        }

        if (im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << fullPath << endl;
            return 1;
        }

        if (imR.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << fullPathR << endl;
            return 1;
        }
        memset(fullPath, 0, 500);
        memset(fullPathR, 0, 500);

        cv::remap(im, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imR, imRightRect, M1r, M2r, cv::INTER_LINEAR);


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        SLAM.TrackStereoVI(imLeftRect, imRightRect, vimuData, iListData[j + 1].timeStamp - imageMsgDelaySec);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[j] = ttrack;
        bool bstop = false;

        while (!SLAM.bLocalMapAcceptKF())
        {
            bstop = true;
        };

    }
    delete [] fullPath;
    delete [] fullPathR;
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath +"V1"+ "_StereoVioKeyframe.txt");// from body(IMU) to world.
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(config._tmpFilePath + "V1"+"_StereoVio.txt"); //from cam to world.
    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    cout << endl << endl << "press any key to shutdown" << endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}



