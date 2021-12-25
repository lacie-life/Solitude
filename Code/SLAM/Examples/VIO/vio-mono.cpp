
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include "common/IMUdata.h"
#include "common/Configparam.h"
#include "system/System.h"

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

void loadImageList(char *imagePath, std::vector<ICell> &iListData)
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
        //string temp1 = line.substr(0,comma).substr(0,10);
        //temp.timeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());
        //cout <<line.substr(0,comma).c_str()<<endl;
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

void loadIMUFile(char *imuPath, std::vector<kms_slam::IMUData> &vimuData)
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

        //imuTimeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());

        // cout << line.substr(0,comma).c_str() << ' ';
        // memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
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
            // cout << line.substr(comma + 1,comma2-comma-1).c_str() << ' ';
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

//    if (argc != 7)
//    {
//        cerr << endl << "Not enough param" << endl;
//        cerr << endl << "Usage: ./project path_to_ORBVOC.TXT path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data  strName" << endl;
//        return 1;
//    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // kms_slam::System SLAM(argv[1], argv[2], kms_slam::System::MONOCULAR, true);

    kms_slam::System SLAM("/home/jun/Github/Master-Thesis/Code/SLAM/Vocabulary/ORBvoc.txt",
                          "/home/jun/Github/Master-Thesis/Code/SLAM/Examples/VIO/EuRoC.yaml",
                          kms_slam::System::MONOCULAR,
                          true);

    kms_slam::ConfigParam config("/home/jun/Github/Master-Thesis/Code/SLAM/Examples/VIO/EuRoC.yaml");

    /**
     * @brief added data sync
     */
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    cout<<"-----------------------------------------------------------------------------"<<endl;
    char *fullPath = new char[500];// = {0};
    memset(fullPath, 0, 500);

    std::vector<kms_slam::IMUData> allimuData;
    std::vector<ICell> iListData;

    loadIMUFile("/home/jun/Github/Data/MH_01_easy/mav0/imu0/data.csv", allimuData);

    loadImageList("/home/jun/Github/Data/MH_01_easy/mav0/cam0/data.csv", iListData);

    //double e = pow(10.0,-9);

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



    // cout << iListData.size() << "------------" << allimuData.size() << endl;
    // cv::waitKey(0);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(iListData.size());

    int nImages = iListData.size();
    if (nImages < 1)
    {
        cerr << endl << "There is no enough images." << endl;
    }

    for (int j = 0; j < iListData.size() - 1; j++)
    {
        std::vector<kms_slam::IMUData> vimuData;

        for (unsigned int i = 0; i < 10; i++)
        {
            // cout << "*************************************************************************" << endl;
            // char temp[10] = {0};

            // substring(temp,imuTime,0,10);
            // cout << "==========================================================================" << endl;
            if (bAccMultiply98)
            {
                allimuData[10 * j + i]._a(0) *= g3dm;
                allimuData[10 * j + i]._a(1) *= g3dm;
                allimuData[10 * j + i]._a(2) *= g3dm;
            }


            //kms_slam::IMUData imudata(allimuData[j]._g(0),allimuData[j]._g(1),allimuData[j]._g(2),
            //                        allimuData[j]._a(0),allimuData[j]._a(1),allimuData[j]._a(2),(double)allimuData[j]._t);

            //kms_slam::IMUData imudata(allimuData[j]._g(0),allimuData[j]._g(1),allimuData[j]._g(2),
            //              allimuData[j]._a(0),allimuData[j]._a(1),allimuData[j]._a(2),j*0.0005+i*0.00005);//j*0.0005+i*0.00005

            //kms_slam::IMUData imudata(allimuData[10*j+i]._g(0),allimuData[10*j+i]._g(1),allimuData[10*j+i]._g(2),
            //       allimuData[10*j+i]._a(0),allimuData[10*j+i]._a(1),allimuData[10*j+i]._a(2),j*0.05+i*0.005);//j*0.0005+i*0.00005

            kms_slam::IMUData imudata(allimuData[10 * j + i]._g(0), allimuData[10 * j + i]._g(1), allimuData[10 * j + i]._g(2),
                                       allimuData[10 * j + i]._a(0), allimuData[10 * j + i]._a(1), allimuData[10 * j + i]._a(2), allimuData[10 * j + i]._t); //j*0.0005+i*0.00005
            vimuData.push_back(imudata);
        }

        // cout << "IMU FINISHED READING" << endl;

        string temp = iListData[j + 1].imgName.substr(0, iListData[j].imgName.size() - 1);

        sprintf(fullPath, "%s/%s", "/home/jun/Github/Data/MH_01_easy/mav0/cam0/data.csv", temp.c_str());
        cv::Mat im = cv::imread(fullPath, 0);

        // cout << fullPath << endl;

        memset(fullPath, 0, 100);

        //  cout << "-----------------------KMS----------------------" << iListData[j].timeStamp << endl;

        static double startT = -1;
        if (startT < 0)
            startT = iListData[j + 1].timeStamp;
        if (iListData[j + 1].timeStamp < startT + config._testDiscardTime)
            im = cv::Mat::zeros(im.rows, im.cols, im.type());

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // SLAM.TrackMonoVI(im, vimuData, j*0.00005);
        // FIX:if imageMsgDelaySec>0,it will be wrong

        // cout << std::setprecision(13) << "img time: " << iListData[j+1].timeStamp << " fist vimu begin() time: " << (*vimuData.begin())._t << endl;


        // SLAM.TrackMonoVI(im, vimuData, j*0.05- imageMsgDelaySec);

        // cout<< std::setprecision(13) <<"Now is Tracking Img at time: "<< iListData[j + 1].timeStamp<<endl;
        SLAM.TrackMonoVI(im, vimuData, iListData[j + 1].timeStamp - imageMsgDelaySec);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[j] = ttrack;



        //if(j == 6)
        //{
        //  usleep(20);
        //cv::waitKey(0);
        //}
        // Wait local mapping end.
        bool bstop = false;

        while (!SLAM.bLocalMapAcceptKF())
        {
            bstop = true;
        };
        //if(bstop)
        //  break;
    }
    delete [] fullPath;
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath +"vio-mono"+ "_norosMonoVio.txt");
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