/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

// Stereo runner for datasets laid out as:
//   <sequence_folder>/images_left/<timestamp_ns>.png
//   <sequence_folder>/images_right/<timestamp_ns>.png
// with no separate timestamps file: filenames themselves (minus the ".png")
// are taken as nanosecond timestamps, and left/right frames are paired by
// matching filename. Any left or right frame without an identically-named
// counterpart on the other side is dropped, since it can't be tracked as a
// stereo pair.

#include<iostream>
#include<algorithm>
#include<iomanip>
#include<chrono>
#include<map>

#include<dirent.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./stereo_general path_to_vocabulary path_to_settings path_to_sequence_folder_1 (path_to_sequence_folder_2 ... path_to_sequence_folder_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = argc - 3;
    bool bFileName = false;
    string file_name;
    // If the last positional-looking argument doesn't exist as a directory, treat it as the trajectory file name.
    {
        string lastArg(argv[argc - 1]);
        DIR *d = opendir(lastArg.c_str());
        if(d == nullptr)
        {
            bFileName = true;
            file_name = lastArg;
        }
        else
        {
            closedir(d);
        }
    }
    const int num_seq_real = bFileName ? num_seq - 1 : num_seq;
    if (bFileName)
        cout << "file name: " << file_name << endl;

    cout << "num_seq = " << num_seq_real << endl;

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageLeft.resize(num_seq_real);
    vstrImageRight.resize(num_seq_real);
    vTimestampsCam.resize(num_seq_real);
    nImages.resize(num_seq_real);

    int tot_images = 0;
    for (seq = 0; seq<num_seq_real; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[seq + 3]);
        string pathLeft = pathSeq + "/images_left";
        string pathRight = pathSeq + "/images_right";

        LoadImages(pathLeft, pathRight, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq_real; seq++)
    {
        // Seq loop
        for(int ni=0; ni<nImages[seq]; ni++)
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni]);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(seq < num_seq_real - 1)
        {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + file_name + ".txt";
        const string f_file =  "f_" + file_name + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

// Lists <dir>/*.png, keyed by the numeric value of the filename (without extension).
static void ListPngFilesByTimestamp(const string &dir, map<unsigned long long, string> &out)
{
    DIR *d = opendir(dir.c_str());
    if(d == nullptr)
    {
        cerr << "ERROR: could not open image directory: " << dir << endl;
        exit(1);
    }

    struct dirent *entry;
    while((entry = readdir(d)) != nullptr)
    {
        string name(entry->d_name);
        if(name.size() <= 4 || name.compare(name.size() - 4, 4, ".png") != 0)
            continue;

        string stem = name.substr(0, name.size() - 4);
        try
        {
            size_t pos;
            unsigned long long ts = std::stoull(stem, &pos);
            if(pos != stem.size())
                throw std::invalid_argument(stem);
            out[ts] = dir + "/" + name;
        }
        catch(const std::exception &)
        {
            cerr << "WARNING: skipping file with non-numeric timestamp name: " << dir << "/" << name << endl;
        }
    }
    closedir(d);
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    map<unsigned long long, string> left, right;
    ListPngFilesByTimestamp(strPathLeft, left);
    ListPngFilesByTimestamp(strPathRight, right);

    vstrImageLeft.reserve(left.size());
    vstrImageRight.reserve(left.size());
    vTimeStamps.reserve(left.size());

    size_t skippedLeft = 0, skippedRight = 0;
    for(const auto &kv : left)
    {
        auto it = right.find(kv.first);
        if(it == right.end())
        {
            skippedLeft++;
            continue;
        }
        vstrImageLeft.push_back(kv.second);
        vstrImageRight.push_back(it->second);
        vTimeStamps.push_back(static_cast<double>(kv.first) / 1e9);
    }
    for(const auto &kv : right)
    {
        if(left.find(kv.first) == left.end())
            skippedRight++;
    }

    cout << endl << vstrImageLeft.size() << " image(s) found with a matching timestamp in both cameras." << endl;

    if(skippedLeft > 0 || skippedRight > 0)
    {
        cerr << "WARNING: " << skippedLeft << " left / " << skippedRight
             << " right frame(s) had no matching timestamp on the other side and were skipped." << endl;
    }

    if(vstrImageLeft.empty())
    {
        cerr << "ERROR: no matching left/right stereo pairs found in " << strPathLeft
             << " and " << strPathRight << endl;
        exit(1);
    }
}
