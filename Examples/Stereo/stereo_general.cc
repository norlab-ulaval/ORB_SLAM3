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
#include<set>
#include<cmath>
#include<fstream>
#include<sstream>

#include<dirent.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

// Reads <pathSeq>/images_meta_left/images_meta_left.csv (if present) and returns each
// frame's exposure-bracket sequence_index (0-3), keyed by the same nanosecond
// timestamp used for the frame's filename. Frames with no matching row, or when the
// CSV itself is missing (e.g. a non-bracketed dataset), get phase -1 (unknown).
void LoadExposurePhases(const string &pathSeq, const vector<double> &vTimeStamps,
                        vector<int> &vExposurePhase);

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
    vector< vector<int> > vExposurePhaseCam;
    vector<int> nImages;

    vstrImageLeft.resize(num_seq_real);
    vstrImageRight.resize(num_seq_real);
    vTimestampsCam.resize(num_seq_real);
    vExposurePhaseCam.resize(num_seq_real);
    nImages.resize(num_seq_real);

    int tot_images = 0;
    for (seq = 0; seq<num_seq_real; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[seq + 3]);
        string pathLeft = pathSeq + "/images_left";
        string pathRight = pathSeq + "/images_right";

        LoadImages(pathLeft, pathRight, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        LoadExposurePhases(pathSeq, vTimestampsCam[seq], vExposurePhaseCam[seq]);
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
    // Viewer.on: 0 in the settings yaml disables the Pangolin viewer for headless/batch runs
    // (defaults to on, so existing configs without this key behave as before).
    bool bUseViewer = true;
    {
        cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
        cv::FileNode node = fSettings["Viewer.on"];
        if(!node.empty())
            bUseViewer = (int)node != 0;
    }
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, bUseViewer);

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

            // Diagnostic per-frame marker (ORBSLAM_LOG_FRAMES=1) so tracking log lines
            // interleaved on stdout can be pinned to a raw frame index / dataset timestamp.
            static const bool bLogFrames = (getenv("ORBSLAM_LOG_FRAMES") != nullptr);
            if(bLogFrames)
                cout << "FRAME ni=" << ni << " ts_ns=" << llround(tframe*1e9) << endl;

            // Diagnostic (ORBSLAM_DIAG_SHARPNESS=1): per-frame blur/noise signature,
            // measured on the raw captured image before any ORB-SLAM3 rectification/
            // processing. Placed here rather than in core because the correct exposure
            // phase (vExposurePhaseCam[seq][ni]) is already a plain local value at this
            // exact point -- no ts-join needed, unlike core-side diagnostics where the
            // phase tag isn't set on the Frame until after construction. Low mean
            // intensity + high Laplacian variance = noise-dominated (short/dark
            // exposure); normal mean + low Laplacian variance = blur-dominated
            // (long/bright exposure) -- the two exposure extremes degrade images in
            // different ways, not just "less usable signal" in both cases the same way.
            static const bool bDiagSharpness = (getenv("ORBSLAM_DIAG_SHARPNESS") != nullptr);
            if(bDiagSharpness)
            {
                cv::Mat grayL, grayR, lapL, lapR;
                if(imLeft.channels()==1) grayL = imLeft;
                else cv::cvtColor(imLeft, grayL, cv::COLOR_BGR2GRAY);
                if(imRight.channels()==1) grayR = imRight;
                else cv::cvtColor(imRight, grayR, cv::COLOR_BGR2GRAY);

                cv::Laplacian(grayL, lapL, CV_64F);
                cv::Laplacian(grayR, lapR, CV_64F);
                cv::Scalar meanL, stdL, meanR, stdR, meanIntensityL;
                cv::meanStdDev(lapL, meanL, stdL);
                cv::meanStdDev(lapR, meanR, stdR);
                cv::meanStdDev(grayL, meanIntensityL, cv::Scalar());

                cout << "SHARPNESS ni=" << ni << " phase=" << vExposurePhaseCam[seq][ni]
                     << " lapVarLeft=" << (stdL[0]*stdL[0])
                     << " lapVarRight=" << (stdR[0]*stdR[0])
                     << " meanIntensityLeft=" << meanIntensityL[0]
                     << " ts_ns=" << llround(tframe*1e9) << endl;
            }

            // Experiment (ORBSLAM_SAT_GATE=1, ORBSLAM_SAT_MAX_FRAC=<float, default 0.30>):
            // skip using a frame for tracking entirely when more than this fraction of
            // its pixels are saturated (>=250/255). Quick test of whether severely
            // blown-out frames (e.g. the July 18 tunnel-exit transient, where bright-
            // phase frames reach up to 86% saturated and mid-phase up to 27%, confirmed
            // via direct measurement) are actively hurting more than the partial
            // information in them helps, vs. the confidence gate in ComputeStereoMatches
            // already filtering out the worst individual keypoint matches. Distinct from
            // the earlier, reverted "drop a whole exposure phase" idea: this is a
            // dynamic, per-frame, content-based skip -- an otherwise-fine dark or mid
            // frame is never skipped, only a frame that happens to be this washed out.
            static const bool bSatGateEnabled = (getenv("ORBSLAM_SAT_GATE") != nullptr);
            static const float fSatMaxFrac = getenv("ORBSLAM_SAT_MAX_FRAC") ?
                (float)atof(getenv("ORBSLAM_SAT_MAX_FRAC")) : 0.30f;
            static const bool bDiagSat = (getenv("ORBSLAM_DIAG_SAT") != nullptr);
            bool bSkipFrame = false;
            if(bSatGateEnabled || bDiagSat)
            {
                cv::Mat grayL;
                if(imLeft.channels()==1) grayL = imLeft;
                else cv::cvtColor(imLeft, grayL, cv::COLOR_BGR2GRAY);
                const float satFrac = (float)cv::countNonZero(grayL >= 250) / (grayL.rows*grayL.cols);
                bSkipFrame = bSatGateEnabled && satFrac > fSatMaxFrac;
                if(bDiagSat)
                    cout << "SAT_CHECK ni=" << ni << " phase=" << vExposurePhaseCam[seq][ni]
                         << " satFrac=" << satFrac << " skipped=" << (bSkipFrame ? 1 : 0)
                         << " ts_ns=" << llround(tframe*1e9) << endl;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the images to the SLAM system
            if(!bSkipFrame)
                SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni], vExposurePhaseCam[seq][ni]);

            // Diagnostic frame export (ORBSLAM_DUMP_FRAMES_DIR=<dir>): saves the same
            // tracked-keypoints-overlaid image the Pangolin viewer shows, one PNG per
            // frame, independent of whether the viewer is active -- for offline video
            // export of a headless run. ORBSLAM_DUMP_START_NS/ORBSLAM_DUMP_END_NS
            // (optional) restrict dumping to a raw-timestamp window so a full-dataset
            // run doesn't have to save every frame just to capture one segment.
            static const char *dumpDir = getenv("ORBSLAM_DUMP_FRAMES_DIR");
            if(dumpDir)
            {
                static const char *startEnv = getenv("ORBSLAM_DUMP_START_NS");
                static const char *endEnv = getenv("ORBSLAM_DUMP_END_NS");
                static const long long dumpStartNs = startEnv ? atoll(startEnv) : -1;
                static const long long dumpEndNs = endEnv ? atoll(endEnv) : -1;
                const long long tsNs = llround(tframe*1e9);
                if((dumpStartNs < 0 || tsNs >= dumpStartNs) && (dumpEndNs < 0 || tsNs <= dumpEndNs))
                {
                    static int dumpIdx = 0;
                    cv::Mat drawn = SLAM.GetFrameDrawerImage();
                    char fname[512];
                    snprintf(fname, sizeof(fname), "%s/frame_%06d.png", dumpDir, dumpIdx++);
                    cv::imwrite(fname, drawn);
                }
            }

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

// Max distinct exposure-phase buckets supported; must match Tracking::NUM_EXPOSURE_PHASES
// and MapPoint::NUM_EXPOSURE_PHASES.
static const int MAX_EXPOSURE_PHASES = 4;

void LoadExposurePhases(const string &pathSeq, const vector<double> &vTimeStamps,
                        vector<int> &vExposurePhase)
{
    vExposurePhase.assign(vTimeStamps.size(), -1);

    const string metaPath = pathSeq + "/images_meta_left/images_meta_left.csv";
    ifstream f(metaPath);
    if(!f.is_open())
        return; // not a bracketed dataset (or metadata not present) -- leave phase unknown

    string header;
    getline(f, header);
    vector<string> cols;
    {
        stringstream ss(header);
        string col;
        while(getline(ss, col, ','))
            cols.push_back(col);
    }
    int tsCol = -1, expCol = -1;
    for(size_t i = 0; i < cols.size(); i++)
    {
        if(cols[i] == "timestamp") tsCol = (int)i;
        if(cols[i] == "exposure_factor") expCol = (int)i;
    }
    if(tsCol < 0 || expCol < 0)
    {
        cerr << "WARNING: " << metaPath << " missing 'timestamp'/'exposure_factor' columns "
             << "-- exposure phase will be left unknown." << endl;
        return;
    }

    // Phase is derived from the frame's ACTUAL exposure_factor, not any nominal bracket
    // "slot" label (e.g. a sequence_index/cycle-position column) -- the camera's AE can
    // override the nominal slot's exposure during rapid lighting changes (confirmed: ~10
    // frames dataset-wide have a "dark slot" position actually shot at 1.0x/4.0x), and
    // using the true exposure keeps same-phase matching/fallback self-consistent even then.
    map<unsigned long long, double> exposureByTs;
    string line;
    while(getline(f, line))
    {
        stringstream ss(line);
        string field;
        vector<string> fields;
        while(getline(ss, field, ','))
            fields.push_back(field);
        if((int)fields.size() <= max(tsCol, expCol))
            continue;
        exposureByTs[stoull(fields[tsCol])] = stod(fields[expCol]);
    }

    // Bucket by distinct exposure_factor value (ascending) rather than hardcoding expected
    // values -- naturally merges e.g. two nominal bracket slots that share the same actual
    // exposure_factor (1.0x) into one phase, and adapts to whatever bracket scheme a given
    // dataset actually used.
    set<double> distinctExposures;
    for(const auto &kv : exposureByTs)
        distinctExposures.insert(kv.second);

    map<double,int> phaseByExposure;
    {
        int phase = 0;
        for(double e : distinctExposures)
        {
            if(phase >= MAX_EXPOSURE_PHASES)
            {
                cerr << "WARNING: " << metaPath << " has more than " << MAX_EXPOSURE_PHASES
                     << " distinct exposure_factor values -- extras left as unknown (-1)." << endl;
                break;
            }
            phaseByExposure[e] = phase++;
        }
    }

    size_t nMissing = 0;
    for(size_t i = 0; i < vTimeStamps.size(); i++)
    {
        auto it = exposureByTs.find(llround(vTimeStamps[i] * 1e9));
        if(it != exposureByTs.end() && phaseByExposure.count(it->second))
            vExposurePhase[i] = phaseByExposure[it->second];
        else
            nMissing++;
    }

    if(nMissing > 0)
        cerr << "WARNING: " << nMissing << " of " << vTimeStamps.size()
             << " frame(s) had no exposure-phase metadata; left as unknown (-1)." << endl;
}
