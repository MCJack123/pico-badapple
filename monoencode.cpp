#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

/*
Structure of the file:
4 bytes - MVIE
2 bytes - width
2 bytes - height
2 bytes - frame rate
2 bytes - frame count

RLE data:
A frame consists of a 2-byte length, followed by a series of 1 or 2-byte words indicating the length of a span of black/white pixels.
If the word is 1 byte, the first byte's high bit is cleared.
If the word is 2 bytes, the first byte's high bit is set, and the next byte has bits 7-14.
Pseudo-C to get a word:
{
    uint16_t c = get();
    if (c & 0x80) c = (c & 0x7F) | (get() << 7);
}
Each frame begins on black, and alternates on each word.
A span of pixels can overflow to the next row(s).
The end of a frame is reached once width*height pixels are decoded.
If a word is 32767, the next word will have the same color.
*/

using namespace std;
using namespace std::chrono;
using namespace cv;

int main(int argc, const char * argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input.mp4> <output.mvd>\n";
        return 1;
    }

    VideoCapture cap(argv[1]);
    ofstream out(argv[2], std::ios::binary);
    string status = "Wrote 0 frames; 0 fps; 00:00 remaining";
    system_clock::time_point start = system_clock::now(), last_frame = start;
    string last_frame_data;
    uint16_t nframes;

    if (!cap.isOpened()) {
        if (out.is_open()) out.close();
        cerr << "Could not open input file\n";
        return 2;
    }
    if (!out.is_open()) {
        cerr << "Could not open output file";
        return 3;
    }

    out.write("MVIE", 4);
    nframes = cap.get(CAP_PROP_FRAME_WIDTH);
    out.write((char*)&nframes, 2);
    nframes = cap.get(CAP_PROP_FRAME_HEIGHT);
    out.write((char*)&nframes, 2);
    nframes = cap.get(CAP_PROP_FPS);
    out.write((char*)&nframes, 2);
    nframes = cap.get(CAP_PROP_FRAME_COUNT);
    out.write((char*)&nframes, 2);

    for (int j = 0; j < nframes; j++) {
        Mat orig, frame;
        stringstream ss;
        bool isWhite = false;
        uint16_t c = 0;
        cap >> orig;
        if (orig.empty()) break;
        GaussianBlur(orig, frame, Size(0, 0), 5);
        addWeighted(orig, 1.75, frame, -0.75, 0, frame);
        for (Point3_<uint8_t> &p : Mat_<Point3_<uint8_t>>(frame)) {
            bool w = (p.x + p.y + p.z) / 3 >= 128;
            if (w != isWhite) {
                if (c < 128) ss.put((char)c);
                else {
                    ss.put(c | 0x80);
                    ss.put(c >> 7);
                }
                isWhite = w;
                c = 0;
            }
            if (++c == 32767) {
                ss.write("\xFF\xFF", 2);
                c = 0;
            }
        }
        if (c) {
            if (c < 128) ss.put((char)c);
            else {
                ss.put(c | 0x80);
                ss.put(c >> 7);
            }
        }

        std::string data = ss.str();
        uint16_t tmp = data.size();
        out.write((char*)&tmp, 2);
        out.write(data.c_str(), data.size());
        out.flush();

        cout << "\r";
        for (int j = 0; j < status.size(); j++) cout << " ";
        double fps = j / ((double)duration_cast<milliseconds>(system_clock::now() - start).count() / 1000.0);
        auto remaining = (start + milliseconds((long)((1.0 / fps) * nframes * 1000))) - system_clock::now();
        status = "Wrote " + to_string(j) + " frames; " +
            to_string(fps) + " fps; " +
            to_string(duration_cast<minutes>(remaining).count()) + ":" + to_string(duration_cast<seconds>(remaining).count() % 60) + " remaining";
        cout << "\r" << status;
        cout.flush();
        last_frame = system_clock::now();
    }

    cout << "\n";
    out.close();
    return 0;
}