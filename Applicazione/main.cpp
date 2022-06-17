#include <fcntl.h>
#include <sys/poll.h>
#include <atomic>
#include <condition_variable>
#include <thread>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "unistd.h"

using namespace cv;

#define INPUT_FRAME_WIDTH 1920
#define OUTPUT_FRAME_WIDTH (INPUT_FRAME_WIDTH -2)
#define INPUT_FRAME_HEIGHT 1080
#define OUTPUT_FRAME_HEIGHT (INPUT_FRAME_HEIGHT - 2)
#define INPUT_FRAME_SIZE (INPUT_FRAME_WIDTH*INPUT_FRAME_HEIGHT)
#define OUTPUT_FRAME_SIZE (OUTPUT_FRAME_WIDTH*OUTPUT_FRAME_HEIGHT)
#define INPUT_VIDEO_FILE "input2.mp4"
#define OUTPUT_VIDEO_FILE "output.avi"
#define DEVICE_FILE_PATH "/dev/msgdma_test"
#define WRITE_FRAMES 1
#define WRITE_OUTPUT_VIDEO 0
#define WRITE_FRAMES_AS_FILE 0
#define AVAILABLE 0
#define WOULD_BLOCK 1

using namespace std;

std::atomic_bool writerDone = {false};
std::atomic<size_t> totalFramesRemaining = {0};

std::mutex m;
std::condition_variable conditionVariable;

int reader() {
    #if WRITE_OUTPUT_VIDEO
    VideoWriter outputVideo;
    outputVideo.open(OUTPUT_VIDEO_FILE, VideoWriter::fourcc('M','J','P','G'), 30, Size(OUTPUT_FRAME_WIDTH,OUTPUT_FRAME_HEIGHT), false);
    #endif

    int fd = open(DEVICE_FILE_PATH, O_RDONLY);

    size_t read_offset = 0;
    char read_buffer[OUTPUT_FRAME_SIZE];

    size_t frameNumber = 0;
    ssize_t ret;
    Mat frame(OUTPUT_FRAME_HEIGHT, OUTPUT_FRAME_WIDTH, CV_8U);

    while (true) {
        while(totalFramesRemaining == 0) {
            if(writerDone) {
                printf("reader done\n");
                close(fd);
                return 0;
            }

            {
                std::unique_lock<std::mutex> lk(m);
                conditionVariable.wait(lk);
            }
        }
        printf("Starting frame %d\n", frameNumber);

        do {
            ret = read(fd, read_buffer + read_offset, OUTPUT_FRAME_SIZE - read_offset);
            if (ret > 0) {
                read_offset += ret;
                //printf("read offset %zu %lu\n", read_offset, OUTPUT_FRAME_SIZE - read_offset);
            } else {
                printf("read returned %d. errno %d\n", ret, errno);
                break;
            }
        } while (read_offset < OUTPUT_FRAME_SIZE);

        std::memcpy(frame.data, read_buffer, OUTPUT_FRAME_SIZE * sizeof(uint8_t));
        read_offset = 0;

        #if WRITE_OUTPUT_VIDEO
        outputVideo.write(frame);
        #endif

        std::string test = "frames/" + std::to_string(frameNumber) + ".jpg";
        #if WRITE_FRAMES
        cv::imwrite(test, frame);
        #elif WRITE_FRAMES_AS_FILE
        std::ofstream myfile;
                    myfile.open (test);
                    myfile.write(read_buffer, OUTPUT_FRAME_SIZE);
                    myfile.close();
        #endif

        printf("Read frame %d\n", frameNumber);
        frameNumber++;

        totalFramesRemaining--;
    }
}

int writer() {
    VideoCapture cap (INPUT_VIDEO_FILE);
    int fd = open(DEVICE_FILE_PATH, O_WRONLY);

    Mat frame;
    Mat grayscale;
    size_t offset;
    ssize_t ret;
    size_t remaining;
    size_t frameNumber = 0;

    if(!cap.isOpened())  // check if we succeeded
        CV_Error(0, "Can not open Video file");

    cap >> frame; // get the next frame from video
    while (!frame.empty()) {
        cvtColor(frame, grayscale, COLOR_RGB2GRAY);

        {
            std::unique_lock<std::mutex> lk(m);
            totalFramesRemaining += 1;
        }
        conditionVariable.notify_all();

        offset = 0;
        remaining = INPUT_FRAME_SIZE;

        while (remaining > 0) {
            ret = write(fd, &grayscale.data[offset], remaining);

            if (ret > 0) {
                offset += ret;
                remaining -= ret;
            } else {
                printf("write returned %d. errno %d\n", ret, errno);
                exit(1);
            }
        }

        printf("Wrote frame %zu \n", frameNumber);
        frameNumber++;

        cap >> frame; // get the next frame from video
    }

    printf("writer done\n");
    close(fd);
    printf("writer closed\n");

    {
        std::unique_lock<std::mutex> lk(m);
        writerDone = true;
    }
    conditionVariable.notify_all();
}

int acceleratorMultithread() {
    std::thread writerThread(::writer);
    std::thread readerThread(::reader);

    writerThread.join();
    readerThread.join();
    return 0;
}

int accelleratorNonBlocking() {
    VideoCapture cap (INPUT_VIDEO_FILE);

#if WRITE_OUTPUT_VIDEO
    VideoWriter outputVideo;
    outputVideo.open(OUTPUT_VIDEO_FILE, VideoWriter::fourcc('M','J','P','G'), 30, Size(OUTPUT_FRAME_WIDTH,OUTPUT_FRAME_HEIGHT), false);
#endif

    int fd = open(DEVICE_FILE_PATH, O_RDWR);

    int flags_before;
    if((flags_before = fcntl(fd, F_GETFL,0) < 0)) return -1;
    if(fcntl(fd,F_SETFL, flags_before | O_NONBLOCK)<0) return -1;

    Mat frame;
    Mat grayscale;
    size_t offset;
    ssize_t ret;
    size_t remaining;

    size_t read_offset = 0;
    char read_buffer[OUTPUT_FRAME_SIZE];

    //nio stuff
    int readStatus = AVAILABLE;
    int writeStatus = AVAILABLE;
    struct pollfd pfd = {};
    Mat resultFrame(OUTPUT_FRAME_HEIGHT, OUTPUT_FRAME_WIDTH, CV_8U);

    int frameRead = 0;
    int frameWritten = 0;

#if WRITE_OUTPUT_VIDEO
    if(!outputVideo.isOpened())  // check if we succeeded
        CV_Error(0, "Can not open OUTPUT Video file");
#endif

    if(!cap.isOpened())  // check if we succeeded
        CV_Error(0, "Can not open INPUT Video file");

    cap >> frame; // get the next frame from video
    while (!frame.empty()) {
        cvtColor(frame, grayscale, COLOR_RGB2GRAY);

        totalFramesRemaining += 1;

        offset = 0;
        remaining = INPUT_FRAME_SIZE;
        while (remaining > 0) {
            if(writeStatus == AVAILABLE) {
                ret = write(fd, &grayscale.data[offset], remaining);

                if (ret > 0) {
                    offset += ret;
                    remaining -= ret;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        writeStatus = WOULD_BLOCK;
                    } else {
                        printf("write returned %zd. errno %d\n", ret, errno);
                        exit(1);
                    }
                }
            }

            if(readStatus == AVAILABLE && totalFramesRemaining > 0) {
                ret = read(fd, read_buffer + read_offset, OUTPUT_FRAME_SIZE - read_offset);
                if(ret > 0) {
                    read_offset += ret;
                    if(read_offset == OUTPUT_FRAME_SIZE) {
                        std::memcpy(resultFrame.data, read_buffer, OUTPUT_FRAME_SIZE * sizeof(uint8_t));
                        read_offset = 0;
                        totalFramesRemaining -= 1;

                        #if WRITE_OUTPUT_VIDEO
                        outputVideo << resultFrame;
                        #endif

                        std::string test = "frames/" + std::to_string(frameRead) +".jpg";

                        #if WRITE_FRAMES_AS_FILE
                        std::ofstream myfile;
                        myfile.open (test);
                        myfile.write(read_buffer, OUTPUT_FRAME_SIZE);
                        myfile.close();
                        #elif WRITE_FRAMES
                        cv::imwrite(test, resultFrame);
                        #endif

                        printf("Read frame %d\n", frameRead);
                        frameRead++;
                    }
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        readStatus = WOULD_BLOCK;
                    } else {
                        printf("read returned %d. errno %d\n", ret, errno);
                        exit(1);
                    }
                }
            }

            if(writeStatus == WOULD_BLOCK || readStatus == WOULD_BLOCK) {
                pfd.fd = fd;
                pfd.events = 0;
                pfd.events |= readStatus == WOULD_BLOCK ? POLLIN : 0;
                pfd.events |= writeStatus == WOULD_BLOCK ? POLLOUT : 0;

                printf("Polling %d %d\n", readStatus, writeStatus);
                int r = poll(&pfd, 1, readStatus != 0 && writeStatus != 0 ? 1000 : 0);
                printf("End poll %d\n", r);
                if (r == 0) continue;
                if (r < 0) {
                    printf("poll returned %d\n", r);
                    exit(1);
                }

                if (pfd.revents & POLLIN) {
                    readStatus = AVAILABLE;
                }
                if (pfd.revents & POLLOUT) {
                    writeStatus = AVAILABLE;
                }

                printf("Polling DONE %d %d\n", readStatus, writeStatus);
            }
        }

        printf("Wrote frame %d\n", frameWritten);
        frameWritten++;
        cap >> frame; // get the next frame from video
    }

    //Go back to blocking IO
    if(fcntl(fd,F_SETFL, flags_before)<0) return -1;

    fsync(fd);

    while(totalFramesRemaining > 0) {
        ret = read(fd, read_buffer + read_offset, OUTPUT_FRAME_SIZE - read_offset);
        if(ret > 0) {
            read_offset += ret;
            if(read_offset == OUTPUT_FRAME_SIZE) {
                std::memcpy(resultFrame.data, read_buffer, OUTPUT_FRAME_SIZE * sizeof(uint8_t));
                read_offset = 0;
                totalFramesRemaining -= 1;

                #if WRITE_OUTPUT_VIDEO
                outputVideo << resultFrame;
                #endif

                printf("Read frame %d\n", frameRead);
                frameRead++;
            }
        } else {
            printf("MULTI_THREAD read returned %d. errno %d\n", ret, errno);
            exit(1);
        }
    }

    close(fd);

    return 0;
}

int main(int argc, char *argv[]) {
    int selection = argc > 1 ? atoi(argv[1]) : 0;
    if(selection == 1) {
        return acceleratorMultithread();
    } else {
        return accelleratorNonBlocking();
    }
}