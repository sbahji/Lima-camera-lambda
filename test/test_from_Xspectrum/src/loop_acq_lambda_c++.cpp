/*
 * loop_acq_lambda_c++.cpp - run scan according to Python script
 *
 * Author: A. Beckmann
 * Date  : April 2022
 *
 */

#include <libxsp.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <csignal>
#include <cstdlib>
#include <random>
#include <chrono>


using namespace xsp;
using namespace std;

int main(int argc, char** argv)
{
    auto n_frames = 1;
    auto shutter_time = 1000.0; // [ms]
    vector<double> thresholds{6.0};
    auto timeout = 4; // [s]
    auto count = 0;

    xsp::setLogHandler([](xsp::LogLevel l, const string& m) {
        switch (l) {
            case xsp::LogLevel::ERROR:
            case xsp::LogLevel::WARN:
            case xsp::LogLevel::INFO:
                cout << l << ": " << m << endl;
                break;
            default:
                // uncomment this, if you want to have DEBUG output
                //cout << l << ": " << m << endl;
                break;
        }
        });

    auto s = xsp::createSystem("/opt/xsp/config/system_Si.yml");
    if (s == nullptr) {
        cout << "ERROR: failed to create system" << endl;
        return 1;
    }

    try {
        s->connect();
        s->initialize();
        while (!s->isReady())
            sleep(1);
    }
    catch(const xsp::RuntimeError& e) {
        cout << "ERROR: could not initialize system: " << e.what() << endl;
        return 1;
    }

    auto d = dynamic_pointer_cast<xsp::lambda::Detector>(s->detector("lambda"));
    auto r = dynamic_pointer_cast<xsp::lambda::Receiver>(s->receiver("lambda/1"));

    // Don't need to wait for RAM to be allocated, since one 64MB page is always
    // allocated. This is enough for a single image to store.
    //while (r->ramAllocated()) sleep(1);

    d->setFrameCount(n_frames);
    d->setThresholds(thresholds);
    d->setShutterTime(shutter_time);

    s->startAcquisition();
    cout << s->isBusy() << endl;
    sleep(timeout);
    cout << s->isBusy() << endl;

    cout << "texp=" << shutter_time << " ms" << endl;
    cout << "pause=" << timeout << " s" << endl;

    auto nbimg = 100;
    for (auto i = 0; i < nbimg; i++) {
        cout << "img N " << i + 1 << endl;

        try {
            s->startAcquisition();
        }
        catch(const xsp::RuntimeError& e) {
            cout << "ERROR: could not start system: " << e.what() << endl;
            return 1;
        }

        sleep(timeout);
        if (s->isBusy()) {
            count++;
            s->stopAcquisition();
            cout << "erreur N " << count << endl;
            sleep(timeout);
        }
    }

    cout << "done..." << endl;
    cout << "NB image total " << nbimg << endl;
    cout << "NB erreur total" << count << endl;

    s->disconnect();
    d = nullptr;
    r = nullptr;
    s = nullptr;
}

