/********************************************/ /**
*  testAcq.cpp
***********************************************/

/********************************************/ /**
*  DEPENDENCIES
***********************************************/
#include <iostream>
#include <unistd.h>
#include "libxsp.h"

/***************************************************************************************************
 * SDK MANAGEMENT
 **************************************************************************************************/
/* typedef unique_ptr<xsp::System> uptr_sys;
typedef shared_ptr<xsp::lambda::Detector> sptr_det;
typedef shared_ptr<xsp::Receiver> sptr_recv; */

using namespace xsp;
int main()
{
    // step 1: create system
    auto s = createSystem("/opt/xsp/config/system_Si.yml");
    if (s == nullptr)
    {
        std::cout << "No system created. Aborting..." << std::endl;
        return -1;
    }
    try
    {
        s->connect();
        s->initialize();
    }
    catch (const RuntimeError &e)
    {
        std::cout << "exception: " << e.what() << std::endl;
        return -1;
    }

    // step 2: get pointers and wait for RAM buffer
    auto d = s->detector("lambda");
    auto r = s->receiver("lambda/1");
    while (!r->ramAllocated())
        sleep(1);

    // step 3: set parameters
    try
    {
        d->setFrameCount(100);
        d->setShutterTime(0.5);
    }
    catch (const RuntimeError &e)
    {
        std::cout << "exception: " << e.what() << std::endl;
        return -1;
    }

    // step 4: run acquisition
    // Note: for Lambda detectors, applications need to wait until sensor HV
    // is set using the following loop:
    // ld = std::dynamic_pointer_cast<lambda::Detector>(d);
    // while (!ld->voltageSettled()) sleep(1);
    try
    {
        d->startAcquisition();
    }
    catch (const RuntimeError &e)
    {
        std::cout << "exception: " << e.what() << std::endl;
        return -1;
    }
    auto n_received = 0;
    while (true)
    {
        auto f = r->frame(1500);
        if (f != nullptr)
        {
            auto dp = reinterpret_cast<const std::uint16_t *>(f->data());
            auto size = f->size();
            // process data: e.g. std::memcpy(<dest>, dp, size);
            r->release(f);
            n_received++;
        }
        else
        {
            // timeout occured, when reading frames
            break;
        }
        if (n_received == 100)
            break;
    }
    // step 5: clean up
    s->disconnect();
    return 0;
}