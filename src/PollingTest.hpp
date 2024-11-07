#ifndef _POLLING_TEST_HPP_
#define _POLLING_TEST_HPP_

class PollingTest
{
   public:
    static void runAllTests();

   private:
    static void periodicPolling();

    static void dataAgeDetection();
};

#endif // IMODBUS_DEVICE_TEST_HPP
