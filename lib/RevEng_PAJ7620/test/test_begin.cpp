#include <ArduinoUnitTests.h>
#include "RevEng_PAJ7620.h"

RevEng_PAJ7620 *sensor;

unittest_setup()
{
  sensor = new RevEng_PAJ7620();
}

unittest_teardown()
{
  delete sensor;
  sensor = NULL;
}

unittest(empty_begin)
{
}

unittest_main()
