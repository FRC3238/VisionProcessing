#!/bin/bash

# location of run_test.py relative to this file
FAKE_WPILIB_DIR=../../fake-wpilib

# location of test modules relative to this file
TEST_MODULES=tests

# use pynetworktables for SmartDashboard/NetworkTables
PYNETWORKTABLES=True #False

FILE_PATH=`dirname $0`

python3 -B "${FILE_PATH}/${FAKE_WPILIB_DIR}/run_test.py" --use-pynetworktables=${PYNETWORKTABLES} --test-modules="${FILE_PATH}/${TEST_MODULES}" $@
exit $?

