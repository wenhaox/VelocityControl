# CMake generated Testfile for 
# Source directory: /Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner
# Build directory: /Users/peter/Desktop/Research/zynq/build/jsoncpp/src/jsontestrunner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(jsoncpp_readerwriter "/opt/homebrew/Frameworks/Python.framework/Versions/3.13/bin/python3.13" "-B" "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "/Users/peter/Desktop/Research/zynq/build/jsoncpp/src/jsontestrunner/jsontestrunner_exe" "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter PROPERTIES  WORKING_DIRECTORY "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/data" _BACKTRACE_TRIPLES "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/CMakeLists.txt;43;add_test;/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/CMakeLists.txt;0;")
add_test(jsoncpp_readerwriter_json_checker "/opt/homebrew/Frameworks/Python.framework/Versions/3.13/bin/python3.13" "-B" "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/runjsontests.py" "--with-json-checker" "/Users/peter/Desktop/Research/zynq/build/jsoncpp/src/jsontestrunner/jsontestrunner_exe" "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/data")
set_tests_properties(jsoncpp_readerwriter_json_checker PROPERTIES  WORKING_DIRECTORY "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/../../test/data" _BACKTRACE_TRIPLES "/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/CMakeLists.txt;47;add_test;/Users/peter/Desktop/Research/zynq/jsoncpp/src/jsontestrunner/CMakeLists.txt;0;")
