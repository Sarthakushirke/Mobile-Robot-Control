if(EXISTS "/home/sarthak/mrc/Mobile-Robot-Control/Week_2/mrc-navigation-assignment-1-main/build/tooltest[1]_tests.cmake")
  include("/home/sarthak/mrc/Mobile-Robot-Control/Week_2/mrc-navigation-assignment-1-main/build/tooltest[1]_tests.cmake")
else()
  add_test(tooltest_NOT_BUILT tooltest_NOT_BUILT)
endif()