# Ctrl_CV_2026

g++ -std=c++17 -I include -I test/mock -o /tmp/test_protocol test/test_protocol.cpp && /tmp/test_protocol && \
g++ -std=c++17 -I include -I test/mock -o /tmp/test_kinematics test/test_kinematics.cpp && /tmp/test_kinematics && \
g++ -std=c++17 -I include -I test/mock -I lib/ChassisComms -o /tmp/test_chassis_comms test/test_chassis_comms.cpp lib/ChassisComms/ChassisComms.cpp && /tmp/test_chassis_comms