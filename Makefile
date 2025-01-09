.PHONY: cpp

cpp:
	@cmake -Bbuild cpp/kinematic_icp/
	@cmake --build build -j$(nproc --all)
