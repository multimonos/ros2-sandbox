.PHONY: build

services:
	ros2 service list

call-sum2:
	echo '/sum2_server ...' \
	; echo '\ninterface:'; ros2 interface show example_interfaces/srv/AddTwoInts \
	; echo '\ntype:'; ros2 service type /sum2_server \
	; echo '\ncall:'; ros2 service call /sum2_server example_interfaces/srv/AddTwoInts "{a: 3, b: 7}" \
	; echo 'done'

reset-counter:
	ros2 service call /number_counter_reset example_interfaces/srv/SetBool "{data: true}"
