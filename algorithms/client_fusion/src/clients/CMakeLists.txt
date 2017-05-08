set(clients_SRC global_func.cpp
				global_func.h
				socket_client.cpp
				socket_client.h
				sick_client.cpp
				sick_client.h
				odo_client.cpp
				odo_client.h
				sick_odo_client.cpp
				sick_odo_client.h
				client_interface.cpp
				client_interface.h
				xtion_client.cpp
				xtion_client.h
				mopenni2.cpp
				mopenni2.h
				client_main.cpp
	)
set(clients_LIBS ${NORMAL_INTER_LIB} ${NORMAL_LIB} ws2_32 OpenNI2 ${OpenCV_LIBS})

add_executable(clients_fusion ${clients_SRC})
target_link_libraries(clients_fusion ${clients_LIBS})