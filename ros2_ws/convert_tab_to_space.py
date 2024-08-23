def convert_tabs_to_spaces(file_path, num_spaces=4):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            new_line = line.replace('\t', ' ' * num_spaces)
            file.write(new_line)

# 파일 경로 지정
#file_path = '/root/ros2_ws/src/mpu9250/mpu9250/imusensor/MPU9250/MPU9250.py'
file_path = '/root/ros2_ws/src/mpu9250/mpu9250/imusensor/filters/madgwick.py'
convert_tabs_to_spaces(file_path)
