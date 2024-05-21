import rospkg

def search_and_paste(file_path, search_word, paste_word):
    with open(file_path, 'r+') as file:
        lines = file.readlines()
        file.seek(0)  # Reset file pointer to the beginning

        for line in lines:
            file.write(line)  # Write the original line to the file

            if search_word in line:
                if not(paste_word + '\n' in lines):
                    file.write(paste_word + '\n')

# Print the path to the franka_gazebo

ros_pack = rospkg.RosPack()
franka_gazebo = ros_pack.get_path('franka_gazebo')

# MODIFY THE PACKAGE
file_path = franka_gazebo + '/package.xml'
print("Change files")
print(file_path)
existing_depend = '<depend>franka_example_controllers</depend>'
new_depend = '  <depend>franka_bimanual_controllers</depend>'

search_and_paste(file_path, existing_depend, new_depend)

# MODIFY CMAKE
file_path = franka_gazebo + '/CMakeLists.txt'
print("Change files in directory")
print(file_path)
existing_depend = 'franka_example_controllers'
new_depend = '  franka_bimanual_controllers'

search_and_paste(file_path, existing_depend, new_depend)