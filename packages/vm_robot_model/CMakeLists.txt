cmake_minimum_required(VERSION 3.8)
# 🌟 修正點 1 & 2：刪除重複行，並將專案名稱對齊資料夾與 package.xml
project(vm_robot_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找 ROS2 基礎編譯工具
find_package(ament_cmake REQUIRED)

# 📌 關鍵：安裝 launch, urdf, 以及新增的 rviz 資料夾到系統共享目錄中
install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
